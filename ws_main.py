import atexit
import subprocess
import threading
import time
import asyncio
import websockets
from threading import Thread
import json
import os
import ssl
import logging

import psutil
from flask import Flask, jsonify, render_template, request, send_from_directory
from motor import TB6612MotorDriver
from vc import INA226

app = Flask(__name__)
app.config['TEMPLATES_AUTO_RELOAD'] = True
motor = None
ina = None
mediamtx_proc = None
ffmpeg_proc = None
is_active = False
client_heartbeats = {}  # 存储客户端 ID 和最后心跳时间
client_heartbeats_lock = threading.Lock()  # 线程锁保护字典
# current_action = None  # 跟踪当前动作，防止重复

# 视频码率（Mbps），用于替换 ffmpeg 命令中的 -b:v 与 -bufsize，默认 1Mbps
video_bitrate_mbps = int(os.getenv("IPC_CAR_BITRATE_MBPS", "1"))

# SSL_PEM_FILE = r"your pem file path"
# SSL_KEY_FILE = r"you key file path"

# 心跳与推流启动宽限配置
HEARTBEAT_TIMEOUT_SECONDS = int(os.getenv("IPC_CAR_HEARTBEAT_TIMEOUT", "5"))
STARTUP_GRACE_SECONDS = int(os.getenv("IPC_CAR_STARTUP_GRACE", "5"))
grace_until_ts = 0.0  # 在此时间点之前，即使没有心跳也不判定断连

# ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
# ssl_ctx.load_cert_chain(SSL_PEM_FILE, SSL_KEY_FILE)

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)

formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
console_handler.setFormatter(formatter)

def init_motor():
    global motor
    if motor is None:
        try:
            motor = TB6612MotorDriver()
            logger.info("Motor initialized successfully")
        except Exception:
            logger.exception("Failed to initialize motor")
            raise

def init_ina():
    global ina
    if ina is None:
        try:
            ina = INA226()
            ina.configure()
            logger.info("INA226 initialized successfully")
        except Exception:
            logger.exception("Failed to initialize INA226")
            raise

def motor_forward():
    logger.info("Moving forward")
    if motor:
        motor.forward()

def motor_backward():
    logger.info("Moving backward")
    if motor:
        motor.backward()

def motor_left():
    logger.info("Turning left")
    if motor:
        motor.left()

def motor_right():
    logger.info("Turning right")
    if motor:
        motor.right()

def motor_stop():
    logger.info("Stopping")
    if motor:
        motor.stop()

def motor_set_speed(speed: int):
    logger.info(f"Set speed {speed}")
    if motor:
        motor.set_speed(speed / 100)

def motor_set_laser(state: int):
    logger.info(f"Set laser {state}")
    if motor:
        motor.set_laser(state)

def cleanup_processes():
    global mediamtx_proc, ffmpeg_proc
    for proc in [mediamtx_proc, ffmpeg_proc]:
        if proc and psutil.pid_exists(proc.pid):
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except psutil.TimeoutExpired:
                proc.kill()
    mediamtx_proc = None
    ffmpeg_proc = None

def apply_bitrate_to_cmd(cmd: list, mbps: int) -> list:
    """将 ffmpeg 命令数组中的 -b:v 与 -bufsize 值替换为给定 Mbps 值。"""
    new_cmd = list(cmd)
    def replace_opt(opt: str, new_value: str) -> None:
        if opt in new_cmd:
            idx = new_cmd.index(opt)
            if idx + 1 < len(new_cmd):
                new_cmd[idx + 1] = new_value
    bitrate_value = f"{mbps}M"
    replace_opt("-b:v", bitrate_value)
    replace_opt("-bufsize", bitrate_value)
    return new_cmd

def kill_existing_processes(process_name):
    for proc in psutil.process_iter(['name', 'pid']):
        if proc.info['name'].lower() == process_name.lower():
            try:
                psutil.Process(proc.info['pid']).terminate()
                psutil.Process(proc.info['pid']).wait(timeout=3)
            except psutil.TimeoutExpired:
                psutil.Process(proc.info['pid']).kill()
            except Exception as e:
                logger.error(f"Error killing {process_name} (pid {proc.info['pid']}): {e}")

def run_rtcwake_after_delay(seconds: int, delay_seconds: float = 1.0) -> None:
    """延时后执行 rtcwake，并捕获输出记录到日志，避免阻塞请求线程。"""
    def worker():
        try:
            time.sleep(delay_seconds)
            # 显式解析 rtcwake 绝对路径，避免 PATH 差异
            rtcwake_bin = '/usr/sbin/rtcwake' if os.path.exists('/usr/sbin/rtcwake') else '/sbin/rtcwake'
            proc = subprocess.Popen(
                [rtcwake_bin, '-m', 'mem', '-s', str(seconds)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            if proc.stdout:
                for line in proc.stdout:
                    logger.info(f"rtcwake: {line.strip()}")
            proc.wait()
            logger.info(f"rtcwake exited with code {proc.returncode}")
        except Exception as e:
            logger.exception(f"rtcwake exec error: {e}")

    Thread(target=worker, daemon=True).start()

def check_heartbeats():
    global is_active
    while True:
        with client_heartbeats_lock:
            current_time = time.time()
            disconnected_clients = []
            for client_id, last_time in client_heartbeats.items():
                # print(client_id, current_time, current_time - last_time)
                if current_time - last_time > HEARTBEAT_TIMEOUT_SECONDS:
                    disconnected_clients.append(client_id)

            for client_id in disconnected_clients:
                logger.info(f"Client {client_id} disconnected")
                del client_heartbeats[client_id]
            # print(client_heartbeats, is_active)
            if not client_heartbeats and is_active:
                # 推流启动后的宽限期内，不因临时无心跳而清理
                if current_time <= grace_until_ts:
                    pass
                else:
                    logger.info("No active clients (beyond grace), stopping motor and stream")
                    motor_stop()
                    motor_set_laser(0)
                    cleanup_processes()
                    is_active = False
        time.sleep(0.5)

async def websocket_handler(websocket):
    # global current_action
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                action = data.get('action')
                if action:
                    if action == 'forward':
                        motor_forward()
                    elif action == 'backward':
                        motor_backward()
                    elif action == 'left':
                        motor_left()
                    elif action == 'right':
                        motor_right()
                    elif action == 'stop':
                        motor_stop()
                    else:
                        await websocket.send(json.dumps({'status': 'error', 'message': 'Invalid action'}))
                        continue
                    await websocket.send(json.dumps({'status': 'ok'}))
            except json.JSONDecodeError:
                logger.warning("Invalid JSON received")
                await websocket.send(json.dumps({'status': 'error', 'message': 'Invalid JSON'}))
            except Exception as e:
                logger.exception(f"WebSocket error: {e}")
                await websocket.send(json.dumps({'status': 'error', 'message': str(e)}))
    except websockets.exceptions.ConnectionClosed:
        logger.info("WebSocket client disconnected, stopping motor")
        motor_stop()
        # cleanup_processes()

@app.route('/')
def index():
    global video_bitrate_mbps
    video_bitrate_mbps = 1
    motor_set_speed(20)
    return render_template('ws_index.html')

# 启用PWA需要ssl支持
# @app.route('/manifest.webmanifest')
# def manifest():
#     return send_from_directory('templates', 'manifest.webmanifest', mimetype='application/manifest+json')

@app.route('/favicon.ico')
def favicon():
    # 使用现有 PNG 作为站点图标，避免 404
    return send_from_directory('static', 'icon_128.png', mimetype='image/png')

@app.route('/control', methods=['POST'])
def control():
    action = request.json.get('action')
    if action == 'stop':
        motor_stop()
    else:
        return jsonify({'status': 'error', 'message': 'Invalid action'}), 400
    return jsonify({'status': 'ok'})

@app.route('/set_speed', methods=['POST'])
def set_speed():
    speed = request.json.get('speed')
    if not isinstance(speed, int) or speed < 10 or speed > 60:
        return jsonify({'status': 'error', 'message': 'Speed must be an integer between 10 and 60'}), 400
    motor_set_speed(speed)
    return jsonify({'status': 'ok'})

@app.route('/set_laser', methods=['POST'])
def set_laser():
    state = request.json.get('state')
    if state not in [0, 1]:
        return jsonify({'status': 'error', 'message': 'State must be 0 or 1'}), 400
    motor_set_laser(state)
    return jsonify({'status': 'ok'})

@app.route('/set_bitrate', methods=['POST'])
def set_bitrate():
    """设置推流码率（Mbps）。允许值：1/2/4/8。"""
    global video_bitrate_mbps
    try:
        data = request.json or {}
        mbps = data.get('mbps')
        if isinstance(mbps, str):
            try:
                mbps = int(mbps)
            except Exception:
                return jsonify({'status': 'error', 'message': 'mbps must be integer'}), 400
        if not isinstance(mbps, int):
            return jsonify({'status': 'error', 'message': 'mbps must be integer'}), 400
        if mbps not in [1, 2, 4, 8]:
            return jsonify({'status': 'error', 'message': 'mbps must be one of 1,2,4,8'}), 400
        video_bitrate_mbps = mbps
        logger.info(f"Video bitrate set to {video_bitrate_mbps} Mbps")
        return jsonify({'status': 'ok'})
    except Exception as e:
        logger.exception(f"Error setting bitrate: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/status_and_heartbeat', methods=['POST'])
def status_and_heartbeat():
    global is_active
    try:
        data = request.json
        client_id = data.get('client_id')
        if not client_id:
            return jsonify({'status': 'error', 'message': 'Missing client_id'}), 400
        with client_heartbeats_lock:
            client_heartbeats[client_id] = time.time()
            is_active = True
        t2 = int(time.time() * 1000)  # 服务器接收时间 (t2, 毫秒)
        if not ina:
            return jsonify({'status': 'error', 'message': 'INA226 not initialized'}), 500
        try:
            bus_voltage = ina.bus_voltage
            current = ina.current
            cpu_percent = psutil.cpu_percent(interval=0.1)
            mem = psutil.virtual_memory()
            mem_percent = mem.percent
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                cpu_temp = int(f.read().strip()) / 1000
            # battery_percent = calculate_battery_percent(bus_voltage)
            t3 = int(time.time() * 1000)  # 服务器发送时间 (t3, 毫秒)
            return jsonify({
                'status': 'ok',
                'bus_voltage': f"{bus_voltage:.3f}",
                'current': f"{current:.3f}",
                'cpu_percent': f"{cpu_percent:.1f}",
                'mem_percent': f"{mem_percent:.1f}",
                'cpu_temp': f"{cpu_temp:.1f}",
                # 'battery_percent': f"{battery_percent:.1f}",
                't2': t2,
                't3': t3
            })
        except Exception as e:
            logger.exception(f"Error getting status: {e}")
            return jsonify({'status': 'error', 'message': str(e)}), 500
    except Exception as e:
        logger.exception(f"Status and heartbeat error: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/start_stream', methods=['POST'])
def start_stream():
    global mediamtx_proc, ffmpeg_proc, grace_until_ts
    try:

        if mediamtx_proc and psutil.pid_exists(mediamtx_proc.pid) and ffmpeg_proc and psutil.pid_exists(ffmpeg_proc.pid):
            return jsonify({'status': 'ok', 'message': 'Already streaming'})
        
        kill_existing_processes('mediamtx')
        kill_existing_processes('ffmpeg')
        
        mediamtx_proc = subprocess.Popen(['/home/radxa/rk3566-ipc-car-public/rtsp/mediamtx', '/home/radxa/rk3566-ipc-car-public/rtsp/mediamtx.yml'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        for _ in range(10):
            if psutil.pid_exists(mediamtx_proc.pid):
                break
            time.sleep(0.1)
        else:
            return jsonify({'status': 'error', 'message': 'Failed to start MediaMTX'}), 500
        try:
            with open('/home/radxa/rk3566-ipc-car-public/rtsp/ffmpeg_cmd.json', 'r') as f:
                ffmpeg_cmd = json.load(f)['cmd']
        except Exception as e:
            logger.exception(f"Error loading ffmpeg_cmd.json: {e}")
            ffmpeg_cmd = [
                "ffmpeg",
                "-f", "v4l2",
                "-input_format", "nv12",
                "-framerate", "30",
                "-video_size", "1920x1440",
                "-i", "/dev/video0",
                "-vf", "fps=30",
                "-c:v", "h264_rkmpp",
                "-profile:v", "main",
                "-level", "4.2",
                "-rc_mode", "CBR",
                "-b:v", "1M",
                "-g", "15",
                "-bufsize", "1M",
                "-flags", "low_delay",
                "-tune", "zerolatency",
                "-preset", "ultrafast",
                "-f", "rtsp",
                "rtsp://0.0.0.0:8554/stream"
            ]
        # 应用码率设置（替换 -b:v 与 -bufsize）
        ffmpeg_cmd = apply_bitrate_to_cmd(ffmpeg_cmd, video_bitrate_mbps)
        ffmpeg_proc = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        for _ in range(10):
            if psutil.pid_exists(ffmpeg_proc.pid):
                break
            time.sleep(0.1)
        else:
            cleanup_processes()
            return jsonify({'status': 'error', 'message': 'Failed to start FFmpeg'}), 500
        # 启动成功后设置一个启动宽限期，避免客户端在 HLS 就绪期间心跳短暂中断被判定为离线
        grace_until_ts = time.time() + STARTUP_GRACE_SECONDS
        logger.info(f"Stream start grace until {grace_until_ts:.0f} (epoch seconds), window={STARTUP_GRACE_SECONDS}s")

        return jsonify({'status': 'ok'})
    except Exception as e:
        logger.exception(f"Error starting stream: {e}")
        cleanup_processes()
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/stop_stream', methods=['POST'])
def stop_stream():
    try:
        data = request.json
        client_id = data.get("client_id")
        if not client_id:
            return jsonify({'status': 'error', 'message': 'Missing client_id'}), 400
        with client_heartbeats_lock:
            active_clients = list(client_heartbeats.keys())  # 创建键的快照
            if len(active_clients) == 1 and client_id in active_clients:
                logger.info(f"Only client {client_id} is active, stopping stream")
                cleanup_processes()
                return jsonify({'status': 'ok', 'message': 'Stream stopped'})
            else:
                logger.info(f"Client {client_id} requested stop, but other clients are active: {active_clients}")
                return jsonify({'status': 'ok', 'message': 'Stream not stopped, other clients active'})
    except Exception as e:
        logger.exception(f"Error processing stop stream request: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/sleep', methods=['POST'])
def sleep():
    try:
        data = request.json or {}
        hours = data.get('hours')
        # 仅支持整数小时，范围 1-72 小时
        if isinstance(hours, float):
            hours = int(hours)
        if not isinstance(hours, int):
            try:
                hours = int(hours)
            except Exception:
                return jsonify({'status': 'error', 'message': 'hours must be integer'}), 400
        if hours < 1 or hours > 72:
            return jsonify({'status': 'error', 'message': 'hours must be between 1 and 72'}), 400

        # 安全处理：先停止运动与关闭外设、清理推流
        motor_stop()
        motor_set_laser(0)
        cleanup_processes()

        seconds = hours * 3600
        logger.info(f"Scheduling rtcwake after {seconds}s (hours={hours})")

        # 后台线程触发 rtcwake，并捕获输出
        run_rtcwake_after_delay(seconds, delay_seconds=1.0)

        return jsonify({'status': 'ok'})
    except Exception as e:
        logger.exception(f"Error scheduling sleep: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

async def run_websocket_server():
    # async with websockets.serve(websocket_handler, "0.0.0.0", 8080, ssl=ssl_ctx):
    async with websockets.serve(websocket_handler, "0.0.0.0", 8080):
        await asyncio.Future()  # 保持运行

def start_websocket_server():
    asyncio.run(run_websocket_server())

if __name__ == '__main__':
    try:
        init_motor()
        init_ina()
        atexit.register(cleanup_processes)
        Thread(target=check_heartbeats, daemon=True).start()
        Thread(target=start_websocket_server, daemon=True).start()
        # logger.info("Starting Flask server on 0.0.0.0:5000")
        # context = (SSL_PEM_FILE, SSL_KEY_FILE)
        # app.run(host='0.0.0.0', port=443, debug=False, ssl_context=context)
        app.run(host='0.0.0.0', port=9090, debug=False)
    except KeyboardInterrupt:
        logger.info("Shutting down")
        cleanup_processes()
    except Exception as e:
        logger.exception(f"Startup error: {e}")
        cleanup_processes()