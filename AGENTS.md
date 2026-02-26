# AGENTS.md

## Cursor Cloud specific instructions

### Project overview

Python Flask + WebSocket IoT project for a WiFi/4G remote-controlled camera car on Radxa ZERO 3W (RK3566). See `README.md` for hardware requirements and wiring.

### Key services

| Service | Port | Entry point |
|---------|------|-------------|
| Flask HTTP server | 9090 | `python ws_main.py` |
| WebSocket server | 8080 | Started as daemon thread inside `ws_main.py` |
| mediamtx (RTSP→HLS) | 8554/8888 | `rtsp/mediamtx rtsp/mediamtx.yml` |
| ffmpeg (camera capture) | — | Launched by `/start_stream` endpoint |

### Running on x86 (Cloud Agent VM)

The project requires Rockchip SBC hardware (GPIO, PWM, I2C, camera). On x86:

- **Use `python dev_run.py`** instead of `python ws_main.py` — it mocks `gpiod` and `periphery` hardware modules so Flask + WebSocket servers start normally.
- All HTTP API endpoints work (control, speed, laser, bitrate, sleep). Hardware commands are no-ops via mocks.
- Video streaming (`/start_stream`) will fail because `mediamtx` and `ffmpeg-rockchip` are ARM-only binaries.
- The `gpiod` pip package (v2.x) has a different API than the v1.x bindings the code uses; this only matters on actual hardware.

### Linting

```
flake8 --max-line-length=150 --ignore=E402,W503,E501 ws_main.py motor.py vc.py
```

Style warnings are present but no blocking errors.

### Dependencies (pip)

`Flask`, `websockets`, `psutil`, `python-periphery`, `gpiod` — no `requirements.txt` exists; documented in README step 4.

### Notes

- The frontend loads Tailwind CSS and hls.js from CDN, so internet access is needed for proper rendering.
- WebSocket URL in `ws_index.html` is hardcoded to `10.0.0.229:8080` — on local dev, the WebSocket indicator shows "disconnected" in the browser, but the server itself is functional (testable via CLI).
- No automated test suite exists in this repo.
