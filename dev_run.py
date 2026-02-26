"""
Development launcher: runs the Flask web server on x86 by mocking
hardware-only modules (gpiod, periphery) that require Rockchip SBC hardware.
This file does NOT modify any existing source code.
"""
import sys
from unittest.mock import MagicMock
from threading import Thread

mock_gpiod = MagicMock()
mock_gpiod.Chip.OPEN_BY_NUMBER = 0
mock_gpiod.LINE_REQ_DIR_OUT = 1
sys.modules["gpiod"] = mock_gpiod

mock_periphery = MagicMock()
sys.modules["periphery"] = mock_periphery

from ws_main import (  # noqa: E402
    app,
    init_motor,
    init_ina,
    check_heartbeats,
    start_websocket_server,
)

if __name__ == "__main__":
    init_motor()
    init_ina()
    Thread(target=check_heartbeats, daemon=True).start()
    Thread(target=start_websocket_server, daemon=True).start()
    app.run(host="0.0.0.0", port=9090, debug=False)
