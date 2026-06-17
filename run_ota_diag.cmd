@echo off
cd /d "%~dp0"
echo.
echo ESP32 Toilet - OTA / boot diagnostics (no trust handshake)
echo ============================================================
echo.
echo Before continuing:
echo   1. Power on the toilet and wait ~30 seconds for boot
echo   2. Disconnect the phone app (only one BLE client at a time)
echo   3. Stay within BLE range of this PC
echo.
pause
python get_ota_diag.py --scan-seconds 25
echo.
pause
