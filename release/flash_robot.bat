@echo off
chcp 65001 > nul
echo =======================================================================
echo          SONIC ROBOT - CÔNG CỤ NẠP FIRMWARE ALL-IN-ONE (ESP32-S3)
echo =======================================================================
echo.
echo Vui lòng đảm bảo:
echo 1. Robot ESP32-S3 đã cắm cáp USB vào máy tính.
echo 2. Đã cài Python (hoặc có sẵn esptool).
echo.
set /p COMPORT="Nhập cổng COM của robot (ví dụ COM3, COM5, COM7) [Mặc định COM5]: "
if "%COMPORT%"=="" set COMPORT=COM5

echo.
echo Đang nạp firmware vào %COMPORT%... Vui lòng không rút cáp!
echo.

python -m esptool --chip esp32s3 -p %COMPORT% -b 921600 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size 16MB --flash_freq 80m 0x0 sonic_robot_firmware_all_in_one_16MB.bin

if %ERRORLEVEL% EQU 0 (
    echo.
    echo =======================================================================
    echo    [THÀNH CÔNG] Firmware đã được nạp hoàn tất! Robot đang khởi động lại.
    echo =======================================================================
) else (
    echo.
    echo =======================================================================
    echo    [LỖI] Không thể nạp firmware. Vui lòng kiểm tra lại cổng COM và cáp kết nối.
    echo =======================================================================
)
echo.
pause
