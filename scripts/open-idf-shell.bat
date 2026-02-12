@echo off
REM --- ESP-IDF version folder installed by Espressif installer ---
@REM Mở ESP-IDF 5.5.1 CMD property --> Target để lấy đường dẫn như sau C:\Espressif\idf_cmd_init.bat" esp-idf-29323a3f5a0574597d6dbaa0af20c775"
@REM --- Set the IDF version to use ---
@REM set IDF_VER=esp-idf-29323a3f5a0574597d6dbaa0af20c775 // 5.5.1
set IDF_VER=esp-idf-b29c58f93b4ca0f49cdfc4c3ef43b562

REM --- Load ESP-IDF environment correctly ---
call "C:\Espressif\idf_cmd_init.bat" %IDF_VER%

REM --- Change to project root directory (one level up from scripts folder) ---
cd ..

cmd