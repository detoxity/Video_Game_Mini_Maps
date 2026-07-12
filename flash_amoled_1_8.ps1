# Flash the ESP32-S3-Touch-AMOLED-1.8 board. Usage: .\flash_amoled_1_8.ps1 [COM port]
param([string]$Port = "COM15")
$T = "$HOME\.espressif\tools"
$env:IDF_PATH = "$HOME\esp\v5.5-rc1\esp-idf"
$env:PATH = "$T\cmake\3.30.2\bin;$T\ninja\1.12.1;$T\ccache\4.10.2\ccache-4.10.2-windows-x86_64;$T\xtensa-esp-elf\esp-14.2.0_20241119\xtensa-esp-elf\bin;$HOME\.espressif\python_env\idf5.5_py3.11_env\Scripts;$env:PATH"
$env:BOARD = "amoled_1_8"
Set-Location $PSScriptRoot
& "$HOME\.espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe" `
  "$env:IDF_PATH\tools\idf.py" -B build_amoled -D SDKCONFIG=build_amoled/sdkconfig -p $Port build flash
