@echo off
REM ===========================================================================
REM  Build the receiver and produce the .hex and .uf2 for the dongle.
REM
REM  Usage:  receiver\tools\build.cmd
REM
REM  Why this script exists at all: the ARM toolchain has to be reachable while
REM  cargo runs. Here that is a hard requirement, not a convenience -
REM  p256-cortex-m4-sys (pulled in by RMK's crypto) shells out to
REM  arm-none-eabi-gcc, and on this machine the only one is the MSYS2 copy
REM  inside QMK_MSYS, which needs its own bin directories on PATH or it dies
REM  with STATUS_DLL_NOT_FOUND (0xc0000135).
REM
REM  The dongle is flashed with the nRF Connect Programmer (.hex), not by drag
REM  and drop: its bootloader is the Nordic USB DFU one, not a UF2 one.
REM ===========================================================================
setlocal
REM QMK_MSYS holds this machine's only arm-none-eabi-gcc, which RMK's crypto
REM build script (p256-cortex-m4-sys) shells out to. If it is not here, whatever
REM is already on PATH is used - CI installs gcc-arm-none-eabi, for instance.
if exist "C:\QMK_MSYS\mingw64\bin" set "PATH=C:\QMK_MSYS\mingw64\bin;C:\QMK_MSYS\opt\qmk\bin;C:\QMK_MSYS\usr\bin;%PATH%"

cd /d "%~dp0.." || exit /b 1

cargo build --release || exit /b 1
cargo objcopy --release -- -O ihex keypoint-nmk-receiver.hex || exit /b 1
cargo hex-to-uf2 --input-path keypoint-nmk-receiver.hex --output-path keypoint-nmk-receiver.uf2 --family nrf52840 || exit /b 1

echo.
echo   keypoint-nmk-receiver.hex   - flash with nRF Connect Programmer (this is the one to use)
echo   keypoint-nmk-receiver.uf2   - only for a dongle running a UF2 bootloader instead
echo.
endlocal