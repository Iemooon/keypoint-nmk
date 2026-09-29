@echo off
REM ===========================================================================
REM  Build both transmitter halves and produce the two UF2 files.
REM
REM  Usage:  keyboard\tools\build.cmd                 (nRF52840, the default)
REM          keyboard\tools\build.cmd nrf52833         (other chip: needs matching
REM                                              [chip] name in board.toml)
REM
REM  Why this script exists at all: the ARM toolchain has to be reachable while
REM  cargo runs (build scripts may shell out to a C compiler), and on this
REM  machine the only arm-none-eabi-gcc is the MSYS2 one inside QMK_MSYS - it
REM  needs its own bin directories on PATH or it dies with STATUS_DLL_NOT_FOUND
REM  (0xc0000135). Doing that here keeps the build reproducible from one place
REM  instead of living in whatever shell happened to be open.
REM
REM  The UF2 family id passed to cargo-hex-to-uf2 must match the chip:
REM    nrf52840 -> 0xada52840      nrf52833 -> 0x621e937a
REM ===========================================================================
setlocal
REM QMK_MSYS holds this machine's only arm-none-eabi-gcc, which RMK's crypto
REM build script (p256-cortex-m4-sys) shells out to. If it is not here, whatever
REM is already on PATH is used - CI installs gcc-arm-none-eabi, for instance.
if exist "C:\QMK_MSYS\mingw64\bin" set "PATH=C:\QMK_MSYS\mingw64\bin;C:\QMK_MSYS\opt\qmk\bin;C:\QMK_MSYS\usr\bin;%PATH%"

if not "%~1"=="" (
    set "FEATURE=%~1"
) else (
    set "FEATURE=nrf52840"
)

cd /d "%~dp0.." || exit /b 1

if "%FEATURE%"=="nrf52840" (
    cargo build --release || exit /b 1
) else (
    cargo build --release --no-default-features --features %FEATURE% || exit /b 1
)

cargo objcopy --release --bin left  -- -O ihex keypoint-nmk-left.hex  || exit /b 1
cargo objcopy --release --bin right -- -O ihex keypoint-nmk-right.hex || exit /b 1

cargo hex-to-uf2 --input-path keypoint-nmk-left.hex  --output-path keypoint-nmk-left.uf2  --family %FEATURE% || exit /b 1
cargo hex-to-uf2 --input-path keypoint-nmk-right.hex --output-path keypoint-nmk-right.uf2 --family %FEATURE% || exit /b 1

echo.
echo   keypoint-nmk-left.uf2    - flash to the LEFT half
echo   keypoint-nmk-right.uf2   - flash to the RIGHT half
echo.
echo   Flashing: double-tap the half's reset button (or hold it while plugging
echo   in), then drag the UF2 onto the BOOT drive that appears.
echo.
endlocal