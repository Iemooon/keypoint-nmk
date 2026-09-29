@echo off
REM ===========================================================================
REM  Build the keypoint-nmk receiver for ALL THREE boards, from one source tree.
REM
REM  Usage:  receiver\tools\build-variants.cmd
REM
REM  The three boards, in the owner's terms:
REM    PCA10059        - the Nordic nRF52840 Dongle. This is the board the
REM                      receiver was developed on, and it gets NO override file:
REM                      board.toml's own [memory]/[flash]/[storage] are its layout,
REM                      so the default build IS the PCA10059 build. (Until
REM                      2026-09-26 this variant was called "52840-dongle"; the
REM                      file name follows the board's real name now.)
REM    52840-nicenano  - nice!nano v2, Adafruit UF2 bootloader.
REM    52833-nicenano  - the 52833 board the owner calls "blue macro".
REM
REM  A plain `receiver\tools\build.cmd` still builds just the PCA10059 image under the
REM  plain name keypoint-nmk-receiver.hex/.uf2. This script turns the board into a build
REM  INPUT rather than a source edit.
REM
REM  What varies between the three is ONLY the bootloader/chip layout, and that
REM  comes from one override file per board (rx\boards\*.toml). board.toml stays
REM  the single source of truth for the keyboard and the link - the matrix, the
REM  channel table, the USB identity and the storage semantics are not
REM  duplicated anywhere, so they cannot drift between the boards.
REM
REM  Why the ARM toolchain has to be on PATH: p256-cortex-m4-sys (pulled in by
REM  RMK's crypto) shells out to arm-none-eabi-gcc, and on this machine the only
REM  one is the MSYS2 copy inside QMK_MSYS, which needs its own bin directories
REM  on PATH or it dies with STATUS_DLL_NOT_FOUND (0xc0000135).
REM
REM  The order matters for build time: the two nRF52840 boards are built
REM  back-to-back (one feature set) and then the nRF52833 board, so cargo
REM  rebuilds the whole dependency graph twice, not three times.
REM ===========================================================================
setlocal
REM QMK_MSYS holds this machine's only arm-none-eabi-gcc, which RMK's crypto
REM build script (p256-cortex-m4-sys) shells out to. If it is not here, whatever
REM is already on PATH is used - CI installs gcc-arm-none-eabi, for instance.
if exist "C:\QMK_MSYS\mingw64\bin" set "PATH=C:\QMK_MSYS\mingw64\bin;C:\QMK_MSYS\opt\qmk\bin;C:\QMK_MSYS\usr\bin;%PATH%"

cd /d "%~dp0.." || exit /b 1

REM   variant              board override file                    cargo chip feature
call :build PCA10059       ""                                   nrf52840
if errorlevel 1 exit /b 1
call :build 52840-nicenano "boards\nrf52840-nicenano.toml"      nrf52840
if errorlevel 1 exit /b 1
call :build 52833-nicenano "boards\nrf52833-nicenano.toml"      nrf52833
if errorlevel 1 exit /b 1

echo.
echo === verifying the three products against the layouts they claim ===
python tools\verify_variant.py
if errorlevel 1 exit /b 1

echo.
echo   keypoint-nmk-receiver-PCA10059.hex        - nRF Connect Programmer (Nordic dongle)
echo   keypoint-nmk-receiver-52840-nicenano.uf2  - drag onto the UF2 drive
echo   keypoint-nmk-receiver-52833-nicenano.uf2  - drag onto the UF2 drive (blue macro)
echo.
echo   Every board also gets the other format (.hex and .uf2). Use the one above:
echo   a .uf2 is only meaningful on a board that has a UF2 bootloader.
endlocal
exit /b 0


REM ---------------------------------------------------------------------------
REM  :build <variant> <override|""> <chip>
REM
REM  setlocal/endlocal around each board is load-bearing, not decoration:
REM  BOARD_OVERRIDE has to disappear again, or the next board would inherit the
REM  previous board's layout.
REM ---------------------------------------------------------------------------
:build
setlocal
set "VARIANT=%~1"
set "OVERRIDE=%~2"
set "CHIP=%~3"
set "PROD=keypoint-nmk-receiver-%VARIANT%"
if "%OVERRIDE%"=="" (set "BOARD_OVERRIDE=") else (set "BOARD_OVERRIDE=%OVERRIDE%")

echo.
echo === %VARIANT% : chip=%CHIP%  override=%OVERRIDE% ===

cargo build --release --no-default-features --features %CHIP% || exit /b 1
cargo objcopy --release --no-default-features --features %CHIP% -- -O ihex %PROD%.hex || exit /b 1
cargo hex-to-uf2 --input-path %PROD%.hex --output-path %PROD%.uf2 --family %CHIP% || exit /b 1

endlocal
exit /b 0