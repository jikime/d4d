@echo off
REM Compile AprilTag to WebAssembly
REM This script requires Emscripten to be installed and activated

echo Compiling AprilTag to WebAssembly...

REM Set paths (adjust these to your specific paths)
set APRILTAG_SRC=..\vendor\apriltag
set OUTPUT_DIR=..\js

REM Create output directory if it doesn't exist
if not exist %OUTPUT_DIR% mkdir %OUTPUT_DIR%

REM Compile the code
emcc -O3 ^
  apriltag_wasm.c ^
  %APRILTAG_SRC%\apriltag.c ^
  %APRILTAG_SRC%\apriltag_quad_thresh.c ^
  %APRILTAG_SRC%\apriltag_pose.c ^
  %APRILTAG_SRC%\tag36h11.c ^
  %APRILTAG_SRC%\tag25h9.c ^
  %APRILTAG_SRC%\tag16h5.c ^
  %APRILTAG_SRC%\tagCircle21h7.c ^
  %APRILTAG_SRC%\tagCircle49h12.c ^
  %APRILTAG_SRC%\tagCustom48h12.c ^
  %APRILTAG_SRC%\tagStandard41h12.c ^
  %APRILTAG_SRC%\tagStandard52h13.c ^
  %APRILTAG_SRC%\common\g2d.c ^
  %APRILTAG_SRC%\common\getopt.c ^
  %APRILTAG_SRC%\common\homography.c ^
  %APRILTAG_SRC%\common\image_u8.c ^
  %APRILTAG_SRC%\common\image_u8x3.c ^
  %APRILTAG_SRC%\common\matd.c ^
  %APRILTAG_SRC%\common\pam.c ^
  %APRILTAG_SRC%\common\pjpeg.c ^
  %APRILTAG_SRC%\common\pjpeg-idct.c ^
  %APRILTAG_SRC%\common\pnm.c ^
  %APRILTAG_SRC%\common\string_util.c ^
  %APRILTAG_SRC%\common\svd22.c ^
  %APRILTAG_SRC%\common\time_util.c ^
  %APRILTAG_SRC%\common\unionfind.c ^
  %APRILTAG_SRC%\common\workerpool.c ^
  %APRILTAG_SRC%\common\zarray.c ^
  %APRILTAG_SRC%\common\zhash.c ^
  %APRILTAG_SRC%\common\zmaxheap.c ^
  %APRILTAG_SRC%\common\pthreads_cross.c ^
  -I%APRILTAG_SRC% ^
  -s WASM=1 ^
  -s ALLOW_MEMORY_GROWTH=1 ^
  -s INITIAL_MEMORY=335544320 ^
  -s MAXIMUM_MEMORY=536870912 ^
  -s MODULARIZE=1 ^
  -s EXPORT_NAME="AprilTagWasm" ^
  -s EXPORTED_FUNCTIONS="['_malloc', '_free', '_initialize_detector', '_set_detector_parameters', '_detect_tags', '_cleanup', '_add_family']" ^
  -s EXPORTED_RUNTIME_METHODS="['cwrap', 'setValue', 'getValue', 'UTF8ToString', 'HEAPU8']" ^
  -s ASSERTIONS=1 ^
  -s ENVIRONMENT="web,worker" ^
  -o %OUTPUT_DIR%\apriltag_wasm.js

echo Compilation complete!
if exist %OUTPUT_DIR%\apriltag_wasm.js (
  echo WebAssembly module built successfully!
  echo Output files:
  echo - %OUTPUT_DIR%\apriltag_wasm.js
  echo - %OUTPUT_DIR%\apriltag_wasm.wasm
) else (
  echo Failed to build WebAssembly module!
)