/*
正在执行任务: C:\Users\17831\ESPIDF\esp\tools\tools\cmake\3.24.0\bin\cmake.exe -G=Ninja -DPYTHON_DEPS_CHECKED=1 -DESP_PLATFORM=1 -B=c:\Users\17831\ESPIDF\esp\frameworks\esp-idf-v5.2.2\examples\bluetooth\bluedroid\ble\gatt_server\build -S=c:\Users\17831\ESPIDF\esp\frameworks\esp-idf-v5.2.2\examples\bluetooth\bluedroid\ble\gatt_server 

-- Found Git: C:/Users/17831/ESPIDF/esp/tools/tools/idf-git/2.39.2/cmd/git.exe (found version "2.39.2.windows.1") 
-- The C compiler identification is GNU 13.2.0
-- The CXX compiler identification is GNU 13.2.0
-- The ASM compiler identification is GNU
-- Found assembler: C:/Users/17831/ESPIDF/esp/tools/tools/xtensa-esp-elf/esp-13.2.0_20230928/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc.exe
-- Detecting C compiler ABI info
CMake Error:
  Running

   'C:/Users/17831/ESPIDF/esp/tools/tools/ninja/1.11.1/ninja.exe' '-C' 'C:/Users/17831/ESPIDF/esp/frameworks/esp-idf-v5.2.2/examples/bluetooth/bluedroid/ble/gatt_server/build/CMakeFiles/CMakeTmp' '-t' 'restat' 'build.ninja'

  failed with:

   ninja: error: failed recompaction: Permission denied



CMake Error at C:/Users/17831/ESPIDF/esp/tools/tools/cmake/3.24.0/share/cmake-3.24/Modules/CMakeDetermineCompilerABI.cmake:57 (try_compile):
  Failed to generate test project build system.
Call Stack (most recent call first):
  C:/Users/17831/ESPIDF/esp/tools/tools/cmake/3.24.0/share/cmake-3.24/Modules/CMakeTestCCompiler.cmake:26 (CMAKE_DETERMINE_COMPILER_ABI)
  C:/Users/17831/ESPIDF/esp/frameworks/esp-idf-v5.2.2/tools/cmake/project.cmake:506 (__project)
  CMakeLists.txt:6 (project)


-- Configuring incomplete, errors occurred!
See also "C:/Users/17831/ESPIDF/esp/frameworks/esp-idf-v5.2.2/examples/bluetooth/bluedroid/ble/gatt_server/build/CMakeFiles/CMakeOutput.log".

 *  终端进程“C:\Users\17831\ESPIDF\esp\tools\tools\cmake\3.24.0\bin\cmake.exe '-G=Ninja', '-DPYTHON_DEPS_CHECKED=1', '-DESP_PLATFORM=1', '-B=c:\Users\17831\ESPIDF\esp\frameworks\esp-idf-v5.2.2\examples\bluetooth\bluedroid\ble\gatt_server\build', '-S=c:\Users\17831\ESPIDF\esp\frameworks\esp-idf-v5.2.2\examples\bluetooth\bluedroid\ble\gatt_server'”已终止，退出代码: 1。 
 */