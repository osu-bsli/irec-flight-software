cmake_minimum_required(VERSION 3.16)
project(irec-flight-software)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

find_package(PkgConfig)
# Order is important here: libgpiodcxx links to libgpiod, so libgpiod has to come AFTER libgpiodcxx
pkg_check_modules(deps IMPORTED_TARGET libgpiodcxx libgpiod)

add_executable(irec-flight-software)

target_compile_options(irec-flight-software PRIVATE
	-ggdb
	#	-fsanitize=address
)

target_link_libraries(irec-flight-software PRIVATE
    PkgConfig::deps
    -static-libstdc++ -static-libgcc
    wiringPi
    #-fsanitize=address
)

add_subdirectory(src)
