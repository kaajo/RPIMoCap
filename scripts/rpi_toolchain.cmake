message(STATUS "Using sysroot path: ${RASPBERRY_ROOT_FS}")

set(TOOLCHAIN_HOST "arm-linux-gnueabihf")

set(TOOLCHAIN_CC "${TOOLCHAIN_HOST}-gcc")
set(TOOLCHAIN_CXX "${TOOLCHAIN_HOST}-g++")
set(TOOLCHAIN_LD "${TOOLCHAIN_HOST}-ld")
set(TOOLCHAIN_AR "${TOOLCHAIN_HOST}-ar")
set(TOOLCHAIN_RANLIB "${TOOLCHAIN_HOST}-ranlib")
set(TOOLCHAIN_STRIP "${TOOLCHAIN_HOST}-strip")
set(TOOLCHAIN_NM "${TOOLCHAIN_HOST}-nm")

set(CMAKE_CROSSCOMPILING TRUE)
set(CMAKE_SYSROOT "${RASPBERRY_ROOT_FS}")

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR "armv7")
set(CMAKE_SYSTEM_VERSION 1)

set(CMAKE_C_COMPILER ${TOOLCHAIN_CC})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_CXX})

set(CMAKE_C_FLAGS "-mcpu=cortex-a53 -mfpu=neon-vfpv4 -mfloat-abi=hard ${COMMON_FLAGS}" CACHE STRING "Flags for Raspberry PI 3")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS}" CACHE STRING "Flags for Raspberry PI 3")

set(LIB_DIRS 
	"${RASPBERRY_ROOT_FS}/opt/vc/lib"
	"${RASPBERRY_ROOT_FS}/lib/${TOOLCHAIN_HOST}"
	"${RASPBERRY_ROOT_FS}/usr/lib"
	"${RASPBERRY_ROOT_FS}/usr/local/lib"
	"${RASPBERRY_ROOT_FS}/usr/lib/${TOOLCHAIN_HOST}"
    "${RASPBERRY_ROOT_FS}/usr/${TOOLCHAIN_HOST}/lib"
)

# You can additionally check the linker paths if you add the flags ' -Xlinker --verbose'
set(COMMON_FLAGS "-I${RASPBERRY_ROOT_FS}/usr/include ")
FOREACH(LIB ${LIB_DIRS})
	set(COMMON_FLAGS "${COMMON_FLAGS} -L${LIB} -Wl,-rpath-link,${LIB}")
ENDFOREACH()

set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${RASPBERRY_ROOT_FS}/usr/lib/${TOOLCHAIN_HOST}")
set(CMAKE_FIND_ROOT_PATH "${CMAKE_INSTALL_PREFIX};${CMAKE_PREFIX_PATH};${RASPBERRY_ROOT_FS}")

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

