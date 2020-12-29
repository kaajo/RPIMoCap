#!/bin/sh
set -e
set -o xtrace

ROOTFS=/media/kaajo/rootfs/
INSTALL_PATH=${ROOTFS}/opt/rpimocap/
PROJECT_DIR=$(dirname "$(readlink -f "$0")")/../

echo "Running RPIMoCap client deploy script from ${PROJECT_DIR} to ${INSTALL_PATH}"

#TODO check this step
sudo ln -sfn /usr/arm-linux-gnueabihf/lib/ld-linux-armhf.so.3 /lib/ld-linux-armhf.so.3

RPATH_FLAGS="-DCMAKE_SKIP_BUILD_RPATH=FALSE -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=TRUE"
TOOLCHAIN_FLAGS="-DCMAKE_TOOLCHAIN_FILE=${PROJECT_DIR}/scripts/rpi_toolchain.cmake"

export CXX=/usr/bin/arm-linux-gnueabihf-g++-8
export CC=/usr/bin/arm-linux-gnueabihf-gcc-8

export PKG_CONFIG_DIR=
export PKG_CONFIG_LIBDIR="${ROOTFS}/usr/lib/arm-linux-gnueabihf/pkgconfig/":"${ROOTFS}/usr/lib/pkgconfig":"${ROOTFS}/usr/share/pkgconfig"
export PKG_CONFIG_SYSROOT_DIR="${ROOTFS}"

sudo mkdir -m 666 -p $INSTALL_PATH
cd ${PROJECT_DIR}
rm -r build-cross
mkdir -p build-cross
cd build-cross
cmake -DRASPBERRY_ROOT_FS=${ROOTFS} -DCMAKE_INSTALL_PREFIX=${INSTALL_PATH} -DCMAKE_BUILD_TYPE=Release -DENABLE_SIM=OFF -DENABLE_TESTS=OFF ${RPATH_FLAGS} ${TOOLCHAIN_FLAGS} ../RPIMoCap/

make -j 4
sudo make install
sudo chmod -R 777 ${INSTALL_PATH}

