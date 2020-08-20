#!/bin/sh
set -e
set -o xtrace

rootfs_path=/media/kaajo/writable
architecture=arm

sudo apt update
sudo apt install -y qemu qemu-user-static binfmt-support libstdc++-9-dev-armhf-cross
sudo cp /usr/bin/qemu-${architecture}-static ${rootfs_path}/usr/bin/

# mount all the things!
! mountpoint -q ${rootfs_path}/dev && sudo mount -o bind /dev ${rootfs_path}/dev
! mountpoint -q ${rootfs_path}/run && echo "" #sudo mount -o bind /run ${rootfs_path}/run
! mountpoint -q ${rootfs_path}/proc && echo "" #sudo mount -o bind /proc ${rootfs_path}/proc
! mountpoint -q ${rootfs_path}/sys && echo "" #sudo mount -o bind /sys ${rootfs_path}/sys

#sudo mount -o bind /dev/pts ${rootfs_path}/dev/pts

QEMU_COMMAND="sudo chroot ${rootfs_path} qemu-${architecture}-static /bin/bash -c "

# Internet connection
sudo cp --remove-destination /etc/resolv.conf ${rootfs_path}/etc/resolv.conf
#${QEMU_COMMAND} "echo nameserver 8.8.4.4 | sudo tee -a /etc/resolv.conf"
${QEMU_COMMAND} "mknod random c 1 8 || true"
${QEMU_COMMAND} "mknod urandom c 1 9 || true"
${QEMU_COMMAND} "mknod -m 666 /dev/null c 1 3 || true"
${QEMU_COMMAND} "mknod -m 666 /dev/zero c 1 5 || true"
${QEMU_COMMAND} "chown root:root /dev/null /dev/zero || true"

# Install packages
${QEMU_COMMAND} "sudo apt update"
${QEMU_COMMAND} "sudo apt install -y build-essential pkg-config libgtest-dev qtbase5-dev libqt5core5a libqt5network5 libqt5gui5 libqt5widgets5 libqt5concurrent5 libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-good libgstreamer-plugins-bad1.0-0 libmosquitto-dev libmosquittopp-dev libeigen3-dev libmsgpack-dev avahi-utils libopencv-dev"

# unmount all the things!
#sudo umount ${rootfs_path}/dev/pts
mountpoint -q ${rootfs_path}/dev && sudo umount ${rootfs_path}/dev
#sudo umount ${rootfs_path}/sys
mountpoint -q ${rootfs_path}/run && sudo umount ${rootfs_path}/run
mountpoint -q ${rootfs_path}/proc && sudo umount ${rootfs_path}/proc

echo "\e[38;2;255;128;0m Now edit calls of _qt5gui_find_extra_libs function in ${rootfs_path}/usr/lib/arm-linux-gnueabihf/cmake/Qt5Gui/Qt5GuiConfigExtrax.cmake. Add prefix \"${rootfs_path}\" to current path otherwise these libraries cannot be found."

