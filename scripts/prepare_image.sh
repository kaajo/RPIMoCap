#!/bin/bash
set -e
set -o xtrace

PROJECT_DIR=$(dirname "$(readlink -f "$0")")/../

cd ${PROJECT_DIR}
mkdir os_image
cd os_image

### Download raspios image
sudo apt update
sudo apt install -y wget unzip qemu qemu-utils qemu-user-static binfmt-support g++-8-arm-linux-gnueabihf

wget https://downloads.raspberrypi.org/raspios_lite_armhf/images/raspios_lite_armhf-2020-12-04/2020-12-02-raspios-buster-armhf-lite.zip -O rpios.zip
unzip rpios.zip

image_path=${PWD}/2020-12-02-raspios-buster-armhf-lite.img

### Resize image 
qemu-img resize ${image_path} 10G
gnome-disk-image-mounter --writable ${image_path} 
sleep 2
read -n 1 -s -r -p "Edit rootfs partition size, then press any key to continue"

rootfs_path=/media/$USER/rootfs/
architecture=arm
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
${QEMU_COMMAND} "sudo apt install -y build-essential pkg-config libgtest-dev qtbase5-dev libqt5core5a libqt5network5 libqt5gui5 libqt5widgets5 libqt5concurrent5 libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-good libgstreamer-plugins-bad1.0-0 libmosquitto-dev libmosquittopp-dev libeigen3-dev libmsgpack-dev avahi-utils libraspberrypi-dev"

### Install OpenCV 4.4 from https://github.com/dlime/Faster_OpenCV_4_Raspberry_Pi
${QEMU_COMMAND} "sudo apt-get install -y libjpeg-dev libpng-dev libtiff-dev libgtk-3-dev libavcodec-extra libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libjasper1 libjasper-dev libatlas-base-dev gfortran libeigen3-dev libtbb-dev python3-dev python3-numpy"
git clone https://github.com/dlime/Faster_OpenCV_4_Raspberry_Pi.git
sudo cp -r Faster_OpenCV_4_Raspberry_Pi/debs/ ${rootfs_path}/opt/OpenCV/
${QEMU_COMMAND} "cd /opt/OpenCV/ && dpkg -i OpenCV*.deb"
${QEMU_COMMAND} "ldconfig"
rm -rf Faster_OpenCV_4_Raspberry_Pi
sudo rm -r ${rootfs_path}/opt/OpenCV/

# Fix pkgconfig
sudo sed -i 's|prefix=/usr/local|prefix=/usr|' ${rootfs_path}/usr/lib/pkgconfig/opencv4.pc

# Fix path to lib
qtextrax=${rootfs_path}/usr/lib/arm-linux-gnueabihf/cmake/Qt5Gui/Qt5GuiConfigExtras.cmake
sudo sed -i 's|_qt5gui_find_extra_libs(EGL \"EGL\" \"\" \"/usr/include/libdrm\")|_qt5gui_find_extra_libs(EGL \"EGL\" \"'${rootfs_path}'\" \"/usr/include/libdrm\")|' ${qtextrax}
sudo sed -i 's|_qt5gui_find_extra_libs(OPENGL \"GLESv2\" \"\" \"\")|_qt5gui_find_extra_libs(OPENGL \"GLESv2\" \"'${rootfs_path}'\" \"\")|' ${qtextrax}

# Enable services
${QEMU_COMMAND} "sudo systemctl enable ssh"

# Install RPIMoCap service
sudo cp ${PROJECT_DIR}/scripts/rpiclient.service ${rootfs_path}/etc/systemd/system/rpiclient.service
${QEMU_COMMAND} "systemctl enable rpiclient"

# unmount all the things!
#sudo umount ${rootfs_path}/dev/pts

mountpoint -q ${rootfs_path}/dev && sudo umount ${rootfs_path}/dev
#sudo umount ${rootfs_path}/sys
mountpoint -q ${rootfs_path}/run && sudo umount ${rootfs_path}/run
mountpoint -q ${rootfs_path}/proc && sudo umount ${rootfs_path}/proc

echo "Image is now ready for cross compilation"

