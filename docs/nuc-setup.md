# Nuc setup for Intel Realsense camera, OpenCV, etc.

## Ubuntu
Tried to install ubuntu server 16.04 multiple times but froze on startup each time!

Reverted to ubuntu server 14.04 which installed succesfully. However, networking was not setup during the installation so was done manually:

__Network__
 - `ip link` to find the name of the ethernet -> p2p1
 - add the following to `/etc/network/interfaces` to enable the ethernet:
        # Ethernet
        allow-hotplug p2p1
        iface p2p1 inet dhcp

- then set the nuc to use the pc as a proxy to allow internet access by adding `http_proxy=mpowell@techpc42:3128; export http_proxy` to `~/.bashrc`. _Note:_ Need to set the pc up with cntlm and allow access from other computers by setting _Gateway_ to 'yes'.


__Direct ethernet to antother pc__
To connect the nuc directly to a pc via an ethernet cable (i.e. no router,etc.)[ a dhcp server can be set up on the pc.](https://www.howtoforge.com/dhcp_server_linux_debian_sarge)

1. **Set static IP on pc:** Edit `/etc/network/interfaces` and add the following:
        iface eth0 inet static
        address xxx.xxx.xxx.xxx(enter your ip here)
        netmask xxx.xxx.xxx.xxx
        gateway xxx.xxx.xxx.xxx(enter gateway ip here,usually the address of the router)

2. **Install dhcp server:**
        sudo apt-get install dhcp3-server

3. **Configure dhcp3:** Edit `/etc/dhcp/dhcpd.conf` and add the following making sure the network is the same as the static one set above.
        ddns-update-style none;

        option domain-name-servers 145.253.2.75, 193.174.32.18;

        authoritative;

        subnet 192.168.0.0 netmask 255.255.255.0 {
        range 192.168.0.200 192.168.0.229;
        option subnet-mask 255.255.255.0;
        option broadcast-address 192.168.0.255;
        option routers 192.168.0.1;
        }
4. Restart server: `/etc/init.d/ics-dhcp3-server restart`
## OpenCV
Download OpenCV (using OpenCV 2.4.13 or OpenCV 3). [Linux install](http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html)

__Required Packages__

    [compiler] sudo apt-get install build-essential
    [required] sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
    [optional] sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

__Build__

    cd ~/opencv-2.4.13
    mkdir release
    cd release
    cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..

Then install opencv to the system:

    make
    sudo make install



## Librealsense
Download/clone the git repo `git clone https://github.com/IntelRealSense/librealsense.git`

Then follow the [install instructions](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md) to build and install the library to the system. This involves updating the kernel to 4.4.

__Note:__
Make sure that the linux-headers, kernel, etc. have matching versions.


## PCL
Install dependencies:
- boost (min 1.4, but needed 1.58 with pcl1.8) `sudo apt-get install libboost1.58.0-all`
- eigen `sudo apt-get install libeigen3-dev`
- flann `sudo apt-get install libflann-dev`
- vtk (download 1.8 from http://www.vtk.org/)

Download pcl

    wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz
    tar xvf pcl-1.8.0.tar.gz

Make and install pcl library:

    cd pcl-1.8.0
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    sudo make -j2
    sudo make install
