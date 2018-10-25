# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|
  config.vm.box = "ubuntu/trusty64"
  config.vm.synced_folder "parametric", "/home/vagrant/ros_ws/src/parametric", create: true

  config.vm.network "public_network", ip: "192.168.1.112"

  config.vm.provider "virtualbox" do |vb|
    # Display the VirtualBox GUI when booting the machine
    vb.gui = true
    vb.name = "ROS"
    vb.cpus = 4
    vb.memory = "4096"
  end
 
  config.vm.provision "shell", inline: <<-SHELL
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    apt-get update
    apt-get install -y lxde ros-indigo-desktop-full python-rosinstall git-core python-argparse python-wstool python-vcstools python-rosdep ros-indigo-control-msgs ros-indigo-joystick-drivers gazebo2 ros-indigo-qt-build ros-indigo-driver-common ros-indigo-gazebo-ros-control ros-indigo-gazebo-ros-pkgs ros-indigo-ros-control ros-indigo-control-toolbox ros-indigo-realtime-tools ros-indigo-ros-controllers ros-indigo-xacro python-wstool ros-indigo-tf-conversions ros-indigo-kdl-parser avahi-autoipd avahi-daemon avahi-utils
    rosdep init
  SHELL

  config.vm.provision "shell", privileged: false, inline: <<-SHELL
    sudo chown vagrant:vagrant /home/vagrant /home/vagrant/* /home/vagrant/**/*
    rosdep update
    echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
    mkdir -p ~/ros_ws/src
    source /opt/ros/indigo/setup.bash
    cd ~/ros_ws
    catkin_make
    catkin_make install
    cd src
    wstool init .
    wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
    wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter_simulator/master/baxter_simulator.rosinstall
    wstool update
    source /opt/ros/indigo/setup.bash
    cd ~/ros_ws
    catkin_make
    catkin_make install
    cd devel
    wget https://github.com/RethinkRobotics/baxter/raw/master/baxter.sh
    chmod u+x baxter.sh
    sudo reboot
  SHELL
end
