# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|
  config.vm.box = "ubuntu/xenial64"

  config.vm.provider "virtualbox" do |vb|
    vb.gui = true
  end

  config.vm.provision "shell", inline: <<-SHELL
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    apt-get update
    sudo apt-get install ros-lunar-desktop-full -y
    sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y
    sudo rosdep init
    sudo su vagrant
    rosdep update
    echo "source /opt/ros/lunar/setup.bash" >> ~/.bashrc
    source ~/.bashrc 
  SHELL
end
