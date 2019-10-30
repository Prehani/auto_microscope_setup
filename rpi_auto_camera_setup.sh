#!/bin/bash bash 
#Start of the script--begin with some trace/debugging options
set -o nounset
set -o errexit

#We are currently on raspbian buster, but just to check
#If this changes in the future, just update 
cat /etc/os-release
os = "buster"

#Initial user loading
init_setup(){
	#Echo statement
	echo "Raspberry Pi Camera setup!"
	#Make sure everything is up to dater 
	sudo apt-get update && sudo apt-get upgrade                                                                                                

	if grep 'opt' /etc/passwd; then
	    #Nothing needs to be done
		echo "opt user already exists!"
	else
		#I think the password setup here is incorrect? 
		#Security is not great here as I would need to type the process
		#in the PID which could create issues/allow other users to see the 
		#password; this isn't relevant now, but something to keep in mind
		sudo adduser opt --gecos "First Last,RoomNumber,WorkPhone,HomePhone" --disabled-password
		#sudo chpasswd << 'END'
		opt:'OPTproject2019'
	fi
}

#Miniconda install
miniconda_install(){ 
	#Begin with a miniconda installation: we need to download the latest
	#version of miniconda as well as check the md5sum and then execute the 
	#install script for the miniconda installer

	echo "Downloading and installing miniconda"
	wget http://repo.continuum.io/miniconda/Miniconda3-latest-Linux-armv7l.sh
	sudo md5sum Miniconda3-latest-Linux-armv7l.sh
	sudo /bin/bash Miniconda3-latest-Linux-armv7l.sh
	
	#Next, add conda to the path to ensure that everything runs smoothly, and 
	#so that we can execute conda commands with ease:
	echo "Adding miniconda to the path and bashrc" 
	#This is wrong, need to append a line to the end 
	sudo nano /home/pi/.bashrc
	export PATH="/home/pi/miniconda3/bin:$PATH" 
	conda
}

#Miniconda setup 
miniconda_setup(){
	#Next, add appropriate channels to install the correct version of python
	conda 
	if $? !=0; then 
		miniconda_install
	fi 
	echo "Adding appropriate channels for rpi:"
	conda config --add channels rpi
	#And create/activate the appropriate environments 
	echo "Creating and activating opt environment"
	#If opt environment is in the list of conda environments, pass effectively 
	if conda info --envs | grep 'opt'; then 
		echo "Environment opt already exists!"
	#Otherwise, continue 
	else 
		conda create --name opt python=3.7 -y 
	fi	
	#Activate the new environment 
	source activate opt
	
	#Lastly, call the conda setup script to install the appropriate packages
	echo "Install script for appropriate python packages"
	declare -a python_packages=("numpy" "matplotlib" "scipy" "scipy" "scikit-image" "imagecodecs-lite" "numba" "h5py" "pyqtgraph")
	for i in "${python_packages[@]}"
	do
		echo "installing" $i 
		conda install $i -y  
	done
	echo "Install tifffile"
	conda install -c conda-forge tifffile 
}

#ROS setup per http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi
ros_setup(){ 
	echo "2.1 Setup ROS Repos"
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	
	sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key \
		 C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	#Ensure everything is up to date	
	sudo apt-get update && sudo apt-get upgrade -y 
	echo "2.2 Install Boostrap Dependencies" 
	sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
	echo "2.3 Initializing rosdep"
	sudo rosdep init 
	rosdep update

	echo "3.1 catkin Workspace Creation"
	pwd
	sudo apt-get install catkin -y 
	mkdir -p ~/ros_catkin_ws
	cd ~/ros_catkin_ws
	rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall 
	wstool init src kinetic-ros_comm-wet.rosinstall

	#Effectively try the install; if it fails, update and try again
	rosinstall_generator robot --rosdistro kinetic --deps --wet-only --tar > kinetic-robot-wet.rosinstall\
	&& wstool init src kinetic-robot-wet.rosinstall 
	if [[ $? -ne 0 ]]; then 
		wstool update -j4 -t src &&\
		rosinstall_generator robot --rosdistro kinetic --deps --wet-only --tar > kinetic-robot-wet.rosinstall &&\
		wstool init src kinetic-robot-wet.rosinstall
	fi 

	echo "3.2 Resolve Dependencies"
	sudo apt-get install cmake -y 

	mkdir -p ~/ros_catkin_ws/external_src
	cd ~/ros_catkin_ws/external_src
	wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
	unzip assimp-3.1.1_no_test_models.zip
	cd assimp-3.1.1
	cmake .
	make
	sudo make install

	rosinstall_generator desktop --rosdistro kinetic --deps --wet-only --exclude collada_parser collada_urdf --tar > kinetic-desktop-wet.rosinstall

	cd ~/ros_catkin_ws
	rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:${os}

	sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic

}

init_setup
miniconda_install
miniconda_setup
ros_setup 

exit $?