# README - Experimental Robotics Lab - Assignment 2 - RoboCLuedo

**Francesco Ganci - 4143910** - Robotics Engineering - A.A. 2021/2022

> Compatible With:
> - ROS1 noetic
> 
> Previous version of this project:
> - [programmatoroSeduto/EsperimentalROboticsLab-Assignment-1 on GitHub](https://github.com/programmatoroSeduto/ExperimentalRoboticsLab-Assignment-1)
> 
> Starter kit:
> - [CarmineD8/erl2 on GitHub](https://github.com/CarmineD8/erl2/) 

![cluedo logo](_media/img/cluedologo.jpg)

## What is this?

...

### Structure of the repository

```
root@b3f448d20720:~/ros_ws/src/robocluedo# tree

```






# How to setup and run the project

Here are the instructions for installing and running the project in your ROS1 environment.

## Prerequisites and Dependencies

Before installing the project, make sure your system satisfies these requirements:

- a working installation of ROS1 (the project is compatible with **ROS1 Noetic**).
	
	I suggest to use the Docker image here: [carms84/exproblab Docker image](https://hub.docker.com/r/carms84/exproblab) 
	
	With a configuration like this one:
	
	```
	docker run -it -p 6080:80 -p 5900:5900 -p 8888:8888 --name RLAB_assignment2 carms84/exproblab
	```
	
	This kind of port forwarding allows also to use Jupyter. 

### DEPT -- ...

...

## Project Setup

...

## Project Configuration

...

## Modules Testing

...

## How to run the project

... only fast way here; if you want to see more, see the detailed description in the docs






# How to generate the documentation

The project has been documented using two tools:

- **Doxygen** easy to use, stable, quick to use, with a handy GUI, but quite unflexible and generating a too much old style HTML documentation (welcome back to 90s!)
- **Sphinx** enables to produce a nice documentation, flexible and extendable with not so much effort, but really painful to use, not stable, it has not a GUI, and uses the horrible reStructured format. [Here](https://devblogs.microsoft.com/cppblog/clear-functional-c-documentation-with-sphinx-breathe-doxygen-cmake/) is an interesting article about Sphinx. 


## Documentation system setup

Here's how to install the abovementioned tools for generating the documentation. 

### DEPT -- Doxygen

Open a console and paste this command: 

```bash
# the engine
sudo apt-get install -y doxygen
# check your installation with the command 
doxygen -v
# 1.8.17

# and the GUI
sudo apt-get install -y doxygen-gui
# install check: this should make an alert window appear
doxywizard --help
```
That's all for the Doxygen setup. You can run the GUI using typing the command `doxywizard` in the shell. 

### DEPT -- Sphinx

*THE EXTENSION 'BREATHE' OF SPHINX REQUIRES DOXYGEN: install it before proceed.*

Installing Sphinx s quite easy:

```bash
# the engine
# the following gives an issue (the repo is not up to date...)
#    sudo apt-get install -y python3-sphinx
# use this instead:
pip3 install sphinx==4.5.0

# check the installation
sphinx-quickstart --version
# if it appears a Python exception like this:
# ImportError: cannot import name 'contextfunction' from 'jinja2' (/usr/local/lib/python3.8/dist-packages/jinja2/__init__.py)
# SEE THE TROUBLESHOOTING section concerning jinja2
```
Let's install also some useful extensions for Sphinx:

```bash
# breathe allows Sphinx to read the Doxygen XML documentation
pip3 install breathe
# see https://github.com/michaeljones/breathe/releases
# latest is 4.33.1

# this is a nice theme which recalls ReadTheDocs (a little bit worse than the orininal one)
pip3 install sphinx-rtd-theme

# this extension lets Sphinx to read the MD format
pip3 install myst-parser
# see https://myst-parser.readthedocs.io/en/latest/
# latest is 0.17.2
```

## Documentation generation

...use the script ...






# Troubleshooting 

Here are some well-known problems that can occur trying to run the project. 


## DOcker VM -- *setrlimit(RLIMIT_CORE): Operation not permitted*

It could appear this "intimidating" message on the screen: `sudo: setrlimit(RLIMIT_CORE): Operation not permitted`. It occurs especially when you attempt to run this code inside a Docker container.

*Who cares*. You can ignore this message, and keep going on, since the commands are effective anyway. See these posts:

- [in container: sudo: setrlimit(RLIMIT_CORE): Operation not permitted on GitHub Issues](https://github.com/sudo-project/sudo/issues/42)
- [sudo: setrlimit(RLIMIT_CORE): Operation not permitted on StackExchange](https://unix.stackexchange.com/questions/578949/sudo-setrlimitrlimit-core-operation-not-permitted)


## Docker VM -- unable to visualize the screen

It is a very common issue of `xvnc11`: it gets stuck often at the first launch the machine. To solve the problem, open a bash in the system hosting the Docker container, and run the command

```bash
docker exec -it <your_container_name> /bin/bash
```

and, inside the bash of the container, copy and paste this:

```bash
rm /tmp/.X1-lock
/usr/bin/Xvfb :1 -screen 0 1024x768x16 &
export DISPLAY=":1"
export HOME="/root"
export USER="/root"
/usr/bin/openbox &
/usr/bin/lxpanel --profile LXDE &
/usr/bin/pcmanfm --desktop --profile LXDE &
x11vnc -display :1 -xkb -forever -shared -repeat &

```

It should solve the problem. The log of the container should include a line lie this: 

```
2022-07-03 10:19:05,166 INFO success: x11vnc entered RUNNING state, process has stayed up for > than 1 seconds (startsecs)
```

Remember to **restart the machine** before working on it. 


## Docker VM - Update/Upgrade ROS

Especially when you use the Docker image [carms84/noetic_ros2](https://hub.docker.com/r/carms84/exproblab), the first time you launch the command `sudo apt-get update`, an error message is print; the output is similar to this one:

```
$$ sudo apt-get update
sudo: setrlimit(RLIMIT_CORE): Operation not permitted
Get:1 http://dl.google.com/linux/chrome/deb stable InRelease [1,811 B]
Get:2 http://mirrors.ubuntu.com/mirrors.txt Mirrorlist [142 B]                                                                                 
Get:3 http://giano.com.dist.unige.it/ubuntu focal InRelease [265 kB]                                                                 
Get:6 http://dl.google.com/linux/chrome/deb stable/main amd64 Packages [1,090 B]                                                                                     
Get:5 http://ubuntu.mirror.garr.it/ubuntu focal-backports InRelease [108 kB]                                                                                         
Get:7 http://security.ubuntu.com/ubuntu focal-security InRelease [114 kB]                                                                        
Get:4 http://giano.com.dist.unige.it/ubuntu focal-updates InRelease [114 kB]                                              
Get:9 http://ubuntu.mirror.garr.it/ubuntu focal-updates/main amd64 Packages [2,185 kB]                                               
Ign:8 http://ubuntu.connesi.it/ubuntu focal-updates/restricted amd64 Packages                                                             
Ign:8 http://giano.com.dist.unige.it/ubuntu focal-updates/restricted amd64 Packages                                                                            
Ign:11 http://ubuntu.connesi.it/ubuntu focal-updates/multiverse amd64 Packages                                                                                 
Ign:12 http://giano.com.dist.unige.it/ubuntu focal-backports/universe amd64 Packages                                                     
Get:13 http://ubuntu.mirror.garr.it/ubuntu focal-backports/main amd64 Packages [51.2 kB]                                                 
Get:14 http://packages.ros.org/ros/ubuntu focal InRelease [4,676 B]                                                                
Get:11 http://ubuntu.mirror.garr.it/ubuntu focal-updates/multiverse amd64 Packages [30.3 kB]                           
Err:14 http://packages.ros.org/ros/ubuntu focal InRelease                                              
  The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
Get:10 http://archive.ubuntu.com/ubuntu focal-updates/universe amd64 Packages [1,153 kB]
Get:15 http://packages.ros.org/ros2/ubuntu focal InRelease [4,679 B]            
Get:16 http://security.ubuntu.com/ubuntu focal-security/restricted amd64 Packages [1,139 kB]  
Err:15 http://packages.ros.org/ros2/ubuntu focal InRelease            
  The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
Get:17 http://security.ubuntu.com/ubuntu focal-security/multiverse amd64 Packages [25.8 kB]
Get:18 http://security.ubuntu.com/ubuntu focal-security/main amd64 Packages [1,771 kB]
Get:8 http://archive.ubuntu.com/ubuntu focal-updates/restricted amd64 Packages [1,214 kB]       
Get:12 http://archive.ubuntu.com/ubuntu focal-backports/universe amd64 Packages [26.0 kB]
Get:19 http://security.ubuntu.com/ubuntu focal-security/universe amd64 Packages [870 kB]      
Fetched 9,079 kB in 7s (1,331 kB/s)                                                                                                                                                                               
Reading package lists... Done
W: An error occurred during the signature verification. The repository is not updated and the previous index files will be used. GPG error: http://packages.ros.org/ros/ubuntu focal InRelease: The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
W: An error occurred during the signature verification. The repository is not updated and the previous index files will be used. GPG error: http://packages.ros.org/ros2/ubuntu focal InRelease: The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
W: Failed to fetch http://packages.ros.org/ros/ubuntu/dists/focal/InRelease  The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
W: Failed to fetch http://packages.ros.org/ros2/ubuntu/dists/focal/InRelease  The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
W: Some index files failed to download. They have been ignored, or old ones used instead.
```

This happens because there are still some old addresses in the list of the repositories. To solve this, here are the commands:

```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# now, update/upgrade
sudo apt-get update -y
sudo apt-get upgrade -y
```

If you're interested, here is the official post explaining this fact: [ROS GPG Key Expiration Incident on ROS Discourse](https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669). Another useful post: [apt update: signatures were invalid: F42ED6FBAB17C654 - ROS Answers: Open Source Q&A Forum](https://answers.ros.org/question/379190/apt-update-signatures-were-invalid-f42ed6fbab17c654/)


## Sphinx - jinja2

The packahe Jinja typically gives some problem; unfortunately it is required during the installation og Jupyter notebook, otherwise, as you try to run `jupyter contrib nbextension install`, it wll appear a very unintellegible Python error. 

In order to avoid this boring error, uninstall and install again this component:

```bash
pip3 uninstall Jinja2
pip3 install Jinja2
```

See this post, not directly related with this situation: [ContextualVersionConflict when starting 0-robot #139
 on HitHub](https://github.com/zero-os-no-longer-used/0-robot/issues/139)




# The project - technical overview

...






# Project Roadmap

## first steps -- repo setup

- [x] workspace creation -- 20220307
- [x] documentation system, first setup with Sphinx and Doxygen -- 20220307
- [x] Sphinx Configuration -- 20220307
	
	:warning: very first configuration, not yet tested.
	
- [x] Doxygen Configuration 
	
	:warning: very first configuration, not yet tested.
	
- [ ] ... 

## Project Documentation

- [ ] template for message description (see RT1_assignment_2)
- [ ] template for service description (see RT1_assignment_2)
- [ ] ...

## aRMOR and aRMOR interface

- [x] download the package from the previous assignment version -- 20220307
- [ ] create a standalone package for the node `cluedo_armor_interface.cpp` and its libraries
- [ ] study the examples in the previous version of the assignment ...
- [ ] ... put the documentation there, *here* ...
- [ ] ... and test the package
- [ ] write the final launch files

## ROSplan, actions and mission control

- [ ] ...

## movement controller 

**SImulation environment**:

- [ ] ...

**MoveIt controller**:

- [ ] ...

**Navigation system and path planning**:

- [ ] ...

## put things together

- [ ] ...

## testing

- [ ] ...

## deployment

- [ ] ...







# Authors and contacts

A project by *Francesco Ganci*, S4143910

- **Email** : _s4143910@studenti.unige.it_

# Useful Links

...
