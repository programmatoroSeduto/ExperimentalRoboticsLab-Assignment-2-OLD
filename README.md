# README - Experimental Robotics Lab - Assignment 2 - RoboCLuedo

**Francesco Ganci - 4143910** - Robotics Engineering - A.A. 2021/2022

> Compatible With:
> - ROS1 noetic
> 
> Previous version of this project:
> - [programmatoroSeduto/EsperimentalROboticsLab-Assignment-1 on GitHub](https://github.com/programmatoroSeduto/ExperimentalRoboticsLab-Assignment-1)
> 
> A minimal starter kit:
> - [CarmineD8/erl2 on GitHub](https://github.com/CarmineD8/erl2/) 

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

## How to run the project

... only fast way here; if you want to see more, see the detailed description in the docs

# How to generate the documentation

## Documentation system setup

...sphynx and Doxygen ...

## Documentation generation

...use the script ...

# Troubleshooting 

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

# The project - technical overview

...

# Authors and contacts

A project by *Francesco Ganci*, S4143910

- **Email** : _s4143910@studenti.unige.it_

# Useful Links

...
