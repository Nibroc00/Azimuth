# Azimuth Vicon GPS System
The Azimuth system allow for GPS relient flight modes indoor by taking motion capture data and outputting GPS data in the form of NMEA protocol. The following are the messages outputed by the system:
- GGA for global postioning data
- VTG for course over ground
- RMC for course over ground and postion
- HDT for heading

Azimuth runs as a ROS node and was developed using ros noetic. ROS is an opensource messageing protocol for robotics. Installation instructions can be found [here](http://wiki.ros.org/ROS/Installation). The Azimuth system gets its data from a rostopic published by [Vicon bridge](https://github.com/ethz-asl/vicon_bridge), a ROS node that uses the Vicon API to access and publish postion information from a vicon motion capture system.

## Enabling Heading from GPS
!!! For ardupiliot copter firmware !!!
To enable heading From GPS, the following paramaters need to be set in Mission Planner:
		AHRS_EKF_TYPE=3
		GPS_TYPE=16
		COMPASS_USE=0
		COMPASS_USE2=0
		EK3_MAG_CAL=5
		EK3_SRC1_YAW=2

## Installing the package 
To install our package, simply git clone the repository to the src directory of your catkin workspace and the catkin_make or catkin_build

## Running the package
To launch the node run the command below, replacing the items curly brackets with your own settings
```console
foo@bar:~$ roslaunch azimuth gps.launch vicon_target:={vicon_target} serial_port:={serial_port} baud:={baud} lat:={lat} lon:={lon}
```

# Running the package as a docker container

## Building the docker container

To build the docker container, clone the repository and navigate to docker/azimuth-gps. Then locate the .env file and adjust the paramters accordingly. Then run the command below to build the container

```console
foo@bar:~$ docker build .
```

## Launching the container
run the command below in the same directory as the docker-compose.yml file, or provide docker-compose with the path to the file.

```console
foo@bar:~$ docker-compose --env-file .env up
```


