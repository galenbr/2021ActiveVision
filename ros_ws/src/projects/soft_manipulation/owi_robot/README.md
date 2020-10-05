# Package for [OWI 535 Robotic Arm Edge!](https://www.amazon.com/OWI-Robotic-Soldering-Required-Extensive/dp/B0017OFRCY/)

## About the Package
This package interfaces with [OWI USB Interface!](https://www.amazon.com/OWI-USB-Interface-Robotic-Arm/dp/B0028MBWS2) to command the robot. The following is a list of available services and their short descriptions. For a full explanation of how to use the service check the section below.

* writecmd.srv - this service writes commands to the robot
* generatecmd.srv - this service generates 3 byte commands for the robot

## Service Documentation

### writecmd.srv

This service sends a 3 byte command to the OWI USB Interface. To try this out, connect the USB to your computer, turn on the power switch for the USB kit and run the *cmd_robot_node*