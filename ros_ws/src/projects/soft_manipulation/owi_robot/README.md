# Package for [OWI 535 Robotic Arm Edge!](https://www.amazon.com/OWI-Robotic-Soldering-Required-Extensive/dp/B0017OFRCY/)

## About the Package
This package interfaces with [OWI USB Interface!](https://www.amazon.com/OWI-USB-Interface-Robotic-Arm/dp/B0028MBWS2) to command the robot. The following is a list of available services and their short descriptions. For a full explanation of how to use the service check the section below.

* writecmd.srv - this service writes commands to the robot
* generatecmd.srv - this service generates 3 byte commands for the robot

## Service Documentation

### writecmd.srv

This service sends a 3 byte command to the OWI USB Interface. To try this out, connect the USB to your computer, turn on the power switch for the USB kit and run the *cmd_robot_node*. To do this run roscore in a terminal.

```
roscore
```

In a new terminal rosrun the node. **Dont forget to source your ws if you don't have it setup in your bascrc!**

```
rosrun cmd_robot_node
```

In a new terminal call the service. **Dont forget to source your ws if you don't have it setup in your bascrc!**

```
rosservice call cmdOwiServer "Str1:'00' Str2:'00' Str3:'01'"
```

This should turn on the LED on the robot. You can set Str3 to '00' to turn off the LED.

