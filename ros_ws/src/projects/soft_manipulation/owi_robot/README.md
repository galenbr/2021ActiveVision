# Package for [OWI 535 Robotic Arm Edge](https://www.amazon.com/OWI-Robotic-Soldering-Required-Extensive/dp/B0017OFRCY/)

## About the Package
This package interfaces with [OWI USB Interface](https://www.amazon.com/OWI-USB-Interface-Robotic-Arm/dp/B0028MBWS2) to command the robot. The following is a list of available services and their short descriptions. For a full explanation of how to use the service check the section below.

* writecmd.srv - this service writes commands to the robot
* generatecmd.srv - this service generates 3 byte commands for the robot

## Service Documentation

### writecmd.srv

This service sends a 3 byte command to the OWI USB Interface. To try this out, connect the USB to your computer, turn on the power switch for the USB kit and run the *cmd_robot_node*. To do this run roscore in a terminal.

```
roscore
```

In a new terminal rosrun the node. **Dont forget to source your ws if you don't have it setup in your bashrc!**

```
rosrun cmd_robot_node
```

In a new terminal call the service. **Dont forget to source your ws if you don't have it setup in your bashrc!**

```
rosservice call cmdOwiServer "Str1:'00' Str2:'00' Str3:'01'"
```

This should turn on the LED on the robot. You can set Str3 to '00' to turn off the LED.

#### Permission Denied Error
```
libusb: error [_get_usbfs_fd] libusb couldn't open USB device /dev/bus/usb/001/025: Permission denied
libusb: error [_get_usbfs_fd] libusb requires write access to USB device nodes.
Error opening device
```

If you receive an error with write permissions to the USB port use the [chmod](https://wiki.linuxquestions.org/wiki/Chmod) command as shoen below. This will elevate permissions for all users and the group on the computer to write to the USB device you receive an error for as in the example below (I received an error for USB001/025). *Make sure to change the USB device ID in the command below to reflect the error!*

```
sudo chmod 666 /dev/bus/usb/001/025
```

