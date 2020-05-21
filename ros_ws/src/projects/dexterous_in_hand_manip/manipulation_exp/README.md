#### Available Services:

Do not forget to source the workspace
```
  source ~/mer_lab/ros_ws/devel/setup.bash
```
1. Resetting sequence for releasing the object, moving the arm away and repositioning the object
```
  rosservice call /ih_manip/reset_sequence "start: 0"
```
2. Preparation sequence for initializing the arm and approaching the object
```
  rosservice call /ih_manip/prepare_grasp "start: 0"
```
3. Grasping sequence to grasp the object
```
  rosservice call /ih_manip/object_grasp "start: 0"
```
4. Gravity exploitation sequence to have a grip from higher position (object must be supported by table)
```
  rosservice call /ih_manip/gravity_exploit "start: 0"
```
5. Prehensile pushing sequence to lower the grip position (object must be supported by table)
```
  rosservice call /ih_manip/prehensile_pushing "start: 0"
```
6. Sliding test sequence to test sliding motions of the hand
```
  rosservice call /ih_manip/sliding_test "start: 0"
```
7. Rotating test sequence to test rotating motions of the hand
```
  rosservice call /ih_manip/rotating_test "start: 0"
```


These services can be used in combination with lower level (non-sequential) services to generate other desired sequences.
