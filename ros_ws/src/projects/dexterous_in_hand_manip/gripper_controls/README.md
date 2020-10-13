#### Available Services:

Do not forget to source the workspace
```
  source ~/mer_lab/ros_ws/devel/setup.bash
```
1. Slide object down on left finger, provide left finger position reference (+ for opening, - for closing direction)
```
  rosservice call /Slide_Left_Finger_Down "data: 0.0"
```
2. Slide object up on left finger, provide right finger position reference (+ for opening, - for closing direction)
```
  rosservice call /Slide_Left_Finger_Up "data: 0.0"
```
3. Slide object down on right finger, provide right finger position reference (+ for opening, - for closing direction)
```
  rosservice call /Slide_Right_Finger_Down "data: 0.0"
```
4. Slide object up on right finger, provide left finger position reference (+ for opening, - for closing direction)
```
  rosservice call /Slide_Right_Finger_Up "data: 0.0"
```
5. Rotate object anticlockwise, provide left finger position reference (+ for opening, - for closing direction)
```
  rosservice call /Rotate_anticlockwise "data: 0.0"
```
6. Rotate object anticlockwise, provide right finger position reference (+ for opening, - for closing direction)
```
  rosservice call /Rotate_clockwise "data: 0.0"
```
7. Hold object, provide position references for both fingers (+ for opening, - for closing direction)
```
  rosservice call /Hold_object "left: 0.0 right: 0.0"
```
7. Set friction of selected finger (finger: 0 for left, 1 for right, high_friction: 0 for low_friction, 1 for high_friction)
```
  rosservice call /set_friction "finger:0 high_friction:false"
```
