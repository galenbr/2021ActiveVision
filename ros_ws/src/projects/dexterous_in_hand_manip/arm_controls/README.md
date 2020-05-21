#### Available Services:

Do not forget to source the workspace
```
  source ~/mer_lab/ros_ws/devel/setup.bash
```
1. Initialize arm pose (near grasp position)
```
  rosservice call /initialize_arm_pose "execute: true"
```
2. Get to grasp pose (object is between fingers)
```
  rosservice call /grasp_pose "execute: true"
```
3. Resting pose away from the objects
```
  rosservice call /rest_pose "execute: true"
```
4. Move up (from current position)
```
  rosservice call /move_up "execute: true"
```
5. Move down (from current position)
```
  rosservice call /move_down "execute: true"
```
