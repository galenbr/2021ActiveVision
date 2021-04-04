import robosuite as suite
from robosuite import load_controller_config
import numpy as np
import roboticstoolbox as rtb
from spatialmath import *

# Load the desired controller's default config as a dict
config = load_controller_config(default_controller="JOINT_POSITION")

# Create environment
env = suite.make("Insert", 
				 robots="Panda", 
				 controller_configs=config,
				 has_renderer=True,
	    		 has_offscreen_renderer=False,
	    		 use_camera_obs=False,
)

#Use IK from Peter Corke's Toolbox
rtb_panda=rtb.models.DH.Panda()
#Define Starting EE Position
T = SE3(0.3, 0.0, 0.6)*SE3.OA([0, -1, 0], [0, 0, -1])
#Compute inverse kinematics
# sol = rtb_panda.ikine_LM(T,rlimit=500)
sol = rtb_panda.ikine_min(T)
print(sol)

#Get panda object
panda=env.robots[0]
#Set joint position to home
#home=np.array([0.0,-0.785,0.0,-2.356, 0.0, 1.57, 0.784])
home=np.array(sol.q)
panda.set_robot_joint_positions(home)

#Set
for i in range(1000):
    # action = np.random.randn(8) # sample random action
    #action=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.1,]) #Last element is gripper
    action=np.array([-0.015,.4,0.0,0.25,0.0,0.0,0.0,0.0,]) #Last element is gripper
    obs, reward, done, info = env.step(action)  # take action in the environment
    #print(obs.keys())
    #print(obs['robot0_proprio-state'])
    print('************************')
    print(panda.ee_force)
    print(panda.ee_torque)
    env.render()  # render on display


#TODO: Spawn robot in pose directly above object (with noise)
#TODO: Spawn a fixed object
#TODO: Spawn Peg in correct position
#TODO: Add proper paramters (friction, inertia, etc. to all objects