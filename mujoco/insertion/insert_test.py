import robosuite as suite
from robosuite import load_controller_config
import numpy as np
import roboticstoolbox as rtb
from spatialmath import *
from robosuite.wrappers import GymWrapper
from SAC_env import SAC_Agent
from common.utils import mini_batch_train

# #Define SAC params
gamma = 0.99
tau = 0.01
alpha = 0.2
a_lr = 3e-4
q_lr = 3e-4
p_lr = 3e-4
buffer_maxlen = 1000000

# Load the desired controller's default config as a dict
config = load_controller_config(default_controller="JOINT_POSITION")

# Create environment
env = GymWrapper(
        suite.make("Insert", 
                    robots="Panda", 
                    controller_configs=config,
                    has_renderer=True,
                    has_offscreen_renderer=False,
                    use_camera_obs=False,
                    )
)

#Use IK from Peter Corke's Toolbox
rtb_panda=rtb.models.DH.Panda()
#Define Starting EE Position
T = SE3(0.3, 0.0, 0.6)*SE3.OA([0, -1, 0], [0, 0, -1])
#Compute inverse kinematics
# sol = rtb_panda.ikine_LM(T,rlimit=500)
sol = rtb_panda.ikine_min(T)
#print(sol)

#Get panda object
panda=env.robots[0]
#Set joint position to home
#home=np.array([0.0,-0.785,0.0,-2.356, 0.0, 1.57, 0.784])
home=np.array(sol.q)
panda.set_robot_joint_positions(home)

#Create SAC agent
agent = SAC_Agent(env, gamma, tau, alpha, q_lr, p_lr, a_lr, buffer_maxlen)

for i_episode in range(20):
    obs = env.reset()
    #Set
    for t in range(100):
        #TODO: Let the Agent take the action
        action = np.random.randn(8) # sample random action
        # action=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.1,]) #Last element is gripper
        # action=np.array([-0.015,.4,0.0,0.25,0.0,0.0,0.0,0.0,]) #Last element is gripper
        obs, reward, done, info = env.step(action)  # take action in the environment

        print('************************')
        print(reward)
        env.render()  # render on display
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break
env.close()
#***********

#TODO: Get peg tip position (Should be known from panda gripper definition)
#TODO: Spawn robot in pose directly above object (with noise)
#TODO: Spawn a fixed object
#TODO: Spawn Peg in correct position
#TODO: Add proper paramters (friction, inertia, etc. to all objects