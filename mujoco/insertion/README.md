# insertion
Peg-in-Hole Insertion Simulation in Mujoco using RL.

## Instructions
### General Usage
- ```source insertion_env/bin/activate```
- ```cd mer_lab/mujoco/insertion/scripts```
- ```python3 insert_test.py```

### Creating new Environments
- Create a new file in robosuite/environments/manipulation using an existing environment as a template.
- Add an associated import line to robosuite/__init__.py.

## Setup
### Assumptions:
- Assumptions in Mujoco README are met.
- LLVM is installed (```sudo apt install llvm```).

### First Time Setup
1. ```python3 -m venv insertion_env```
2. ```source insertion_env/bin/activate```
3. ```easy_install llvmlite```
	-Verify that a .so file is built: ```sudo find / -name libllvmlite.so*```
4. ```pip install numba==0.43.1```
5. ```pip install mujoco-py robosuite roboticstoolbox-python```
6. Replace robosuite (/home/<YOUR_USERNAME>/insertion_env/lib/python3.6/site-packages/robosuite) with the custom version in this folder (mer_lab/mujoco/insertion/robosuite).

## Notes
- This was tested on Ubuntu 18.04 with Python 3.6.9

## References
- https://robosuite.ai/docs/index.html
- https://github.com/vikashplus/franka_sim