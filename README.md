# Applying reinforcement learning to drone simulation

Applying drone simulation to reinforcement learning right here.

If using Anaconda:

```
conda create --name yourenvnamehere
conda install -r requirements.txt
```

If using venv:

```
python3 -m venv venv
pip3 install -r requirements.txt
```


### How to set up:

Install Unreal Engine 4: https://www.unrealengine.com/en-US/get-now

Set up AirSim plugin afterwards: https://microsoft.github.io/AirSim/build_linux/

Install the AirSim blocks environment: https://github.com/jimzers/airsim-blocks

Make sure you've set up a virtual environment with the packages described in `requirements.txt` before proceeding


### How to run:

First start up Unreal Engine and open up the "Blocks" project.

Then press "Play" in the editor. Press "No" when the simulator prompts you to run in car mode.

Now you should be able to run the Python code to interface with the environment.

Note: If you want to stop the program before the Airsim connection has been severed, then you must first stop the Unreal Engine simulation (with ESC) before stopping your Python code. Otherwise your UE4 will crash. Or it might just crash either way. Hurray for UE4 on Linux!


### Summary of files:

`a_to_b.py`

Moves the drone around 4 points, and takes pictures at each point.


`act_on_detection.py`

Test file to experiment with red color detection and moving based on sensing red color in front of the drone.


`basic_movement_class.py`

Velocity movement drone class to interface with the drone. Has options for logging movements and reading inputs from text files.


`detect_red_color.py`

Test file to test color detection on drone.


`movement_class_position.py`

Position-based movement (e.g. move to XYZ) to interface with the drone. Has options for logging movements and reading inputs from text files.


`stop_n_go.py`

Test file to experiment with continued action from the drone based on color detection.


`testing.py`

Copy of testing file from Airsim repo to check if Airsim connection and Python API is properly installed and working.


`testingDQN.py`

Attempt to use custom OpenAI Gym environment to learn drone movement.
