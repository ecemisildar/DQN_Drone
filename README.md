# DQN_Drone
Reinforcement learning drone application using dqn

This repository includes a custom drone, its custom gazebo environment and a reinforcement algorithm called deep Q learning. The goal is to teach the hexacopter how to fly without crashing the objects in the environment using just a depth camera. 

In order to run the code with rviz and gazebo type `roslaunch air_drone drone_env.launch` on the terminal.

## Explanations about the package
### launch 
Contains 3 files `drone_env.launch` launches both rviz and gazebo. 
### msg
Contains 2 custom messages `MotorSpeed.msg` takes the propeller name and velocities, while `Pose.msg` takes the current position and orientation of the drone.
### src
Contains a python node and a cpp plugin. Python node is for implementing the RL algorithm, publishing proper velocities to the motors and subscribing to the current position of the drone. The plugin is for implementing the physical properties of the drone in the gazebo environment and also for handling the custom messages.
### urdf
Contains a drone model, links and joints in order to simulate the drone in the simulation environments.
### world
Contains my custom environment for gazebo simulation which has walls and some obstacles.

## Resources
My primary resources when I make this repo are [Drone model urdf, pluign](https://gitlab.com/dagothar/kair_drone/-/tree/master?ref_type=heads) and [DQN algorithm](https://stable-baselines3.readthedocs.io/en/master/modules/dqn.html). 

