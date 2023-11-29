# Drone Navigation in Agricultural Environments using Deep Reinforcement Learning

<p align="center">
  <img src="Screenshot%202023-11-29%20005517.png" alt="The drone trying to reach a target, i.e. a weed" width="500" height="500"/>
</p>

Python implementation of an agent training environment representing the drone achieving goals via deep reinforcement learning.

## Description

This project aims to develop an autonomous navigation system for drones, specifically designed for agricultural environments. It uses advanced path planning algorithms, such as RRT and RRT*, to allow drones to navigate effectively, avoiding obstacles and reaching objectives in complex scenarios. A key component of the system is the use of machine learning models for decision making, using reinforced learning techniques.

## Technologies and Libraries

The project is developed in Python 3.11.5, making use of the following libraries:
- Pygame for simulation and visualization of the environment.
- OpenCV (cv2) for image processing.
- Numpy for mathematical calculations and data manipulation.
- Gymnasium for the creation and management of reinforced learning environments.
- Stable Baselines3 and SAC (Soft Actor-Critic) to implement reinforcement learning models for autonomous drone control.
- Other standard Python libraries like math, os, random, sys.

## Installation and Configuration

To install and configure this project, follow the steps below:

1. **Clone the repository**:
    Clone the project to your local computer using the `git clone [repository URL]` command.

2. **Installing Dependencies**:
    The project requires several Python libraries, which can be installed via pip. Major dependencies include:
    - Pygame
    - OpenCV (cv2)
    - Numpy
    - Gymnasium
    - Stable Baselines3 and SAC for reinforcement learning
    - Other standard dependencies like math, os, random, sys

    Run the command `pip install -r requirements.txt` (be sure to create a `requirements.txt` file with all necessary dependencies).

3. **Model Configuration**:
    - Ensure that the reinforcement learning model (e.g. a saved SAC model) is correctly placed in the path specified in the script files (e.g. `model_path` in `model_actions.py`).

4. **Environment Configuration**:
    - Edit image and asset paths to fit your directory structure. Files such as `agricultural_land_drone.py` and `environment.py` include references to image and asset paths that may need adjustment.

5. **Execution of Scripts**:
    - Once the installation and configuration is complete, you can run the various Python scripts to test and visualize the drone's navigation system.

## Usage

After installing and configuring the project, you can use the following scripts to explore different features:

1. **agricultural_land_drone.py**:
    - This script simulates navigating a drone in an agricultural environment.
    - Start the program with `python agricultural_land_drone.py`.
    - The drone will use path planning algorithms to navigate and avoid obstacles.

2. **agricultural_land_rrt_star.py**:
    - Implements the RRT* algorithm for route planning in agricultural contexts.
    - Run with `python agricultural_land_rrt_star.py` to see how the drone calculates the route.

3. **model_actions.py**:
    - Contains the actions or templates used in the project.
    - It is a helper module used by other scripts, so it does not need to be run directly.

4. **square_obstacles_drone.py** and **square_obstacles_rrt_classic.py**:
    - These scripts handle scenarios with square obstacles.
    - Use variants of RRT algorithms for navigation.
    - Start them with `python square_obstacles_drone.py` or `python square_obstacles_rrt_classic.py`.

5. **environment.py**:
    - Configure and manage the simulation environment.
    - It is used by the other scripts for the drone simulation environment.

6. **training.py**:
    - Used to train reinforcement learning models.
    - Run this script if you want to train or refine your drone's control model.

Each script offers a different perspective on the drone's navigation and route planning capabilities, demonstrating the effectiveness of the algorithms and techniques employed in the project.

## Contribute

*(Guidelines for those who wish to contribute to the project.)*

## License

*(Details on the license under which the project is released.)*
