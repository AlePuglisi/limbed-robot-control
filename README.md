# legged-robot-control
MATLAB | Simulink | robotics-toolbox | Legged Robot | Modeling and Control | URDF to DH 

This Repository contains the MATLAB code and Simulink scheme for modeling and control of LIMBERO+GRIEEL (legged robot prototype of Space Robotics Lab (SRL), Tohoku University ).<br/>

This is a "sequel" of my work at SRL as a research student, for my Master's thesis at Politecnico di Milano. <br/>
At SRL I updated the ROS 2 software and implemented stability algorithms. <br/>
Now I'm working on modeling and performance analysis of an alternative control structure. 

## Objectives

### In the previous episode:

<img width=450 height=390 src=https://github.com/AlePuglisi/legged-robot-control/blob/main/image_video/transform_sequence_algorithm_GIF.gif>

The software I used at SRL is written in C++ and is based on ROS 2 for communication and control.<br/>
The robot joint controllers are implemented using ros2_control, and the simulation is based on Gazebo Classic.<br/>
For the tuning of the joint_trajectory_controller and the refinement of robot [URDF](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html), I take an iterative tuning approach.<br/>

Some of my tasks as a member of the SRL Rover Team have been: 
- Update the C++ nodes for high/low-level control, with 3 additional joints in each limb (new end-effector transformable module)
- Update the XACRO description to make the robot reconfigurable (use launch arguments to choose end-effector mode).
- Retune URDF parameters to make the Gazebo simulation more reliable (friction, dumping, Gazebo tags, etc..)
- Update ros2_control configuration, and retune the joint controller PID gains
- According to support polygon stability theory, implement an algorithm for base positioning and end-effector module transformation.
- Retune DYNAMIXEL servomotors with Dynamixel Wizard, by trial and error.
- Update DYNAMIXEL Servomotors PID gains from a CSV, in a C++ node (controlling the servomotors)
- Set up the software to open/close the Gripper-Wheel module, with proper timing.
- Set up the Robot and do experiments on the real Hardware. <br/>
  (Accounting for Motor torque limitations, by compensation with a pulley system)
  
### In the next episode: 
Because of the lack of a model-based rigorous control tuning, I took a different approach.<br/> 
Also, to provide better insight into the algorithm performances, a different modeling of the leg based on DH is taken. <br/>
With this model, I attempt to define new useful performance indices.

Given the robot URDF:
- Convert URDF into Denvit Hartemberg model, and define it in the robotics-toolbox as a [SerialLink](https://www.petercorke.com/RTB/r9/html/SerialLink.html) <br/>
  (Notice, I didn't use the MATLAB [Robotics System Toolbox](https://it.mathworks.com/products/robotics.html), but the previous version of Peter Corke [toolbox](https://petercorke.com/toolboxes/robotics-toolbox/).
  I took this choice because using rigidBodyTree for dynamic simulation is computationally demanding)
- Single limb dynamic model identification 
- Tune independent joint controllers for the robot leg
- Performance analysis of the model-based controller gains, and gain correction if needed.
- Implement a gravity compensation algorithm (then include it in ROS 2). 
- Expand to 4-legs simulation and control
- Define new performance index for legged-robots
- Analyze the quality of some base postures, using defined indices
- Implement the new joint controllers in ROS 2, and simulate using Gazebo.

## Brief Implementation Description 

- ### MODELING AND CONTROL TUNING 
    ### Step 1: URDF to DH
    To better handle the robot joint controllers and model identification, I take the main [xacro](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html) file and convert it into a single 
    URDF.<br/>
    Be sure to have the proper package installed
    ```bash
    sudo apt install ros-<distro>-xacro
    ```
    Then you can run the command for conversion: 
    ```bash
    xacro robot.xacro -o robot.urdf
    ```
    
    Then, after defining an equivalent DH frame description, I identify the DH parameters from the URDF rigidBodyTree object created in the MATLAB script.<br/>
    At this point, I can initialize the robot leg as a SerialLink object. 
    
    Take a look at the [MATLAB](https://github.com/AlePuglisi/legged-robot-control/blob/main/limberoGrieel_leg_DH.m) script (commented as clear as possible) for all the passages. 
    
     ... README WIP ...
    ### Step 2: Dynamic parameter Identification 
    (Already implemented in MATLAB)
    
    ### Step 3: Control Tuning 
    (Already done for the servomotor model, independent joint controller) 

- ### MATLAB SINGLE LIMB SIMULATION
  (Simulink scheme implemented)
  
- ### MATLAB 4-LIMB KINEMATIC SIMULATION
  (Not yet implemented ... check if the dynamic simulation of the closed-kinematic chain is feasible with some SerialLink connection)
  
- ### POSTURE MANIPULABILITY ANALYSIS
  (Some tests done for single limb, soon extension for legged closed chain structure)
  
- ### ROS2+Gazebo MODEL-BASED CONTROLLER PERFORMANCE ANALYSIS
  (code implemented, controller gains adjustment and correction needed for stability) 
  



