# UniLace
Simulation environment for [**Benchmarking and Simulating Bimanual Robot Shoe Lacing**](https://ieeexplore.ieee.org/abstract/document/10629040) published on *IEEE Robotics and Automation Letters*. 

https://github.com/user-attachments/assets/c65985c7-a8ca-4be4-94d5-fe680df4ea77


## Project Structure
The project is structured with two main components: the ROS workspace and the Unity project. 
The ROS workspace contains the ROS packages for the simulation and the Unity project contains the simulation environment. 
The project structure is as follows:
```
├── catkin_ws
│   └── src
│       └── uni_lace
│           ├── uni_lace
│           └── uni_lace_msgs
│           └── robot_driver_unity
├── UniLace
│   ├── Assets
│       ├── Resources
│       ├── RosMessages
│       ├── Scenes
│       ├── Scripts
│   ├── Build
|   |   ├── UniLaceLive
|   |   └── UniLaceGym
|   |   └── Demo
├── Dockerfile
├── Makefile
```

This Unity project is composed of two scenes: **UniLaceLive** and **UniLaceGym**. An extra **Demo** scene is for fast showcasing simulated shoe lacing.

Experiments included in the submission are completed with *UniLaceLive*.
All scenes are prebuilt and can be used without Unity.

## Prerequisites
### System requirements
* This project has been tested on Ubuntu 18.04, 20.04 and 22.04.
* Docker and Nvidia-docker installed.
* Unity installation is required ONLY for [Advanced Usage](#Advanced-Usage).

## Installation
1. Clone this repo
   ```
   git clone git@github.com:ImperialCollegeLondon/UniLace.git
   ```
2. Download the Docker image from Docker Hub
   ```
   cd UniLace
   make install-from-hub
   ```
   (Alternatively,) Build the Docker image from the Dockerfile
   ```
   cd UniLace
   make install-from-source
   ```
3. Download prebuilt executables from this [link](https://1drv.ms/u/c/ff232397a18bbe91/EVao1qGjYFVPkw2sxmOxkBkBBgmLOGvA0kBKSFg_SM26gw?e=yyg40U). Extract to the UniLace folder. Check the folder structure to ensure the executables are in the correct location as shown in the [Project Structure](#Project-Structure).

### Installing baseline system
```
make install-baseline
```

## Running the project
### Demo scene
The demo scene is created to showcase shoe lacing in a short time (less than 2 minutes on the testing machine).
1. To run this scene, simply run the following command:
    ```
    make demo
    ```

### UniLaceLive scene
This scene is where the baseline system is tested.
1. Launch the ROS wrapper (included in the uni_lace package) with the Unity executable. This can be done easily by
    ```
    make start-live
    ```
2. Run the baseline system:
    ```
    make run-baseline
    ```
3. Stop the simulation
The simulation can be stopped by pressing `Ctrl+C` in the terminal or by pressing `Esc` with the Unity window selected.

### UniLaceGym scene
This scene is connected with a Python wrapper. Running the testing policy will call `step` once which moves the robot into a different configuration and shows the observations.
Then it calls the `reset` function and moves the scene back to the default setup. The scene will close itself after the reset function is called.
1. Launch the Python wrapper with the Unity executable:
    ```
    make start-gym
    ```
2. Follow the instructions in the terminal to run the step and reset functions.

## Usage
### Changing the rendering perspective
The rendering perspective can be changed with keyboard inputs. This is activated by pressing the `Enter` key. 
The perspective is controlled by:
* `W` and `S`, `A` and `D` keys to translate the camera forward and backward, left and right respectively.
* `Space` and `Shift` keys to move the camera up and down.
* `I` and `K`, `J` and `L` keys to rotate the camera upward and downward, left and right respectively.

Once the desired perspective is achieved, press `Q` to lock the camera.

### Changing parameters
Simulation-related parameters include: 
* shoelace parameters
* obi solver paramters
* simulation parameters
* task parameters
* scene object parameters
* randomised parameters

All parameters are stored in `param.yaml`. More information on specific parameters is included in the file. The parameters are loaded into the simulation automatically and there is no need to rebuild the environments.


## UniLaceLive
This scene considers the simulation as a live system. Simulation information is published in ROS topics.
The baseline system is tested with this scene. Following is what happens when the simulation starts.

* `ParamManager` queries parameters from the param ROS service (therefore the ROS scripts must be launched first). 
* `Main` contains the main process of the simulation. It constructs the scene objects according to the parameters.
* `ArmController` and `GripperController` interface with ROS to control the robot.
* `ShoeManger` and `ShoelaceManager` control the construction of the shoe and shoelace.
* `RGBCameraWrapper` and `DepthCameraWrapper` publish images to ROS.
* `AgletCollisionManager` monitors the collision between aglets and grippers for attachments.
* `ShoeUpperCollisionManager` monitors the collision between the aglets and the eyestays to avoid penetration.
* `GroundTruthManager` publishes ground truth information as JSON string to ROS, including:
    * Aglet poses
    * Eyelet poses
    * Rope tension
    * Eyelets to be laced
    * Success
    * Shoelace branch length
* `ObiColliderCreator` adds Obi colliders to target objects.
* `FPSDisplay` controls information rendered to the top-right corner of the screen.

> **_NOTE:_** In this project, `Upper` is used interchangeably with `Eyestay`.

## UniLaceGym
UniLaceGym contains the same components as `UniLaceLive` but wrapped command and observation in `unity_step` and `unity_reset` ROS services. The Python script `uni_lace_gym.py` in `uni_lace` package uses these services and wraps them into an OpenAI Gym environment.
* Observations
    * RGBD images
    * Left arm joint states
    * Left gripper state
    * Right arm joint
    * Right gripper state

* Actions
    * Left arm joint target positions
    * Left arm joint max velocities
    * Left gripper target state
    * Right arm joint target positions
    * Right arm joint max velocities
    * Right gripper target state

## Individual Components

### Shoelace Manager
* This script controls generating the aglets, Obi Rope and related components. 
* It provides functions to estimate the tension of the entire Obi Rope and the regions around two ends (last 5 elements), as well as the branch lengths after experiments.
* It includes aglet collision managers `AgletCollisionManager` which controls the attachment of the aglets. In simulation, it is difficult to rely on friction to keep the aglet between fingers. Therefore, the aglets are attached to grippers and they are detached when the gripper opens or the rope end tension is over threshold after certain iterations.

### Shoe Manager
* This script manages the eyelets, Obi Softbody (the eyestays, also named shoe uppers in the project) and related components.
* It generates particle eyelet groups and the bottom layer particle group in runtime. The particle eyelet groups are connected to eyelet objects for retrieving the eyelet poses, the bottom layer particle group is used to attach the eyestays onto the shoe sole.
* The shoe upper collision manager `ShoeUpperCollisionManager` is created in the main processes instead. They monitors the Obi particle collisions and reports instances involved with aglets and the shoe upper. If the collision distance is negative, the main process starts to monitor this collision. If it persists after certain iterations (10 by default), the aglet is considered to have penetrated the eyestay and the experiment is declared failed.

### Robot Controllers
* There are two types of controllers: GripperController and ArmController.
* Each controllers also has a ROS version which subscribe to trajectory commands in `trajectory_msgs/JointTrajectory` format and publishes the joint states in `sensor_msgs/JointState` format.
* To incorporate the controllers into the MoveIt planning framework, `joint_trajectory_action` servers are written in the `robot_driver_unity` package. They are just wrappers around the controllers inside Unity.

### Cameras
* There are two types of camera wrappers: RGBCameraWrapper and DepthCameraWrapper. They maintain the camera intrinsics and provide interfaces to retrieve rendered images.
* Each wrappers also has a ROS version which publishes the camera images in `sensor_msgs/Image` format with the camera intrinsics in `sensor_msgs/CameraInfo` format.
* The depth images are encoded differently from normal depth images. Following are code snippets to convert images to numpy arrays:
    * Reading the RGB images to numpy arrays:
        ```
        reshaped = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, 3))
        img = np.flipud(reshaped)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        ```
    * Reading the depth images to numpy arrays:
        ```
        depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape((depth_msg.height, depth_msg.width))
        depth = np.flipud(depth)*1000
        ```

### Randomisation
The randomisation is controlled by the `param-live.yaml` file. The randomisation parameters are:
* Shoe position
* Shoelace position
* Light intensity
* Camera depth noise
We model the camera depth noise as a Gaussian distribution.
The error rate is modelled after the [Intel RealSense D435i](https://www.intelrealsense.com/depth-camera-d435/#:~:text=~28%20cm-,Depth%20Accuracy,-%3A%0A%3C2%25%20at) (2% of depth),
as well as the [Intel RealSense L515](https://dev.intelrealsense.com/docs/lidar-camera-l515-datasheet)  (0.25% of depth).

### Coordinate spaces
* Unity uses a left-hand coordinate system which is different from ROS. [`ROSTCPConnector.ROSGeometry`](https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/ROSGeometry.md) is used to convert the coordinates. `Utils` also provides some functions for the conversion in different datatypes. 
* Scale in this environment is 1/10 of the normal size (1 unit = 0.1 meter).

### Resources

* Rendering Materials
    * The rope, aglet and eyelet materials are customisable. 
    * The materials can be in the `param-live.yaml` file. 
    * Several materials are included in the `Resources/Materials` folder.
    * New materials can be added by putting the material file in the `Resources/Materials` folder and adding the file name to the `param-live.yaml` file.

* Collision Materials
    * Friction is controlled by the Obi Collision Materials. The desk and the placemat use the same material `DeskObiMaterial` in `Resources/Materials`. 
    * The materials for the shoe and shoelace are created online and can be tuned in the `param-live.yaml` file.
    * The friction is controlled by the `dynamicFriction` and `staticFriction` parameters. More information can be found in this [documentation](https://obi.virtualmethodstudio.com/manual/6.3/collisionmaterials.html).
    * If the two objects involved in a collision have different materials, by default, the average friction from the two materials is used.
    * This can be changed by opening the Unity Project and modifying the `ShoeManager` and `ShoelaceManager` scripts.

## Advanced Usage
The following changes would require making changes to and rebuilding the Unity project. 
The project is built on Unity 2022.3.22f1.

### Prerequisites
* Unity 2022.3.22f1
* Make sure you have Linux build support installed with Unity.
* Obi Rope and Obi Softbody assets: import manually after opening the Unity project.
* Upon first opening the project, Unity will ask to import TMP libraries. Click on `Import TMP Essentials`.

### Collision table
To reduce computation, the objects involved in the simulation are separated into different groups and the following rule is applied to disable unnecessary collision checking.
|        | Place Mat     | Rope Ends | Shoe Sole | Shoe Upper | Grippers | Eyelets | Aglets | Rope |
|:------:|:----------:|:---------:|:---------:|:----------:|:--------:|:-------:|:------:|:----:|
| Rope   |:heavy_check_mark:|:heavy_check_mark:|:heavy_check_mark:|:heavy_check_mark:|:heavy_check_mark:|:heavy_check_mark:|:heavy_check_mark:|:heavy_check_mark:|
| Aglets |:heavy_check_mark:|:heavy_multiplication_x:|:heavy_check_mark:|:heavy_check_mark:|:heavy_check_mark:|:heavy_check_mark:|:heavy_check_mark:|      |
| Eyelets|:heavy_check_mark:|:heavy_check_mark:|:heavy_check_mark:|:heavy_multiplication_x:|:heavy_check_mark:|:heavy_check_mark:|        |      |
| Grippers|:heavy_multiplication_x:|:heavy_multiplication_x:|:heavy_check_mark:|:heavy_check_mark:|:heavy_check_mark:|         |        |      |
| Shoe Upper|:heavy_check_mark:|:heavy_check_mark:|:heavy_multiplication_x:|:heavy_check_mark:|          |         |        |      |
| Shoe Sole|:heavy_multiplication_x:|:heavy_check_mark:|:heavy_check_mark:|            |          |         |        |      |
| Rope Ends|:heavy_check_mark:|:heavy_check_mark:|           |            |          |         |        |      |
| Place Mat|:heavy_check_mark:|           |           |            |          |         |        |      |

### Customising shoe models
Within the environment, we included a scanned model of an Adidas Stan Smith. following steps are involved to replace this shoe model:
1. An enclosed shoe model, this can be from a 3D scan or CAD design software.
1. Segment the model into 3 components: the shoe tongue, shoe upper and shoe sole. This can be done using Blender following this [tutorial](https://www.youtube.com/watch?v=fVOYv8HdMxI).
1. Export the components in stl format and substitute the files in `Asset/Resources`.
1. Create an Obi Softbody blueprint with the Upper model. This can be done by following the [Obi Softbody tutorial](https://obi.virtualmethodstudio.com/manual/6.3/softbodies.html).

### Customising robots
To use a different robot, the following steps are required:

1. Add the URDF into the Asset folder.
1. Follow the [tutorial](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/urdf_importer/urdf_tutorial.md) to import the URDF to a scene and save it as a prefab in `Asset/Resources/Robots`.
1. Modifying the joint names and params in the config folder of `robot_unity_driver`.
1. Modifying the robot and joint names for the controller settings in the param file.

## Acknowledgement
* Special thanks to [Cedric Goubard](https://www.imperial.ac.uk/personal-robotics/people/phd-students/cedric-goubard/) for his help on the Docker and containerisation of this project.
