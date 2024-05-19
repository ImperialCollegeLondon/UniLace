using System.Collections.Generic;
using System.Threading.Tasks;
using System.Collections;
using System.Linq;
using UnityEngine;
using RosMessageTypes.Geometry;
using RosQuaternion = RosMessageTypes.Geometry.QuaternionMsg;
using RosHeader = RosMessageTypes.Std.HeaderMsg;
using RosJointState = RosMessageTypes.Sensor.JointStateMsg;
using RosFloat64 = RosMessageTypes.Std.Float64Msg;
using Transform = UnityEngine.Transform;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;

[RequireComponent(typeof(ROSConnection))]
public class GripperControllerROS : GripperController
{
    private static Timer timer = new Timer();

    //Define Variables
    private float publish_rate_control = 0;
    private float update_time_step;

    // ROS Connector
    private ROSConnection rosConnector;

    // ROS communication related variables
    private string positionTopicName;
    private string effortTopicName;
    private string jointStateTopicName;
    private string frame_id = "";

    private RosMessageTypes.Sensor.JointStateMsg left_joint_state_msg;
    private RosMessageTypes.Sensor.JointStateMsg right_joint_state_msg;
    private RosMessageTypes.Sensor.JointStateMsg gripper_state_msg;


    public async void TrajectoryCallback(RosFloat64 trajectory)
    {
        if (trajectory != null)
        {
            SetPositionTarget((float)trajectory.data/25);
        }
        else {
            // *****************************************
            // Empty trajectory, cancel current execution. 
            // (The trajectory has already been executed under current implementation,
            // skipping this fornow.)
            // *****************************************
        }
    }

    public async void EffortCallback(RosFloat64 effort)
    {
        if (effort != null)
        {            
            SetEffortTarget((float)effort.data);
        }
        else 
        {
            // *****************************************
            // Empty trajectory, cancel current execution. 
            // (The trajectory has already been executed under current implementation,
            // skipping this fornow.)
            // *****************************************
        }
    }

    public RosJointState FetchGripperState()
    {
        // Read gripper joint states
        gripper_state_msg.header.seq ++;
        timer.Now(gripper_state_msg.header.stamp);
        for (int i = 0; i < numGripperJoints; i++) {
            gripper_state_msg.position[i] = jointArticulationBodies[i].jointPosition[0]/10; // to mm
            gripper_state_msg.velocity[i] = jointArticulationBodies[i].jointVelocity[0]/10;
            gripper_state_msg.effort[i] = jointArticulationBodies[i].jointAcceleration[0]/10;
            statePercentage[i] = jointArticulationBodies[i].jointPosition[0]/jointArticulationBodies[i].xDrive.upperLimit;
        }
        return gripper_state_msg;
    }
    
    private void InitializeMessage()
    {
        // Initialise joint state messages
        gripper_state_msg = new RosJointState
        {
            header = new RosHeader { frame_id = frame_id },
            name = jointNames.ToArray(),
            position = new double[numGripperJoints],
            velocity = new double[numGripperJoints],
            effort = new double[numGripperJoints]
        };
    }

    // private void FixedUpdate()
    void Update()
    {
        base.Update();

        // publish joint states
        if (publish_rate_control >= 0.1) // control the publish rate at 10 fps
        {
            publish_rate_control = 0;
            FetchGripperState();
            rosConnector.Publish(jointStateTopicName, gripper_state_msg);
        }
        else 
        {
            publish_rate_control += Time.deltaTime;
        }
    }

    async void Start()
    {
        positionTopicName = gripperName + "_position_cmd";
        effortTopicName = gripperName + "_effort_cmd";
        jointStateTopicName = gripperName + "/joint_states";

        rosConnector = ROSConnection.instance;

        base.Start();

        update_time_step = Time.fixedDeltaTime;

        InitializeMessage();
    
        rosConnector.RegisterPublisher<RosJointState>(jointStateTopicName);
        rosConnector.Subscribe<RosFloat64>(positionTopicName, TrajectoryCallback);
        rosConnector.Subscribe<RosFloat64>(effortTopicName, EffortCallback);

    }

}
