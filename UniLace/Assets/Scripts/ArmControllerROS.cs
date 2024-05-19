using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System;
using RosMessageTypes.Geometry;
using UnityEngine;
using RosQuaternion = RosMessageTypes.Geometry.QuaternionMsg;
using RosBool = RosMessageTypes.Std.BoolMsg;
using RosFloat = RosMessageTypes.Std.Float32Msg;
using RosJointState = RosMessageTypes.Sensor.JointStateMsg;
using RosJointTrajectory = RosMessageTypes.Trajectory.JointTrajectoryMsg;
using RosHeader = RosMessageTypes.Std.HeaderMsg;
using Transform = UnityEngine.Transform;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;

[RequireComponent(typeof(ROSConnection))]
public class ArmControllerROS : ArmController
{
    private static Timer timer = new Timer();

    // Define variables 
    private float publish_rate_control = 0;
    private float update_time_step;

    // ROS Connector
    private ROSConnection rosConnector;
    
    // ROS communication related variables
    private string pathTopicName;
    private string controllerStateTopicName;
    private string jointStateTopicName;
    private string frame_id = "";

    private bool controller_states = false;
    private RosBool controller_state_msg;
    private RosJointState joint_state_msg;


    /// <summary>
    /// jointArticulationBody.jointPosition: rad
    /// state_msg.position: rad
    /// joint1XDrive.target: deg
    /// active_target: rad
    /// </summary>

    public void TrajectoryCallback(RosJointTrajectory trajectory)
    {
        if (trajectory.points.Length != 0)
        {
            // Debug.Log("Trajectory received.");
            StartCoroutine(ExecuteTrajectories(trajectory));
        }
        else
        {
            StopMotion();
        }
    }
    
    private IEnumerator ExecuteTrajectories(RosJointTrajectory trajectory)
    {
        executing=true;
        // deploy joint trajectory
        for (int jointConfigIndex  = 0 ; jointConfigIndex < trajectory.points.Length; jointConfigIndex++)
        {
            List<float> wp = new List<float>();
            for (int joint = 0; joint < jointArticulationBodies.Count; joint++)
            {
                int joint_index = Array.IndexOf(trajectory.joint_names, unityJointNames[joint]);
                wp.Add((float)trajectory.points[jointConfigIndex].positions[joint_index]);
            }
            wayPoints.Add(wp);
        }

        yield return new WaitUntil(() => !executing);
        controller_states = false;
        controller_state_msg.data = controller_states;
        // publish the message
        rosConnector.Publish(controllerStateTopicName, controller_state_msg);
        controller_states = true;
    }

    public RosJointState FetchJointStates()
    {
        joint_state_msg.header.seq ++;
        timer.Now(joint_state_msg.header.stamp);
        for (int i = 0; i < numRobotJoints; i++) {
            int n_i = natural_index[i];
            joint_state_msg.position[i] = jointArticulationBodies[n_i].jointPosition[0];
            joint_state_msg.velocity[i] = jointArticulationBodies[n_i].jointVelocity[0];
            joint_state_msg.effort[i] = jointArticulationBodies[n_i].jointAcceleration[0];
        }
        return joint_state_msg;
    }

    // Initialise joint state messages
    private void InitializeMessage()
    {
        controller_state_msg = new RosBool(false);

        joint_state_msg = new RosJointState
        {
            header = new RosHeader { frame_id = frame_id },
            name = natural_index.Select(i => unityJointNames[i]).ToArray(), // to natural index
            position = new double[numRobotJoints],
            velocity = new double[numRobotJoints],
            effort = new double[numRobotJoints]
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
            FetchJointStates();
            rosConnector.Publish(jointStateTopicName, joint_state_msg);
        }
        else 
        {
            publish_rate_control += Time.deltaTime;
        }
    }

    public void speedRatioCallback(RosFloat speed_ratio)
    {
        changeSpeedRatio(speed_ratio.data);
    }

    async void Start()
    {
        pathTopicName = armName + "/joint_path_command";
        controllerStateTopicName = armName + "/controller_states";
        jointStateTopicName = armName + "/joint_states";

        rosConnector = ROSConnection.instance;
        rosConnector.Subscribe<RosFloat>(armName + "/speed_ratio", speedRatioCallback);

        base.Start();

        InitializeMessage();

        rosConnector.RegisterPublisher<RosBool>(controllerStateTopicName);
        rosConnector.RegisterPublisher<RosJointState>(jointStateTopicName);
        rosConnector.Subscribe<RosJointTrajectory>(pathTopicName, TrajectoryCallback);

        update_time_step = Time.fixedDeltaTime;
    }

}
