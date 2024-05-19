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

public class ArmController : MonoBehaviour
{
    // Define variables 
    protected int numRobotJoints;
    
    // ROS communication related variables
    public string armName;
    protected List<string> unityJointNames = new List<string>(); // set automatically
    [HideInInspector] public List<string> jointNames = new List<string>(); // command input order

    // Unity Objects
    public GameObject arm;
    public List<float> initialConfig = new List<float>();
    protected List<ArticulationBody> jointArticulationBodies = new List<ArticulationBody>();
    protected List<int> natural_index = new List<int>{0,1,3,4,5,6,2};

    public float stiffness = 100000f;
    public float damping = 1000f;
    public float forceLimit = 1000f;
    public float jointFriction = 10f;
    public float angularDamping = 10f;
    public float speed = 1f; // percentage
    // public float speed = 5f; // unit: deg/s
    // public float speed = 50f; // Units: degree/s
    // public float torque = 100f; // Units: Nm or N
    // public float acceleration = 20f;// Units: m/s^2 / degree/s^2
    public List<float> activeTarget = new List<float>();
    protected List<List<float>> wayPoints = new List<List<float>>();
    public bool executing=false;
    public List<float> distance;

    /// <summary>
    /// jointArticulationBody.jointPosition: rad
    /// state_msg.position: rad
    /// joint1XDrive.target: deg
    /// active_target: rad
    /// </summary>

    public void AddWaypoints(List<float[]> wayPoints, string[] wayPointJointNames=null)
    {
        if (wayPointJointNames==null) {
            wayPointJointNames = jointNames.ToArray();
        }
        foreach (var wayPoint in wayPoints)
        {
            AddWaypoint(wayPoint, wayPointJointNames);
        }
    }

    public void AddWaypoint(float[] wayPoint, string[] wayPointJointNames=null)
    {
        // check if the waypoint is valid
        if (wayPoint.Length!=jointArticulationBodies.Count) {
            return;
        }
        if (wayPointJointNames==null) {
            wayPointJointNames = jointNames.ToArray();
        }
        executing=true; // here or wayponts
        List<float> wp = new List<float>();
        for (int joint = 0; joint < jointArticulationBodies.Count; joint++)
        {
            int joint_index = Array.IndexOf(wayPointJointNames, unityJointNames[joint]);
            wp.Add(wayPoint[joint_index]);
        }
        wayPoints.Add(wp);
    }

    public float[] GetJointPositions()
    {
        return jointArticulationBodies.Select(r => r.jointPosition[0]).ToArray();
    } 

    // protected void FixedUpdate()
    protected void Update()
    {
        if (executing) {
            List<float> jointPositions = jointArticulationBodies.Select(r => r.jointPosition[0]).ToList();
            distance = jointPositions.Zip(activeTarget, (x, y) => Mathf.Abs(x - y)).ToList();
            float thresh = wayPoints.Count>0 ? 0.192f : 0.0026f; // 0.192=11 deg = z15 in rapid; 0.0026 = 0.15 deg = z0 in rapid
            if (distance.All(d => d<thresh)) {
                if (wayPoints.Count!=0) {
                    activeTarget = wayPoints[0];
                    wayPoints.RemoveAt(0);
                }
                else {
                    executing=false;
                }
            }
            for (int i=0; i<jointArticulationBodies.Count; i++) {
                var drive = jointArticulationBodies[i].xDrive;
                // drive.target = Mathf.MoveTowards(jointArticulationBodies[i].jointPosition[0], activeTarget[i], speed*Mathf.Deg2Rad*Time.deltaTime)*Mathf.Rad2Deg;
                drive.target = activeTarget[i]*Mathf.Rad2Deg;
                jointArticulationBodies[i].xDrive = drive;
                // float target = Mathf.MoveTowards(jointArticulationBodies[i].jointPosition[0], activeTarget[i], speed*Mathf.Deg2Rad*Time.deltaTime)*Mathf.Rad2Deg;
                // SetJointPosition(jointArticulationBodies[i], target);
            }
        }
    }

    public void StopMotion() 
    {
        wayPoints = new List<List<float>>();
        for (int i=0; i<jointArticulationBodies.Count; i++) {
            activeTarget[i] = jointArticulationBodies[i].jointPosition[0];
            // jointArticulationBodies[i].jointAcceleration = new ArticulationReducedSpace(0f);
            jointArticulationBodies[i].jointForce = new ArticulationReducedSpace(0f);
            jointArticulationBodies[i].jointVelocity = new ArticulationReducedSpace(0f);
        }
        executing = false;
    }

    public void SetJointPosition(UrdfJointRevolute joint, float angle)
    {
        // set joint position
        ArticulationBody articulation_body = joint.gameObject.GetComponent<ArticulationBody>();
        articulation_body.jointPosition = new ArticulationReducedSpace((float)angle*Mathf.Deg2Rad);
        // articulation_body.jointAcceleration = new ArticulationReducedSpace(0f);
        articulation_body.jointForce = new ArticulationReducedSpace(0f);
        articulation_body.jointVelocity = new ArticulationReducedSpace(0f);
        // set joint goal 
        var joint1XDrive = articulation_body.xDrive;
        joint1XDrive.target = angle;
        articulation_body.xDrive = joint1XDrive;
    }

    public void SetJointPosition(ArticulationBody articulation_body, float angle)
    {
        // set joint position
        articulation_body.jointPosition = new ArticulationReducedSpace((float)angle*Mathf.Deg2Rad);
        // articulation_body.jointAcceleration = new ArticulationReducedSpace(0f);
        articulation_body.jointForce = new ArticulationReducedSpace(0f);
        articulation_body.jointVelocity = new ArticulationReducedSpace(0f);
        // set joint goal 
        var joint1XDrive = articulation_body.xDrive;
        joint1XDrive.target = angle;
        articulation_body.xDrive = joint1XDrive;
    }

    public void changeSpeedRatio(float speed_ratio)
    {
        for (int i=0; i<jointArticulationBodies.Count; i++) {
            var joint1XDrive = jointArticulationBodies[i].xDrive;
            speed = speed_ratio;
            joint1XDrive.damping = damping/speed;
            jointArticulationBodies[i].xDrive = joint1XDrive;
        }
    }

    protected async void Start()
    {
        // Extract all articulation bodies with revolute joints 
        UrdfJointRevolute[] revoluteJoints = arm.GetComponentsInChildren<UrdfJointRevolute>();

        activeTarget = initialConfig.Select(r=> (float)r * Mathf.Deg2Rad).ToList();

        for (int i=0; i<revoluteJoints.Length; i++) {
            // revoluteJoints[i].gameObject.AddComponent<JointControl>();
            jointArticulationBodies.Add(revoluteJoints[i].gameObject.GetComponent<ArticulationBody>());
            unityJointNames.Add(revoluteJoints[i].jointName);
            SetJointPosition(revoluteJoints[i], initialConfig[i]);
            // jointArticulationBodies[i].mass = 0.1f;
            jointArticulationBodies[i].useGravity = false;
            // jointArticulationBodies[i].GetComponent<UrdfInertial>().useUrdfData=true;
            // jointArticulationBodies[i].inertiaTensor = Vector3.zero;
            // jointArticulationBodies[i].inertiaTensorRotation = Quaternion.Euler(0,0,0);
            // jointArticulationBodies[i].jointFriction = jointFriction;
            jointArticulationBodies[i].angularDamping = angularDamping;
            var joint1XDrive = jointArticulationBodies[i].xDrive;
            joint1XDrive.stiffness = stiffness;
            joint1XDrive.damping = damping;
            joint1XDrive.forceLimit = forceLimit;
            jointArticulationBodies[i].xDrive = joint1XDrive;
        }
        // Debug.Log("Number: " + jointArticulationBodies.Count);
        numRobotJoints = revoluteJoints.Length;
    }

}
