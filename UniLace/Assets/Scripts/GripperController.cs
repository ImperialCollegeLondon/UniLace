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
public class GripperController : MonoBehaviour
{
    public string gripperName;
    //Define Variables
    private readonly float jointAssignmentWait = 0.1f;
    protected int numGripperJoints;

    public float stiffness = 10000f;
    public float damping = 10f;
    public float forceLimit = 100f;
    public float jointFriction = 10f;
    public float angularDamping = 10f;
    public float speed = 0.1f; // Units: m/s

    private float target_percentage = 0f;
    protected List<float> statePercentage = new List<float>();
    public bool executing = false;

    public float agletWidth = 0.005f;

    // ROS communication related variables
    protected List<string> jointNames = new List<string>();

    // Unity Objects
    public GameObject gripper;
    protected List<ArticulationBody> jointArticulationBodies = new List<ArticulationBody>();

    public enum gripperStates {Open, Opening, Closing, Closed, Holding};

    [HideInInspector] public gripperStates gripper_state = gripperStates.Closed;

    // private IEnumerator gripperTo(float percentage)
    // {
    //     for (int i=0; i<jointArticulationBodies.Count; i++) {
    //         var drive = jointArticulationBodies[i].xDrive;
    //         drive.target = drive.upperLimit * percentage; // may need to modify this for other grippers
    //         jointArticulationBodies[i].xDrive = drive;
    //     }
    //     yield return new WaitForSeconds(jointAssignmentWait);
    // }
    
    public void CloseGripper() {target_percentage = 0f;}
    public void OpenGripper() {target_percentage = 1f;}

    public void SetPositionTarget(float percentage) {
        executing = true;
        float new_target_percentage = Mathf.Clamp(percentage, 0f, 1f);
        if (new_target_percentage>target_percentage) {
            gripper_state = gripperStates.Opening;
        }
        else if (new_target_percentage<target_percentage) {
            gripper_state = gripperStates.Closing;
        }
        target_percentage = new_target_percentage;
    }

    public void SetEffortTarget(float effort) {
        executing = true;
        if (effort>0) {
            gripper_state = gripperStates.Closing;
            target_percentage = 0f;
        }
        if (effort<0) {
            gripper_state = gripperStates.Opening;
            target_percentage = 1f;
        }
    }

    public float GetJointPosition() {
        return jointArticulationBodies[0].jointPosition[0]/10; // to mm
    }

    public void Stop() {
        target_percentage = jointArticulationBodies[0].xDrive.target/jointArticulationBodies[0].xDrive.upperLimit;
    }

    public void Hold() {
        target_percentage = agletWidth/2/jointArticulationBodies[0].xDrive.upperLimit;
    }

    // protected void FixedUpdate()
    protected void Update()
    {
        float target=0;
        for (int i=0; i<jointArticulationBodies.Count; i++) {
            var drive = jointArticulationBodies[i].xDrive;
            target = Mathf.MoveTowards(drive.target, target_percentage*drive.upperLimit, speed * Time.deltaTime); // may need to modify this for other grippers
            drive.target = target;
            jointArticulationBodies[i].xDrive = drive;
        }

        if (gripper_state == gripperStates.Holding) {}
        else if (target_percentage*jointArticulationBodies[0].xDrive.upperLimit==target && target>0) {
            gripper_state = gripperStates.Open;
        }
        else if (target_percentage*jointArticulationBodies[0].xDrive.upperLimit==target && target==0) {
            gripper_state = gripperStates.Closed;
        }

        if (!Mathf.Approximately(statePercentage[0], target_percentage)) {
            executing = false;
        }
    }

    protected async void Start()
    {

        // Extract all articulation bodies with primatic joints 
        UrdfJointPrismatic[] prismaticJoints = gripper.GetComponentsInChildren<UrdfJointPrismatic>();
        for (int i=0; i<prismaticJoints.Length; i++) {
            jointArticulationBodies.Add(prismaticJoints[i].gameObject.GetComponent<ArticulationBody>());
            jointArticulationBodies[i].useGravity = false;
            // jointArticulationBodies[i].jointFriction = jointFriction;
            // jointArticulationBodies[i].angularDamping = angularDamping;
            var drive = jointArticulationBodies[i].xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            jointArticulationBodies[i].xDrive = drive;
            jointNames.Add(prismaticJoints[i].jointName);
            statePercentage.Add(jointArticulationBodies[i].jointPosition[0]/drive.upperLimit);
        }
        numGripperJoints = prismaticJoints.Length;

        // Close grippers by default
        // StartCoroutine(gripperTo(0));
        CloseGripper();
        
        // set aglet width manually
    }

}
