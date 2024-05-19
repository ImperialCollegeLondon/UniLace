using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Linq;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector;
using RosPose = RosMessageTypes.Geometry.PoseMsg;
using RosPoseStamped = RosMessageTypes.Geometry.PoseStampedMsg;
using RosPoseArray = RosMessageTypes.Geometry.PoseArrayMsg;
using RosHeader = RosMessageTypes.Std.HeaderMsg;
using RosPoint = RosMessageTypes.Geometry.PointMsg;
using RosQuaternion = RosMessageTypes.Geometry.QuaternionMsg;
using RosFloat = RosMessageTypes.Std.Float32Msg;
using Obi;


public class GroundTruthManager : MonoBehaviour
{
    private Transform aglet1;
    private Transform aglet2;
    private List<List<int>> shoe_particle_ids = new List<List<int>>();
    private ObiSolver solver;
    public string robotName = "robot";
    private Transform robot2world;
    // private bool initialised = false;
    private Quaternion to_aglet_frame = Quaternion.Euler(0, 0, -90f);
    private Quaternion to_eyelet_frame = Quaternion.Euler(-90f, 0, -90f);

    public bool usingSoftbody = true;
    [HideInInspector] public ShoelaceManager sl_manager;
    [HideInInspector] public ShoeManager shoe_manager;

    [HideInInspector] public GroundTruth gt = new GroundTruth();

    // Start is called before the first frame update
    protected async void Start()
    {
        gt.success = false;
        robot2world = GameObject.Find(robotName).transform;
        aglet1 = sl_manager.aglets[0].transform;
        aglet2 = sl_manager.aglets[1].transform;
    }

    public void UpdateMessage() {
        // Update aglet states
        gt.aglet1.position = Utils.Unity2ROS(robot2world.InverseTransformPoint(aglet1.position));
        gt.aglet1.rotation = Utils.Unity2ROS(Quaternion.Inverse(robot2world.rotation)*aglet1.rotation*to_aglet_frame);
        gt.aglet2.position = Utils.Unity2ROS(robot2world.InverseTransformPoint(aglet2.position));
        gt.aglet2.rotation = Utils.Unity2ROS(Quaternion.Inverse(robot2world.rotation)*aglet2.rotation*to_aglet_frame);

        // Update eyelet states
        if (usingSoftbody) {
            List<Pose> eyeletPoses = shoe_manager.GetParticleEyeletPoses();
            // convert to robot space
            gt.eyelets = eyeletPoses.Select(p => new Pose(Utils.Unity2ROS(robot2world.InverseTransformPoint(p.position)),
                (Utils.Unity2ROS(Quaternion.Inverse(robot2world.rotation)*p.rotation*to_eyelet_frame)))).ToList();    
        }
        else {
            List<Pose> eyeletPoses = shoe_manager.GetEyeletPoses();
            // convert to robot space
            gt.eyelets = eyeletPoses.Select(p => new Pose(Utils.Unity2ROS(robot2world.InverseTransformPoint(p.position)),
                (Utils.Unity2ROS(Quaternion.Inverse(robot2world.rotation)*p.rotation*to_eyelet_frame)))).ToList();
        }

        // monitor tension at the rope ends
        gt.rope_tension1 = sl_manager.end_a_tension;
        gt.rope_tension2 = sl_manager.end_b_tension;
        gt.rope_tension = sl_manager.rope_tension;
    }

    public GroundTruth GetGroundTruth() {
        UpdateMessage();
        // return a copy of the ground truth
        return gt;
    }
}
