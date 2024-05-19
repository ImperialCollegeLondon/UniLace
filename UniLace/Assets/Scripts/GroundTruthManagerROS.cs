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
using RosString = RosMessageTypes.Std.StringMsg;
using Obi;

public class GroundTruthManagerROS : GroundTruthManager
{
    private static Timer timer = new Timer();

    private ROSConnection rosConnector;
    private float publish_rate_control = 0;
    private float update_time_step;
    private RosString message = new RosString();
    private string topicName = "/groundtruth";

    // Start is called before the first frame update
    async void Start()
    {
        // foreach (System.Reflection.MethodInfo m in typeof(ObiSoftbody).GetMethods())
        //     Debug.Log(m.Name);

        update_time_step = Time.fixedDeltaTime;
        
        // SET ROS CONNECTION
        rosConnector = ROSConnection.instance;
        rosConnector.RegisterPublisher<RosString>(topicName);

        base.Start();
    }

    // publish messages
    // private void FixedUpdate()
    void Update()
    {
        // if (!initialised) return;
        if (publish_rate_control >= 0.1) // control the publish rate at 10 fps
        {
            publish_rate_control = 0;
            UpdateMessage();
            message.data = JsonUtility.ToJson(gt);
            rosConnector.Send(topicName, message);
        }
        else 
        {
            publish_rate_control += update_time_step;
        }
        // Debug.Log("Finish "+(Time.time*1000).ToString());
    }

}
