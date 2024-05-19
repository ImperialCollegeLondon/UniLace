using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosPose = RosMessageTypes.Geometry.PoseMsg;
using RosPoint = RosMessageTypes.Geometry.PointMsg;
using RosPoseArray = RosMessageTypes.Geometry.PoseArrayMsg;
using RosQuaternion = RosMessageTypes.Geometry.QuaternionMsg;

static class Utils
{
    public static Vector3 Unity2ROS(Vector3 pos) {
        var FLUSpace = pos.To<FLU>();
        return new Vector3(FLUSpace.x, FLUSpace.y, FLUSpace.z);
    }

    public static Quaternion Unity2ROS(Quaternion rot) {
        var FLUSpace = rot.To<FLU>();
        return new Quaternion(FLUSpace.x, FLUSpace.y, FLUSpace.z, FLUSpace.w);
    }

    public static Vector3 ROS2Unity(Vector3 point) {
        // to ROS point
        var p = new RosPoint(point.x, point.y, point.z);
        // to Unity point
        return p.From<FLU>();
    }

    public static Quaternion ROS2Unity(Quaternion quaternion) {
        // to ROS quaternion
        var q = new RosQuaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        // to Unity quaternion
        return q.From<FLU>();
    }

    public static int Sign(float value)
    {
        if (value>0) return 1;
        else if (value==0) return 0;
        else return -1;
    }

    public static Vector3 Sign(Vector3 value)
    {
        return new Vector3(Sign(value.x), Sign(value.y), Sign(value.z));
    }
    
    [System.Diagnostics.Conditional("UNITY_EDITOR")]
    public static void LogMessage(string message)
    {
        Debug.Log(message);
    }

    public static Material CreateMaterial(Color color)
    {
        Material material = new Material(Shader.Find("Standard"));
        material.color = color;
        return material;
    }

}