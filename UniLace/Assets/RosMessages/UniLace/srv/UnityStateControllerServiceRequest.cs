//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UniLace
{
    [Serializable]
    public class UnityStateControllerServiceRequest : Message
    {
        public const string k_RosMessageName = "uni_lace_msgs/UnityStateControllerService";
        public override string RosMessageName => k_RosMessageName;

        // request
        public Geometry.PoseArrayMsg real_pos;

        public UnityStateControllerServiceRequest()
        {
            this.real_pos = new Geometry.PoseArrayMsg();
        }

        public UnityStateControllerServiceRequest(Geometry.PoseArrayMsg real_pos)
        {
            this.real_pos = real_pos;
        }

        public static UnityStateControllerServiceRequest Deserialize(MessageDeserializer deserializer) => new UnityStateControllerServiceRequest(deserializer);

        private UnityStateControllerServiceRequest(MessageDeserializer deserializer)
        {
            this.real_pos = Geometry.PoseArrayMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.real_pos);
        }

        public override string ToString()
        {
            return "UnityStateControllerServiceRequest: " +
            "\nreal_pos: " + real_pos.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}