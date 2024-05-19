//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UniLace
{
    [Serializable]
    public class UnityStateControllerServiceResponse : Message
    {
        public const string k_RosMessageName = "uni_lace_msgs/UnityStateControllerService";
        public override string RosMessageName => k_RosMessageName;

        // response
        public Geometry.PoseArrayMsg sim_pos;

        public UnityStateControllerServiceResponse()
        {
            this.sim_pos = new Geometry.PoseArrayMsg();
        }

        public UnityStateControllerServiceResponse(Geometry.PoseArrayMsg sim_pos)
        {
            this.sim_pos = sim_pos;
        }

        public static UnityStateControllerServiceResponse Deserialize(MessageDeserializer deserializer) => new UnityStateControllerServiceResponse(deserializer);

        private UnityStateControllerServiceResponse(MessageDeserializer deserializer)
        {
            this.sim_pos = Geometry.PoseArrayMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.sim_pos);
        }

        public override string ToString()
        {
            return "UnityStateControllerServiceResponse: " +
            "\nsim_pos: " + sim_pos.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
