using System;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.BuiltinInterfaces;
using RosTimeMsg = RosMessageTypes.BuiltinInterfaces.TimeMsg;
using RosHeaderMsg = RosMessageTypes.Std.HeaderMsg;
using RosCameraInfoMsg = RosMessageTypes.Sensor.CameraInfoMsg;
using RosImageMsg = RosMessageTypes.Sensor.ImageMsg;
using RosCompressedImageMsg = RosMessageTypes.Sensor.CompressedImageMsg;
using RosPoseMsg = RosMessageTypes.Geometry.PoseStampedMsg;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.Serialization;
using Unity.Robotics.ROSTCPConnector;
using RosCameraInfoGenerator = Unity.Robotics.ROSTCPConnector.MessageGeneration.CameraInfoGenerator;
using RosMessageExtensions = Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageExtensions;


/// <summary>
/// Useful Links:
/// https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/com.unity.robotics.ros-tcp-connector/Runtime/Messages/Sensor/msg/ImageMsg.cs
/// </summary>


public class RGBCameraWrapperROS : RGBCameraWrapper
{
    ROSConnection rosConnector;
    private string ImagetopicName;
    private string cameraInfoTopicName;
    private string cameraPoseTopicName;
    private string frameId;
    public string robotName;
    public string poseFrame;
    // Publish the cube's position and rotation every N seconds
    private float publishMessageFrequency = 0.1f;
    public float frameRate = 10f;
    private int imgMsgSize;
    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;
    public float seed=1;
    private uint seqNum = 0;
    private Transform robot_local;
    private Quaternion to_aglet_frame = Quaternion.Euler(0, 90f, 90f);
    // private RosCompressedImageMsg message;
    private RosImageMsg message;
    private RosCameraInfoMsg cameraInfoMessage;
    private RosPoseMsg poseMessage;
    private static Timer timer = new Timer();

    async void Start()
    {
        ImagetopicName = cameraName+"/color/image_raw";
        cameraInfoTopicName = cameraName+"/color/camera_info";
        frameId = cameraName+"_color_optical_frame";
        cameraPoseTopicName = cameraName+"/pose";
        publishMessageFrequency = 1/frameRate;
        timeElapsed = publishMessageFrequency*seed/100f;

        // start the ROS connection
        rosConnector = ROSConnection.instance;
        rosConnector.RegisterPublisher<RosImageMsg>(ImagetopicName);
        rosConnector.RegisterPublisher<RosCameraInfoMsg>(cameraInfoTopicName);
        rosConnector.RegisterPublisher<RosPoseMsg>(cameraPoseTopicName);

        var timeMessage = new RosTimeMsg();
        var headerMsg = new RosHeaderMsg(seqNum, timeMessage, frameId);
        int image_step = 3;
        message = new RosImageMsg();
        message.width = (uint) resolutionWidth;
        message.height = (uint) resolutionHeight;
        imgMsgSize = image_step * resolutionWidth; 
        message.step = (uint) imgMsgSize;
        message.encoding = "rgb8";

        // Initialise the RGB camera
        base.Start();

        // Camera.onPostRender += UpdateImage;
        cameraInfoMessage = RosCameraInfoGenerator.ConstructCameraInfoMessage(imageCamera, headerMsg, 0.0f, 0.01f);
        // cameraInfoMessage.k = GetIntrinsic(imageCamera);

        robot_local = GameObject.Find(robotName).transform;
        poseMessage = new RosPoseMsg();
        poseMessage.header.frame_id = poseFrame;
    }

    void LateUpdate()
    {
        timeElapsed += Time.deltaTime;
        if (texture2D != null && timeElapsed > publishMessageFrequency) {
            // execute as coroutine to wait for the EndOfFrame before starting capture
            StartCoroutine(
                UpdateMessage());
            timeElapsed = 0;
        }
    }

    public IEnumerator UpdateMessage()
    {
        // yield return new WaitForEndOfFrame();
        var renderRT =
            RenderTexture.GetTemporary(resolutionWidth, resolutionHeight, 24, RenderTextureFormat.Default, RenderTextureReadWrite.Default, 1);

        // render to offscreen texture (readonly from CPU side)
        RenderTexture.active = renderRT;
        imageCamera.targetTexture = renderRT;
        
        imageCamera.Render();
        texture2D.ReadPixels(rect, 0, 0);
        // texture2D.Apply(); // only necessary if wrting to the texture after this
        yield return null;

        // imageCamera.targetTexture.Release();
        if ( imageCamera.targetTexture != null ) {
            imageCamera.targetTexture = null;
            RenderTexture.ReleaseTemporary(renderRT);
            RenderTexture.active = null;
        }

        // Update image message
        var timeMessage = new RosTimeMsg();
        var headerMsg = new RosHeaderMsg(seqNum, timeMessage, frameId);
        message.header = headerMsg;
        message.data = texture2D.GetRawTextureData();

        // Update camera info message
        cameraInfoMessage.header = headerMsg;

        // Update pose message
        poseMessage.pose.position = robot_local.InverseTransformPoint(imageCamera.transform.position).To<FLU>();
        poseMessage.pose.orientation = (Quaternion.Inverse(robot_local.transform.rotation) * imageCamera.transform.rotation*to_aglet_frame).To<FLU>();

        seqNum += 1;

        // send mesages out
        rosConnector.Send(ImagetopicName, message);
        rosConnector.Send(cameraInfoTopicName, cameraInfoMessage);
        rosConnector.Send(cameraPoseTopicName, poseMessage);
    }
}