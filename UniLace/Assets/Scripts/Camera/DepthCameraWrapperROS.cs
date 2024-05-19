using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using RosTimeMsg = RosMessageTypes.BuiltinInterfaces.TimeMsg;
using RosHeaderMsg = RosMessageTypes.Std.HeaderMsg;
using RosCameraInfoMsg = RosMessageTypes.Sensor.CameraInfoMsg;
using RosImageMsg = RosMessageTypes.Sensor.ImageMsg;
using RosCompressedImageMsg = RosMessageTypes.Sensor.CompressedImageMsg;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosCameraInfoGenerator = Unity.Robotics.ROSTCPConnector.MessageGeneration.CameraInfoGenerator;
using RosMessageExtensions = Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageExtensions;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.Rendering;
using Unity.Mathematics;
using UnityEngine.Experimental.Rendering;

/// <summary>
/// useful links:
/// Understanding depth texture: https://docs.unity3d.com/Manual/SL-CameraDepthTexture.html
/// Understanding Unity camera: https://docs.unity3d.com/Manual/PhysicalCameras.html
/// Understanding texture formats: https://docs.unity3d.com/ScriptReference/TextureFormat.html
/// Understanding raw texture data: https://docs.unity3d.com/ScriptReference/Texture2D.GetRawTextureData.html
/// Retrieving intrinsics: https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/com.unity.robotics.ros-tcp-connector/Runtime/Extensions/CameraInfoGenerator.cs
/// Retrieving intrinsics: https://github.com/Unity-Technologies/com.unity.perception/issues/548 (the results are not good)
/// Realsense intrinsics: https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#intrinsic-camera-parameters
/// Realsense intrinsics: https://dev.intelrealsense.com/docs/projection-in-intel-realsense-sdk-20
/// </summary>

public class DepthCameraWrapperROS : DepthCameraWrapper
{
    ROSConnection rosConnector;
    private string ImagetopicName;
    private string cameraInfoTopicName;
    private string frameId;
    
    // Publish the cube's position and rotation every N seconds
    private float publishMessageFrequency = 0.1f;
    public float frameRate = 10f;
    private int img_data_size;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;
    public float seed=1;

    private uint seqNum = 0;
    // private RosCompressedImageMsg message;
    [HideInInspector] public RosImageMsg message;
    [HideInInspector] public RosCameraInfoMsg cameraInfoMessage;


    async void Start()
    {
        base.Start();
        ImagetopicName = cameraName+"/aligned_depth_to_color/image_raw";
        cameraInfoTopicName = cameraName+"/aligned_depth_to_color/camera_info";
        frameId = cameraName+"_color_optical_frame";
        publishMessageFrequency = 1/frameRate;
        timeElapsed = publishMessageFrequency*seed/100f;

        // start the ROS connection
        rosConnector = ROSConnection.GetOrCreateInstance();
        rosConnector.RegisterPublisher<RosImageMsg>(ImagetopicName);
        rosConnector.RegisterPublisher<RosCameraInfoMsg>(cameraInfoTopicName);

        // initialise the messages
        var timeMessage = new RosTimeMsg();
        var headerMsg = new RosHeaderMsg(seqNum, timeMessage, frameId);
        cameraInfoMessage = RosCameraInfoGenerator.ConstructCameraInfoMessage(imageCamera, headerMsg);
        // cameraInfoMessage.k = GetIntrinsic(imageCamera);

        int image_step = 4;
        message = new RosImageMsg();
        message.width = (uint) resolutionWidth;
        message.height = (uint) resolutionHeight;
        img_data_size = image_step * resolutionWidth; 
        message.step = (uint) img_data_size;
        message.encoding = "rgba8";
    }

    // void FixedUpdate()
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
            RenderTexture.GetTemporary(resolutionWidth, resolutionHeight, depth, format, readWrite, antiAliasing);

        // render to offscreen texture (readonly from CPU side)
        RenderTexture.active = renderRT;
        imageCamera.targetTexture = renderRT;
        
        imageCamera.Render();
        texture2D.ReadPixels(rect, 0, 0);
        yield return null;

        // imageCamera.targetTexture.Release();
        if ( imageCamera.targetTexture != null ) {
            imageCamera.targetTexture = null;
            RenderTexture.ReleaseTemporary(renderRT);
            RenderTexture.active = null;
        }

        var timeMessage = new RosTimeMsg();
        var headerMsg = new RosHeaderMsg(seqNum, timeMessage, frameId);
        message.header = headerMsg;
        message.data = texture2D.GetRawTextureData();

        // Camera Info message
        cameraInfoMessage.header = headerMsg;

        // seqNum += 1;

        // Publish messages
        rosConnector.Publish(ImagetopicName, message);
        rosConnector.Publish(cameraInfoTopicName, cameraInfoMessage);
    }
}
