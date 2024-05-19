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
using RosHeader = RosMessageTypes.Std.HeaderMsg;
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

public class DepthCameraWrapper : MonoBehaviour
{
    public string cameraName = "unity_camera";
    // The game object
    public Camera imageCamera;
    public double[] cameraIntrinsics;
    public int resolutionWidth = 1280;
    public int resolutionHeight = 720;
    public float noiseVariance = 0; // variance of the gaussian noise

    protected int depth = 32;
    protected RenderTextureFormat format = RenderTextureFormat.RFloat;
    protected RenderTextureReadWrite readWrite = RenderTextureReadWrite.Default;
    protected int antiAliasing = 1;

    protected Texture2D texture2D;
    protected Rect rect;

    protected async void Start()
    {
        imageCamera = GetComponent<Camera>();
        var cameraInfoMessage = RosCameraInfoGenerator.ConstructCameraInfoMessage(imageCamera, new RosHeader());
        cameraIntrinsics = cameraInfoMessage.K;

        // RFloat: reder only the red channel (32bits)
        texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RFloat, false); 
        rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
        var depthShader = Shader.Find("Hidden/DepthCamera");
        //set up camera shader
        var cb = new CommandBuffer();
        cb.SetGlobalFloat("_noiseVariance", noiseVariance);
        imageCamera.AddCommandBuffer(CameraEvent.BeforeForwardOpaque, cb);
        imageCamera.AddCommandBuffer(CameraEvent.BeforeFinalPass, cb);
        imageCamera.SetReplacementShader(depthShader, "");
        imageCamera.backgroundColor = Color.white;
        imageCamera.clearFlags = CameraClearFlags.SolidColor;
        imageCamera.allowHDR = false;
        imageCamera.allowMSAA = false;
    }
    
    public void OnDestroy()
    {
        if (imageCamera.targetTexture != null)
        {
            imageCamera.targetTexture.Release();
        }
        if (texture2D != null)
        {
            Destroy(texture2D);
        }
        if (imageCamera != null)
        {
            imageCamera.RemoveAllCommandBuffers();
            Destroy(imageCamera);
        }
    }
    
    public void Render()
    {
        // yield return new WaitForEndOfFrame();
        var renderRT = RenderTexture.GetTemporary(resolutionWidth, resolutionHeight, 
                depth, format, readWrite, antiAliasing);

        // render to offscreen texture (readonly from CPU side)
        RenderTexture.active = renderRT;
        imageCamera.targetTexture = renderRT;
        
        imageCamera.Render();
        texture2D.ReadPixels(rect, 0, 0);

        if ( imageCamera.targetTexture != null ) {
            imageCamera.targetTexture = null;
            RenderTexture.ReleaseTemporary(renderRT);
            RenderTexture.active = null;
        }
        // return texture2D.GetRawTextureData();
    }

    public byte[] FetchImage()
    {
        return texture2D.GetRawTextureData();
    }
}
