using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
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
using RosHeader = RosMessageTypes.Std.HeaderMsg;

/// <summary>
/// RGB Camera class that captures images from the camera
/// Call FetchImage() to render the image data to texture2D,
/// then use texture2D.GetRawTextureData() to get the image data
/// </summary>


public class RGBCameraWrapper : MonoBehaviour
{
    public string cameraName = "unity_camera";
    // The game object
    protected Camera imageCamera;
    public double[] cameraIntrinsics;
    public int resolutionWidth = 1280;
    public int resolutionHeight = 720;
    protected int depth = 24;
    protected RenderTextureFormat format = RenderTextureFormat.Default;
    protected RenderTextureReadWrite readWrite = RenderTextureReadWrite.Default;
    protected Texture2D texture2D;
    protected Rect rect;
    public bool rendering = false;

    public async void Start()
    {
        imageCamera = GetComponent<Camera>();
        var cameraInfoMessage = RosCameraInfoGenerator.ConstructCameraInfoMessage(imageCamera, new RosHeader());
        cameraIntrinsics = cameraInfoMessage.K;
        // Initialize game Object
        texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
        rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
        var rgbShader = Shader.Find("Hidden/RGBShader");
        imageCamera.SetReplacementShader(rgbShader, "");
        imageCamera.backgroundColor = Color.gray;
        imageCamera.clearFlags = CameraClearFlags.SolidColor;
        imageCamera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
    }

    public void Render()
    // public IEnumerator Render()
    {
        // rendering = true;
        // yield return null;
        // get image data using texture2D.GetRawTextureData() after this function
        var renderRT = RenderTexture.GetTemporary(resolutionWidth, resolutionHeight, 
                depth, format, readWrite, 1);

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
        rendering = false;
        // return texture2D.GetRawTextureData();
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
            Destroy(imageCamera);
        }
        
    }

    public byte[] FetchImage()
    {
        // wait for the rendering flag to be false

        return texture2D.GetRawTextureData();
    }
}