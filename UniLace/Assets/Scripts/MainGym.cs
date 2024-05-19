using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Linq;
using System.IO;
using System;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using Random=UnityEngine.Random;
using UnityEditor;
using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;
using Unity.Mathematics;
using Obi;
// using TMPro;
using RosMessageTypes.UniLace;

/// <summary>
/// Tunneling in Obi only happens due to:
/// 1. Telepotation of objects. This does not exist in this project since nothing is moved by
///     directly changing the transform. (apart from the initialisation stage)
/// 2. Constraints violate contact conditions. This happens when the rope is taut.
/// 
/// Useful links:
/// Tunneling in Obi: https://obi.virtualmethodstudio.com/forum/thread-2968-post-10130.html
/// Performance: https://docs.unity3d.com/Manual/ProfilerCPU.html
/// </summary>

public class MainGym : MonoBehaviour
{
    [HideInInspector] public ShoelaceManager shoelace;
    [HideInInspector] public ShoeManager shoe;
    [HideInInspector] public SlParams pm;
    private ObiSolver obiSolver;

    public bool debug = false;
    public bool demoMode = false;
    public bool useRobot = true;
    // DEFINE OBI RELATED VARIABLES
    private ObiFixedUpdater obiUpdater;

    [HideInInspector] public Transform robot2world;

    private float end_a_tension;
    private float end_b_tension;
    private float rope_tension;
    private GameObject shoe_sole;
    private GameObject robot;

    private Vector3 agletInitPosA=new Vector3(-2.1f, 0.36f, 2.75f);
    private Vector3 agletInitPosB=new Vector3(2.1f, 0.36f, 2.75f);

    private int counter = 0; 
    public int agletMonitorCounterA = 0;
    public int agletMonitorCounterB = 0;
    public int shoelaceTensionCounter = 0;
    public int endATensionCounter = 0;
    public int endBTensionCounter = 0;

    // DEFINE ROS CONNECTION VARIABLES
    // private ROSConnection rosConnector;
    private string stepServiceName = "/unity_step";
    private string resetServiceName = "/unity_reset";
    private string infoServiceName = "/unity_metric_info";

    // DEFINE SIMULATION PARAMETERS
    private int horizon=100;
    private int horizon_counter=0;
    private bool waitforstart=true;
    [HideInInspector] public bool generated=false;

    private RandomisedParams defaultRandParams = new RandomisedParams();

    private ArmController armLeftController;
    private ArmController armRightController;
    private GripperController gripperLeftController;
    private GripperController gripperRightController;
    private List<RGBCameraWrapper> rgbCameras = new List<RGBCameraWrapper>();
    private List<DepthCameraWrapper> depthCameras = new List<DepthCameraWrapper>();
    private GroundTruthManager groundTruthManager;
    private ShoeUpperCollisionManager shoeUpperCollisionManager;

    public List<bool> eyeletToBeLaced = new List<bool>();
    public List<float> eyestayInitGaps = new List<float>();

    // public TMP_Text statusText;
    // timer for no motion 
    private float noMotionTimer = 0;
    private GameObject MainCamera;
    protected FpsDisplay fpsDisplay;

    protected async void Start()
    {
        MainCamera = GameObject.Find("MainCamera");
        MainCamera.SetActive(false);
        fpsDisplay = gameObject.AddComponent<FpsDisplay>();
        // GET PARAMETERS
        ParamManager pmManager = new ParamManager();
        await pmManager.WaitForParam();
        pm = pmManager.p;
        Utils.LogMessage("Parameter Received.");

        // SETUP SIMULATION PARAMETERS
        // Limit framerate to cinematic 24fps.
        QualitySettings.vSyncCount = 0; // Set vSyncCount to 0 so that using .targetFrameRate is enabled.
        Application.targetFrameRate = 24;
        Time.timeScale = pm.time_scale;
        Time.fixedDeltaTime = pm.time_step;
        horizon = pm.horizon;
        debug = pm.debug;

        defaultRandParams.light_intensity = 1;
        defaultRandParams.shoe_position = GameObject.Find("StanSmith").transform.position;

        // set gym services
        ROSConnection rosConnector = ROSConnection.GetOrCreateInstance();
        rosConnector.ImplementService<UniLaceResetServiceRequest, 
                UniLaceResetServiceResponse>(resetServiceName, Reset);
        rosConnector.ImplementService<UniLaceStepServiceRequest, 
                UniLaceStepServiceResponse>(stepServiceName, Step);
        rosConnector.ImplementService<UniLaceInfoServiceRequest, 
                UniLaceInfoServiceResponse>(infoServiceName, GetMetricInfo);

        // GENERATE THE SCENE
        await GenerateScene();

        generated=true;
    }

    public async Task GenerateScene() {
        MainCamera.SetActive(false);
        demoMode = pm.demo_mode;
        eyeletToBeLaced = Enumerable.Repeat(true, pm.num_eyelets).ToList();

        // GENERATE THE ROBOT
        if (useRobot) {
            robot = Instantiate(Resources.Load<GameObject>("Robots/"+pm.robot.name), 
                    pm.robot.pose.position, 
                    pm.robot.pose.rotation);
            robot.name = pm.robot.name;
            robot.transform.localScale = new Vector3(10, 10, 10);
        }
        else {
            robot = new GameObject(pm.robot.name);
        }
        robot2world = robot.transform;
        
        // Add arm controllers
        armLeftController = gameObject.AddComponent<ArmController>();
        armLeftController.armName = pm.robot.left_arm.arm_name;
        armLeftController.arm = GameObject.Find(pm.robot.left_arm.base_link);
        armLeftController.jointNames = pm.robot.left_arm.joint_names;
        armLeftController.initialConfig = pm.robot.left_arm.initial_config;
        armLeftController.stiffness = pm.robot.left_arm.stiffness;
        armLeftController.damping = pm.robot.left_arm.damping;
        armLeftController.forceLimit = pm.robot.left_arm.force_limit;
        armLeftController.jointFriction = pm.robot.left_arm.joint_friction;
        armLeftController.angularDamping = pm.robot.left_arm.angular_damping;
        armLeftController.speed = pm.robot.left_arm.speed;

        armRightController = gameObject.AddComponent<ArmController>();
        armRightController.armName = pm.robot.right_arm.arm_name;
        armRightController.arm = GameObject.Find(pm.robot.right_arm.base_link);
        armRightController.jointNames = pm.robot.right_arm.joint_names;
        armRightController.initialConfig = pm.robot.right_arm.initial_config;
        armRightController.stiffness = pm.robot.right_arm.stiffness;
        armRightController.damping = pm.robot.right_arm.damping;
        armRightController.forceLimit = pm.robot.right_arm.force_limit;
        armRightController.jointFriction = pm.robot.right_arm.joint_friction;
        armRightController.angularDamping = pm.robot.right_arm.angular_damping;
        armRightController.speed = pm.robot.right_arm.speed;

        // Add gripper controllers
        gripperLeftController = gameObject.AddComponent<GripperController>();
        gripperLeftController.gripperName = pm.robot.left_gripper.gripper_name;
        gripperLeftController.stiffness = pm.robot.left_gripper.stiffness;
        gripperLeftController.damping = pm.robot.left_gripper.damping;
        gripperLeftController.forceLimit = pm.robot.left_gripper.force_limit;
        gripperLeftController.jointFriction = pm.robot.left_gripper.joint_friction;
        gripperLeftController.angularDamping = pm.robot.left_gripper.angular_damping;
        gripperLeftController.speed = pm.robot.left_gripper.speed;
        gripperLeftController.gripper = GameObject.Find(pm.robot.left_gripper.base_link);

        gripperRightController = gameObject.AddComponent<GripperController>();
        gripperRightController.gripperName = pm.robot.right_gripper.gripper_name;
        gripperRightController.stiffness = pm.robot.right_gripper.stiffness;
        gripperRightController.damping = pm.robot.right_gripper.damping;
        gripperRightController.forceLimit = pm.robot.right_gripper.force_limit;
        gripperRightController.jointFriction = pm.robot.right_gripper.joint_friction;
        gripperRightController.angularDamping = pm.robot.right_gripper.angular_damping;
        gripperRightController.speed = pm.robot.right_gripper.speed;
        gripperRightController.gripper = GameObject.Find(pm.robot.right_gripper.base_link);

        // SET RANDOMISED PARAMS
        // randomise intensity of all lights
        Light[] lights = FindObjectsOfType<Light>();
        foreach (Light light in lights) {
            light.intensity = defaultRandParams.light_intensity + 
                    Random.Range(-pm.random_range_light_intensity, pm.random_range_light_intensity);
        }
        // randomise the configuration of the shoelace
        Vector3 noise = Vector3.Scale(new Vector3(Random.value, Random.value, Random.value), pm.random_range_shoelace_position);
        agletInitPosA = pm.aglet_initial_position_l+noise;
        agletInitPosB = pm.aglet_initial_position_r+noise;
        // set the initial configuration of the shoelace
        List<Vector3> points = new List<Vector3>();
        // add noise to every point
        for (int i=0; i<pm.rope_num_ctrl_pts; i++) {
            // convert to unity coordinates
            points.Add(robot2world.TransformPoint(Utils.ROS2Unity(pm.rope_init_config[i])));
            points[i] += noise;
        }
        if (pm.aglet_initial_position_l==Vector3.zero)
            agletInitPosA = points[0];
        else
            agletInitPosA = pm.aglet_initial_position_l+noise;
        if (pm.aglet_initial_position_r==Vector3.zero)
            agletInitPosB = points[^1];
        else
            agletInitPosB = pm.aglet_initial_position_r+noise;
        // set the initial configuration of the shoe
        Vector3 shoePositionNoise = Vector3.Scale(new Vector3(Random.value, Random.value, Random.value), pm.random_range_shoe_position);
        

        // CONFIGURE OBI SOLVER AND UPDATER
        GameObject solverObject = GameObject.Find("Obi Solver");
        if (solverObject==null)
            solverObject = new GameObject("Obi Solver", typeof(ObiSolver));
        ObiSolver obiSolver = solverObject.GetComponent<ObiSolver>();
        // Configure the solver
        obiSolver.gravity = new Vector3(0.0f, pm.solver_gravity, 0.0f); // Vector3.zero;
        obiSolver.parameters.damping = pm.solver_damping;
        obiSolver.parameters.surfaceCollisionIterations = pm.solver_surface_collision_iterations;
        obiSolver.distanceConstraintParameters.iterations = pm.solver_distance_constraint_iterations;
        obiSolver.shapeMatchingConstraintParameters.iterations = pm.solver_shape_matching_iterations;
        obiSolver.particleCollisionConstraintParameters.iterations = pm.solver_particle_collision_constraint_iterations;
        obiSolver.collisionConstraintParameters.iterations = pm.solver_collision_constraint_iterations;
        obiSolver.parameters.collisionMargin = pm.solver_collision_margin;
        obiSolver.parameters.maxDepenetration = pm.solver_collision_max_depenetration;
        obiSolver.PushSolverParameters();

        obiUpdater = solverObject.GetComponent<ObiFixedUpdater>();
        if (obiUpdater==null)
            obiUpdater = solverObject.AddComponent<ObiFixedUpdater>();
            if (obiUpdater.solvers.Count==0)
                obiUpdater.solvers.Add(obiSolver); // add the solver to the updater:
        obiUpdater.substeps = pm.solver_updater_substeps;

        // GENERATE CAMERAS
        foreach (CameraParams cp in pm.cameras) {
            if (cp.type.Contains("rgb")) {
                // create an rgb camera
                GameObject cameraObj = new GameObject(cp.name);
                cameraObj.transform.parent = GameObject.Find(cp.parent).transform;
                cameraObj.transform.localScale = Vector3.one;
                Camera camera = cameraObj.AddComponent<Camera>();
                camera.name = cp.name;
                camera.enabled = false;
                camera.fieldOfView = cp.fov;
                camera.usePhysicalProperties = true;
                camera.farClipPlane = cp.clipping_depth[1];
                camera.nearClipPlane = cp.clipping_depth[0];
                camera.sensorSize = cp.sensor_size;
                camera.focusDistance = cp.focus_distance;
                camera.focalLength = cp.focus_length;
                camera.lensShift = cp.lens_shift;
                camera.transform.localPosition = cp.position;
                camera.transform.localEulerAngles = cp.rotation;
                camera.targetDisplay = 8;
                // Create an RGB camera Wrapper
                RGBCameraWrapper rgbCam = cameraObj.AddComponent<RGBCameraWrapper>();
                rgbCam.cameraName = cp.name;
                rgbCam.resolutionWidth = (int)cp.resolution[0];
                rgbCam.resolutionHeight = (int)cp.resolution[1];
                rgbCameras.Add(rgbCam);
            } 
            if (cp.type.Contains("d")) {
                // create a depth camera
                GameObject cameraObj = new GameObject(cp.name);
                cameraObj.transform.parent = GameObject.Find(cp.parent).transform;
                cameraObj.transform.localScale = Vector3.one;
                Camera camera = cameraObj.AddComponent<Camera>();
                camera.name = cp.name+"Depth";
                camera.enabled = false;
                camera.fieldOfView = cp.fov;
                camera.usePhysicalProperties = true;
                camera.farClipPlane = cp.clipping_depth[1];
                camera.nearClipPlane = cp.clipping_depth[0];
                camera.sensorSize = cp.sensor_size;
                camera.focusDistance = cp.focus_distance;
                camera.focalLength = cp.focus_length;
                camera.lensShift = cp.lens_shift;
                camera.transform.localPosition = cp.position;
                camera.transform.localEulerAngles = cp.rotation;
                camera.targetDisplay = 8;
                // Create a Depth camera Wrapper
                DepthCameraWrapper depthCam = cameraObj.AddComponent<DepthCameraWrapper>();
                depthCam.cameraName = cp.name;
                depthCam.resolutionWidth = (int)cp.resolution[0];
                depthCam.resolutionHeight = (int)cp.resolution[1];
                depthCam.noiseVariance = cp.depth_noise_stddev;
                depthCameras.Add(depthCam);
            }
        }

        // GENERATE THE SHOELACE
        shoelace = gameObject.AddComponent<ShoelaceManager>();
        shoelace.Init(obiSolver,
                      ropeInitialStates:points,
                      pm:pm,
                      debug:debug);
        shoelace.agletCollisionManager0.gripper_left_controller = gripperLeftController;
        shoelace.agletCollisionManager0.gripper_right_controller = gripperRightController;
        shoelace.agletCollisionManager0.gripper_left = gripperLeftController.gripper;
        shoelace.agletCollisionManager0.gripper_right = gripperRightController.gripper;

        shoelace.agletCollisionManager1.gripper_left_controller = gripperLeftController;
        shoelace.agletCollisionManager1.gripper_right_controller = gripperRightController;
        shoelace.agletCollisionManager1.gripper_left = gripperLeftController.gripper;
        shoelace.agletCollisionManager1.gripper_right = gripperRightController.gripper;
                
        gripperLeftController.agletWidth = shoelace.agletWidth;
        gripperRightController.agletWidth = shoelace.agletWidth;

        // GENERATE THE SHOE
        shoe = gameObject.AddComponent<ShoeManager>();
        shoe.Init(obiSolver,
                  pm:pm,
                  shoePositionNoise:defaultRandParams.shoe_position+shoePositionNoise);
        shoeUpperCollisionManager = 
                shoe.shoeUpperObj.AddComponent<ShoeUpperCollisionManager>();
        shoeUpperCollisionManager.agletWidth = shoelace.agletWidth;

        // GENERATE GROUNDTRUTH MANAGER
        groundTruthManager = gameObject.AddComponent<GroundTruthManager>();
        groundTruthManager.robotName = pm.robot.name;
        groundTruthManager.usingSoftbody = false;
        groundTruthManager.sl_manager = shoelace;
        groundTruthManager.shoe_manager = shoe;

        // MOVE AGLETS TOWARDS INITIAL POSITIONS
        bool movedAglets = false;
        while (!movedAglets) {
            bool movedAglet0 = shoelace.MoveAgletTowardsPose(0, agletInitPosA, Quaternion.Euler(0, 0, 0), Time.deltaTime);
            bool movedAglet1 = shoelace.MoveAgletTowardsPose(1, agletInitPosB, Quaternion.Euler(0, 0, 0), Time.deltaTime);
            movedAglets = movedAglet0 && movedAglet1;
            await Task.Yield();
        }
        shoelace.UpdateRopeEndCollisionFilters();
        shoelace.SetShoelaceRestLength();
        eyestayInitGaps = shoe.EstimateEyestayGaps();
        // statusText.text = "Status: Ready";
        fpsDisplay.sysMessage = "Status: Ready";
        // enable the main camera
        MainCamera.SetActive(true);
        fpsDisplay.showFPS = true;
    }


    public async Task<UniLaceStepServiceResponse> Step(UniLaceStepServiceRequest request)
    {
        // Preocess the service messages
        UniLaceStepRequest action = JsonUtility.FromJson<UniLaceStepRequest>(request.act.data);
        UniLaceStepServiceResponse responseMsg = new UniLaceStepServiceResponse();

        if (demoMode) {
            // directly move the aglets to the target positions
            shoelace.MoveAgletTowardsPose(0,
                    robot2world.TransformPoint(Utils.ROS2Unity(action.aglet_pose_a.position)),
                    robot2world.rotation*Utils.ROS2Unity(action.aglet_pose_a.rotation),
                    Time.deltaTime);
            shoelace.MoveAgletTowardsPose(1, 
                    robot2world.TransformPoint(Utils.ROS2Unity(action.aglet_pose_b.position)),
                    robot2world.rotation*Utils.ROS2Unity(action.aglet_pose_b.rotation),
                    Time.deltaTime);
            await Task.Yield();
            UniLaceStepResponse response = new UniLaceStepResponse();
            response.ground_truth = groundTruthManager.GetGroundTruth();
            responseMsg.obs.data = JsonUtility.ToJson(response);
            return responseMsg;
        }

        // create a timer
        System.Diagnostics.Stopwatch stopwatch = new System.Diagnostics.Stopwatch();
        stopwatch.Start();

        Utils.LogMessage("Received Action: "+request.act.data);

        // Deploy actions
        armLeftController.AddWaypoint(action.left_arm);
        armLeftController.changeSpeedRatio(action.left_arm_vel);
        gripperLeftController.SetPositionTarget(action.left_gripper);
        armRightController.AddWaypoint(action.right_arm);
        armRightController.changeSpeedRatio(action.right_arm_vel);
        gripperRightController.SetPositionTarget(action.right_gripper);

        stopwatch.Stop();
        Utils.LogMessage("Read Time Usage: "+stopwatch.ElapsedMilliseconds.ToString()+"ms");
        stopwatch.Reset();
        stopwatch.Start();

        // wait for horizon number of frames
        while (horizon_counter<horizon) {
            await Task.Yield();
            horizon_counter += 1;
        }
        horizon_counter = 0;

        stopwatch.Stop();
        Utils.LogMessage("Step wait Time Usage: "+stopwatch.ElapsedMilliseconds.ToString()+"ms");
        stopwatch.Reset();
        stopwatch.Start();

        // make observation
        MakeObservation(out responseMsg.obs.data, out responseMsg.obs_raw.data);

        // print time usage
        stopwatch.Stop();
        Utils.LogMessage("Observation Time Usage: "+stopwatch.ElapsedMilliseconds.ToString()+"ms");
        return responseMsg;
    }

    private void MakeObservation(out string obs, out byte[] obs_raw) {
        UniLaceStepResponse response = new UniLaceStepResponse();
        // render the cameras
        List<RGBDImage> rgbd_images = new List<RGBDImage>();
        List<Task> tasks = new List<Task>();
        foreach (RGBCameraWrapper rgbCam in rgbCameras) {
            // Task t = Task.Run(() => rgbCam.Render());
            // tasks.Add(t);
            rgbCam.Render();
        }
        foreach (DepthCameraWrapper depthCam in depthCameras) {
            // Task t = Task.Run(() => depthCam.Render());
            // tasks.Add(t);
            depthCam.Render();
        }
        // await Task.WhenAll(tasks.ToArray());
        // read the textures
        int rbgId = 0;
        int depthId = 0;
        int raw_id = 0;
        List<byte[]> raws = new List<byte[]>();
        for (int i=0; i<pm.cameras.Count; i++) {
            RGBDImage rgbdImage = new RGBDImage();
            if (pm.cameras[i].type.Contains("rgb")) {
                raws.Add(rgbCameras[rbgId].FetchImage());
                int new_raw_id = raw_id + raws[^1].Length;
                rgbdImage.raw_range_rgb = new Vector2Int(raw_id, new_raw_id);
                raw_id = new_raw_id;
                rgbdImage.name = rgbCameras[rbgId].cameraName;
                rgbdImage.resolution = new Vector2Int(rgbCameras[rbgId].resolutionWidth, 
                        rgbCameras[rbgId].resolutionHeight);
                rgbdImage.camera_pose = new Pose(rgbCameras[rbgId].gameObject.transform.position, 
                        rgbCameras[rbgId].gameObject.transform.rotation);
                rgbdImage.camera_intrinsics = 
                        rgbCameras[rbgId].cameraIntrinsics.Select(x => (float)x).ToArray();
                rbgId += 1;
            }
            else {
                // rgbdImage.rgb = new byte[0];
                rgbdImage.raw_range_rgb = new Vector2Int(raw_id, raw_id);
            }
            if (pm.cameras[i].type.Contains("d")) {
                raws.Add(depthCameras[depthId].FetchImage());
                int new_raw_id = raw_id + raws[^1].Length;
                rgbdImage.raw_range_depth = new Vector2Int(raw_id, new_raw_id);
                raw_id = new_raw_id;
                if (!pm.cameras[i].type.Contains("rgb")) {
                    rgbdImage.name = depthCameras[depthId].cameraName;
                    rgbdImage.resolution = new Vector2Int(depthCameras[depthId].resolutionWidth, 
                            depthCameras[depthId].resolutionHeight);
                    rgbdImage.camera_pose = new Pose(depthCameras[depthId].gameObject.transform.position, 
                            depthCameras[depthId].gameObject.transform.rotation);
                    rgbdImage.camera_intrinsics =
                            depthCameras[depthId].cameraIntrinsics.Select(x => (float)x).ToArray();
                }
                depthId += 1;
            }
            rgbd_images.Add(rgbdImage);
        }
        response.rgbd_images = rgbd_images.ToArray();
        response.left_arm = armLeftController.GetJointPositions();
        response.left_gripper = gripperLeftController.GetJointPosition();
        response.right_arm = armRightController.GetJointPositions();
        response.right_gripper = gripperRightController.GetJointPosition();
        response.ground_truth = groundTruthManager.GetGroundTruth();
        obs = JsonUtility.ToJson(response);
        obs_raw = ByteCombine(raws);
    }

    // function combines list of byte arrays into one byte array
    public byte[] ByteCombine(List<byte[]> arrays)
    {
        byte[] ret = new byte[arrays.Sum(x => x.Length)];
        int offset = 0;
        foreach (byte[] data in arrays)
        {
            System.Buffer.BlockCopy(data, 0, ret, offset, data.Length);
            offset += data.Length;
        }
        return ret;
    }


    private async Task<UniLaceResetServiceResponse> Reset(UniLaceResetServiceRequest request)
    {
        // reset the scene
        Utils.LogMessage("Resetting the scene.");
        generated=false;
        // statusText.text = "Status: Initialising";
        fpsDisplay.sysMessage = "Status: Initialising";
        // remove created objects and components
        Destroy(shoelace);
        Destroy(shoe);
        Destroy(groundTruthManager);
        foreach (RGBCameraWrapper rgbCam in rgbCameras) {
            Destroy(rgbCam);
        }
        rgbCameras.Clear();
        foreach (DepthCameraWrapper depthCam in depthCameras) {
            Destroy(depthCam);
        }
        depthCameras.Clear();
        Destroy(armLeftController);
        Destroy(armRightController);
        Destroy(gripperLeftController);
        Destroy(gripperRightController);
        Destroy(robot);

        // wait for the next frame
        await Task.Yield();
        
        // generate the scene
        await GenerateScene();

        generated=true;
        // SceneManager.LoadScene(SceneManager.GetActiveScene().name);
        Utils.LogMessage("Scene Reset.");
        // waitforstart=true;

        // prepare the response
        UniLaceResetServiceResponse response = new UniLaceResetServiceResponse();
        MakeObservation(out response.obs.data, out response.obs_raw.data);
        return response;
    }

    protected void Pause() {
        Time.timeScale = 0;
    }

    void Continue() {
        Time.timeScale = 1;
    }

    void Stop() {
        Application.Quit();
    }

    // void FixedUpdate() 
    protected void Update()
    {
        // do not start untill all components are generated
        if (!generated) return;
        
        // check how long the simulation has been running without motion
        if (armLeftController.executing || armRightController.executing || 
                gripperLeftController.executing || gripperRightController.executing) {
            noMotionTimer = 0;
        }
        else {
            noMotionTimer += Time.deltaTime;
            if (noMotionTimer>pm.no_motion_time_thresh) {
                // statusText.text = "Status: Failed (No motion)";
                fpsDisplay.sysMessage = "Status: Failed (No motion after "+pm.no_motion_time_thresh+"s)";
                Utils.LogMessage("No motion after "+pm.no_motion_time_thresh+"s!");
                Pause();
            }
        }

        // check if eyelets have been laced
        if (shoelace.agletCollisionManager0.monitoredEyeletIndex>=0) {
            // transform eyelet position to aglet frame
            Vector3 eyeletPos = shoelace.aglets[0].transform.InverseTransformPoint(
                    shoe.eyelets[shoelace.agletCollisionManager0.monitoredEyeletIndex].position)*0.1f; // aglet scale 0.1
            float distance = eyeletPos.magnitude;
            float distanceZ = eyeletPos.z;
            // check if distance is within threshold
            bool inRange = distance<pm.eyelet_outer_radius && eyeletPos.z>0 && eyeletPos.z<pm.lace_thresh_z;
            if (inRange) {
                Vector3 agletZ = shoelace.aglets[0].transform.forward;
                Vector3 eyeletY = shoe.eyelets[shoelace.agletCollisionManager0.monitoredEyeletIndex].up;
                // check if the z axis of the aglet is aligned with the y axis of the eyelet
                float angle = Vector3.Angle(agletZ, eyeletY);
                bool aligned = angle<pm.lace_thresh_angle || angle>180-pm.lace_thresh_angle;
                if (aligned) {
                    Utils.LogMessage("Marking eyelet "+shoelace.agletCollisionManager0.monitoredEyeletIndex+
                            " as laced with a distance of "+eyeletPos.z);
                    // set the eyelet to be laced
                    eyeletToBeLaced[shoelace.agletCollisionManager0.monitoredEyeletIndex] = false;
                    groundTruthManager.gt.eyelet_to_be_laced = eyeletToBeLaced;
                    // eyelet has been laced
                    shoelace.agletCollisionManager0.monitoredEyelet = null;
                    shoelace.agletCollisionManager0.monitoredEyeletIndex = -1;
                }
            }
            if (distance > 1) {
                Utils.LogMessage("Stop monitoring eyelet "+shoelace.agletCollisionManager0.monitoredEyeletIndex+
                        ".");
                shoelace.agletCollisionManager0.monitoredEyelet = null;
                shoelace.agletCollisionManager0.monitoredEyeletIndex = -1;
            }
        }
        if (shoelace.agletCollisionManager1.monitoredEyeletIndex>=0) {
            // transform eyelet position to aglet frame
            Vector3 eyeletPos = shoelace.aglets[1].transform.InverseTransformPoint(
                    shoe.eyelets[shoelace.agletCollisionManager1.monitoredEyeletIndex].position)*0.1f;
            float distance = eyeletPos.magnitude;
            float distanceZ = eyeletPos.z;
            // check if distance is within threshold
            bool inRange = distance<pm.eyelet_outer_radius && eyeletPos.z>0 && eyeletPos.z<pm.lace_thresh_z;
            if (inRange) {
                Vector3 agletZ = shoelace.aglets[1].transform.forward;
                Vector3 eyeletY = shoe.eyelets[shoelace.agletCollisionManager1.monitoredEyeletIndex].up;
                // check if the z axis of the aglet is aligned with the y axis of the eyelet
                float angle = Vector3.Angle(agletZ, eyeletY);
                bool aligned = angle<pm.lace_thresh_angle || angle>180-pm.lace_thresh_angle;
                if (aligned) {
                    Utils.LogMessage("Marking eyelet "+shoelace.agletCollisionManager1.monitoredEyeletIndex+
                            " as laced with a distance of "+eyeletPos.z);
                    // set the eyelet to be laced
                    eyeletToBeLaced[shoelace.agletCollisionManager1.monitoredEyeletIndex] = false;
                    groundTruthManager.gt.eyelet_to_be_laced = eyeletToBeLaced;
                    // eyelet has been laced
                    shoelace.agletCollisionManager1.monitoredEyelet = null;
                    shoelace.agletCollisionManager1.monitoredEyeletIndex = -1;
                }
            }
            if (distance > 1) {
                Utils.LogMessage("Stop monitoring eyelet "+shoelace.agletCollisionManager1.monitoredEyeletIndex+
                        ".");
                shoelace.agletCollisionManager1.monitoredEyelet = null;
                shoelace.agletCollisionManager1.monitoredEyeletIndex = -1;
            }
        }

        // check if all eyelets have been laced
        if (!groundTruthManager.gt.success && !eyeletToBeLaced.Contains(true)) {
            // check if the two aglets are near the shoe
            Vector3 agletPosA = shoelace.aglets[0].transform.position;
            Vector3 agletPosB = shoelace.aglets[1].transform.position;
            Vector3 shoePos = shoe.shoeUpperObj.transform.position;
            // get the distance between the aglets and the shoe on the xy plane
            float distanceA = Vector3.Distance(new Vector3(agletPosA.x, agletPosA.y, 0), 
                    new Vector3(shoePos.x, shoePos.y, 0));
            float distanceB = Vector3.Distance(new Vector3(agletPosB.x, agletPosB.y, 0),
                    new Vector3(shoePos.x, shoePos.y, 0));
            // check if the distances are within the threshold
            if (distanceA>pm.success_radius && distanceB>pm.success_radius) {
                // statusText.text = "Status: Success";
                fpsDisplay.sysMessage = "Status: Success";
                Utils.LogMessage("All eyelets have been laced.");
                groundTruthManager.gt.success = true;
            }
        }

        // monitor collision between aglets and shoe upper
        if (shoeUpperCollisionManager.monitoredAglet==0) {
            // if insertion is not detected
            if (shoelace.agletCollisionManager0.monitoredEyeletIndex<0) {
                // accumulate steps of collision
                agletMonitorCounterA++;
                if (agletMonitorCounterA>pm.aglet_collision_time_thresh) {
                    // statusText.text = "Status: Failed (Aglet A penetrated shoe upper)";
                    Utils.LogMessage("Aglet A penetrated shoe upper!");
                    shoelace.agletCollisionManager0.DetachFromStepParent();
                    // Pause();
                }
            }
        }
        else {
            agletMonitorCounterA = 0;
        }
        if (shoeUpperCollisionManager.monitoredAglet==1) {
            // if insertion is not detected
            if (shoelace.agletCollisionManager1.monitoredEyeletIndex<0) {
                // accumulate steps of collision
                agletMonitorCounterB++;
                if (agletMonitorCounterB>pm.aglet_collision_time_thresh) {
                    // statusText.text = "Status: Failed (Aglet B penetrated shoe upper)";
                    Utils.LogMessage("Aglet B penetrated shoe upper!");
                    shoelace.agletCollisionManager1.DetachFromStepParent();
                    // Pause();
                }
            }
        }
        else {
            agletMonitorCounterB = 0;
        }

        // monitor tension at the rope ends
        end_a_tension = shoelace.GetEndATension();
        if (end_a_tension>pm.tension_thresh_tips) {
            // count
            endATensionCounter++;
            if (endATensionCounter>pm.tension_time_thresh) {
                // statusText.text = "Status: Failed (End A over threshold)";
                Utils.LogMessage("End A over threshold!");
                shoelace.agletCollisionManager0.DetachFromStepParent();
                // Pause();
            }
        }
        else {
            endATensionCounter = 0;
        }
        end_b_tension = shoelace.GetEndBTension();
        if (end_b_tension>pm.tension_thresh_tips) {
            // count
            endBTensionCounter++;
            if (endBTensionCounter>pm.tension_time_thresh) {
                // statusText.text = "Status: Failed (End B over threshold)";
                Utils.LogMessage("End B over threshold!");
                shoelace.agletCollisionManager1.DetachFromStepParent();
                // Pause();
            }
        }
        else {
            endBTensionCounter = 0;
        }
        rope_tension = shoelace.GetShoelaceTension();
        if (rope_tension>pm.tension_thresh) {
            // count
            shoelaceTensionCounter++;
            if (shoelaceTensionCounter>pm.tension_time_thresh) {
                // statusText.text = "Status: Failed (Rope tension over threshold)";
                fpsDisplay.sysMessage = "Status: Failed (Rope tension over threshold)";
                Utils.LogMessage("Rope tension over threshold!");
                Pause();
            }
        }
        else {
            shoelaceTensionCounter = 0;
        }

        // process debug display
        // statusText.text = "Rope tension: "+rope_tension.ToString("F2");
        fpsDisplay.debugMessage = "Rope tension: "+rope_tension.ToString("F2") + "\n" +
                "No motion for: "+noMotionTimer.ToString("F0") + "s";

        // stop the simulation when requested
        if (Input.GetKey("escape"))
        {
            Stop();
        }

        if (Input.GetKey("c"))
        {
            Continue();
        }

    }

    public async Task<UniLaceInfoServiceResponse> GetMetricInfo(UniLaceInfoServiceRequest request) {
        bool render = request.render.data;
        UniLaceInfoServiceResponse response = new UniLaceInfoServiceResponse();
        MetricInfo metricInfo = new MetricInfo();
        // stop simulation
        Pause();
        metricInfo.branch_lengths = shoelace.EstimateBranchLengths(
            shoe.eyelets[pm.num_eyelets-2].position, shoe.eyelets[pm.num_eyelets-1].position, true);
        metricInfo.eyestay_gaps = shoe.EstimateEyestayGaps();
        metricInfo.eyestay_gaps_initial = eyestayInitGaps;
        string message = "Branch lengths: " + metricInfo.branch_lengths.x.ToString("F3") + ", " + 
                metricInfo.branch_lengths.y.ToString("F3") + "\n";
        // message += "Eyestay gaps: ";
        // for (int i=0;i<pm.num_eyelets/2;i++) 
        //     message += (metricInfo.eyestay_gaps[i]/eyestayInitGaps[i]).ToString("F3") + ", ";
        Utils.LogMessage(message);
        // statusText.text = "Status: Success \n" + message;
        fpsDisplay.sysMessage = "Status: Success \n" + message;
        response.info.data = JsonUtility.ToJson(metricInfo);
        return response;
    }

//     public async Task<bool> WaitForGenerating() {
//         while (generated == false)
//             await Task.Yield();
//         return true;
//     }
}
