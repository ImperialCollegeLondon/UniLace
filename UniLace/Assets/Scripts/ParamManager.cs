using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.UniLace;
using RosPose = RosMessageTypes.Geometry.PoseMsg;
using RosPoint = RosMessageTypes.Geometry.PointMsg;


/// <summary>
/// Fetches parameters from the ROS server
/// </summary>

[System.Serializable]
public struct GroundTruth {
    public Pose aglet1;
    public Pose aglet2;
    public List<Pose> eyelets;
    public float rope_tension1;
    public float rope_tension2;
    public float rope_tension;
    public List<bool> eyelet_to_be_laced;
    public bool success;
}

[System.Serializable]
public struct MetricInfo {
    public Vector2 branch_lengths;
    public List<float> eyestay_gaps_initial;
    public List<float> eyestay_gaps;
}


[System.Serializable]
public struct RandomisedParams {
    public Vector3 shoe_position;
    public Vector3 shoelace_position;
    public float light_intensity;
}


[System.Serializable]
public class UniLaceStepRequest
{
    public float[] left_arm;
    public float left_arm_vel;
    public float left_gripper;
    public float[] right_arm;
    public float right_arm_vel;
    public float right_gripper;
    public Pose aglet_pose_a;
    public Pose aglet_pose_b; 
}

[System.Serializable]
public class RGBDImage
{
    // public byte[] rgb;
    // public byte[] depth;
    public string name;
    public Vector2Int raw_range_rgb;
    public Vector2Int raw_range_depth;
    public Vector2Int resolution;
    public Pose camera_pose;
    public float[] camera_intrinsics;
}

[System.Serializable]
public class UniLaceStepResponse
{
    public RGBDImage[] rgbd_images;
    public float[] left_arm;
    public float left_gripper;
    public float[] right_arm;
    public float right_gripper;
    public GroundTruth ground_truth;
}

[System.Serializable]
public class CameraParams
{
    public string name;
    public string type;
    public float fov;
    public float focus_distance;
    public float focus_length;
    public Vector2 sensor_size;
    public Vector2 lens_shift;
    public List<float> clipping_depth;
    public string parent;
    public Vector2 resolution;
    public Vector3 position;
    public Vector3 rotation;
    public float depth_noise_stddev;
    public int frame_rate;
}

[System.Serializable]
public class ArmParams
{
    public string arm_name;
    public string base_link;
    public List<string> joint_names;
    public List<float> initial_config;
    public float stiffness;
    public float damping;
    public float force_limit;
    public float joint_friction;
    public float angular_damping;
    public float speed;
}

[System.Serializable]
public class GripperParams
{
    public string gripper_name;
    public string base_link;
    public float stiffness;
    public float damping;
    public float force_limit;
    public float joint_friction;
    public float angular_damping;
    public float speed;
}

[System.Serializable]
public class RobotParams
{
    public string name;
    public string base_link;
    public Pose pose;
    public ArmParams left_arm;
    public ArmParams right_arm;
    public GripperParams left_gripper;
    public GripperParams right_gripper;
}

[System.Serializable]
public class SlParams
{
    public bool debug;
    public bool demo_mode;
    // shoelace param
    public float rope_length;
    public float rope_radius;
    public int rope_num_ctrl_pts;
    public int rope_pooled_particles;
    public float rope_resolution;
    public float rope_stretch_compliance;
    public float rope_stretching_scale;
    public float rope_bend_compliance;
    public float rope_max_bending;
    public float rope_particle_mass;
    public float rope_damping;
    public float rope_gravity;
    public List<float> rope_collision_material;
    public string rope_material;
    public float aglet_size;
    public float aglet_mass;
    public Vector3 aglet_initial_position_l;
    public Vector3 aglet_initial_position_r;
    public List<string> aglet_materials;
    public List<float> aglet_controller_pid;
    public float aglet_controller_tolerance;
    public float aglet_translation_speed_limit;
    public float aglet_rotation_speed_limit;
    public List<Vector3> rope_init_config;
    public List<CameraParams> cameras;

    // shoe param
    public List<float> shoe_collision_material;
    public float upper_particle_mass;
    public float upper_deformation_resistance;
    public float upper_max_deformation;
    public float upper_plastic_yield;
    public float upper_plastic_creep;
    public float upper_plastic_recovery;
    public float upper_lower_border_thresh;
    public float eyelet_mass;
    public float eyelet_outer_radius;
    public List<string> eyelet_materials;

    // robot param
    public RobotParams robot;

    // solver param
    public int solver_updater_substeps;
    public float solver_collision_margin;
    public float solver_collision_max_depenetration;
    public int solver_distance_constraint_iterations;
    public int solver_shape_matching_iterations;
    public int solver_particle_collision_constraint_iterations;
    public int solver_collision_constraint_iterations;
    public int solver_surface_collision_iterations;
    public float solver_damping;
    public float solver_gravity;

    // simulation param
    public float time_scale;
    public float time_step;
    public int horizon;
    // public bool use_gym;

    // task param
    public int num_eyelets;
    public float success_radius;
    public float tension_thresh_tips;
    public float tension_thresh;
    public float tension_time_thresh;
    public int aglet_collision_time_thresh;
    public float no_motion_time_thresh;
    public float lace_thresh_z;
    public float lace_thresh_angle;
    public Vector3 random_range_shoe_position;
    public Vector3 random_range_shoelace_position;
    public float random_range_light_intensity;
}

public class ParamManager
{
    private string paramServiceName = "/unilace_param_service";

    public bool getting_param = true;
    public SlParams p;

    // Start is called before the first frame update
    public ParamManager()
    {
        ROSConnection rosConnector = ROSConnection.GetOrCreateInstance();
        rosConnector.RegisterRosService<UniLaceParamServiceRequest, UniLaceParamServiceResponse>(paramServiceName);

        UniLaceParamServiceRequest paramServiceRequest = new UniLaceParamServiceRequest();
        rosConnector.SendServiceMessage<UniLaceParamServiceResponse>(paramServiceName, paramServiceRequest, Param_CB);
    }

    void Param_CB(UniLaceParamServiceResponse response)
    {
        p = JsonUtility.FromJson<SlParams>(response.params_json.data);
        getting_param = false;
    }

    public async Task<bool> WaitForParam() {
        while (getting_param == true)
            await Task.Yield();
        return true;
    }
}
