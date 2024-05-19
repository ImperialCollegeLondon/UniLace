using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Linq;
using Obi;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;


/// <summary>
/// The shoe is seprated into four parts: shoe upper, shoe sole and tongue, and eyelets.
/// The shoe upper is modelled as a softbody
/// The shoe sole and tongue are modelled as rigidbodies
/// The eyelets have no volume and only used for retieving the position of the eyelets
/// 
/// The Upper has three types of particles: the eyelets, the lower border, and the rest of the upper
/// the eyelets are the particles in the vincinity of the eyelets
/// the lower border are the particles in the vincinity of the lower border of the upper
/// 
/// Lower border particles are retrieved with raycasting from the particles down to the shoe sole
/// https://docs.unity3d.com/ScriptReference/Physics.Raycast.html
/// Closest point may also be used to retrived the lower border particles, 
/// it did not perform well in this setup:
/// https://docs.unity3d.com/ScriptReference/Collider.ClosestPoint.html
/// https://docs.unity3d.com/ScriptReference/MeshCollider.html
/// 
/// Shoe upper blueprint cannot be generated online due to:
/// 1. the time required is too long
/// 2. it's separated into too many frames and the generating logic will be too complex
/// 
/// Possible Improvements:
/// 1. Use Obi Bones to model the eyelets
/// https://obi.virtualmethodstudio.com/manual/6.3/bonesetup.html
/// https://docs.blender.org/manual/en/latest/animation/armatures/bones/introduction.html
/// 
/// 
/// Additional links:
/// Unity Execution Order: https://docs.unity3d.com/Manual/ExecutionOrder.html
/// Obi Distance Fields: https://obi.virtualmethodstudio.com/manual/6.3/distancefields.html
/// </summary>

public class ShoeManager : MonoBehaviour
{
    // simulation parameters
    public bool debug = false; // debug mode
    private ObiSolver solver; // obi solver
    // shoe upper parameters
    public GameObject shoeUpperObj;
    public ObiSoftbody shoeUpper;
    public float shoeUpperDeformationResistance = 1.0f;
    public float shoeUpperMaxDeformation = 0.01f;
    public float shoeUpperPlasticYield = 1.0f;
    public float shoeUpperPlasticCreep = 0.0f;
    public float shoeUpperPlasticRecovery = 0.0f;
    public float shoeUpperLowerBorderThresh = 0.02f;
    private List<List<int>> shoeParticleEyeletIds = new List<List<int>>();
    protected List<string> eyeletMat = new List<string> {"BlueEyelets", "RedEyelets"};
    private ObiCollisionMaterial shoeCollisionMaterial;

    // shoe sole and tongue parameters
    public GameObject shoeObj;
    private GameObject shoeSoleObj;
    public float shoeUpperParticleMass = 0.1f;
    private Vector3 shoePositionNoise;

    // eyelet parameters
    public List<Transform> eyelets = new List<Transform>();
    public float eyeletMass = 0.1f;
    public int numberOfEyelets = 2;
    public float eyeletParticleThresh=0.6f;

    [System.Serializable]
    private struct eyeletPose
    {
        public string name;
        public List<float> pose;
    }

    [System.Serializable]
    private struct eyeletPoses
    {
        public List<eyeletPose> poses;
    }

    // constructor
    public void Init(ObiSolver solver,
                        SlParams pm,
                        Vector3 shoePositionNoise,
                        float eyeletParticleThresh=0.6f)
    {
        this.solver = solver;
        this.eyeletMass = pm.eyelet_mass;
        this.shoePositionNoise = shoePositionNoise;
        numberOfEyelets = pm.num_eyelets;
        eyeletParticleThresh = pm.eyelet_outer_radius;
        shoeUpperParticleMass = pm.upper_particle_mass;
        shoeUpperDeformationResistance = pm.upper_deformation_resistance;
        shoeUpperMaxDeformation = pm.upper_max_deformation;
        shoeUpperPlasticYield = pm.upper_plastic_yield;
        shoeUpperPlasticCreep = pm.upper_plastic_creep;
        shoeUpperPlasticRecovery = pm.upper_plastic_recovery;
        shoeUpperLowerBorderThresh = pm.upper_lower_border_thresh;
        eyeletMat = pm.eyelet_materials;
        // create shoe obi collision material
        shoeCollisionMaterial = ScriptableObject.CreateInstance<ObiCollisionMaterial>();
        shoeCollisionMaterial.dynamicFriction = pm.shoe_collision_material[0];
        shoeCollisionMaterial.staticFriction = pm.shoe_collision_material[1];
        shoeCollisionMaterial.frictionCombine = Oni.MaterialCombineMode.Average;
        shoeCollisionMaterial.stickinessCombine = Oni.MaterialCombineMode.Minimum;
        Generate();
        // set the shoe to a randomised position
        shoeObj.transform.position = shoePositionNoise;
    }

    public void OnDestroy() {
        // remove the attachements and destroy the eyelets
        foreach (Transform eyelet in eyelets) {
            Destroy(eyelet.gameObject);
        }
        foreach (ObiParticleAttachment attachment in shoeUpperObj.GetComponents<ObiParticleAttachment>()) {
            Destroy(attachment);
        }
        // foreach (ObiParticleGroup group in shoeUpper.blueprint.groups) {
        //     Destroy(group);
        // }
    }

    private void Generate()
    {
        // generate shoe upper
        // load eyelet poses
        TextAsset eyeletsPosesAsset = (TextAsset)Resources.Load("Configs/eyelets");
        eyeletPoses eyelet_poses = JsonUtility.FromJson<eyeletPoses>(eyeletsPosesAsset.text);

        // generate shoe sole and tongue
        shoeObj = GameObject.Find("StanSmith");
        // set shoe sole collision material
        shoeSoleObj = GameObject.Find("StanSmith/StanSmithSole");
        shoeSoleObj.GetComponent<ObiCollider>().CollisionMaterial = shoeCollisionMaterial;
        // process the shoe upper
        shoeUpperObj = GameObject.Find("Obi Solver/StanSmithUpper");
        shoeUpper = shoeUpperObj.GetComponent<ObiSoftbody>();
        shoeUpper.deformationResistance = shoeUpperDeformationResistance;
        shoeUpper.maxDeformation = shoeUpperMaxDeformation;
        shoeUpper.plasticYield = shoeUpperPlasticYield;
        shoeUpper.plasticCreep = shoeUpperPlasticCreep;
        shoeUpper.plasticRecovery = shoeUpperPlasticRecovery;
        shoeUpper.collisionMaterial = shoeCollisionMaterial;
        Material[] shoe_materials = shoeUpperObj.GetComponent<SkinnedMeshRenderer>().materials;
        List<List<int>> shoeParticleEyeletIds = new List<List<int>>();
        List<ObiParticleGroup> shoe_particle_groups = new List<ObiParticleGroup>();
        Utils.LogMessage("Found "+shoeUpper.blueprint.groups.Count.ToString()+" groups");
        for (int i=0;i<eyelet_poses.poses.Count;i++) {
            // set eyelet colours
            if (i<numberOfEyelets) 
                shoe_materials[i+1] = Resources.Load<Material>(i%2==0?"Materials/"+eyeletMat[0]:"Materials/"+eyeletMat[1]);
            if (shoeUpper.blueprint.groups.Exists(g=>g.name.Contains("Eyelet"))) {
            // if (false) {
                // Eyelet particle group exists
                int i_temp = shoeUpper.blueprint.groups.FindIndex(g=>g.name.Contains("Eyelet_"+i.ToString()));
                List<int> group_particle_ids = new List<int>();
                for (int j=0; j<shoeUpper.blueprint.groups[i_temp].particleIndices.Count; j++) {
                    group_particle_ids.Add(shoeUpper.blueprint.groups[i_temp].particleIndices[j]);
                }
                shoeParticleEyeletIds.Add(group_particle_ids);
                shoe_particle_groups.Add(shoeUpper.blueprint.groups[i_temp]);
            }
            else {
                Vector3 location = new Vector3(-eyelet_poses.poses[i].pose[1], eyelet_poses.poses[i].pose[0], eyelet_poses.poses[i].pose[2]);
                location = shoeObj.transform.TransformPoint(FLU.ConvertToRUF(location));
                // if (i==0)
                // Utils.LogMessage(location.ToString());
                // Add new particle group
                List<int> group_particle_ids = new List<int>();
                var group = ScriptableObject.CreateInstance<ObiParticleGroup>();
                group.name = "Eyelet_"+i.ToString();
                // find nearby particle indices
                for (int j=0; j<shoeUpper.solverIndices.Length; ++j){
                    // retrieve the particle index in the solver:
                    int solverIndex = shoeUpper.solverIndices[j];
                    if (Vector3.Distance(location, solver.positions[solverIndex])<eyeletParticleThresh) {
                        group_particle_ids.Add(solverIndex);
                        group.particleIndices.Add(solverIndex);
                        // change the mass of the particles
                        // solver.invMasses[solverIndex] = 1.0f/1; // 1.0f/shoeUpperParticleMass;
                    }
                }


            // // change the mass of the particles
            // for (int j=0; j<group_particle_ids.Count; j++) {
            //     solver.invMasses[group_particle_ids[j]] = 1.0f/shoeUpperParticleMass;
            // }

                shoeParticleEyeletIds.Add(group_particle_ids);
                shoe_particle_groups.Add(group);

            }
        }
        shoeUpper.GetComponent<SkinnedMeshRenderer>().materials = shoe_materials;

        // // add markers to the particles for visualisation
        // for (int j=0; j<shoeParticleEyeletIds[0].Count; j++) {
        //     // add a marker to the particle for visualisation
        //     GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        //     marker.transform.position = solver.positions[shoeParticleEyeletIds[0][j]];
        //     marker.transform.localScale = new Vector3(0.01f, 0.01f, 0.01f);
        //     marker.GetComponent<MeshRenderer>().material = Resources.Load<Material>("Materials/Marker");
        // }

        // change the mass of the particles
        for (int i=0; i<shoeUpper.solverIndices.Length; i++) {
            solver.invMasses[shoeUpper.solverIndices[i]] = 1.0f/shoeUpperParticleMass;
        }

        // generate eyelets
        for (int i=0;i<eyelet_poses.poses.Count;i++) {
            GameObject eyelet_go = GameObject.Instantiate(Resources.Load("Models/Eyelet", typeof(GameObject))) as GameObject;
            // GameObject eyelet_go = createCapsuleEyelet("Eyelet "+i.ToString(), 
            //                     Vector3.zero,
            //                     Quaternion.identity,
            //                     new Vector3(0.6f, 0.3f, 0.6f),
            //                     shoeObj.transform
            //                     );
            eyelet_go.transform.parent = shoeObj.transform;
            Vector3 location = new Vector3(-eyelet_poses.poses[i].pose[1], eyelet_poses.poses[i].pose[0], eyelet_poses.poses[i].pose[2]);
            eyelet_go.transform.localPosition = FLU.ConvertToRUF(location);
                // if (i==0)
                // Utils.LogMessage(eyelet_go.transform.position.ToString());
            // eyelet_go.transform.localRotation = Quaternion.Euler(Mathf.Pow(-1, i+1)*90, 90, 0);
            eyelet_go.transform.localRotation = Quaternion.Euler(eyelet_poses.poses[i].pose[7]*Mathf.Rad2Deg, -eyelet_poses.poses[i].pose[9]*Mathf.Rad2Deg, eyelet_poses.poses[i].pose[8]*Mathf.Rad2Deg);
            eyelet_go.transform.localScale = new Vector3(6f,6f,6f);
            eyelet_go.name = "Eyelet_"+i.ToString();
            eyelet_go.tag = "eyelet";
            // eyelet_go.layer = LayerMask.NameToLayer("nobody"); // does not collide with anything, only used for retrieving the pose
            eyelet_go.GetComponent<MeshRenderer>().enabled = false; // disable rendering to save computation
            // get the only child of eyelet_go
            var colliderObj = eyelet_go.transform.GetChild(0).gameObject;
            // colliderObj.SetActive(false);
            colliderObj.name = eyelet_go.name;
            // colliderObj.layer = LayerMask.NameToLayer("eyelet");
            // change eyelet params
            Rigidbody rb = eyelet_go.GetComponent<Rigidbody>();
            rb.mass = eyeletMass;
            eyelets.Add(eyelet_go.transform);

            // add obi particle attachment
            ObiParticleAttachment attachment = shoeUpperObj.AddComponent<ObiParticleAttachment>();
            attachment.target = eyelet_go.transform;
            attachment.particleGroup = shoe_particle_groups[i];
            attachment.attachmentType = ObiParticleAttachment.AttachmentType.Dynamic; // so that the eyelets move backs
        }

        // create a new particle group for the lower border of the upper
        List<int> lower_particle_ids = new List<int>();
        var lower_group = ScriptableObject.CreateInstance<ObiParticleGroup>();
        lower_group.name = "LowerBorder";
        for (int j=0; j<shoeUpper.solverIndices.Length; ++j){
            // retrieve the particle index in the solver:
            int solverIndex = shoeUpper.solverIndices[j];
            if (BordersMesh(solver.positions[solverIndex], shoeSoleObj.GetComponent<MeshCollider>(), shoeUpperLowerBorderThresh)) {
                lower_particle_ids.Add(solverIndex);
                lower_group.particleIndices.Add(solverIndex);

                // // add a marker to the particle for visualisation
                // GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                // marker.transform.position = solver.positions[solverIndex];
                // marker.transform.localScale = new Vector3(0.01f, 0.01f, 0.01f);
                // marker.GetComponent<MeshRenderer>().material = Resources.Load<Material>("Materials/Marker");

            }
        }
        // create attachment for the lower border with the shoe sole
        ObiParticleAttachment lower_attachment = shoeUpperObj.AddComponent<ObiParticleAttachment>();
        lower_attachment.target = shoeObj.transform;
        lower_attachment.particleGroup = lower_group;
        // lower_attachment.constrainOrientation = true;
    }

    public List<float> EstimateEyestayGaps() {
        List<float> gaps = new List<float>();
        for (int i=0; i<eyelets.Count/2; i++) {
            gaps.Add(Vector3.Distance(eyelets[i*2].position, eyelets[i*2+1].position));
        }
        return gaps;
    }

    private GameObject createCapsuleEyelet(string name, Vector3 position, Quaternion rotation, Vector3 scale, Transform parent) {
        // generate capsule eyelet
        GameObject eyelet = new GameObject("Eyelet"+name);
        int numCapsules=36;
        for (int i=0; i<numCapsules; i++) {
            GameObject capsule = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            capsule.name = "Capsule"+i.ToString();
            capsule.transform.position = new Vector3(Mathf.Cos(360/numCapsules*i)*0.1f, 0, Mathf.Sin(360/numCapsules*i)*0.1f);
            capsule.transform.rotation = Quaternion.Euler(0, 0, 0); 
            capsule.transform.localScale = new Vector3(0.05f,0.05f,0.05f);
            capsule.transform.parent = eyelet.transform;
            capsule.GetComponent<MeshRenderer>().enabled = false;
            ObiCollider collider = capsule.AddComponent<ObiCollider>();
        }
        eyelet.transform.position = position;
        eyelet.transform.rotation = rotation;
        eyelet.transform.localScale = scale;
        eyelet.transform.parent = parent;
        return eyelet;
    }

    private bool BordersMesh(Vector3 point, MeshCollider coll, float thresh=0.01f) {
        // create a ray from the point to the mesh
        Ray ray = new Ray(point, Vector3.down);
        RaycastHit hit;
        if (coll.Raycast(ray, out hit, thresh)) {
            return true;
        }
        return false;

        // // check if a point is on the border of or inside a mesh
        // Vector3 closestPoint = coll.ClosestPoint(point);
        // if (Vector3.Distance(point, closestPoint)<thresh) {
        //         // add a marker to the particle for visualisation
        //         GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        //         marker.transform.position = closestPoint;
        //         marker.transform.localScale = new Vector3(0.01f, 0.01f, 0.01f);
        //         marker.GetComponent<MeshRenderer>().material = Resources.Load<Material>("Materials/Marker");
        //     return true;
        // }
        // return false;
    }

    public List<Pose> GetEyeletPoses() {
        List<Pose> eyeletPoses = new List<Pose>();
        for (int i=0; i<eyelets.Count; i++) {
            Pose pose = new Pose();
            pose.position = eyelets[i].position;
            pose.rotation = eyelets[i].rotation;
            eyeletPoses.Add(pose);
        }
        return eyeletPoses;
    }

    public List<Pose> GetParticleEyeletPoses() {
        List<Pose> eyeletPoses = new List<Pose>();
        foreach (List<int> group in shoeParticleEyeletIds) {
            Pose pose = new Pose();
            List<Vector3> particlePositions = new List<Vector3>();
            foreach (int id in group) {
                particlePositions.Add((Vector3)solver.positions[id]);
            }
            pose.position = new Vector3(
                particlePositions.Average(x=>x.x),
                particlePositions.Average(x=>x.y),
                particlePositions.Average(x=>x.z));
            eyeletPoses.Add(pose);
        }
        return eyeletPoses;
    }

}