using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Linq;
using Obi;
using UnityEngine;
using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;
using Unity.Mathematics;


/// <summary>
/// Generates an ObiRope object based on the given parameters.
/// </summary>
/// <param name="points">The list of control points for the rope.</param>
/// <param name="material">The material to be used for rendering the rope.</param>
/// <param name="colliderFilterRope">The collision filter for the rope.</param>
/// <param name="colliderFilterRopeEnd">The collision filter for the rope's end points.</param>
/// <param name="rope_radius">The radius of the rope.</param>
/// <param name="resolution">The resolution of the rope.</param>
/// <param name="pooled_particles">The number of pooled particles for the rope.</param>
/// <param name="name">The name of the rope.</param>
/// <param name="stretch_compliance">The stretch compliance of the rope.</param>
/// <param name="stretching_scale">The stretching scale of the rope.</param>
/// <param name="bend_compliance">The bend compliance of the rope.</param>
/// <param name="max_bending">The maximum bending of the rope.</param>
/// <param name="mass">The mass of the rope.</param>
/// <param name="damping">The damping of the rope.</param>
/// <param name="self_collision">Whether self-collision is enabled for the rope.</param>
/// <returns>The generated ObiRope object.</returns>


public class ShoelaceManager : MonoBehaviour
{
    // simulation parameters
    public bool debug = false; // debug mode
    private ObiSolver solver; // obi solver
    // rope parameters
    public ObiRope rope; // obi rope
    private List<Vector3> ropeInitialStates = new List<Vector3>(); // initial states of the rope
    // public List<Vector3> ropeInitialStates = new List<Vector3>(); // initial states of the rope
    public int colliderFilterRopeEnd;
    private int colliderFilterRope;
    public Material ropeMaterial;
    public ObiCollisionMaterial ropeColliderMaterial;

    public float ropeGravity=-9.8f;
    public float ropeRadius=0.015f;
    public float ropeResolution=0.5f;
    public int ropePooledParticles=0;
    public float ropeStretchCompliance=0.0f;
    public float ropeStretchingScale=0.99f;
    public float ropeBendCompliance=0.0f;
    public float ropeMaxBending=0.015f;
    public float ropeMass=0.1f;
    private List<int> ropeEndParticleIds;
    private int endLength = 5;
    public float end_a_rest_length;
    public float end_b_rest_length;
    public float rope_rest_length;
    public float end_a_tension;
    public float end_b_tension;
    public float rope_tension;

    // aglet parameters
    public GameObject agletObject;
    public List<GameObject> aglets = new List<GameObject>();
    private int colliderFilterAglet;
    public bool agletUseGravity = false;
    public bool agletIsKinematic = false;
    public float agletWidth;
    public float agletSize = 1f;
    public float agletMass = 0.05f; // 0.5f
    public float agletTranslationSpeed = 1f;
    public float agletRotationSpeed = 90f;
    public List<string> agletMat = new List<string>(new string[] {"RedAglet", "BlueAglet"});

    public AgletCollisionManager agletCollisionManager0;
    public AgletCollisionManager agletCollisionManager1;
    public List<PID3D> agletControllers = new List<PID3D>();

    // constructor
    public void Init(ObiSolver solver,
                            List<Vector3> ropeInitialStates,
                            SlParams pm,
                            bool debug=false) {
        this.solver = solver;
        // rope parameters
        // this.ropeInitialStates = pm.rope_init_config;
        this.ropeInitialStates = ropeInitialStates;
        this.ropeRadius = pm.rope_radius;
        this.ropeResolution = pm.rope_resolution;
        this.ropePooledParticles = pm.rope_pooled_particles;
        this.ropeGravity = -pm.rope_gravity;
        this.ropeStretchCompliance = pm.rope_stretch_compliance;
        this.ropeStretchingScale = pm.rope_stretching_scale;
        this.ropeBendCompliance = pm.rope_bend_compliance;
        this.ropeMaxBending = pm.rope_max_bending;
        this.ropeMass = pm.rope_particle_mass;
        this.agletMass = pm.aglet_mass;
        this.agletSize = pm.aglet_size;
        this.agletTranslationSpeed = pm.aglet_translation_speed_limit;
        this.agletRotationSpeed = pm.aglet_rotation_speed_limit;
        agletMat = pm.aglet_materials;
        for (int i=0; i<2; i++) {
            agletControllers.Add(new PID3D(pm.aglet_controller_pid[0],
                                            pm.aglet_controller_pid[1],
                                            pm.aglet_controller_pid[2],
                                            pm.aglet_controller_tolerance));
        }

        this.ropeMaterial = Resources.Load<Material>("Materials/"+pm.rope_material);

        // Create new collision material
        // ropeColliderMaterial = Resources.Load<ObiCollisionMaterial>("Materials/IceObiMaterial");
        ropeColliderMaterial = ScriptableObject.CreateInstance<ObiCollisionMaterial>();
        ropeColliderMaterial.dynamicFriction = pm.rope_collision_material[0];
        ropeColliderMaterial.staticFriction = pm.rope_collision_material[1];
        ropeColliderMaterial.frictionCombine = Oni.MaterialCombineMode.Average;
        ropeColliderMaterial.stickinessCombine = Oni.MaterialCombineMode.Minimum;

        // set up the collision filters
        colliderFilterRopeEnd = ObiUtils.MakeFilter((1 << 0) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7), 6);
        colliderFilterRope = ObiUtils.MakeFilter(ObiUtils.CollideWithEverything ^ (1 << 2), 0);
        colliderFilterAglet = ObiUtils.MakeFilter((1 << 0) | (1 << 1) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 7), 1);

        Utils.LogMessage("Generating Obi Rope.");
        
        Generate();
        this.solver.OnBeginStep += Solver_OnBeginStep;

        Utils.LogMessage("Generated "+rope.restLength/10+"m long Obi Rope.");
    }

    public void OnDestroy() {
        // remove the objects and components created in generate function
        if (rope != null) {
            solver.OnBeginStep -= Solver_OnBeginStep;
            Destroy(rope.gameObject);
        }
        if (agletObject != null) {
            Destroy(agletObject);
        }

    }


    private async Task Generate() {
        // CREATING A BLUPRINT
        // create the blueprint: (ltObiRopeBlueprint, ObiRodBlueprint)
        ObiRopeBlueprint ropeBlueprint = ScriptableObject.CreateInstance<ObiRopeBlueprint>();
        ropeBlueprint.thickness = ropeRadius; // radius of the particles (metres)
        ropeBlueprint.resolution = ropeResolution;
        ropeBlueprint.pooledParticles = ropePooledParticles;

        // AddControlPoint: position, inTangentVector, outTangentVector, normal, mass, rotationalMass, thickness, filter, color, name
        // obi rod use 'normal' to define the orientation at each control point. not used in ropes
        // first point does not need in anchor
        // last point does not need out anchor
        Vector3 temp = (ropeInitialStates[1]-ropeInitialStates[0])/2;
        ropeBlueprint.path.Clear();

        for (int i=0; i<ropeInitialStates.Count; i++) {
            Vector3 inTan = i==0 ? temp : (ropeInitialStates[i-1]-ropeInitialStates[i])/2;
            Vector3 outTan = i==ropeInitialStates.Count-1 ? temp : (ropeInitialStates[i+1]-ropeInitialStates[i])/2;
            ropeBlueprint.path.AddControlPoint(ropeInitialStates[i], inTan, outTan, Vector3.one, ropeMass, ropeMass, 1, colliderFilterRope, Color.white, i.ToString());
        }
        // ropeBlueprint.path.AddControlPoint(ropeInitialStates[0], Vector3.one, temp, Vector3.one, ropeMass, ropeMass, 1, colliderFilterRopeEnd, Color.white, "0");
        // if (ropeInitialStates.Count >= 3)
        //     for (int i=1; i<ropeInitialStates.Count-1; i++) {
        //         Vector3 inTan = (ropeInitialStates[i-1]-ropeInitialStates[i])/2;
        //         Vector3 outTan = (ropeInitialStates[i+1]-ropeInitialStates[i])/2;            
        //         ropeBlueprint.path.AddControlPoint(ropeInitialStates[i], inTan, outTan, Vector3.one, ropeMass, ropeMass, 1, colliderFilterRope, Color.white, i.ToString());
        //     }
        // temp = (ropeInitialStates[^2]-ropeInitialStates[^1])/2;
        // ropeBlueprint.path.AddControlPoint(ropeInitialStates[^1], temp, Vector3.one, Vector3.one, ropeMass, ropeMass, 1, colliderFilterRopeEnd, Color.white, (ropeInitialStates.Count-1).ToString());

        ropeBlueprint.path.FlushEvents();

        // generate the particle representation of the rope (wait until it has finished):
        ropeBlueprint.Generate();

        // CREATING AN ACTOR
        GameObject ropeObject = new GameObject("Obi Rope", typeof(ObiRope), typeof(ObiRopeExtrudedRenderer)); // create a rope:
        
        rope = ropeObject.GetComponent<ObiRope>(); // get component references:
        rope.selfCollisions = true;
        rope.surfaceCollisions = true;
        rope.stretchingScale = ropeStretchingScale;
        rope.stretchCompliance = ropeStretchCompliance;
        rope.bendCompliance = ropeBendCompliance;
        rope.maxBending = ropeMaxBending;

        ObiPathSmoother path_smoother = ropeObject.GetComponent<ObiPathSmoother>();
        // path_smoother.decimation = 0.15f;
        path_smoother.smoothing = 2;

        ObiRopeExtrudedRenderer ropeRenderer = ropeObject.GetComponent<ObiRopeExtrudedRenderer>(); // ObiRopeLineRenderer   
        ropeRenderer.section = Resources.Load<ObiRopeSection>("DefaultRopeSection"); // load the default rope section:
        ropeRenderer.thicknessScale = 1.0f;
        ropeRenderer.uvScale = new Vector2(1,2);

        rope.ropeBlueprint = ScriptableObject.Instantiate(ropeBlueprint); // instantiate and set the blueprint:
        rope.transform.parent = solver.transform; // parent the cloth under a solver to start simulation:

        // CHANGE ROPE MATERIAL
        MeshRenderer meshRenderer = ropeObject.GetComponent<MeshRenderer>();
        meshRenderer.material = ropeMaterial;

        rope.collisionMaterial = ropeColliderMaterial;

        ropeObject.transform.localScale = new Vector3(1f,1f,1f);
        // ropeObject.GetComponent<Renderer>().material = Resources.Load<Material>("Materials/Green");

        #if UNITY_EDITOR
        if (debug) {
            var paticle_renderer = ropeObject.AddComponent<ObiParticleRenderer>();
            paticle_renderer.shader = Resources.Load<Shader>("ObiMaterials/ParticleShader");
        }
        #endif
        
        // store the end particle ids
        ropeEndParticleIds = new List<int>(new int[] { 
            rope.elements[0].particle1, rope.elements[0].particle2, rope.elements[1].particle1, rope.elements[1].particle2,
            rope.elements[^1].particle1, rope.elements[^1].particle2, rope.elements[^2].particle1, rope.elements[^2].particle2});

        // generate aglets
        agletObject = new GameObject("Aglets");
        // agletObject.AddComponent<ArticulationBody>();
        for (int i=0; i<2; i++) {
            GameObject aglet = GameObject.Instantiate(Resources.Load("Models/aglet", typeof(GameObject))) as GameObject;
            aglet.name = "Aglet "+(char)((char)'A'+i);
            int paritcle_id = i==0 ? rope.elements[0].particle1 : rope.elements[^1].particle2;
            aglet.transform.position = solver.transform.InverseTransformPoint(solver.positions[paritcle_id]);
            aglet.transform.rotation = Quaternion.Euler(0, 0, 0);
            aglet.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f)*agletSize;
            aglet.transform.parent = agletObject.transform;
            aglet.GetComponent<Renderer>().material = 
                    Resources.Load<Material>(i==0 ? "Materials/"+agletMat[0] : "Materials/"+agletMat[1]);
            // aglet.AddComponent<AgletCollisionManager>();
            ObiCollider collider = aglet.AddComponent<ObiCollider>();
            collider.CollisionMaterial = Resources.Load<ObiCollisionMaterial>("Materials/IceObiMaterial");
            collider.Filter = colliderFilterAglet;
            
            ObiParticleAttachment attachment = rope.gameObject.AddComponent<ObiParticleAttachment>();
            attachment.target = aglet.transform;
            attachment.particleGroup = i==0 ? rope.blueprint.groups[0] : rope.blueprint.groups[^1];

            Rigidbody aglet_body = aglet.AddComponent<Rigidbody>();
            aglet_body.isKinematic = agletIsKinematic;
            aglet_body.useGravity = agletUseGravity;
            aglet_body.mass = agletMass;
            aglet_body.drag = 5f; // slows down linear motion 0.5f
            aglet_body.angularDrag = 5f; // slows down rotational motion 1.5f

            // ArticulationBody aglet_body = aglet.AddComponent<ArticulationBody>();
            // aglet_body.jointType = ArticulationJointType.FixedJoint;

            aglets.Add(aglet);
        }
        agletWidth = aglets[0].GetComponent<BoxCollider>().size.x*aglets[0].transform.localScale.x;
        
        // // Store initial positions TODO wrong shoelace indices
        // foreach (Vector3 position in solver.positions)
        //     ropeInitialStates.Add(position);
        
        agletCollisionManager0 = 
                aglets[0].AddComponent<AgletCollisionManager>();
          
        agletCollisionManager1 =
                aglets[1].AddComponent<AgletCollisionManager>();   
    }

    public Vector2 EstimateBranchLengths(Vector3 eyelet_a, Vector3 eyelet_b, bool display=true) {
        // get positions of all particles
        List<Vector3> particle_positions = new List<Vector3>();
        for (int i=0; i<rope.elements.Count; i++) {
            particle_positions.Add(solver.positions[rope.elements[i].particle1]);
        }
        // get the index next to the final eyelets
        int index_a;
        int index_b;
        // rank the particles by distance to the eyelets
        IEnumerable<int> ids = Enumerable.Range(0, particle_positions.Count);
        ids = ids.Zip(particle_positions, (f, s) => (ids: f, particle_positions: s))
                .OrderBy(x => Vector3.Distance(x.particle_positions, eyelet_a))
                .Select(x => x.ids);
        int index_1 = ids.First();
        ids = Enumerable.Range(0, particle_positions.Count);
        ids = ids.Zip(particle_positions, (f, s) => (ids: f, particle_positions: s))
                .OrderBy(x => Vector3.Distance(x.particle_positions, eyelet_b))
                .Select(x => x.ids);
        int index_2 = ids.First();
        index_a = index_1<index_2 ? index_1 : index_2;
        index_b = index_1>index_2 ? index_1 : index_2;
        // sum the distance between the particles
        float length_a = 0;
        float length_b = 0;
        GameObject markers_a = new GameObject("Markers_A");
        GameObject markers_b = new GameObject("Markers_B");
        for (int i=0; i<index_a-1; i++) {
            length_a += Vector3.Distance(solver.positions[rope.elements[i].particle1], 
                                    solver.positions[rope.elements[i+1].particle1]);
            GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            marker.name = "Particle"+i.ToString();
            marker.transform.position = solver.positions[rope.elements[i].particle1];
            marker.transform.localScale = Vector3.one*0.05f;
            marker.transform.parent = markers_a.transform;
            marker.GetComponent<SphereCollider>().enabled = false;
            marker.GetComponent<MeshRenderer>().material = Resources.Load<Material>("Materials/Marker");
        }
        for (int i=index_b; i<rope.elements.Count-1; i++) {
            length_b += Vector3.Distance(solver.positions[rope.elements[i].particle1], solver.positions[rope.elements[i+1].particle1]);
            GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            marker.name = "Particle"+i.ToString();
            marker.transform.position = solver.positions[rope.elements[i].particle1];
            marker.transform.localScale = Vector3.one*0.05f;
            marker.transform.parent = markers_b.transform;
            marker.GetComponent<SphereCollider>().enabled = false;
            marker.GetComponent<MeshRenderer>().material = Resources.Load<Material>("Materials/Marker2");
        }
        length_b += Vector3.Distance(solver.positions[rope.elements[^1].particle1], solver.positions[rope.elements[^1].particle2]);
        return new Vector2(length_a, length_b);
    }

    public void UpdateRopeEndCollisionFilters() {
        // set the collision filters for the rope end particles
        solver.filters[rope.solverIndices[0]] = colliderFilterRopeEnd;
        solver.filters[rope.solverIndices[1]] = colliderFilterRopeEnd;
        solver.filters[rope.solverIndices[^1]] = colliderFilterRopeEnd;
        solver.filters[rope.solverIndices[^2]] = colliderFilterRopeEnd;
    }

    public float GetShoelaceTension() {
        return rope.CalculateLength()/rope_rest_length; 
    }

    public void SetShoelaceRestLength() {
        rope_rest_length = rope.restLength;
        end_a_rest_length = 0;
        end_b_rest_length = 0;
        for (int i=0; i<endLength; i++) {
            end_a_rest_length += rope.elements[i].restLength;
            end_b_rest_length += rope.elements[^(i+1)].restLength;
        }
    }

    public float GetEndATension() {
        float end_a_length = 0;
        for (int i=0; i<endLength; i++) {
            end_a_length += Vector3.Distance(solver.positions[rope.elements[i].particle1], 
                                            solver.positions[rope.elements[i].particle2]);
        }
        end_a_tension = end_a_length/end_a_rest_length;
        
        // float end_a_length1 = Vector3.Distance(solver.transform.InverseTransformPoint(solver.positions[ropeEndParticleIds[0]]), 
        //                                         solver.transform.InverseTransformPoint(solver.positions[ropeEndParticleIds[1]]));
        // float end_a_length2 = Vector3.Distance(solver.transform.InverseTransformPoint(solver.positions[ropeEndParticleIds[2]]), 
        //                                         solver.transform.InverseTransformPoint(solver.positions[ropeEndParticleIds[3]]));
        // end_a_tension = (end_a_length1+end_a_length2)/(rope.elements[0].restLength+rope.elements[1].restLength);

        return end_a_tension;
    }

    public float GetEndBTension() {
        float end_b_length = 0;
        for (int i=0; i<endLength; i++) {
            end_b_length += Vector3.Distance(solver.positions[rope.elements[^(i+1)].particle1], 
                                            solver.positions[rope.elements[^(i+1)].particle2]);
        }
        end_b_tension = end_b_length/end_b_rest_length;

        // float end_b_length1 = Vector3.Distance(solver.transform.InverseTransformPoint(solver.positions[ropeEndParticleIds[4]]), 
        //                                         solver.transform.InverseTransformPoint(solver.positions[ropeEndParticleIds[5]]));
        // float end_b_length2 = Vector3.Distance(solver.transform.InverseTransformPoint(solver.positions[ropeEndParticleIds[6]]), 
        //                                         solver.transform.InverseTransformPoint(solver.positions[ropeEndParticleIds[7]]));
        // end_b_tension = (end_b_length1+end_b_length2)/(rope.elements[^1].restLength+rope.elements[^2].restLength);

        return end_b_tension;
    }

    // public void MoveAgletTowardsPose(int agletId, Vector3 position, Quaternion rotation, float time) {
    //     aglets[agletId].transform.position = Vector3.MoveTowards(aglets[agletId].transform.position, position, agletTranslationSpeed*time);
    //     aglets[agletId].transform.rotation = Quaternion.RotateTowards(aglets[agletId].transform.rotation, rotation, agletRotationSpeed*time);
    // }

    public bool MoveAgletTowardsPose(int agletId, Vector3 position, Quaternion rotation, float time) {
        agletControllers[agletId].SetTarget(position);
        Vector3 force = agletControllers[agletId].Update(aglets[agletId].transform.position, time);
        force = Vector3.ClampMagnitude(force, agletTranslationSpeed);
        aglets[agletId].GetComponent<Rigidbody>().AddForce(force, ForceMode.VelocityChange);
        aglets[agletId].transform.rotation = Quaternion.RotateTowards(aglets[agletId].transform.rotation, rotation, agletRotationSpeed*time);
        bool targetReached = agletControllers[agletId].targetReached;
        if (targetReached) {
            agletControllers[agletId].Reset();
        }
        return targetReached;
    }

    // public async Task MoveAgletToPose(int agletId, Vector3 position, Quaternion rotation) {
    //     agletControllers[agletId].SetTarget(position);
    //     while (!agletControllers[agletId].targetReached || aglets[agletId].transform.rotation != rotation) {
    //         Vector3 force = agletControllers[agletId].Update(aglets[agletId].transform.position, Time.fixedDeltaTime);
    //         aglets[agletId].GetComponent<Rigidbody>().AddForce(force, ForceMode.VelocityChange);
    //         aglets[agletId].transform.rotation = Quaternion.RotateTowards(aglets[agletId].transform.rotation, rotation, agletRotationSpeed*Time.fixedDeltaTime);
    //         await Task.Yield();
    //     }
    //     agletControllers[agletId].Reset();
    // }

    private void Solver_OnBeginStep(ObiSolver solver, float stepTime)
    {
        var indices = new NativeArray<int>(rope.solverIndices, Allocator.TempJob);

        var job = new CustomGravityJob
        {
            indices = indices,
            velocities = solver.velocities.AsNativeArray<float4>(),
            gravityStrength = ropeGravity,
            deltaTime = stepTime,
        };

        job.Schedule(indices.Length, 32).Complete();
    }

    [BurstCompile]
    struct CustomGravityJob : IJobParallelFor
    {
        [ReadOnly][DeallocateOnJobCompletion] public NativeArray<int> indices;
        [NativeDisableParallelForRestriction] public NativeArray<float4> velocities;
        [ReadOnly] public float gravityStrength;
        [ReadOnly] public float deltaTime;

        public void Execute(int i)
        {
            var index = indices[i];
            var vel = velocities[index];
            vel.xyz += new float3(0,gravityStrength * deltaTime,0);
            velocities[index] = vel;
        }
    }
}