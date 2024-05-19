using System.Collections.Generic;
using System.Collections;
using System.Linq;
using UnityEngine;
using Unity.Robotics.UrdfImporter;

/// <summary>
/// Parenthold transition order: Step parent, Previous step parent, Biological parent
/// Assumptions:
/// 1. Aglet does not slip out of the gripper (aglet set to kinematic when attached to gripper),
///     shoelace tension is monitored so that this holds.
/// 2. Fingers effectively regulates the aglet orientation.
/// 
/// Additional notes:
/// 1. why rigidbody + rixedjoint instead of articulation body?
///    - articulation body should not be modified at runtime 
///    (https://forum.unity.com/threads/how-to-work-with-configurable-articulation-chains.1184662/)
/// 2. Understanding Fixed Joint:
///   - https://docs.unity3d.com/Manual/class-FixedJoint.html
///   
/// </summary>

public class AgletCollisionManager : MonoBehaviour
{
    public GameObject step_parent=null;
    public GameObject previous_step_parent=null;
    private GameObject bio_parent=null;
    private bool in_collision=false;
    [HideInInspector] public GripperController gripper_left_controller;
    [HideInInspector] public GripperController gripper_right_controller;
    [HideInInspector] public GameObject gripper_left;
    [HideInInspector] public GameObject gripper_right;
    private bool in_contact_gripper_left=false;
    private bool in_contact_gripper_right=false;
    private Dictionary<GameObject, GripperController> gripper_controllers = new Dictionary<GameObject, GripperController>();
    private Dictionary<GameObject, bool> gripper_contact = new Dictionary<GameObject, bool>();
    private bool lock_update = false;
    private FixedJoint fixedJoint;
    public GameObject monitoredEyelet=null;
    public int monitoredEyeletIndex=-1;
    
    void Start() {
        bio_parent = gameObject.transform.parent.gameObject;
        gripper_controllers.Add(gripper_left, gripper_left_controller);
        gripper_controllers.Add(gripper_right, gripper_right_controller);
        gripper_contact.Add(gripper_left, in_contact_gripper_left);
        gripper_contact.Add(gripper_right, in_contact_gripper_right);
    }

    void OnCollisionEnter(Collision collision)
    {
        Utils.LogMessage(gameObject.name+" collided with "+collision.gameObject.name);
        if (collision.gameObject.name.Contains("gripper")) {
            GameObject gripper = collision.gameObject.name.Contains("gripper_r")?gripper_right:gripper_left;
            // get aglet position in gripper space
            Vector3 pos_temp = gripper.transform.InverseTransformPoint(gameObject.transform.position);
            // check if aglet-gripper distance in x-z plane is within the gripper
            float distance = Mathf.Sqrt(pos_temp.x*pos_temp.x+pos_temp.z*pos_temp.z);
            if (distance>0.165f) {
                Utils.LogMessage("Small contract region. Aglet cannot be grasped.");
                return;
            } 
            gripper_contact[gripper] = true;
            // check if closing
            if (gripper_controllers[gripper].gripper_state==GripperController.gripperStates.Closing) {
                // attach to gripper
                Attach(gripper);
            }
        }
    }

    private void OnTriggerEnter(Collider collider)
    {
        Utils.LogMessage("Start monitoring eyelet "+collider.gameObject.name);
        // check if contact object is an eyelet
        if (collider.gameObject.name.Contains("Eyelet")) {
            // start monitoring aglet eyelet distance
            monitoredEyelet = collider.gameObject;
            monitoredEyeletIndex = int.Parse(monitoredEyelet.name.Split('_')[1]);
        }
    }


    void Update() {
        // if (lock_update) return;
        if (step_parent != null && (gripper_controllers[step_parent].gripper_state==GripperController.gripperStates.Opening ||
            gripper_controllers[step_parent].gripper_state==GripperController.gripperStates.Open)) {
            // detach from gripper
            gripper_contact[step_parent] = false;
            Detach(step_parent);
        }
    }

    void Attach(GameObject new_parent) {
        Utils.LogMessage("Attaching to "+new_parent.name);
        if (gameObject.transform.parent.gameObject==bio_parent) {
            previous_step_parent = null;
        }
        else {
            previous_step_parent = step_parent;
        }

        if (new_parent==bio_parent) {
            step_parent = null;
            gameObject.transform.parent = bio_parent.transform;
            gameObject.layer = bio_parent.layer;
            Destroy(gameObject.GetComponent<FixedJoint>());
            gameObject.GetComponent<Rigidbody>().useGravity = true;
        }
        else {
            if (gameObject.GetComponent<FixedJoint>()==null)
                gameObject.AddComponent<FixedJoint>();
            step_parent = new_parent;
            gripper_controllers[step_parent].gripper_state = GripperController.gripperStates.Holding;
            // stop gripper closing
            gameObject.layer = step_parent.layer; // avoid unnecessary collision detection
            gripper_controllers[step_parent].Hold(); // gripper stops closing
            gameObject.transform.parent = step_parent.transform;
            // project attached object to the gripper space
            Vector3 pos_temp = new_parent.transform.InverseTransformPoint(gameObject.transform.position);
            Vector3 rot_temp = (Quaternion.Inverse(new_parent.transform.rotation) * gameObject.transform.rotation).eulerAngles;
            // remove the y rotation and translation
            // if (rot_temp.y>180) rot_temp.y -= 360;
            // rot_temp.y = rot_temp.y>0?90f:-90f;
            // rot_temp.y = (float)((int)rot_temp.y/90f*90f);
            pos_temp.z = 0;
            // apply the rotation and translation
            gameObject.transform.position = new_parent.transform.TransformPoint(pos_temp);
            gameObject.transform.rotation = new_parent.transform.rotation * Quaternion.Euler(rot_temp);
            gameObject.GetComponent<FixedJoint>().connectedArticulationBody = step_parent.GetComponent<ArticulationBody>();
            gameObject.GetComponent<Rigidbody>().useGravity = false;
        }
        Utils.LogMessage("Attached to "+new_parent.name);
    }

    public void Detach(GameObject step_parent) {
        Utils.LogMessage("Detaching from "+step_parent.name);
        // check if previous step parent is still holding
        if (previous_step_parent != null && gripper_controllers[previous_step_parent].gripper_state==GripperController.gripperStates.Holding) {
                // transfer to the previous step parent
                Attach(previous_step_parent);
            }
        else {
            // transfer to the biological parent
            Attach(bio_parent);
        }
    }

    public void DetachFromStepParent() {
        if (step_parent != null) {
            gripper_contact[step_parent] = false;
            Detach(step_parent);
        }
    }

}
