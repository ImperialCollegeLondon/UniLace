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
using RosMessageTypes.UniLace;


public class Demo : MainGym
{
    private List<Vector3> eyelet_pos_lib = new List<Vector3>();
    private List<Vector3> eyelet_rot_lib = new List<Vector3>();
    private List<int> aglet_id_sequence = new List<int>();

    private int eyelet_id_cur = 0;
    private float precision = 0.01f;

    private int aglet_target_id;
    private Vector3 aglet_target_position;
    private Vector3 aglet_target_rotation;

    private bool initialised=false;

    // Start is called before the first frame update
    async void Start()
    {
        base.Start();
        await WaitForGenerating();

        // add waypoints
        for (int i=0;i<pm.num_eyelets;i++) {
            int aglet_id = 0;
            if (i%4==1 || i%4==2)
                aglet_id = 1;
            if (i%2==0)
                addPointsLeft(shoe.eyelets[i].position, 
                    (shoe.eyelets[i].rotation*Quaternion.Euler(-90,0,0)).eulerAngles, 
                    -shoe.eyelets[i].up, 
                    aglet_id,
                    i/2);
            else
                addPointsRight(shoe.eyelets[i].position, 
                    (shoe.eyelets[i].rotation*Quaternion.Euler(-90,0,0)).eulerAngles, 
                    -shoe.eyelets[i].up, 
                    aglet_id,
                    i/2);
        }
        Debug.Log("Added "+eyelet_pos_lib.Count.ToString()+" points");

        // initialise the targets
        aglet_target_id = aglet_id_sequence[0];
        aglet_target_position = eyelet_pos_lib[0];
        aglet_target_rotation = eyelet_rot_lib[0];

        initialised = true;
    }
    

    void addPointsLeft(Vector3 position, Vector3 rotation, Vector3 up, int id, int eyelet_id)
    {
        eyelet_pos_lib.Add(position+up*0.5f);
        aglet_id_sequence.Add(id);  
        eyelet_rot_lib.Add(rotation);

        eyelet_pos_lib.Add(position);
        aglet_id_sequence.Add(id);
        eyelet_rot_lib.Add(rotation);

        eyelet_pos_lib.Add(position+new Vector3(0.15f, -0.1f, 0.2f));
        aglet_id_sequence.Add(id);
        eyelet_rot_lib.Add(new Vector3(0f, 0, 0));

        eyelet_pos_lib.Add(position+new Vector3(0.3f, 1f, 0.2f));
        aglet_id_sequence.Add(id);
        eyelet_rot_lib.Add(new Vector3(0f, 0f, 0));

        eyelet_pos_lib.Add(position+new Vector3(5f-eyelet_id*0.5f, 1f, 0.5f));
        aglet_id_sequence.Add(id);
        eyelet_rot_lib.Add(new Vector3(0f, 90f, 0));
    }

    void addPointsRight(Vector3 position, Vector3 rotation, Vector3 up, int id, int eyelet_id)
    {
        eyelet_pos_lib.Add(position+up*0.5f);
        aglet_id_sequence.Add(id);
        eyelet_rot_lib.Add(rotation);

        eyelet_pos_lib.Add(position);
        aglet_id_sequence.Add(id);
        eyelet_rot_lib.Add(rotation);

        eyelet_pos_lib.Add(position+new Vector3(-0.15f, -0.1f, 0.2f));
        aglet_id_sequence.Add(id);
        eyelet_rot_lib.Add(new Vector3(0f, 0, 0));

        eyelet_pos_lib.Add(position+new Vector3(-0.3f, 1f, 0.2f));
        aglet_id_sequence.Add(id);
        eyelet_rot_lib.Add(new Vector3(0f, 0f, 0));

        eyelet_pos_lib.Add(position+new Vector3(-(5f-eyelet_id*0.5f), 1f, 0.5f));
        aglet_id_sequence.Add(id);
        eyelet_rot_lib.Add(new Vector3(0f, -90f, 0));
    }

    // Update is called once per frame
    void Update()
    {
        base.Update();
        if (!initialised) return;

        // start fake insertion routine
        float distance = Vector3.Distance(shoelace.aglets[aglet_target_id].transform.position, aglet_target_position);
        if (distance>precision)
        {
            // move the aglet towards the target eyelet
            shoelace.MoveAgletTowardsPose(aglet_target_id, aglet_target_position, Quaternion.Euler(aglet_target_rotation), Time.deltaTime);
        }
        else
        {
            eyelet_id_cur += 1;
            if (eyelet_id_cur>=eyelet_pos_lib.Count) 
            {
                initialised=false;
                Utils.LogMessage("Finished");
                Pause();
                // calculate the branch lengths and eyestay gaps
                Vector2 branchLengths = shoelace.EstimateBranchLengths(
                        shoe.eyelets[pm.num_eyelets-2].position, 
                        shoe.eyelets[pm.num_eyelets-1].position, true);
                List<float> eyestayGaps = shoe.EstimateEyestayGaps();
                string message = "Branch lengths: " + branchLengths.x.ToString("F3") + ", " + branchLengths.y.ToString("F3") + "\n";
                // message += "Eyestay gaps: ";
                // for (int i=0;i<pm.num_eyelets/2;i++) 
                //     message += (eyestayGaps[i]/eyestayInitGaps[i]).ToString("F3") + ", ";
                Utils.LogMessage(message);
                // statusText.text = "Status: Success \n" + message;
                fpsDisplay.sysMessage = "Status: Success \n" + message;
                return;
            }
            // update the target
            aglet_target_id = aglet_id_sequence[eyelet_id_cur];
            aglet_target_position = eyelet_pos_lib[eyelet_id_cur];
            aglet_target_rotation = eyelet_rot_lib[eyelet_id_cur];
        }
    }

    public async Task<bool> WaitForGenerating() {
        while (generated == false)
            await Task.Yield();
        return true;
    }
}