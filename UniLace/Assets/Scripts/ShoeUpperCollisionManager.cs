using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Obi;

/// <summary>
/// This script is used to monitor the collision between the shoe upper and the aglet.
/// If the aglet is not properly inserted (i.e. at the upper), the collision will be detected.
/// If the collision persists for a certain number of iterations, the task will be marked as failed.
/// The monitoring is done in the Main scipts.
/// 
/// More about Obi on collision:
/// https://obi.virtualmethodstudio.com/manual/6.3/scriptingcollisions.html
/// </summary>

public class ShoeUpperCollisionManager : MonoBehaviour
{
 	ObiSolver solver;
    public float agletWidth = 0.005f;
	public int monitoredAglet = -1;
	// add a marker
	// public GameObject marker;

	void Start(){
		// // add a marker
		// marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
		// marker.transform.localScale = Vector3.one * 0.05f;
		// marker.GetComponent<Renderer>().material.color = Color.red;
		// marker.SetActive(false);
		
		solver = GameObject.Find("Obi Solver").GetComponent<ObiSolver>();
		solver.OnCollision += Solver_OnCollision;
	} 
    
    void Solver_OnCollision(object sender, Obi.ObiSolver.ObiCollisionEventArgs e)
    {
        var world = ObiColliderWorld.GetInstance();
		bool inCollisionAglet = false;

		// iterate over all contacts (this is predicted contact)
		foreach (Oni.Contact contact in e.contacts)
		{
			// check if there are penetrations
			if (contact.distance < 0)
			{
				ObiColliderBase col = world.colliderHandles[contact.bodyB].owner;
				// check if is an aglet
				if (col != null && col.gameObject.name.Contains("Aglet"))
				{
					// check if involved actor is the upper
					int particleIndex = solver.simplices[contact.bodyA];
					ObiSolver.ParticleInActor pa = solver.particleToActor[particleIndex];
					if (pa.actor.gameObject.name.Contains("Rope")) continue;

					// process the collision
					inCollisionAglet = true;
					// // move the marker to contact point
					// int particleIndex = solver.simplices[contact.bodyA];
					// marker.transform.position = solver.positions[particleIndex];
					// marker.SetActive(true);
					// check if is a penetration.
					// Utils.LogMessage("Aglet contact, distance: " + contact.distance);
                    // if (contact.distance < -agletWidth)  {
					// start monitoring is not
					if (monitoredAglet == -1 && contact.distance < -0.0001f)  {
                    	// start monitoring the aglet, report failure if lasts certain iterations
						monitoredAglet = col.gameObject.name[^1]-'A';
                        Utils.LogMessage("Aglet evasion, distance: " + contact.distance + " at " + monitoredAglet);
                    }
				}
			}
		}
		if (!inCollisionAglet && monitoredAglet != -1) {
			// not in contact, stop monitoring the aglet
			monitoredAglet = -1;
		}
    }
}
