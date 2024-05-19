using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using Obi;

public class ObiColliderCreator : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        MeshCollider[] meshColliders = gameObject.GetComponentsInChildren<MeshCollider>();
        for (int i=0; i<meshColliders.Length; i++) {
            if (meshColliders[i].gameObject.GetComponent<ObiCollider>() == null) {
                ObiCollider collider = meshColliders[i].gameObject.AddComponent<ObiCollider>();
                if (meshColliders[i].gameObject.name.Contains("finger")) {
                    collider.CollisionMaterial = Resources.Load<ObiCollisionMaterial>("Materials/IceObiMaterial.asset");
                }
                int mask = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5);
                int filter = ObiUtils.MakeFilter(mask, 3);
                collider.Filter = filter;
            }
        }
    }

}
