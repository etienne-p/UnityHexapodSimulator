using UnityEngine;
using System.Collections;

[ExecuteInEditMode]
public class CameraController : MonoBehaviour
{
    [Range(0, 24)]
    public float distance = 1.0f;
    public Vector3 rotation;
    public Transform target;

    void Update()
    {
        if (target == null)
            return;
        transform.position = target.position + Quaternion.Euler(rotation) * (Vector3.forward * distance);
        transform.LookAt(target);
    }
}
