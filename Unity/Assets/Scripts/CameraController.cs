using UnityEngine;
using System.Collections;

public class CameraController : MonoBehaviour
{

    public GameObject drone;

    private Vector3 offset;

    void Start()
    {
        offset = transform.position - drone.transform.position;
    }

    void LateUpdate()
    {
        transform.position = drone.transform.position + offset;
    }
}


