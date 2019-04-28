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
        float currentAngle = transform.eulerAngles.y;
        float desiredAngle = drone.transform.eulerAngles.y;
        float angle = Mathf.LerpAngle(currentAngle, desiredAngle, Time.deltaTime*5);
         
        Quaternion rotation = Quaternion.Euler(0, angle, 0);
        transform.position = drone.transform.position + (rotation * offset);
         
        transform.LookAt(drone.transform);
    }
}