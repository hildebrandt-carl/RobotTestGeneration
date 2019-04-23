using UnityEngine;
using System.Collections;

public class RotatorR : MonoBehaviour
{

    void Update()
    {
        transform.Rotate(new Vector3(0, 0, -4182) * Time.deltaTime);
    }
}