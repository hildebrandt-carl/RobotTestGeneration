using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text.RegularExpressions;

public class LoadEnvironment : MonoBehaviour
{
    public Transform obstacleObject;
    public Transform wallObject;
    public Transform goalObject;
    public Transform rainObject;

    public GameObject sun;

    public bool Day = false;
    public bool Raining = false;
    public string FileName = "test1.txt";

    // Place the objects
    void Start()
    {
        readTextFile(FileName);
    }

    void readTextFile(string file_path)
    {
        StreamReader inp_stm = new StreamReader(file_path);

        while (!inp_stm.EndOfStream)
        {
            string inp_ln = inp_stm.ReadLine();

            if (inp_ln.Length > 0)
            { 
                string FirstLetter = inp_ln.Substring(0, 1);
                string Value = "";
                Debug.Log(FirstLetter);

                switch (FirstLetter)
                {
                    // Block
                    case "B":
                        Value = Regex.Match(inp_ln, @"\(([^)]*)\)").Groups[1].Value ;
                        string[] o_loc = Regex.Split(Value, @",");
                        Instantiate(obstacleObject, new Vector3(float.Parse(o_loc[0]), float.Parse(o_loc[1]), float.Parse(o_loc[2])), Quaternion.identity);
                        Debug.Log("Here1");
                        break;
                    // Wall
                    case "W":
                        Value = Regex.Match(inp_ln, @"\(([^)]*)\)").Groups[1].Value ;
                        string[] w_loc = Regex.Split(Value, @",");
                        Transform wallClone;
                        wallClone = Instantiate(wallObject, new Vector3(float.Parse(w_loc[0]), float.Parse(w_loc[1]), float.Parse(w_loc[2])), Quaternion.identity);
                        wallClone.transform.localScale = new Vector3(float.Parse(w_loc[3]),float.Parse(w_loc[4]),float.Parse(w_loc[5]));
                        break;
                    // Goal
                    case "G":
                        Value = Regex.Match(inp_ln, @"\(([^)]*)\)").Groups[1].Value;
                        string[] g_loc = Regex.Split(Value, @",");
                        Instantiate(goalObject, new Vector3(float.Parse(g_loc[0]), float.Parse(g_loc[1]), float.Parse(g_loc[2])), Quaternion.identity);
                        break;
                    // Rain
                    case "R":
                        Value = Regex.Match(inp_ln, @"\(([^)]*)\)").Groups[1].Value;
                        Raining = (Value == "1");
                        if (Raining)
                        {
                            Instantiate(rainObject, new Vector3(0, 0, 0), Quaternion.identity);
                        }
                        break;
                    // Day
                    case "D":
                        Value = Regex.Match(inp_ln, @"\(([^)]*)\)").Groups[1].Value;
                        Day = (Value == "1");
                        sun.SetActive(Day);
                        break;
                    default:
                        Debug.Log("Not Found");
                        break;
                }
            }
        }

        inp_stm.Close();
    }
}




