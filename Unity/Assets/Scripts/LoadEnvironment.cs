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

	public GameObject[] cage_walls;

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
				//Debug.Log(FirstLetter);

				switch (FirstLetter)
				{
					// Block
					case "B":
						Value = Regex.Match(inp_ln, @"\(([^)]*)\)").Groups[1].Value ;
						string[] o_loc = Regex.Split(Value, @",");
						Instantiate(obstacleObject, new Vector3(float.Parse(o_loc[0]), float.Parse(o_loc[1]), float.Parse(o_loc[2])), Quaternion.identity);
						//Debug.Log("Here1");
						break;
					// Wall
					case "W":
						// Parse the string to get the information from it
						Value = Regex.Match(inp_ln, @"\(([^)]*)\)").Groups[1].Value ;
						string[] w_loc = Regex.Split(Value, @",");

						// Get the two points of the wall
						Vector3 Pos1 = new Vector3(float.Parse(w_loc[1]), 0, float.Parse(w_loc[0])) ;
						Vector3 Pos2 = new Vector3(float.Parse(w_loc[3]), 0, float.Parse(w_loc[2])) ;

						// Caclulate the center of the wall
						Vector3 centerPosition = (Pos1 + Pos2)/2 ;
						centerPosition.y = float.Parse(w_loc[4])/2 ;

						// Calculate the distance of the wall
						float dx = Pos2.x - Pos1.x ;
						float dy = Pos2.z - Pos1.z ;
						float distance = Mathf.Sqrt(Mathf.Pow(dx, 2) + Mathf.Pow(dy, 2)) ;
						Debug.Log(distance) ;

						// Create the rotation of the wall
						float wall_angle = Mathf.Atan2(dy, dx) * Mathf.Rad2Deg ;
						Quaternion rotation = Quaternion.Euler(0, -1 * wall_angle, 0) ;
						Debug.Log(wall_angle) ;

						// Create the scaling parameters
						Vector3 scaling = new Vector3(distance, float.Parse(w_loc[4]), 1) ;

						// Create the wall and scale it
						Transform wallClone = Instantiate(wallObject, centerPosition, rotation);
						wallClone.transform.localScale = scaling;

						// End of creating wall
						break;
					// Goal
					case "G":
						// Parse the string to get the information from it
						Value = Regex.Match(inp_ln, @"\(([^)]*)\)").Groups[1].Value;
						string[] g_loc = Regex.Split(Value, @",");

						// Get the cetner of the waypoint
						float x = float.Parse(g_loc[0]) ;
						float y = float.Parse(g_loc[1]) ;
						float h = float.Parse(g_loc[2]) ;
						Vector3 pos = new Vector3(y, h, x) ;

						// Instantiate the goal
						Instantiate(goalObject, pos, Quaternion.identity);

						// End of creating waypoint (goal)
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
					// Day
					case "C":
						Value = Regex.Match(inp_ln, @"\(([^)]*)\)").Groups[1].Value;
						if (Value == "0")
						{
							foreach(GameObject cage_wall in cage_walls)
							{
								cage_wall.GetComponent<MeshRenderer>().enabled = false;
								cage_wall.GetComponent<BoxCollider>().enabled = false;
							}
						}
						sun.SetActive(Day);
						break;
					default:
						//Debug.Log("Not Found");
						break;
				}
			}
		}
		inp_stm.Close();
	}
}