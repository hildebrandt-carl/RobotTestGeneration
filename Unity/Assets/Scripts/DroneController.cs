using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System.Threading;
using Newtonsoft.Json;
using MessageSpec;
using System.Collections.Generic;
using System.Collections;
using System.IO;
using System.Text.RegularExpressions;

public class DroneController : MonoBehaviour
{
    
    public string connectionIP = "127.0.0.1";
    public int connectionPort = 25001;

    private IPAddress localAdalAdd;
    private TcpListener listener;
    private TcpClient client;
    private Vector3 pos = Vector3.zero;
    private string data = "";
    private bool running;
    private bool Collision = false;
    private Thread mThread;
    private WorldState_t worldState;
    private DroneState_t DroneState;

    // Runs on every screen
    private void Update()
    {
        // If we have received data
       if (data != "")
       {
            // Conver the JSON over to a WorldState
            worldState = JsonConvert.DeserializeObject<WorldState_t>(data);

            // Display the different peices of informaiton
            //Debug.Log(ListToVector3(worldState.Position));
            //Debug.Log(ListToVector3(worldState.Rotation));

            // Apply translation and rotation
            transform.position = ListToVector3(worldState.Position);
            transform.rotation = Quaternion.Euler(ListToVector3(worldState.Rotation));

            if(worldState.Reset == true)
            {
                Collision = false;
            }
        }
    }

    // Runs when we are starting the program
    private void Start()
    {
        Debug.Log("Starting Program");

        // Loads a config file to get the correct IP and port
        loadConfigFile();

        // Start a new thread for connecting to TCP connection
        ThreadStart ts = new ThreadStart(GetInfo);
        mThread = new Thread(ts);
        mThread.Start();
    }

    // Loads the config file finding the port number and IP address used to connect to ROS
	void loadConfigFile()
	{
		StreamReader inp_stm = new StreamReader("config.txt");
		while (!inp_stm.EndOfStream)
		{
			string inp_ln = inp_stm.ReadLine();
			if (inp_ln.Length > 0)
			{ 
				string Instruction = inp_ln.Substring(0, 2);
				string Value = "";

				switch (Instruction)
				{
					// Port
					case "PT":
						Value = Regex.Match(inp_ln, @"\(([^)]*)\)").Groups[1].Value ;
						string[] pt_loc = Regex.Split(Value, @",");

                        connectionPort = int.Parse(pt_loc[0]);
                        Debug.Log(connectionPort);
						break;
					// IP
					case "IP":
						// Parse the string to get the information from it
						Value = Regex.Match(inp_ln, @"\(([^)]*)\)").Groups[1].Value ;
						string[] ip_loc = Regex.Split(Value, @",");
                        
                        connectionIP = ip_loc[0];
                        Debug.Log(connectionIP);
						break;
					default:
						//Debug.Log("Not Found");
						break;
				}
			}
		}
		inp_stm.Close();
	}

    // This is started in a new thread
    void GetInfo()
    {
        // Create a TCP server
        localAdalAdd = IPAddress.Parse(connectionIP);
        listener = new TcpListener(IPAddress.Any, connectionPort);
        listener.Start();

        // Accept pending connection request
        client = listener.AcceptTcpClient();

        // While the application is running
        running = true;
        while (running)
        {
            // Accept the data
            Connection();
        }

        // Stop the connection
        client.Close();
        listener.Stop();
    }

    // Processes the TCP data
    void Connection()
    {
        // Create a new network stream
        NetworkStream nwStream = client.GetStream();
        byte[] buffer = new byte[client.ReceiveBufferSize];

        // Read the data from the TCP connection
        int bytesRead = nwStream.Read(buffer, 0, client.ReceiveBufferSize);
        string dataRecieved = Encoding.UTF8.GetString(buffer, 0, bytesRead);

        // Get the data
        data = "";
        if (dataRecieved != null)
        {
            if (dataRecieved != "")
            {
                data = dataRecieved;

                // Create the drone state
                DroneState = new DroneState_t();
                DroneState.CollsionObject = "Test" ;
                DroneState.Collision = Collision ;
                
                // Serialize the state of the drone
                string SerializedDroneState = JsonConvert.SerializeObject(DroneState);

                // Convert the string to bytes
                byte[] byteDroneState = Encoding.ASCII.GetBytes(SerializedDroneState);
                int numberBytes = byteDroneState.GetLength(0);

                // Send the data over
                nwStream.Write(byteDroneState, 0, numberBytes);
            }
        }       
    }

    // Check for collisions
    void OnTriggerEnter(Collider other)
    {
        Debug.Log("Collision Detected!");
        Collision = true;
    }

    public static Vector3 ListToVector3(IList<float> list)
    {
        return new Vector3(list[0], list[1], list[2]);
    }

    public static Quaternion ListToQuaternion(IList<float> list)
    {
        return new Quaternion(list[0], list[1], list[2], list[3]);
    }
}