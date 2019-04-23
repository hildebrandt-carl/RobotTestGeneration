using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System.Threading;
using Newtonsoft.Json;
using MessageSpec;
using System.Collections.Generic;
using System.Collections;

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
            //Debug.Log(worldState.ID);
            //Debug.Log(worldState.Name);
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

        // Start a new thread for connecting to TCP connection
        ThreadStart ts = new ThreadStart(GetInfo);
        mThread = new Thread(ts);
        mThread.Start();
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
                DroneState.ID = "drone1" ;
                DroneState.Name = "drone" ;
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