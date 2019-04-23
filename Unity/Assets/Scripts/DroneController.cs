using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System.Threading;
using Newtonsoft.Json;
using MessageSpec;
using System.Collections.Generic;


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
    private Thread mThread;
    private WorldState_t state;

    private void Update()
    {
       if (data != "")
       {
            state = JsonConvert.DeserializeObject<WorldState_t>(data);

            
            Debug.Log(state.ID);
            Debug.Log(state.Name);
            Debug.Log(ListToVector3(state.Position));
            Debug.Log(ListToVector3(state.Rotation));

            // Apply translation and rotation
            transform.position = ListToVector3(state.Position);
            transform.rotation = Quaternion.Euler(ListToVector3(state.Rotation));
        }

    }

    private void Start()
    {
        Debug.Log("Starting Program");
        ThreadStart ts = new ThreadStart(GetInfo);
        mThread = new Thread(ts);
        mThread.Start();
    }

    void GetInfo()
    {
        localAdalAdd = IPAddress.Parse(connectionIP);
        listener = new TcpListener(IPAddress.Any, connectionPort);
        listener.Start();

        client = listener.AcceptTcpClient();

        running = true;
        while (running)
        {
            Connection();
        }

        listener.Stop();
    }

    void Connection()
    {
        NetworkStream nwStream = client.GetStream();
        byte[] buffer = new byte[client.ReceiveBufferSize];

        int bytesRead = nwStream.Read(buffer, 0, client.ReceiveBufferSize);
        string dataRecieved = Encoding.UTF8.GetString(buffer, 0, bytesRead);

        data = "";
        if (dataRecieved != null)
        {
            if (dataRecieved != "")
            {
                data = dataRecieved;
            }
        }       
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