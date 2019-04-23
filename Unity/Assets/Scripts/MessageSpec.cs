using System.Collections.Generic;

namespace MessageSpec
{
    // Camera class for decoding the ZMQ messages.
    public class WorldState_t
    {
        public IList<float> Position { get; set; }
        public IList<float> Rotation { get; set; }
        public bool Reset { get; set; }
    }

    public class DroneState_t
    {
        public string CollsionObject { get; set; }
        public bool Collision { get; set; }
    }
    
}