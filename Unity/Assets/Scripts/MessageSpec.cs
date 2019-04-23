using System.Collections.Generic;

namespace MessageSpec
{
    // Camera class for decoding the ZMQ messages.
    public class WorldState_t
    {
        public string ID { get; set; }
        public string Name { get; set; }
        public IList<float> Position { get; set; }
        public IList<float> Rotation { get; set; }
    }
    
}