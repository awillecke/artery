//package scenarios.platooning;

import artery.inet.World;
import artery.traci.TraCIBaseTrafficManager;

module Highway extends artery.inet.World
{    
    parameters:
        string traffic_type;
        
    submodules:
        traffic: <traffic_type> like TraCIBaseTrafficManager {
            parameters:
                @display("p=20,40");
        }
}

