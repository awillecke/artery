package artery.application.platooning.traffic;

import artery.traci.TraCIBaseTrafficManager;

simple TurnTrafficManager like TraCIBaseTrafficManager {

    parameters:
        //platoon insertion time
        double insertTime @unit("s");
        //platoon insertion interval
        double insertCarInterval @unit("s");
        //platoon insertion speed
        double insertSpeed @unit("kmph");
        //how many vehicles to inject
        int nCars;
        //lanes
        int lane;
        //sumo vehicle type for platooning cars
        string platooningVType;
        string insertRoute;
        @class(TurnTrafficManager);
}
