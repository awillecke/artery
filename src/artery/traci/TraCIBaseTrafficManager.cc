//
// Copyright (C) 2013-2016 Michele Segata <segata@ccs-labs.org>, Stefan Joerer <joerer@ccs-labs.org>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include "TraCIBaseTrafficManager.h"

Define_Module(TraCIBaseTrafficManager);

const auto traciInitSignal = omnetpp::cComponent::registerSignal("traci.init");
const auto traciStepSignal = omnetpp::cComponent::registerSignal("traci.step");

void TraCIBaseTrafficManager::initialize(int stage) {

	cSimpleModule::initialize(stage);

	if (stage == 0) {

		//empty all vectors
		vehicleTypeIds.clear();
		vehiclesCount.clear();
		laneIds.clear();
		roadIds.clear();
		routeIds.clear();
		laneIdsOnEdge.clear();
		routeStartLaneIds.clear();
		vehicleInsertQueue.clear();

		insertInOrder = true;

		//search for the scenario manager. it will be needed to inject vehicles
		traci::Core *tarci_core = check_and_cast<traci::Core*>(getParentModule()->getSubmodule("traci")->getSubmodule("core"));
		lite_api = &tarci_core->getLiteAPI();
		ASSERT2(lite_api, "cannot find traciCoreModule");

		//reset vehicles counter
		vehCounter = 0;
		initScenario = false;
		isTraciStartup = false;
		isTraciInitialized = false;

//		cModule* traci = getParentModule()->getSubmodule("traci");
		tarci_core->subscribe(traciInitSignal, this);
		tarci_core->subscribe(traciStepSignal, this);

		//start vehicles insertion
		insertVehiclesTrigger = new cMessage("insertVehiclesTrigger");
	}
}

void TraCIBaseTrafficManager::receiveSignal(cComponent*, simsignal_t signalId, const simtime_t&, cObject*)
{
    if (signalId == traciStepSignal) {
		EV_INFO << "TraCIBaseTrafficManager: traciStepSignal\n";
		if (isTraciStartup) {
			//get the time passed at first step
			updateInterval = simTime();
			EV_INFO << "TraCIBaseTrafficManager: updateInterval is " << updateInterval << "\n";
			scheduleAt(simTime() + updateInterval, insertVehiclesTrigger);
			isTraciStartup = false;
		}
	}
	else if (signalId == traciInitSignal) {
		EV_INFO << "TraCIBaseTrafficManager: traciInitSignal\n";
		isTraciInitialized = true;
		isTraciStartup = true;
	}
}

void TraCIBaseTrafficManager::handleSelfMsg(cMessage *msg) {

	if (msg == insertVehiclesTrigger) {

		if (isTraciInitialized) {
			insertVehicles();
		}

		scheduleAt(simTime() + updateInterval, insertVehiclesTrigger);
		return;
	}
}

void TraCIBaseTrafficManager::handleMessage(cMessage *msg) {
	if (msg->isSelfMessage()) {
		handleSelfMsg(msg);
	}
}

void TraCIBaseTrafficManager::finish() {
	if (insertVehiclesTrigger) {
		cancelAndDelete(insertVehiclesTrigger);
		insertVehiclesTrigger = 0;
	}
}

int TraCIBaseTrafficManager::findVehicleTypeIndex(std::string vehType) {

	unsigned int i;

	for (i = 0; i < vehicleTypeIds.size(); i++) {
		if (vehicleTypeIds[i].compare(vehType) == 0) {
			return i;
		}
	}

	return -1;

}

void TraCIBaseTrafficManager::loadSumoScenario() {

	//commandInterface = manager->getCommandInterface();

	//get all the vehicle types
	if(vehicleTypeIds.size()==0) {
		std::list<std::string> vehTypes = getVehicleTypeIds();
		EV << "Having currently " << vehTypes.size() << " vehicle types" << std::endl;
		for (std::list<std::string>::const_iterator i = vehTypes.begin(); i != vehTypes.end(); ++i) {
			if(i->compare("DEFAULT_VEHTYPE")!=0) {
				EV << "found vehType " << (*i) << std::endl;
				vehicleTypeIds.push_back(*i);
				//set counter of vehicles for this vehicle type to 0
				vehiclesCount.push_back(0);
			}
		}
	}
	//get all roads
	if(roadIds.size()==0) {
		std::list<std::string> roads = getRoadIds();
		EV << "Having currently " << roads.size() << " roads in the scenario" << std::endl;
		for (std::list<std::string>::const_iterator i = roads.begin(); i != roads.end(); ++i) {
			EV << *i << std::endl;
			roadIds.push_back(*i);
		}
	}
	//get all lanes
	if(laneIds.size()==0) {
		std::list<std::string> lanes = getLaneIds();
		EV << "Having currently " << lanes.size() << " lanes in the scenario" << std::endl;
		for (std::list<std::string>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
			EV << *i << std::endl;
			laneIds.push_back(*i);
			std::string edgeId = getRoadId(*i);
			laneIdsOnEdge[edgeId].push_back(*i);
		}
	}
	//get all routes
	if(routeIds.size()==0) {
		std::list<std::string> routes = getRouteIds();
		EV << "Having currently " << routes.size() << " routes in the scenario" << std::endl;
		for (std::list<std::string>::const_iterator i = routes.begin(); i != routes.end(); ++i) {
			std::string routeId = *i;
			EV << routeId << std::endl;
			routeIds.push_back(routeId);
			std::list<std::string> routeEdges = getRoadIds(routeId);
			std::string firstEdge = *(routeEdges.begin());
			EV << "First Edge of route " << routeId << " is " << firstEdge << std::endl;
			routeStartLaneIds[routeId] = laneIdsOnEdge[firstEdge];
		}
	}
	//inform inheriting classes that scenario is loaded
	scenarioLoaded();
}

void TraCIBaseTrafficManager::insertVehicles() {

	//if not already done, load all roads, all vehicle types, etc...
	if (!initScenario) {
		loadSumoScenario();
		initScenario = true;
	}

	//insert the vehicles in the queue
	for (InsertQueue::iterator i = vehicleInsertQueue.begin(); i != vehicleInsertQueue.end(); ++i) {
		std::string route = routeIds[i->first];
		EV << "process " << route << std::endl;
		std::deque<struct Vehicle>::iterator vi = i->second.begin();
		while (vi != i->second.end() && i->second.size() != 0) {
			bool suc = false;
			struct Vehicle v = *vi;
			std::string type = vehicleTypeIds[v.id];
			std::stringstream veh;
			veh << type << "." << vehiclesCount[v.id];

			//do we need to put this vehicle on a particular lane, or can we put it on any?

			if (v.lane == -1 && !insertInOrder) {

				//try to insert that into any lane
				for (unsigned int laneId = 0; !suc && laneId < routeStartLaneIds[route].size(); laneId++) {
					EV << "trying to add " << veh.str() << " with " << route << " vehicle type " << type << std::endl;
					suc = addVehicle(veh.str(), type, route, -2, v.position, v.speed, laneId);
					if (suc) break;
				}
				if (!suc) {
					//if we did not manager to insert a car on any lane, then this route is full and we can just stop
					//TODO: this is not true if we want to insert a vehicle not at the beginning of the route. fix this
					break;
				} else {
					EV << "successful inserted " << veh.str() << std::endl;
					vi = i->second.erase(vi);
					vehiclesCount[v.id] = vehiclesCount[v.id] + 1;
				}

			}
			else {

				//try to insert into desired lane
				EV << "trying to add " << veh.str() << " with " << route << " vehicle type " << type << std::endl;
				suc = addVehicle(veh.str(), type, route, -2, v.position, v.speed, v.lane);

				if (suc) {
					EV << "successful inserted " << veh.str() << std::endl;
					vi = i->second.erase(vi);
					vehiclesCount[v.id] = vehiclesCount[v.id] + 1;
				}
				else {
					if (!insertInOrder) {
						vi++;
					}
					else {
						break;
					}
				}

			}
		}
	}
}

void TraCIBaseTrafficManager::addVehicleToQueue(int routeId, struct Vehicle v) {
	vehicleInsertQueue[routeId].push_back(v);
}



std::list<std::string> TraCIBaseTrafficManager::getRoadIds()
{
	Enter_Method("getRoadIds");
	std::vector<std::string> vec = lite_api->edge().getIDList();
	return std::list<std::string>(vec.begin(), vec.end());
}

std::list<std::string> TraCIBaseTrafficManager::getLaneIds()
{
	Enter_Method("getLaneIds");
	std::vector<std::string> vec = lite_api->lane().getIDList();
	return std::list<std::string>(vec.begin(), vec.end());
}

std::list<std::string> TraCIBaseTrafficManager::getRouteIds()
{
	Enter_Method("getRouteIds");
	std::vector<std::string> vec = lite_api->route().getIDList();
	return std::list<std::string>(vec.begin(), vec.end());
}

std::list<std::string> TraCIBaseTrafficManager::getVehicleTypeIds()
{
	Enter_Method("getVehicleTypeIds");
	std::vector<std::string> vec = lite_api->vehicletype().getIDList();
	return std::list<std::string>(vec.begin(), vec.end());
}

std::string TraCIBaseTrafficManager::getRoadId(const std::string laneId)
{
	Enter_Method("getRoadId");
	return lite_api->lane().getEdgeID(laneId);
}

std::list<std::string> TraCIBaseTrafficManager::getRoadIds(const std::string routeId)
{
	Enter_Method("getRoadIds");
	std::vector<std::string> vec = lite_api->route().getEdges(routeId);
	return std::list<std::string>(vec.begin(), vec.end());
}

bool TraCIBaseTrafficManager::addVehicle(const std::string& vehicleID, const std::string& typeID,
		const std::string& routeID, simtime_t depart, const float departPos, const float departSpeed,
		const unsigned int departLane)
{

	Enter_Method("addVehicle");

	int32_t emitTime = (depart < 0) ? (-1) : (floor(depart.dbl() * 1000));

	lite_api->vehicle().add(vehicleID, routeID, typeID, std::to_string(emitTime),
			std::to_string((unsigned int) departLane), std::to_string((float) departPos),
			std::to_string((float) departSpeed));
	return true;
}

