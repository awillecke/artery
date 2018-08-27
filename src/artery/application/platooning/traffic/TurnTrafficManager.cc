#include "TurnTrafficManager.h"

Define_Module(TurnTrafficManager);

void TurnTrafficManager::initialize(int stage) {

	TraCIBaseTrafficManager::initialize(stage);

	if (stage == 0) {

		insertTime = SimTime(par("insertTime").doubleValue());
		insertSpeed = par("insertSpeed").doubleValue();

		nCars = par("nCars").longValue();
		lane = par("lane").longValue();
		insertCarInterval = par("insertCarInterval").doubleValue();
		platooningVType = par("platooningVType").stdstringValue();

		insertRoute = par("insertRoute").stdstringValue();

		insertVehilceMessage = new cMessage("");
		scheduleAt(insertTime, insertVehilceMessage);

		nCarsInserted = 0;
	}

}

void TurnTrafficManager::handleSelfMsg(cMessage *msg) {

	TraCIBaseTrafficManager::handleSelfMsg(msg);

	if (msg == insertVehilceMessage) {
		EV_INFO << "insertVehilceMessage\n";
		if (nCarsInserted < nCars) {
			insertVehicle();
			scheduleAt(simTime() + insertCarInterval, insertVehilceMessage);
		}
	}

}

void TurnTrafficManager::scenarioLoaded() {
	automated.id = findVehicleTypeIndex(platooningVType);
	automated.lane = lane;
	automated.position = 0;
	automated.speed = insertSpeed/3.6;

	std::list<std::string> routes = getRouteIds();
	for (std::list<std::string>::iterator i = routes.begin(); i != routes.end(); ++i) {
		std::string routeId = *i;
		if (routeId == insertRoute) {
			insertRouteId = std::distance(routes.begin(), i);
		}
	}
}

void TurnTrafficManager::insertVehicle() {
	addVehicleToQueue(insertRouteId, automated);
	nCarsInserted++;
}

void TurnTrafficManager::finish() {
	TraCIBaseTrafficManager::finish();
	cancelAndDelete(insertVehilceMessage);
	insertVehilceMessage = 0;
}

