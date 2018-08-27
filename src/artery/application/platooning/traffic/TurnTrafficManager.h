#ifndef TURNTRAFFICMANAGER_H_
#define TURNTRAFFICMANAGER_H_

#include "artery/traci/TraCIBaseTrafficManager.h"

class TurnTrafficManager : public TraCIBaseTrafficManager
{

	public:

		virtual void initialize(int stage);
		virtual void finish();

		TurnTrafficManager() : TraCIBaseTrafficManager() {

		}

	protected:
		SimTime insertTime;
		double insertSpeed;

		int nCars;
		int lane;
		int nCarsInserted;
		double insertCarInterval;
		std::string platooningVType;

		std::string insertRoute;
		int insertRouteId;

		cMessage *insertVehilceMessage;

		//vehicles to be inserted
		struct Vehicle automated;

		virtual void scenarioLoaded();

		void insertVehicle();

		virtual void handleSelfMsg(cMessage *msg);

};

#endif
