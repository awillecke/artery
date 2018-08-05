//
// Copyright (C) 2018 Alexander Willecke <willecke@ibr.cs.tu-bs.de>
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

#ifndef PLATOONINGSERVICE_H_
#define PLATOONINGSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/platooning/messages/UnicastMessage_m.h"
#include "artery/application/platooning/CC_Const.h"

//TODO: Replacement for findSubModule
#include "veins/base/utils/FindModule.h"

/**
*  @brief This service encapsulates the Plexe modules and provides wrapper functions to TraCI.
*  It also handles messages from/to the middleware.
*/
class PlatooningService : public ItsG5Service
{
	public:
		PlatooningService();

		void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;
		void trigger() override;
		void receiveSignal(cComponent*, omnetpp::simsignal_t, cObject*, cObject*) override;

		void sendPlatooningUnicast(omnetpp::cMessage* msg);
		void sendUp(omnetpp::cMessage *msg);

		/* Wrapper functions to interact with TraCI
		 * For more information, see http://plexe.car2x.org/api/
		 * */
		unsigned int getLanesCount();
		void setPlatoonLeaderData(double leaderSpeed, double leaderAcceleration, double positionX, double positionY,
				double time);
		void setPrecedingVehicleData(double speed, double acceleration, double positionX, double positionY, double time);
		void getVehicleData(double &speed, double &acceleration, double &controllerAcceleration, double &positionX,
				double &positionY, double &time);
		void setGenericInformation(int type, const void* data, int length);
		void getGenericInformation(int type, const void* params, int paramsLength, void *result);
		void setCruiseControlDesiredSpeed(double desiredSpeed);
		void setActiveController(enum Plexe::ACTIVE_CONTROLLER activeController);
		enum Plexe::ACTIVE_CONTROLLER getActiveController();
		void setCACCConstantSpacing(double spacing);
		double getCACCConstantSpacing();
		void setACCHeadwayTime(double headway);
		void setFixedAcceleration(int activate, double acceleration);
		bool isCrashed();
		bool isCruiseControllerInstalled();
		void setLaneChangeAction(enum Plexe::PLATOONING_LANE_CHANGE_ACTION action);
		enum Plexe::PLATOONING_LANE_CHANGE_ACTION getLaneChangeAction();
		void setFixedLane(int laneIndex);
		void getRadarMeasurements(double &distance, double &relativeSpeed);
		void setControllerFakeData(double frontDistance, double frontSpeed, double frontAcceleration, double leaderSpeed,
				double leaderAcceleration);
		double getDistanceToRouteEnd();
		double getDistanceFromRouteBegin();
		double getACCAcceleration();

		double getCurrentSpeed();

		libsumo::TraCIPosition getPosition();

		std::string getVehicleId();

	protected:
		void initialize() override;
		void finish() override;
		void handleMessage(omnetpp::cMessage*) override;

	private:
		std::string vehicleId;
		const VehicleDataProvider *mVehicleDataProvider;
		omnetpp::cMessage* m_self_msg;

		int serviceIn;
		int serviceOut;

};

#endif /* PLATOONINGSERVICE_H_ */
