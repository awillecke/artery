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

#include "artery/application/platooning/PlatooningService.h"
#include "artery/application/platooning/UnicastProtocol.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cpacket.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>

#include <boost/optional/optional_io.hpp>

using namespace omnetpp;
using namespace vanetza;

static const simsignal_t scSignalUnicastReceived = cComponent::registerSignal("PlatooningUnicastReceived");

Define_Module (PlatooningService)

PlatooningService::PlatooningService()
{
}

void PlatooningService::indicate(const btp::DataIndication& ind, cPacket* packet)
{
	UnicastMessage *unicast;
	unicast = static_cast<UnicastMessage *>(packet);
	assert(unicast);

	omnetpp::cObject obj(std::move(*unicast));
	emit(scSignalUnicastReceived, &obj);

	sendUp(unicast);

	delete packet;
}

void PlatooningService::initialize()
{
	ItsG5Service::initialize();
	m_self_msg = new cMessage("Platooning Service");
	subscribe(scSignalUnicastReceived);

	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	vehicleId = vehicle.getVehicleId();

	mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();

	serviceIn = findGate("serviceIn");
	serviceOut = findGate("serviceOut");
}

void PlatooningService::finish()
{
	cancelAndDelete(m_self_msg);
	ItsG5Service::finish();
}

void PlatooningService::handleMessage(cMessage* msg)
{
	Enter_Method("handleMessage");

	if (msg == m_self_msg) {
		EV_INFO << "PlatooningService: self message\n";
	}
	else if (msg->getArrivalGateId() == serviceIn) {
	}
	else if (msg->getArrivalGateId() == serviceOut) {
		sendPlatooningUnicast(msg);
	}

	delete msg;
}

void PlatooningService::trigger()
{
	Enter_Method("trigger");
}

void PlatooningService::receiveSignal(cComponent* source, simsignal_t signal, cObject*, cObject*)
{
}

void PlatooningService::sendPlatooningUnicast(cMessage* msg)
{
	using namespace vanetza;

	UnicastMessage *unicast;
	unicast = dynamic_cast<UnicastMessage *>(msg);
	assert(unicast);

	btp::DataRequestB request;
	request.destination_port = host_cast<PlatooningService::port_type>(getPortNumber());
	request.gn.its_aid = 145; //TODO: register somewhere
	request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::_1_S, 1 };
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
	request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

	this->request(request, dynamic_cast<omnetpp::cPacket *>(unicast->dup()));

}

void PlatooningService::sendUp(cMessage *msg)
{
	UnicastProtocol* protocol = FindModule<UnicastProtocol*>::findSubModule(this);
	protocol->publicHandleLowerMsg(msg);
}

unsigned int PlatooningService::getLanesCount()
{
	Enter_Method("getLanesCount");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	return vehicle.getLiteAPI().vehicle().getLanesCount(vehicleId);
}

int PlatooningService::getLaneIndex()
{
	Enter_Method("getLaneIndex");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	return vehicle.getLiteAPI().vehicle().getLaneIndex(vehicleId);
}

void PlatooningService::setPlatoonLeaderData(double leaderSpeed, double leaderAcceleration, double positionX,
		double positionY, double time)
{
	Enter_Method("setPlatoonLeaderData");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	vehicle.getLiteAPI().vehicle().setPlatoonLeaderData(vehicleId, leaderSpeed, leaderAcceleration, positionX,
			positionY, time);
}

void PlatooningService::setPrecedingVehicleData(double speed, double acceleration, double positionX, double positionY,
		double time)
{
	Enter_Method("setPrecedingVehicleData");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	vehicle.getLiteAPI().vehicle().setPrecedingVehicleData(vehicleId, speed, acceleration, positionX, positionY, time);
}

void PlatooningService::getVehicleData(double &speed, double &acceleration, double &controllerAcceleration,
		double &positionX, double &positionY, double &time)
{
	Enter_Method("getVehicleData");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	vehicle.getLiteAPI().vehicle().getVehicleData(vehicleId, speed, acceleration, controllerAcceleration, positionX,
			positionY, time);
}

void PlatooningService::setGenericInformation(int type, const void* data, int length)
{
	Enter_Method("setGenericInformation");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	vehicle.getLiteAPI().vehicle().setGenericInformation(vehicleId, type, data, length);
}

void PlatooningService::getGenericInformation(int type, const void* params, int paramsLength, void *result)
{
	Enter_Method("getGenericInformation");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	vehicle.getLiteAPI().vehicle().getGenericInformation(vehicleId, type, params, paramsLength, result);
}

void PlatooningService::setCruiseControlDesiredSpeed(double desiredSpeed)
{
	Enter_Method("setCruiseControlDesiredSpeed");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	vehicle.getLiteAPI().vehicle().setCruiseControlDesiredSpeed(vehicleId, desiredSpeed);
}

void PlatooningService::setActiveController(enum Plexe::ACTIVE_CONTROLLER activeController)
{
	Enter_Method("setActiveController");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	vehicle.getLiteAPI().vehicle().setActiveController(vehicleId, static_cast<int>(activeController));
}

enum Plexe::ACTIVE_CONTROLLER PlatooningService::getActiveController()
{
	Enter_Method("getActiveController");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	return static_cast<Plexe::ACTIVE_CONTROLLER>(vehicle.getLiteAPI().vehicle().getActiveController(vehicleId));
}

void PlatooningService::setCACCConstantSpacing(double spacing)
{
	Enter_Method("setCACCConstantSpacing");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	vehicle.getLiteAPI().vehicle().setCACCConstantSpacing(vehicleId, spacing);
}

double PlatooningService::getCACCConstantSpacing()
{
	Enter_Method("getCACCConstantSpacing");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	return vehicle.getLiteAPI().vehicle().getCACCConstantSpacing(vehicleId);
}

void PlatooningService::setACCHeadwayTime(double headway)
{
	Enter_Method("setACCHeadwayTime");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	vehicle.getLiteAPI().vehicle().setACCHeadwayTime(vehicleId, headway);
}

void PlatooningService::setFixedAcceleration(int activate, double acceleration)
{
	Enter_Method("setFixedAcceleration");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	vehicle.getLiteAPI().vehicle().setFixedAcceleration(vehicleId, activate, acceleration);
}

void PlatooningService::setFixedLane(int laneIndex)
{
	Enter_Method("setFixedLane");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	vehicle.getLiteAPI().vehicle().setFixedLane(vehicleId, laneIndex);
}

void PlatooningService::getRadarMeasurements(double &distance, double &relativeSpeed)
{
	Enter_Method("getRadarMeasurements");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	vehicle.getLiteAPI().vehicle().getRadarMeasurements(vehicleId, distance, relativeSpeed);
}

bool PlatooningService::isCrashed()
{
	Enter_Method("isCrashed");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	return vehicle.getLiteAPI().vehicle().isCrashed(vehicleId);
}

bool PlatooningService::isCruiseControllerInstalled()
{
	Enter_Method("isCruiseControllerInstalled");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	return vehicle.getLiteAPI().vehicle().isCruiseControllerInstalled(vehicleId);
}

void PlatooningService::setLaneChangeAction(enum Plexe::PLATOONING_LANE_CHANGE_ACTION action)
{
	Enter_Method("setLaneChangeAction");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	vehicle.getLiteAPI().vehicle().setLaneChangeAction(vehicleId, action);
}

enum Plexe::PLATOONING_LANE_CHANGE_ACTION PlatooningService::getLaneChangeAction()
{
	Enter_Method("getLaneChangeAction");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	return static_cast<Plexe::PLATOONING_LANE_CHANGE_ACTION>(vehicle.getLiteAPI().vehicle().getLaneChangeAction(
			vehicleId));
}

void PlatooningService::setControllerFakeData(double frontDistance, double frontSpeed, double frontAcceleration,
		double leaderSpeed, double leaderAcceleration)
{
	Enter_Method("setControllerFakeData");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	vehicle.getLiteAPI().vehicle().setControllerFakeData(vehicleId, frontDistance, frontSpeed, frontAcceleration,
			leaderSpeed, leaderAcceleration);
}

double PlatooningService::getDistanceToRouteEnd()
{
	Enter_Method("getDistanceToRouteEnd");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	return vehicle.getLiteAPI().vehicle().getDistanceToRouteEnd(vehicleId);
}

double PlatooningService::getDistanceFromRouteBegin()
{
	Enter_Method("getDistanceFromRouteBegin");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	return vehicle.getLiteAPI().vehicle().getDistanceFromRouteBegin(vehicleId);
}

double PlatooningService::getACCAcceleration()
{
	Enter_Method("getACCAcceleration");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	return vehicle.getLiteAPI().vehicle().getACCAcceleration(vehicleId);
}

double PlatooningService::getCurrentSpeed()
{
	Enter_Method("getCurrentSpeed");
	return mVehicleDataProvider->speed().value();
}

libsumo::TraCIPosition PlatooningService::getPosition()
{
	Enter_Method("getCurrentPosition");
	auto& vehicle = getFacilities().get_const<traci::VehicleController>();
	return vehicle.getLiteAPI().vehicle().getPosition(vehicleId);
}

std::string PlatooningService::getVehicleId()
{
	return vehicleId;
}
