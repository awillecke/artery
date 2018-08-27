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

#include "artery/application/platooning/maneuver/TurnManeuverScenario.h"

Define_Module(TurnManeuverScenario);

void TurnManeuverScenario::initialize(int stage) {

	BaseScenario::initialize(stage);

	if (stage == 0)

		//name the FSMs
		leaderFsm.setName("leaderFsm");
		followerFsm.setName("followerFsm");
		joinerFsm.setName("joinerFsm");

		//get pointer to application
		appl = FindModule<BaseApp*>::findSubModule(getParentModule());

		startManeuverMessage = new cMessage("startManeuverMessage");
		checkSignalMessage = new cMessage("checkSignalMessage");
		startDiscoveryMessage = new cMessage("startDiscoveryMessage");
		startDiscoveryInterval = par("startDiscoveryInterval").doubleValue();

		checkSignalInterval = par("checkSignalInterval").doubleValue();
		scheduleAt(simTime() + checkSignalInterval, checkSignalMessage);

	if (stage == 1) {
		// after init, each vehicle is in its own platoon an a leader by default
		service->setActiveController(Plexe::DRIVER);
		role = NONE;

		protocol = FindModule<BaseProtocol*>::findSubModule(getParentModule());

		//connect maneuver application to protocol
		protocol->registerApplication(MANEUVER_TYPE, gate("lowerLayerIn"), gate("lowerLayerOut"));

	}

}

void TurnManeuverScenario::finish() {
	BaseScenario::finish();
}

ManeuverMessage *TurnManeuverScenario::generateMessage() {
	ManeuverMessage *msg = new ManeuverMessage();
	msg->setVehicleId(positionHelper->getId());
	msg->setPlatoonId(positionHelper->getPlatoonId());
	msg->setPlatoonLane(service->getLaneIndex());
	msg->setPlatoonSpeed(vehicleData.speed);
	return msg;
}

void TurnManeuverScenario::handleSelfMsg(cMessage *msg) {
	BaseScenario::handleSelfMsg(msg);

	if (msg == checkSignalMessage) {
		checkSignal();
	}
	else if (msg == startManeuverMessage) {
		startManeuver(msg);
	}
	else if (msg == startDiscoveryMessage) {
		handleJoinerMsg(msg);
	}
}

void TurnManeuverScenario::startManeuver(cMessage *msg) {
	double distance;
	double relativeSpeed;

	service->getRadarMeasurements(distance, relativeSpeed);
	EV_INFO << "Dist: " << distance << " relSpeed: " << relativeSpeed << "\n";

	if (distance == -1) {
		// first vehicle at first position -> new leader
		role = LEADER;

		positionHelper->setLeaderId(positionHelper->getId());
		positionHelper->setIsLeader(true);
		positionHelper->setPlatoonLane(-1);
		positionHelper->setPlatoonId(positionHelper->getId());

		vehicleData.joinerId = -1;
		vehicleData.joinLane = service->getLaneIndex();
		vehicleData.speed = service->getCurrentSpeed();
		vehicleData.formation.push_back(positionHelper->getId());

		handleLeaderMsg(new cMessage("initFSM"));

		EV_INFO << "Leader waiting for discovers\n";
	}
	else {
		role = JOINER;

		//we are also interested in receiving beacons: the joiner must compute
		//its distance to the front vehicle while approaching it
		protocol->registerApplication(BaseProtocol::BEACON_TYPE, gate("lowerLayerIn"), gate("lowerLayerOut"));

		handleJoinerMsg(new cMessage("initFSM"));

		scheduleAt(simTime(), startDiscoveryMessage);
		EV_INFO << "Joiner sending discovery\n";
	}

}

void TurnManeuverScenario::handleLowerMsg(cMessage *msg) {
	switch (role) {
		case LEADER:
			handleLeaderMsg(msg);
			break;
		case FOLLOWER:
			handleFollowerMsg(msg);
			break;
		case JOINER:
			handleJoinerMsg(msg);
			break;
		default:
//			ASSERT(false);
			delete msg;
			break;
	};

}

void TurnManeuverScenario::sendUnicast(cPacket *msg, int destination) {
	UnicastMessage *unicast = new UnicastMessage("", MANEUVER_TYPE);
	unicast->setDestination(destination);
	unicast->setChannel(0);
	unicast->encapsulate(msg);
	sendDown(unicast);
}

void TurnManeuverScenario::handleLeaderMsg(cMessage *msg) {

	//this message can be a self message, or a unicast message
	//with an encapsulated beacon or maneuver message
	ManeuverMessage *maneuver = 0;
	cPacket *encapsulated = 0;
	//maneuver message to be sent, if needed
	ManeuverMessage *toSend;

	//first check if this is a unicast message, and in case if it is a beacon or a maneuver
	UnicastMessage *unicast = dynamic_cast<UnicastMessage *>(msg);
	if (unicast) {
		encapsulated = unicast->decapsulate();
		maneuver = dynamic_cast<ManeuverMessage *>(encapsulated);
	}

	//check current leader status
	FSM_Switch(leaderFsm) {
		case FSM_Exit(LS_INIT): {
			FSM_Goto(leaderFsm, LS_LEADING);
			break;
		}
		case FSM_Exit(LS_LEADING): {
			// discover for platoon leaders don't have a valid ID
			if (maneuver && maneuver->getPlatoonId() == INVALID_PLATOON_ID) {
				EV_INFO << "Got unicast with invalid INVALID_PLATOON_ID from " << maneuver->getVehicleId() << "\n";
				// message is a discovery
				if (maneuver->getMessageType() == JM_DISCOVER_PLATOON) {
					EV_INFO << "Got JM_DISCOVER_PLATOON\n";
					// joiner is on same lane
					if (maneuver->getPlatoonLane() == service->getLaneIndex()) {
						toSend = generateMessage();
						toSend->setMessageType(LM_OFFER_PLATOON);
						//send platoon info to the joiner
						sendUnicast(toSend, maneuver->getVehicleId());
						EV_INFO << "Joiner is on same lane -> replying with LM_OFFER_PLATOON\n";
						FSM_Goto(leaderFsm, LS_LEADING);
					}
				}
			}
			//when getting a message, and being in the LEADING state, we need
			//to check if this is a join request. if not just ignore it
			else if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()) {
				if (maneuver->getMessageType() == JM_REQUEST_JOIN) {
					EV_INFO << "JM_REQUEST_JOIN from " << maneuver->getVehicleId() <<"\n";
					toSend = generateMessage();
					toSend->setMessageType(LM_MOVE_IN_POSITION);
					//this will be the front vehicle for the car which will join
					toSend->setFrontVehicleId(vehicleData.formation.back());
					//save some data. who is joining?
					vehicleData.joinerId = maneuver->getVehicleId();
					//send a positive ack to the joiner
					sendUnicast(toSend, vehicleData.joinerId);
					EV_INFO << "Sending LM_MOVE_IN_POSITION\n";

					FSM_Goto(leaderFsm, LS_WAIT_JOINER_IN_POSITION);
				}
			}
			else {
				EV_INFO << "Received PlatoonId " << maneuver->getPlatoonId() << " but expected " << positionHelper->getPlatoonId() << "\n";
			}
			break;
		}
		case FSM_Exit(LS_WAIT_JOINER_IN_POSITION): {

			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()) {
				//the joiner is now in position and is ready to join
				if (maneuver->getMessageType() == JM_IN_POSITION) {

					//tell him to join the platoon
					toSend = generateMessage();
					toSend->setMessageType(LM_JOIN_PLATOON);
					sendUnicast(toSend, vehicleData.joinerId);

					FSM_Goto(leaderFsm, LS_WAIT_JOINER_TO_JOIN);

				}
			}

			break;
		}
		case FSM_Exit(LS_WAIT_JOINER_TO_JOIN): {

			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()) {
				//the joiner has joined the platoon
				if (maneuver->getMessageType() == JM_IN_PLATOON) {
					//add the joiner to the list of vehicles in the platoon
					vehicleData.formation.push_back(vehicleData.joinerId);
					toSend = generateMessage();
					toSend->setMessageType(LM_UPDATE_FORMATION);
					toSend->setPlatoonFormationArraySize(vehicleData.formation.size());
					for (unsigned int i = 0; i < vehicleData.formation.size(); i++) {
						toSend->setPlatoonFormation(i, vehicleData.formation[i]);
					}
					//send to all vehicles
					sendUnicast(toSend, -1);

					positionHelper->setPlatoonSize(vehicleData.formation.size());

					EV_INFO << "positionHelper:\n"
							<< "platoonId: " << positionHelper->getPlatoonId() << "\n"
							<< "platoonLane: " << positionHelper->getPlatoonLane() << "\n"
							<< "platoonSize: " << positionHelper->getPlatoonSize() << "\n"
							<< "highestId: " << positionHelper->getHighestId() << "\n"
							<< "isLeader: " << positionHelper->isLeader() << "\n"
							<< "nLanes: " << positionHelper->getLanesCount() << "\n"
							<< "frontId: " << positionHelper->getFrontId() << "\n";


					FSM_Goto(leaderFsm, LS_LEADING);

				}

			}

			break;
		}

	}

	if (encapsulated) {
		delete encapsulated;
	}
	if (unicast) {
		delete unicast;
	}

}

void TurnManeuverScenario::handleJoinerMsg(cMessage *msg) {

	//this message can be a self message, or a unicast message
	//with an encapsulated beacon or maneuver message
	ManeuverMessage *maneuver = 0;
	PlatooningBeacon *beacon = 0;
	cPacket *encapsulated = 0;
	//maneuver message to be sent, if needed
	ManeuverMessage *toSend;

	//first check if this is a unicast message, and in case if it is a beacon or a maneuver
	UnicastMessage *unicast = dynamic_cast<UnicastMessage *>(msg);
	if (unicast) {
		encapsulated = unicast->decapsulate();
		maneuver = dynamic_cast<ManeuverMessage *>(encapsulated);
		beacon = dynamic_cast<PlatooningBeacon *>(encapsulated);
	}

	EV_INFO << "joinerFsm: " << joinerFsm.str() << "\n";

	//check current joiner status
	FSM_Switch(joinerFsm) {

		//init state, just move to the idle state
		case FSM_Exit(JS_INIT): {
			FSM_Goto(joinerFsm, JS_IDLE);
			break;
		}

		case FSM_Exit(JS_IDLE): {
			//if this is a self message triggering the beginning of procedure, then ask for leaders via broadcast
			EV_INFO << "JS_IDLE: msg = " << msg->str() << "\n";
			if (msg == startDiscoveryMessage) {
				toSend = generateMessage();
				toSend->setPlatoonId(INVALID_PLATOON_ID);
				toSend->setPlatoonLane(service->getLaneIndex());
				toSend->setMessageType(JM_DISCOVER_PLATOON);
				EV_INFO << "Sending JM_DISCOVER_PLATOON via broadcast\n";
				sendUnicast(toSend, -1);
				scheduleAt(simTime() + startDiscoveryInterval, startDiscoveryMessage);
				FSM_Goto(joinerFsm, JS_WAIT_OFFER);
			}
			break;
		}

		case FSM_Exit(JS_WAIT_OFFER): {
			//a leader offered a platoon, ask for joining
			if (maneuver && maneuver->getMessageType() == LM_OFFER_PLATOON) {

				if (maneuver->getPlatoonLane() == service->getLaneIndex()) {
					EV_INFO << "Got LM_OFFER_PLATOON for same lane from " << maneuver->getVehicleId() << "\n";
					//cancel timer
					if (startDiscoveryMessage->isScheduled())
						cancelEvent(startDiscoveryMessage);

					// set leader data in position helper
					int leader = maneuver->getVehicleId();
					positionHelper->setLeaderId(leader);
					positionHelper->setPlatoonId(leader);
					positionHelper->setFrontId(-1);
					positionHelper->setIsLeader(false);
					positionHelper->setPlatoonLane(-1);

					// send join request
					toSend = generateMessage();
					toSend->setMessageType(JM_REQUEST_JOIN);
					sendUnicast(toSend, positionHelper->getLeaderId());
					EV_INFO << "Sending JM_REQUEST_JOIN\n";
					FSM_Goto(joinerFsm, JS_WAIT_REPLY);
				}
			}
			if (msg == startDiscoveryMessage) {
				FSM_Goto(joinerFsm, JS_IDLE);
				scheduleAt(simTime(), startDiscoveryMessage);
			}
			break;
		}

		case FSM_Exit(JS_WAIT_REPLY): {

			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()) {

				//if the leader told us to move in position, we can start approaching the platoon
				if (maneuver->getMessageType() == LM_MOVE_IN_POSITION) {
					//save some data about the platoon
					positionHelper->setFrontId(maneuver->getFrontVehicleId());
					positionHelper->setPlatoonLane(maneuver->getPlatoonLane());
					positionHelper->setPlatoonSize(maneuver->getPlatoonFormationArraySize());
					vehicleData.joinLane = maneuver->getPlatoonLane();

					FSM_Goto(joinerFsm, JS_MOVE_IN_POSITION);
				}

			}
			break;
		}

		case FSM_Exit(JS_MOVE_IN_POSITION): {

			//if we get data, just feed the fake CACC
			if (beacon && beacon->getVehicleId() == positionHelper->getFrontId()) {
				//get front vehicle position
				inet::Coord frontPosition(beacon->getPositionX(), beacon->getPositionY(), 0);
				//get my position
				libsumo::TraCIPosition traciPosition = service->getPosition();
				inet::Coord position(traciPosition.x, traciPosition.y);
				//compute distance (-4 because of vehicle length)
				double distance = position.distance(frontPosition) - 4;
				//if we are in position, tell the leader about that
				EV_INFO << "dist" << distance<< " to front " << positionHelper->getFrontId() << "\n";
				if (distance < 1.3) {
					toSend = generateMessage();
					toSend->setMessageType(JM_IN_POSITION);
					sendUnicast(toSend, positionHelper->getLeaderId());
					FSM_Goto(joinerFsm, JS_WAIT_JOIN);
				}
			}
			break;
		}

		case FSM_Exit(JS_WAIT_JOIN): {

			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()) {

				//if we get confirmation from the leader, switch from faked CACC to real CACC
				if (maneuver->getMessageType() == LM_JOIN_PLATOON) {
					service->setActiveController(Plexe::CACC);
					service->setCACCConstantSpacing(6);
					service->setACCHeadwayTime(1.2);

				}


				EV_INFO << "positionHelper:\n"
						<< "platoonId: " << positionHelper->getPlatoonId() << "\n"
						<< "platoonLane: " << positionHelper->getPlatoonLane() << "\n"
						<< "platoonSize: " << positionHelper->getPlatoonSize() << "\n"
						<< "highestId: " << positionHelper->getHighestId() << "\n"
						<< "isLeader: " << positionHelper->isLeader() << "\n"
						<< "nLanes: " << positionHelper->getLanesCount() << "\n"
						<< "frontId: " << positionHelper->getFrontId() << "\n";

				//tell the leader that we're now in the platoon
				toSend = generateMessage();
				toSend->setMessageType(JM_IN_PLATOON);
				sendUnicast(toSend, positionHelper->getLeaderId());
				FSM_Goto(joinerFsm, JS_FOLLOW);

			}

			break;

		}

		case FSM_Exit(JS_FOLLOW): {

			//we're now following. if we get an update of the formation, change it accordingly
			role = FOLLOWER;
			handleFollowerMsg(new cMessage("initFSM"));

			FSM_Goto(joinerFsm, JS_FOLLOW);

			break;
		}

	}

	if (encapsulated) {
		delete encapsulated;
	}
	if (unicast) {
		delete unicast;
	}

}

void TurnManeuverScenario::handleFollowerMsg(cMessage *msg) {

	//this message can be a self message, or a unicast message
	//with an encapsulated beacon or maneuver message
	ManeuverMessage *maneuver = 0;
	cPacket *encapsulated = 0;

	//first check if this is a unicast message, and in case if it is a beacon or a maneuver
	UnicastMessage *unicast = dynamic_cast<UnicastMessage *>(msg);
	if (unicast) {
		encapsulated = unicast->decapsulate();
		maneuver = dynamic_cast<ManeuverMessage *>(encapsulated);
	}

	//check current follower status
	FSM_Switch(followerFsm) {

		case FSM_Exit(FS_INIT): {
			FSM_Goto(followerFsm, FS_FOLLOW);
			break;
		}

		case FSM_Exit(FS_FOLLOW): {

			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()) {
				if (maneuver->getMessageType() == LM_UPDATE_FORMATION) {
					vehicleData.formation.clear();
					for (unsigned int i = 0; i < maneuver->getPlatoonFormationArraySize(); i++) {
						vehicleData.formation.push_back(maneuver->getPlatoonFormation(i));
					}

					positionHelper->setPlatoonSize(maneuver->getPlatoonFormationArraySize());

					EV_INFO << "positionHelper:\n"
							<< "platoonId: " << positionHelper->getPlatoonId() << "\n"
							<< "platoonLane: " << positionHelper->getPlatoonLane() << "\n"
							<< "platoonSize: " << positionHelper->getPlatoonSize() << "\n"
							<< "highestId: " << positionHelper->getHighestId() << "\n"
							<< "isLeader: " << positionHelper->isLeader() << "\n"
							<< "nLanes: " << positionHelper->getLanesCount() << "\n"
							<< "frontId: " << positionHelper->getFrontId() << "\n";
				}
			}

			break;
		}

	}

	if (encapsulated) {
		delete encapsulated;
	}
	if (unicast) {
		delete unicast;
	}

}

void TurnManeuverScenario::checkSignal() {

	if (service->getCurrentSpeed() <= 0.01) {

		EV_INFO << "Vehicle halted with " << service->getCurrentSpeed() << "\n";
		int signals = service->getSignalStates();

		if ((signals & VEH_SIGNAL_BLINKER_LEFT) || (signals & VEH_SIGNAL_BLINKER_RIGHT)) {
			EV_INFO << "Vehicle wants to turn" << signals << "\n";
			scheduleAt(simTime() + checkSignalInterval, startManeuverMessage);
			return;
		}
	}

	scheduleAt(simTime() + checkSignalInterval, checkSignalMessage);
}
