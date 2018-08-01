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

#include "artery/application/platooning/utilities/PlatooningBaseModule.h"

Define_Module (PlatooningBaseModule);

void PlatooningBaseModule::initialize(int stage)
{
	if (stage == 0) {
		upperLayerIn = findGate("upperLayerIn");
		upperLayerOut = findGate("upperLayerOut");
		lowerLayerIn = findGate("lowerLayerIn");
		lowerLayerOut = findGate("lowerLayerOut");
		upperControlIn = findGate("upperControlIn");
		upperControlOut = findGate("upperControlOut");
		lowerControlIn = findGate("lowerControlIn");
		lowerControlOut = findGate("lowerControlOut");
	}
}

void PlatooningBaseModule::handleMessage(cMessage* msg)
{

	/* imported handler from BaseLayer class */
	if (msg->isSelfMessage()) {
		handleSelfMsg(msg);
	}
	else if (msg->getArrivalGateId() == upperLayerIn) {
		//recordPacket(PassedMessage::INCOMING,PassedMessage::UPPER_DATA,msg);
		handleUpperMsg(msg);
	}
	else if (msg->getArrivalGateId() == upperControlIn) {
		//recordPacket(PassedMessage::INCOMING,PassedMessage::UPPER_CONTROL,msg);
		handleUpperControl(msg);
	}
	else if (msg->getArrivalGateId() == lowerControlIn) {
		//recordPacket(PassedMessage::INCOMING,PassedMessage::LOWER_CONTROL,msg);
		handleLowerControl(msg);
	}
	else if (msg->getArrivalGateId() == lowerLayerIn) {
		//recordPacket(PassedMessage::INCOMING,PassedMessage::LOWER_DATA,msg);
		handleLowerMsg(msg);
	}
	else if (msg->getArrivalGateId() == -1) {
		/* Classes extending this class may not use all the gates, f.e.
		 * BaseApplLayer has no upper gates. In this case all upper gate-
		 * handles are initialized to -1. When getArrivalGateId() equals -1,
		 * it would be wrong to forward the message to one of these gates,
		 * as they actually don't exist, so raise an error instead.
		 */
		throw cRuntimeError("No self message and no gateID?? Check configuration.");
	}
	else {
		/* msg->getArrivalGateId() should be valid, but it isn't recognized
		 * here. This could signal the case that this class is extended
		 * with extra gates, but handleMessage() isn't overridden to
		 * check for the new gate(s).
		 */
		throw cRuntimeError("Unknown gateID?? Check configuration or override handleMessage().");
	}
}

void PlatooningBaseModule::sendDown(cMessage *msg)
{
//    recordPacket(PassedMessage::OUTGOING,PassedMessage::LOWER_DATA,msg);
	send(msg, lowerLayerOut);
}

void PlatooningBaseModule::sendUp(cMessage *msg)
{
//    recordPacket(PassedMessage::OUTGOING,PassedMessage::UPPER_DATA,msg);
	send(msg, upperLayerOut);
}

void PlatooningBaseModule::sendControlUp(cMessage *msg)
{
//    recordPacket(PassedMessage::OUTGOING,PassedMessage::UPPER_CONTROL,msg);
	if (gate(upperControlOut)->isPathOK())
		send(msg, upperControlOut);
	else {
		EV << "BaseLayer: upperControlOut is not connected; dropping message" << std::endl;
		delete msg;
	}
}

void PlatooningBaseModule::sendControlDown(cMessage *msg)
{
//    recordPacket(PassedMessage::OUTGOING,PassedMessage::LOWER_CONTROL,msg);
	if (gate(lowerControlOut)->isPathOK())
		send(msg, lowerControlOut);
	else {
		EV << "BaseLayer: lowerControlOut is not connected; dropping message" << std::endl;
		delete msg;
	}
}

int PlatooningBaseModule::numInitStages() const
{
	return 2;
}

void PlatooningBaseModule::finish()
{

}
