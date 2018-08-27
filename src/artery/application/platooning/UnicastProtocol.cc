//
// Copyright (c) 2012-2016 Michele Segata <segata@ccs-labs.org>
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

#include "artery/application/platooning/UnicastProtocol.h"

//to get recv power from lower layer
//#include "veins/base/phyLayer/PhyToMacControlInfo.h"
//#include "veins/modules/phy/DeciderResult80211.h"

Define_Module(UnicastProtocol);

void UnicastProtocol::initialize(int stage)
{

	PlatooningBaseModule::initialize(stage);

	if (stage == 0)
	{
		//by default we have acks
		enableAck = true;

		//get gates
		upperLayerIn = findGate("upperLayerIn");
		upperLayerOut = findGate("upperLayerOut");
		upperControlIn = findGate("upperControlIn");
		upperControlOut = findGate("upperControlOut");

		//get parameters
		queueSize = par("queueSize").longValue();
		maxAttempts = par("maxAttempts").longValue();
		ackTimeout = par("ackTimeout").doubleValue();

		//timeout message
		timeout = new cMessage("timeout");

		//sequence number
		sequenceNumber = 0;

		//at init time, no current messages are waiting for acks obviously
		currentMsg = 0;

		//packet loss rate
		packetLossRate = par("packetLossRate").doubleValue();

	}

}

void UnicastProtocol::handleUpperMsg(cMessage *msg)
{

	UnicastMessage *unicast = dynamic_cast<UnicastMessage *>(msg);
	ASSERT2(unicast, "received a message from app layer which is not of type UnicastMessage");

	//first, set timestamp of intended sending time
	unicast->setTimestamp();

	if (queueSize != 0 && queue.size() == queueSize)
	{
		//queue is full. cannot enqueue the packet
		UnicastProtocolControlMessage *queueFull = new UnicastProtocolControlMessage();
		queueFull->setControlCommand(FULL_QUEUE);
		send(queueFull, upperControlOut);
		delete msg;
		return;
	}

	if (unicast->getDestination() != -1) {
		//save packet into queue
		queue.push(unicast);
		//if the packet we just inserted is the only one in the queue
		//then tell the protocol to immediately process it
		if (queue.size() == 1)
		{
			processNextPacket();
		}
	}
	else {
		//send message down directly
		sendMessageDown(unicast->getDestination(), unicast->decapsulate(), unicast->getEncapsulationId(), unicast->getPriority(), unicast->getTimestamp(), unicast->getKind());
		delete unicast;
	}

}

void UnicastProtocol::handleUpperControl(cMessage *msg)
{

	UnicastProtocolControlMessage *controlMsg = dynamic_cast<UnicastProtocolControlMessage *>(msg);
	ASSERT2(controlMsg, "message coming from control gate is not a control message");

	switch (controlMsg->getControlCommand())
	{

		case SET_MAC_ADDRESS:
			macAddress = controlMsg->getCommandValue();
			break;

		case DISABLE_ACKS:
			enableAck = false;
			break;

		case ENABLE_ACKS:
			enableAck = true;
			break;

		default:
			ASSERT2(0, "control message contains unknown command");
			break;

	}

	delete msg;

}

void UnicastProtocol::sendMessageDown(int destination, cPacket *msg, int encapsulatedId, int priority, SimTime timestamp, short kind)
{

	//this function cannot be called if we are still waiting for the ack
	//for another packet. this unicast protocol is simple, nothing like TCP
	if (currentMsg != 0 && destination != -1) {
		EV_INFO << "Still waiting for ACK, and Message is no broadcast!\n";
		return;
	}

	UnicastMessage *unicast = new UnicastMessage("unicast");

	//set basic fields
	unicast->setSource(macAddress);
	unicast->setDestination(destination);
	//NOTICE: autoincrementing the sequence number
	if (destination == -1) {
		unicast->setSequenceNumber(-1);
	}
	else {
		unicast->setSequenceNumber(sequenceNumber++);
	}
	unicast->setType(DATA);
	//the length of source, destination and sequence number is set to 0 because
	//these are actually fields which are present at the mac layer. this unicast
	//protocol is just implementing the unicast mechanism which is still missing
	//in the 1609.4 MAC implementation. The size of this unicast message will then
	//be the size of the encapsulated message
	unicast->setByteLength(0);
	unicast->setPriority(priority);
	unicast->setTimestamp(timestamp);
	unicast->setKind(kind);
	unicast->setChannel(0);
	//encapsulate message. NOTICE that we are encapsulating the message directly
	//decapsulated from the message coming from the application. we could use
	//msg->dup(), but then, since msg has been decapsulated, we would have to
	//free it
	unicast->encapsulate(msg);

	sendDown(unicast);

	//TODO: check whether to leave this here or somewhere else
	//if we are sending a unicast packet, schedule ack timeout
	if (destination != -1)
	{
		EV_INFO << "send seq " << unicast->getSequenceNumber() << " to " << unicast->getDestination() << "\n";

		currentMsg = unicast->dup();
		nAttempts = 0;
		scheduleAt(simTime() + SimTime(ackTimeout), timeout);
	}

}

void UnicastProtocol::sendAck(const UnicastMessage *msg)
{

	UnicastMessage *unicast = new UnicastMessage("unicast");

	unicast->setSource(macAddress);
	unicast->setDestination(msg->getSource());
	unicast->setSequenceNumber(msg->getSequenceNumber());
	unicast->setByteLength(0);
	unicast->setPriority(0);
	unicast->setChannel(msg->getChannel());
	unicast->setType(ACK);

	EV_INFO << "send ACK for " << msg->getSequenceNumber() << "\n";

	sendDown(unicast);

}

void UnicastProtocol::resendMessage()
{
	EV_INFO << "resend attempt " << nAttempts << ": " << currentMsg->getSource() << " -> " << currentMsg->getDestination() << " with seq " << currentMsg->getSequenceNumber() << "\n";
	sendDown(currentMsg->dup());

	scheduleAt(simTime() + SimTime(ackTimeout + ackTimeout * nAttempts), timeout);
	nAttempts++;
}

void UnicastProtocol::handleUnicastMessage(const UnicastMessage *msg)
{

	ASSERT2(msg->getType() == DATA, "handleUnicastMessage cannot handle ACK frames");

	int destination = msg->getDestination();
	int source = msg->getSource();
	//-1 means never seen a message from the sender of the message
	int expectedSequenceNumber = -1;
	std::map<int, int>::iterator sequenceNumberIt;

	if (destination == macAddress)
	{

		//message is directed to this node
		EV_INFO << "message for node, seq " << msg->getSequenceNumber() << "\n";

		//first of all check whether this is a duplicate
		sequenceNumberIt = receiveSequenceNumbers.find(source);

		if (sequenceNumberIt != receiveSequenceNumbers.end())
		{
			expectedSequenceNumber = sequenceNumberIt->second;
		}

		if (msg->getSequenceNumber() >= expectedSequenceNumber)
		{
			EV_INFO << "expectedSequenceNumber, wanted " << expectedSequenceNumber << " or higher\n";
			//we have never seen this message, we have to send it up to to the application
			//notice that we do not decapsulate, because the upper layer may want to know
			//the sender address, so we just pass up the entire frame
			send(msg->dup(), upperLayerOut);

			//update next expected sequence number
			receiveSequenceNumbers[source] = msg->getSequenceNumber() + 1;
		}

		//if it is a new message or a duplicate, we have anyhow to send the ack
		if (enableAck)
		{
			sendAck(msg);
		}

	}
	else
	{
		if (destination == -1)
		{
			//message is broadcast. directed to this node but no need to ack
			send(msg->dup(), upperLayerOut);
			//update next expected sequence number
			receiveSequenceNumbers[source] = msg->getSequenceNumber() + 1;
		}
	}

}

void UnicastProtocol::handleAckMessage(const UnicastMessage *ack)
{

	ASSERT2(ack->getType() == ACK, "handleAckMessage cannot handle DATA frames");

	//if ack is not directed to this node, just drop it
	if (ack->getDestination() != macAddress)
	{
		EV_INFO << "got ACK for node " << ack->getDestination() << "\n";
		return;
	}

	if (currentMsg == 0)
	{
		//we have received an ack we were not waiting for. do nothing
		EV_INFO << "unexpected ACK from " << ack->getSource() << " to " << ack->getDestination() << " with seq " << ack->getSequenceNumber() << "\n";
	}
	else
	{

		int msgDestination, ackSource;
		int msgSequence, ackSequence;

		msgDestination = currentMsg->getDestination();
		ackSource = ack->getSource();
		msgSequence = currentMsg->getSequenceNumber();
		ackSequence = ack->getSequenceNumber();

//		ASSERT2(msgDestination == ackSource && msgSequence == ackSequence, "received a wrong ACK");
		EV_INFO << "got ACK for seq " << ack->getSequenceNumber() << ", current is " << msgSequence << "\n";

		//we've got the ack. stop timeout timer
		if (timeout->isScheduled())
			cancelEvent(timeout);
		//message has been correctly received by the destination. move on to next packet
		queue.pop();


		delete currentMsg;
		currentMsg = 0;
		nAttempts = 0;

		processNextPacket();

	}

}

void UnicastProtocol::handleLowerMsg(cMessage *msg)
{
	// get our unicast message out
	UnicastMessage *unicast = dynamic_cast<UnicastMessage *>(msg);
	ASSERT2(unicast, "no unicast message inside the WSM message");

	//TODO: pass up also received power information

	double r = dblrand();
	if (r < packetLossRate) {
		//discard the message
		delete unicast;
		return;
	}

	switch (unicast->getType())
	{
		case DATA:
			handleUnicastMessage(unicast);
			break;
		case ACK:
			handleAckMessage(unicast);
			break;
		default:
			ASSERT2(0, "unknown unicast message received");
			break;
	}

	delete unicast;

}

void UnicastProtocol::handleSelfMsg(cMessage *msg)
{

	if (msg == timeout)
	{

		//if we have a timeout, we should have a message to re-send
		ASSERT2(currentMsg != 0, "ack timeout occurred with no current message");

		if (nAttempts < maxAttempts)
		{
			//we try again to send
			resendMessage();
		}
		else
		{
			EV_DEBUG << "maxAttempts for packet " << currentMsg->getSource() << " to " << currentMsg->getDestination() << "(" << currentMsg->getSequenceNumber() << ")\n";
			//we tried maxAttempts time with no success. discard the
			//message and tell the error to the application
			UnicastProtocolControlMessage *sendError = new UnicastProtocolControlMessage("sendError");
			sendError->setControlCommand(SEND_FAIL);
			//include the message so that application knows which packet has been dropped
			sendError->encapsulate(currentMsg->dup());
			send(sendError, upperControlOut);

			//the packet that we unsuccessfully tried to send is no more needed. delete it
			queue.pop();
			delete currentMsg;
			currentMsg = 0;
			nAttempts = 0;

		}

	}

}

void UnicastProtocol::processNextPacket()
{

	//the queue is empty. no packet to process
	if (queue.empty())
	{
		return;
	}

	//get the message from the queue
	UnicastMessage *toSend = queue.front();

	//send message down
	sendMessageDown(toSend->getDestination(), toSend->decapsulate(), toSend->getEncapsulationId(), toSend->getPriority(), toSend->getTimestamp(), toSend->getKind());

	delete toSend;

}

void UnicastProtocol::publicHandleLowerMsg(cMessage *msg)
{
    Enter_Method("publicHandleLowerMsg");
    handleLowerMsg(msg->dup());
}

void UnicastProtocol::finish()
{
	if (timeout) {
		cancelAndDelete(timeout);
		timeout = 0;
	}
	PlatooningBaseModule::finish();
}

UnicastProtocol::UnicastProtocol()
{
	timeout = 0;
}

UnicastProtocol::~UnicastProtocol()
{
}
