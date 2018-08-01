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

#ifndef PLATOONING_BASE_MODULE_H
#define PLATOONING_BASE_MODULE_H

#include <omnetpp.h>
#include <omnetpp/csimplemodule.h>
#include <assert.h>

using namespace omnetpp;


/**
*  @brief This module provides functions and attributes which where provided
*  by the BaseApplLayer and IBaseApplLayer classes, which Plexe modules
*  inherited from.
*/
class PlatooningBaseModule : public cSimpleModule
{

	protected:

		/** @name gate ids*/
		/*@{*/
		int upperLayerIn;
		int upperLayerOut;
		int lowerLayerIn;
		int lowerLayerOut;
		int upperControlIn;
		int upperControlOut;
		int lowerControlIn;
		int lowerControlOut;
		/*@}*/

	public:
		PlatooningBaseModule()
		{
		}

		/** @brief Initialization of the module and some variables*/
		virtual void initialize(int);

		void handleMessage(cMessage*) override;

		int numInitStages() const override;

		void finish();

	protected:
		/**
		 * @name Handle Messages
		 * @brief Functions to redefine by the programmer
		 *
		 * These are the functions provided to add own functionality to your
		 * modules. These functions are called whenever a self message or a
		 * data message from the upper or lower layer arrives respectively.
		 *
		 **/
		/*@{*/

		/**
		 * @brief Handle self messages such as timer...
		 *
		 * Define this function if you want to process timer or other kinds
		 * of self messages
		 **/
		virtual void handleSelfMsg(cMessage* msg)
		{
			EV << "PlatooningBaseModule: handleSelfMsg not redefined; delete msg\n";
			delete msg;
		}
		;

		/**
		 * @brief Handle messages from lower layer
		 *
		 * Redefine this function if you want to process messages from lower
		 * layers.
		 *
		 * The basic application layer just silently deletes all messages it
		 * receives.
		 **/
		virtual void handleLowerMsg(cMessage* msg)
		{
			EV << "PlatooningBaseModule: handleLowerMsg not redefined; delete msg\n";
			delete msg;
		}
		;

		/**
		 * @brief Handle control messages from lower layer
		 *
		 * The basic application layer just silently deletes all messages it
		 * receives.
		 **/
		virtual void handleLowerControl(cMessage* msg)
		{
			EV << "PlatooningBaseModule: handleLowerControl not redefined; delete msg\n";
			delete msg;
		}
		;

		/** @brief Handle messages from upper layer
		 *
		 * This function is pure virtual here, because there is no
		 * reasonable guess what to do with it by default.
		 */
		virtual void handleUpperMsg(cMessage *msg)
		{
			EV << "PlatooningBaseModule: handleUpperMsg not redefined; delete msg\n";
			delete msg;
		}

		/** @brief Handle control messages from upper layer */
		virtual void handleUpperControl(cMessage *msg)
		{
			EV << "PlatooningBaseModule: handleUpperControl not redefined; delete msg\n";
			delete msg;
		}

		/**
		 * @name Convenience Functions
		 * @brief Functions for convenience - NOT to be modified
		 *
		 * These are functions taking care of message encapsulation and
		 * message sending. Normally you should not need to alter these.
		 *
		 * All these functions assume that YOU do all the necessary handling
		 * of control information etc. before you use them.
		 **/
		/*@{*/

		/** @brief Sends a message to the lower layer
		 *
		 * Short hand for send(msg, lowerLayerOut);
		 *
		 * You have to take care of encapsulation We recommend that you
		 * use a pair of functions called encapsMsg/decapsMsg.
		 */
		void sendDown(cMessage *msg);

		/** @brief Sends a message to the upper layer
		 *
		 * Short hand for send(msg, upperLayerOut);
		 * You have to take care of decapsulation and deletion of
		 * superflous frames. We recommend that you use a pair of
		 * functions decapsMsg/encapsMsg.
		 */
		void sendUp(cMessage *msg);

		/** @brief Sends a control message to an upper layer */
		void sendControlUp(cMessage *msg);

		/** @brief Sends a control message to a lower layer */
		void sendControlDown(cMessage *msg);
};

#endif
