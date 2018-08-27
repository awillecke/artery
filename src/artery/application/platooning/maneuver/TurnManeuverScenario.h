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

#ifndef TURNSCENARIO_H_
#define TURNSCENARIO_H_

#include "artery/application/platooning/scenarios/BaseScenario.h"
#include "artery/application/platooning/protocols/BaseProtocol.h"

#include "artery/application/platooning/messages/ManeuverMessage_m.h"

#include "artery/application/platooning/apps/BaseApp.h"
#include "artery/application/platooning/PlatooningService.h"

#include "inet/common/geometry/common/Coord.h"

class TurnManeuverScenario : public BaseScenario
{
	// taken from SUMOs MSVehicle.h
    enum Signalling {
        /// @brief Everything is switched off
        VEH_SIGNAL_NONE = 0,
        /// @brief Right blinker lights are switched on
        VEH_SIGNAL_BLINKER_RIGHT = 1,
        /// @brief Left blinker lights are switched on
        VEH_SIGNAL_BLINKER_LEFT = 2,
        /// @brief Blinker lights on both sides are switched on
        VEH_SIGNAL_BLINKER_EMERGENCY = 4,
        /// @brief The brake lights are on
        VEH_SIGNAL_BRAKELIGHT = 8,
        /// @brief The front lights are on (no visualisation)
        VEH_SIGNAL_FRONTLIGHT = 16,
        /// @brief The fog lights are on (no visualisation)
        VEH_SIGNAL_FOGLIGHT = 32,
        /// @brief The high beam lights are on (no visualisation)
        VEH_SIGNAL_HIGHBEAM = 64,
        /// @brief The backwards driving lights are on (no visualisation)
        VEH_SIGNAL_BACKDRIVE = 128,
        /// @brief The wipers are on
        VEH_SIGNAL_WIPER = 256,
        /// @brief One of the left doors is opened
        VEH_SIGNAL_DOOR_OPEN_LEFT = 512,
        /// @brief One of the right doors is opened
        VEH_SIGNAL_DOOR_OPEN_RIGHT = 1024,
        /// @brief A blue emergency light is on
        VEH_SIGNAL_EMERGENCY_BLUE = 2048,
        /// @brief A red emergency light is on
        VEH_SIGNAL_EMERGENCY_RED = 4096,
        /// @brief A yellow emergency light is on
        VEH_SIGNAL_EMERGENCY_YELLOW = 8192
    };

	public:

    	static const int MANEUVER_TYPE = 12348;

		virtual void initialize(int stage);
		virtual void finish();

	protected:
		//define the roles
		enum JOIN_ROLE {LEADER, FOLLOWER, JOINER, NONE};

		struct VEHICLE_DATA {
			double				speed;		//speed of the platoon
			int					joinLane;	//the lane chosen for joining the platoon
			int					joinerId;	//the id of the vehicle joining the platoon
			std::vector<int>	formation;	//list of vehicles in the platoon
		};
		//define the states for each role
		typedef enum _LEADER_STATES {
			LS_INIT = 0,
			LS_LEADING = FSM_Steady(1),
			LS_WAIT_JOINER_IN_POSITION = FSM_Steady(2),
			LS_WAIT_JOINER_TO_JOIN = FSM_Steady(3)
		} LEADER_STATES;
		typedef enum _JOINER_STATES {
			JS_INIT = 0,
			JS_IDLE = FSM_Steady(1),
			JS_WAIT_OFFER = FSM_Steady(2),
			JS_WAIT_REPLY = FSM_Steady(3),
			JS_MOVE_IN_POSITION = FSM_Steady(4),
			JS_WAIT_JOIN = FSM_Steady(5),
			JS_JOIN_PLATOON = FSM_Steady(6),
			JS_FOLLOW = FSM_Steady(7)
		} JOINER_STATES;
		typedef enum _FOLLOWER_STATES {
			FS_INIT = 0,
			FS_FOLLOW = FSM_Steady(1)
		} FOLLOWER_STATES;
		//define the messages that can be sent by each role
		enum LEADER_MSGS {
			LM_OFFER_PLATOON = 0,
			LM_MOVE_IN_POSITION = 1,
			LM_JOIN_PLATOON = 2,
			LM_UPDATE_FORMATION = 3
		};
		enum JOINER_MSGS {
			JM_DISCOVER_PLATOON = 4,
			JM_REQUEST_JOIN = 5,
			JM_IN_POSITION = 6,
			JM_IN_PLATOON = 7
		};
		//the state machine handler
		cFSM leaderFsm, joinerFsm, followerFsm;
		//the role of this vehicle
		JOIN_ROLE role;
		//the position of this vehicle in the platoon
		int position;
		//data known by the vehicle
		struct VEHICLE_DATA vehicleData;

		cMessage *startManeuverMessage;
		cMessage *startDiscoveryMessage;
		double startDiscoveryInterval;

		cMessage *checkSignalMessage;
		double checkSignalInterval;

		BaseApp *appl;
		//pointer to protocol
		BaseProtocol *protocol;

	public:
		TurnManeuverScenario() {
			appl = 0;
		}

	protected:

		virtual void handleSelfMsg(cMessage *msg);
		virtual void handleLowerMsg(cMessage *msg);

		ManeuverMessage *generateMessage();
		void startManeuver(cMessage *msg);
		void sendUnicast(cPacket *msg, int destination);

		void handleLeaderMsg(cMessage *msg);
		void handleJoinerMsg(cMessage *msg);
		void handleFollowerMsg(cMessage *msg);

		void checkSignal();

};

#endif
