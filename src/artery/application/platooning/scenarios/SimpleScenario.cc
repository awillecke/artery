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

#include "artery/application/platooning/scenarios/SimpleScenario.h"

Define_Module(SimpleScenario);

void SimpleScenario::initialize(int stage) {

	BaseScenario::initialize(stage);

	if (stage == 0)
		//get pointer to application
		appl = FindModule<BaseApp*>::findSubModule(getParentModule());

	if (stage == 1) {

		EV_INFO << "Cruise Controller available: " << std::boolalpha << service->isCruiseControllerInstalled();
		if (service->isCruiseControllerInstalled()) {
			EV_INFO << ", using " << service->getActiveController() << "\n";
		}
		EV_INFO << "\n";

		//set the active controller
		if (positionHelper->isLeader()) {
			service->setActiveController(Plexe::DRIVER);
			service->setACCHeadwayTime(leaderHeadway);
		}

	}

}

void SimpleScenario::finish() {
	BaseScenario::finish();
}

void SimpleScenario::handleSelfMsg(cMessage *msg) {
	BaseScenario::handleSelfMsg(msg);
}
