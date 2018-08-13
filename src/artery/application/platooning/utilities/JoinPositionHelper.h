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

#ifndef JOINPOSITIONHELPER_H_
#define JOINPOSITIONHELPER_H_

#include "artery/application/platooning/utilities/BasePositionHelper.h"

class JoinPositionHelper : public BasePositionHelper
{

	public:

		virtual void initialize(int stage);
		virtual void finish();

		virtual bool isInSamePlatoon(int vehicleId);

	public:

		static int getIdFromExternalId(std::string externalId);

		JoinPositionHelper() : BasePositionHelper() {
		}

};

#endif
