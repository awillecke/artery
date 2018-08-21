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

package artery.application.platooning;

import src.artery.application.platooning.UnicastProtocol;
import src.artery.application.platooning.protocols.BaseProtocol;
import src.artery.application.platooning.utilities.BasePositionHelper;

module PlatooningService like artery.application.ItsG5Service
{
    parameters:
        @class(PlatooningService);
        @signal[PlatooningUnicastReceived](type=cObject);
        string scenario_type;
        string helper_type;
        string appl_type;
        string protocol_type;

        @display("i=block/app2;bgb=214.88,131.92");


    gates:
        input serviceIn;
        output serviceOut;
    submodules:

        helper: <helper_type> like artery.application.platooning.utilities.BasePositionHelper {
            @display("p=174.76,27.2");
        }

        scenario: <scenario_type> like artery.application.platooning.scenarios.BaseScenario {
            @display("p=104.72,27.2");
        }

        appl: <appl_type> like artery.application.platooning.apps.BaseApp {
            @display("p=34.68,27.2");
        }

        prot: <protocol_type> like artery.application.platooning.protocols.BaseProtocol {
            @display("p=104.72,90.44");
        }

        unicast: artery.application.platooning.UnicastProtocol {
            @display("p=35.28,90.72");
        }
    connections allowunconnected:
//		upperLayer <--> unicast.lowerLayer;

        unicast.upperControlIn <-- prot.lowerControlOut;
        unicast.upperControlOut --> prot.lowerControlIn;

        unicast.upperLayerIn <-- prot.lowerLayerOut;
        unicast.upperLayerOut --> prot.lowerLayerIn;

        serviceOut <-- unicast.lowerLayerOut;
        serviceIn --> unicast.lowerLayerIn;

}
