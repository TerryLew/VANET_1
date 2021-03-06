//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "veins/modules/application/traci/TraCIDemoRSU11p.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
//using Veins::TraCIMobility;
//using Veins::TraCIMobilityAccess;

Define_Module(TraCIDemoRSU11p);

void TraCIDemoRSU11p::onWSA(WaveServiceAdvertisment* wsa) {
}

void TraCIDemoRSU11p::onWSM(WaveShortMessage* wsm) {
}

TraCIDemoRSU11p::~TraCIDemoRSU11p() {
    output.close();
}
