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

#ifndef TraCIDemoRSU11p_H
#define TraCIDemoRSU11p_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIConstants.h"
#include <unordered_map>

/**
 * Small RSU Demo using 11p
 */
class TraCIDemoRSU11p : public BaseWaveApplLayer {
	protected:
		virtual void onWSM(WaveShortMessage* wsm);
		virtual void onWSA(WaveServiceAdvertisment* wsa);
		std::vector<std::vector<double> > v;
    //std::unordered_map<std::string, int> flow={{"edge1",0},{"edge4",0}};
    //int cnt = 0;
	public:
		~TraCIDemoRSU11p();
};

#endif