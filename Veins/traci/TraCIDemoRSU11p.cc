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
    //std::cout << "here "<<simTime()<<"\n";
    //if this RSU receives a WSA for service 42, it will tune to the chan
    //if (wsa->getPsid() == 42) {
    //    mac->changeServiceChannel(wsa->getTargetChannel());
    //}
}

void TraCIDemoRSU11p::onWSM(WaveShortMessage* wsm) {
    //std::cout << "here 2 "<<simTime()<<"\n";
    //Veins::TraCIScenarioManager* manager = Veins::TraCIScenarioManagerAccess().get();
            //traci = manager->getCommandInterface();
    //double flow1 = traci->getLastStepVehicleNumber("flow1");
    //double flow2 = traci->getLastStepVehicleNumber("flow2");
    //double flow3 = traci->getLastStepVehicleNumber("flow3");
    //double flow4 = traci->getLastStepVehicleNumber("flow4");
    //double l1 = 1000, l2 = 800;
    //TraCICommandInterface::Road road1 = TraCICommandInterface::Road(traci, "edge1");
    //TraCICommandInterface::Road road2 = TraCICommandInterface::Road(traci, "edge2");
    //TraCICommandInterface::Road road3 = TraCICommandInterface::Road(traci, "edge3");
    //TraCICommandInterface::Road road4 = TraCICommandInterface::Road(traci, "edge4");

    //double t1 = road1.getCurrentTravelTime()+road2.getCurrentTravelTime();
    //double t2 = road3.getCurrentTravelTime()+road4.getCurrentTravelTime();
    //double x1t1 = (flow1+flow2)/l1;
    //double x2t2 = (flow3+flow4)/l2;
    //v.push_back(std::vector<double>({double(simTime().raw()/pow(10,12)), x1t1+x2t2}));

    //std::cout <<"RSU receives from node "<<wsm->getSenderAddress() <<": " << wsm->getWsmData()<<"\n";
    /*
    Veins::TraCIScenarioManager* manager = Veins::TraCIScenarioManagerAccess().get();
    traci = manager->getCommandInterface();
    int flow1 = traci->getLastStepVehicleNumber("flow1");
    int flow4 = traci->getLastStepVehicleNumber("flow4");
    //std::cout << "link 1 #vehicle: " << f1 <<"\n";
    //std::cout << "link 4 #vehicle: " << f4 <<"\n";
    TraCICommandInterface::Road road1 = TraCICommandInterface::Road(traci, "edge1");
    TraCICommandInterface::Road road2 = TraCICommandInterface::Road(traci, "edge2");
    TraCICommandInterface::Road road3 = TraCICommandInterface::Road(traci, "edge3");
    TraCICommandInterface::Road road4 = TraCICommandInterface::Road(traci, "edge4");
    int time1 = road1.getCurrentTravelTime() + road2.getCurrentTravelTime();
    int time4 = road4.getCurrentTravelTime() + road3.getCurrentTravelTime();
    //std::string str = "time" + std::to_string(time1) + " " + std::to_string(time4);
    std::string str = "flow" + std::to_string(flow1) + " " + std::to_string(flow4);
    WaveShortMessage* wsm1 = new WaveShortMessage();
    populateWSM(wsm1);
    //std::string str = "flowData 10 20";
    wsm1->setWsmData(str.c_str());
    //std::cout << "RSU sent "<<str<<"\n";
    
    //wsm->setSenderAddress(myId);
    //std::string str = "abc";
    //wsm->setWsmData(str.c_str());
    sendDelayedDown(wsm1->dup(), 2 + uniform(0.01,0.2));
     */
}

TraCIDemoRSU11p::~TraCIDemoRSU11p() {
    output.close();
    /*
std::cout << "REQMAP:\n";
    for(auto & i : reqMap) {
        std::cout << i.first << ": ";
        for(auto & j : i.second) std::cout << j.first << "("<<j.second<<")" <<", ";
        std::cout<<"\n";
    }
    */
    /*
std::cout << "HITMAP:\n";
    for(auto & i : BaseWaveApplLayer::hitMap) {
        std::cout << i.first << ": ";
        for(auto & j : i.second) std::cout << j << ", ";
        std::cout<<"\n";
    }
*/
    /*
    std::ofstream output("/Users/yuyangzheng/desktop/systemCost.txt");
    for(auto & i : v) {
        for(auto & j : i) output << j <<" ";
        output << "\n";
    }
    */
}
