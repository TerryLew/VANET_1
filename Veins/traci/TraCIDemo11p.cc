//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
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

#include "veins/modules/application/traci/TraCIDemo11p.h"
#include <vector>
#include <math.h>

Define_Module(TraCIDemo11p);

std::unordered_set<int> TraCIDemo11p::hostSt;

void TraCIDemo11p::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    throughputSignal = registerSignal("throughput");
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
    }
    hostSt.insert(myId);
    
    Veins::TraCIScenarioManager* manager = Veins::TraCIScenarioManagerAccess().get();
        traci = manager->getCommandInterface();

        double flow1 = traci->getLastStepVehicleNumber("flow1a")+traci->getLastStepVehicleNumber("flow1b");
        double flow2 = traci->getLastStepVehicleNumber("flow2");

        TraCICommandInterface::Road road1 = TraCICommandInterface::Road(traci, "edge1");
        TraCICommandInterface::Road road2 = TraCICommandInterface::Road(traci, "edge2");

        double rate = 6000.0, l1 = 1000, l2 = 500, v_max = 15.65, cap1 = 1500, cap2 = 1500, alpha=0.2, beta=10, lane1 = 2, lane2 = 1;
        double weight = 0.2;
        double c_max_1 = 150;
        double c_max_2 = 150;

        double r = 361.0;
        double p = 1.0/20.0;

        auto c = getCommunicationCost();
        double c1 = c.first;
        double c2 = c.second;

        double v1 = road1.getMeanSpeed();
        double v2 = road2.getMeanSpeed();
        double t1 = l1/v1;
        double t2 = l2/v2;
        double x1 = flow1/l1*v1;
        double x2 = flow2/l2*v2;

        double J1 = weight*t1+(1-weight)*c1;
        double J2 = weight*t2+(1-weight)*c2;

        double systemCost = t1*x1+t2*x2;

        output << simTime() <<" " << systemCost << "\n";
        outputFlow << simTime() <<" "<<x1*3600<<"\n";
    
        double x, speed;
        if(simTime() < 20) {
            Mac1609_4 *mac = FindModule<Mac1609_4*>::findSubModule(getParentModule());
            if(t1 > t2) {//choose route 1
                traciVehicle->changeRoute("edge2", 9999);
                double u = flow1/l1;
                x = u*v1*3600;
                mac->setParametersForBitrate(BITRATES_80211P[b1]);
            } else {//choose route 2
                traciVehicle->changeRoute("edge1", 9999);
                double u = flow2/l2;
                x = u*v2*3600;
                speed = v_max/(1.0+alpha*pow(x/(lane2*cap2), beta));
                mac->setParametersForBitrate(BITRATES_80211P[b2]);
            }
        } else {
            Mac1609_4 *mac = FindModule<Mac1609_4*>::findSubModule(getParentModule());

            if( J1 <= J2) {//choose route 1
                traciVehicle->changeRoute("edge2", 9999);
                double u = flow1/l1;
                x = u*v1*3600;
                speed = v_max/(1.0+alpha*pow(x/(lane1*cap1), beta));
                traciVehicle->setSpeed(speed);
                mac->setParametersForBitrate(BITRATES_80211P[b1]);
            } else {//choose route 2
                traciVehicle->changeRoute("edge1", 9999);
                double u = flow2/l2;
                x = u*v2*3600;
                speed = v_max/(1.0+alpha*pow(x/(lane2*cap2), beta));
                traciVehicle->setSpeed(speed);
                mac->setParametersForBitrate(BITRATES_80211P[b2]);
            }
        }
        std::cout << simTime() <<": "<< c1 <<" " << c2 <<" "<< t1<< " "<<t2 <<" " << x1*3600 << " " << x2*3600 << "\n";
}

void TraCIDemo11p::onWSA(WaveServiceAdvertisment* wsa) {
    if (currentSubscribedServiceId == -1) {
        mac->changeServiceChannel(wsa->getTargetChannel());
        currentSubscribedServiceId = wsa->getPsid();
        if  (currentOfferedServiceId != wsa->getPsid()) {
            stopService();
            startService((Channels::ChannelNumber) wsa->getTargetChannel(), wsa->getPsid(), "Mirrored Traffic Service");
        }
    }
}

void TraCIDemo11p::onWSM(WaveShortMessage* wsm) {
    findHost()->getDisplayString().updateWith("r=16,green");
    std::string msg = wsm->getWsmData();

    if (mobility->getRoadId()[0] != ':' && msg.size() > 4) {
        if (msg.substr(0,4)=="req@") {
            std::string req = msg.substr(4);
            int prob = rand()%10;
            if (prob==1) {
                WaveShortMessage* wsm1 = new WaveShortMessage();
                std::string str = "content@"+std::to_string(wsm->getSenderAddress());
                wsm1->setWsmData(str.c_str());
                populateWSM(wsm1);
                sendDown(wsm1);
            }
        } else if (msg.size() >= 8 && msg.substr(0,8)=="content@") {
            if (std::to_string(myId) != msg.substr(8)) return;
                BaseWaveApplLayer::hitMap[simTime()].insert(std::make_pair(myId, mobility->getRoadId()));
                emit(throughputSignal, long(wsm->getBitLength()));
        } else {
            std::cout<<myId<<" receive data "<< msg << "\n";
        }
    }
}

void TraCIDemo11p::handleSelfMsg(cMessage* msg) {
    if (WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg)) {
        //send this message on the service channel until the counter is 3 or higher.
        //this code only runs when channel switching is enabled
        sendDown(wsm->dup());
        wsm->setSerial(wsm->getSerial() +1);
        if (wsm->getSerial() >= 3) {
            //stop service advertisements
            stopService();
            delete(wsm);
        }
        else {
            scheduleAt(simTime()+1, wsm);
        }
    }
    else {
        BaseWaveApplLayer::handleSelfMsg(msg);
    }
}

void TraCIDemo11p::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);

    // stopped for for at least 10s?
    if (simTime() - lastDroveAt >= 2 && sentMessage == false) {
        sentMessage = true;

        WaveShortMessage* wsm = new WaveShortMessage();
        populateWSM(wsm);
        wsm->setWsmData(mobility->getRoadId().c_str());

        //host is standing still due to crash
        if (dataOnSch) {
            startService(Channels::SCH2, 42, "Traffic Information Service");
            //started service and server advertising, schedule message to self to send later
            scheduleAt(computeAsynchronousSendingTime(1,type_SCH),wsm);
        }
        else {
            //send right away on CCH, because channel switching is disabled
            sendDown(wsm);
        }
    }
    lastDroveAt = simTime();
}

std::pair<double, double> TraCIDemo11p::getCommunicationCost() {
    double t = 20;
    simtime_t cur = simTime();
    auto start = hitMap.lower_bound(cur-t);
    auto end = hitMap.lower_bound(cur);
    if(end!=hitMap.end()) end++;
    double c1 = 0, c2 = 0;
    std::unordered_set<int> vehicles1, vehicles2;
    for(auto itr = start; itr != end; itr++) {
        for(auto i = itr->second.begin(); i != itr->second.end(); i++) {
            if(i->second == "edge1") {
                c1+=1048576;
                vehicles1.insert(i->first);
            }
            else if (i->second == "edge2") {
                c2+=1048576;
                vehicles2.insert(i->first);
            }
        }
    }
    c1 = c1/vehicles1.size()/t/(1024*1024);
    c2 = c2/vehicles2.size()/t/(1024*1024);

    c1 = fmin(150.0, 1.0/c1);//20
    c2 = fmin(150.0, 1.0/c2);//20
    return std::make_pair(c1, c2);
}
