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
    //choose route
    Veins::TraCIScenarioManager* manager = Veins::TraCIScenarioManagerAccess().get();
        traci = manager->getCommandInterface();
////one route start

        //TraCICommandInterface::Road road1 = TraCICommandInterface::Road(traci, "e1");
        //double l1 = 1000, v_max = 25, cap = 2000, alpha=0.2, beta=10, lane = 3.0;
        //double flow = manager->drivingVehicleCount;
        //double v1 = road1.getMeanSpeed();
        //double u = flow/l1;
        //double x = u*v1*3600.0;//veh/h
        //std::cout << x << "\n";
        //double speed = v_max/(1.0+alpha*pow(x/(lane*cap), beta));
        //std::cout << "flow = " << x << ", v = " << speed << "\n";
        //traciVehicle->setSpeed(speed);

////one route code end
////two route code start

        double flow1 = traci->getLastStepVehicleNumber("flow1a")+traci->getLastStepVehicleNumber("flow1b");//+traci->getLastStepVehicleNumber("flow1c");
        //double flow1 = traci->getLastStepVehicleNumber("flow1");
        double flow2 = traci->getLastStepVehicleNumber("flow2");
        //double flow2 = traci->getLastStepVehicleNumber("flow2a")+traci->getLastStepVehicleNumber("flow2b")+traci->getLastStepVehicleNumber("flow2c");
        //double flow3 = traci->getLastStepVehicleNumber("flow3a")+traci->getLastStepVehicleNumber("flow3b");
        //double flow4 = traci->getLastStepVehicleNumber("flow4a")+traci->getLastStepVehicleNumber("flow4b");

        //TraCICommandInterface::Road road1 = TraCICommandInterface::Road(traci, "e1");
        //TraCICommandInterface::Road road2 = TraCICommandInterface::Road(traci, "e2");
        TraCICommandInterface::Road road1 = TraCICommandInterface::Road(traci, "edge1");
        TraCICommandInterface::Road road2 = TraCICommandInterface::Road(traci, "edge2");
        //TraCICommandInterface::Road road3 = TraCICommandInterface::Road(traci, "edge3");
        //TraCICommandInterface::Road road4 = TraCICommandInterface::Road(traci, "edge4");

        //double v1 = road1.getMeanSpeed()/2.0+road2.getMeanSpeed()/2.0;
        //double v2 = road3.getMeanSpeed()/2.0+road4.getMeanSpeed()/2.0;

        double rate = 4000.0, l1 = 1000, l2 = 500, v_max = 15.65, cap1 = 1500, cap2 = 1500, alpha=0.2, beta=10, lane1 = 2, lane2 = 1;
        double weight = 0.2;
        //int b1i = 0, b2i = 3;

        double r = 361.0;
        double p = 1.0/10.0;
        //double st = simTime().raw()/pow(10,12);
        //double b1 = 50.0, b2 = 50.0*(1.0-st/500);
        //double u1 = (flow1+flow2)/l1;
        //double u2 = (flow3+flow4)/l2;
        //double f1 = u1*v1;
        //double f2 = u2*v2;
        //double h1 = 1.0-exp(-2*r*p*u1);
        //double h2 = 1.0-exp(-2*r*p*u2);

////
        auto c = getCommunicationCost();
        double c1 = c.first;
        double c2 = c.second;
        //double t1 = road1.getCurrentTravelTime();
        //double t2 = road2.getCurrentTravelTime();

        double v1 = road1.getMeanSpeed();
        //v1 = v1==0.0?road1.getMaxSpeed();
        double v2 = road2.getMeanSpeed();
        //v2 = v2==0.0?road2.getMaxSpeed();
        //double v1 = l1/t1;
        //double v2 = l2/t2;

        double t1 = l1/v1;
        double t2 = l2/v2;

        double x1 = flow1/l1*v1;
        double x2 = flow2/l2*v2;
        //double t1 = l1/v1;
        //double t2 = l2/v2;

        c1 = std::fmin(150, 2.0*r/sqrt(l1)*sqrt(x1*t1)/(1-exp(-2.0*r*p*x1*t1/l1))/(std::fmax(0,10.0-simTime().raw()/pow(10,12)/1000.0*10.0)));
        c2 = std::fmin(150, 2.0*r/sqrt(l2)*sqrt(x2*t2)/(1-exp(-2.0*r*p*x2*t2/l2))/(10));

        //double c1 = std::fmin(150, 2.0*r/sqrt(l1)*sqrt(x1*t1)/(1-exp(-2.0*r*p*x1*t1/l1))/(BITRATES_80211P[b1i]/1000000.0));
        //double c2 = std::fmin(150, 2.0*r/sqrt(l2)*sqrt(x2*t2)/(1-exp(-2.0*r*p*x2*t2/l2))/(BITRATES_80211P[b2i]/1000000.0));
        //std::cout << c1 << ","<<c2<<"\n";
        //double t1 = road1.getCurrentTravelTime() + road2.getCurrentTravelTime();
        //double t2 = road4.getCurrentTravelTime() + road3.getCurrentTravelTime();
        //double t1 = l1/v_max*(1.0+alpha*pow(flow1/l1*v1*3600/(lane1*cap), beta));
        //double t2 = l2/v_max*(1.0+alpha*pow(flow2/l2*v2*3600/(lane2*cap), beta));

        double J1 = weight*t1+(1-weight)*c1;
        double J2 = weight*t2+(1-weight)*c2;
        //std::cout << t1 << ","<<c1<<"    "<< t2 << ","<< c2 << "\n";
        //std::cout << simTime() <<": "<< J1 << ", "<<J2<< "\n";
        //double x1 = (flow1+flow2)/l1*v1*3600;
        //double x2 = (flow3+flow4)/l2*v2*3600;
        //double x1 = rate*(flow1/(flow1+flow2))/3600;//flow1/l1*v1;
        //double x2 = rate/3600-x1;//flow2/l2*v2;
        double systemCost = t1*x1+t2*x2;
        //output << simTime() <<" "<< J1 << " "<<J2<< " "<< systemCost << " "<< c1 <<" " << c2 <<" "<< t1<< " "<<t2<< "\n";
        //std::cout << simTime() <<": "<< c1 <<" " << c2 <<" "<< t1<< " "<<t2<< "\n";
        output << simTime() <<" " << systemCost << "\n";
        outputFlow << simTime() <<" "<<x1*3600<<"\n";
        double x, speed;
        //std::cout << simTime() << ": "<< flow1/l1*v1*3600 << "\n";
        if(simTime() < 20) {
            Mac1609_4 *mac = FindModule<Mac1609_4*>::findSubModule(getParentModule());
            //if(t1 <= t2) {//choose route 1
            if(t1 > t2) {//choose route 1
                traciVehicle->changeRoute("edge2", 9999);
                //double u = (flow1+flow2)/l1;
                double u = flow1/l1;
                x = u*v1*3600;
                //x = rate*(flow1/(flow1+flow2));
                speed = v_max/(1.0+alpha*pow(x/(lane1*cap1), beta));
                //traciVehicle->slowDown(speed, 1);
                //traciVehicle->setSpeed(speed);
                mac->setParametersForBitrate(BITRATES_80211P[b1]);
            } else {//choose route 2
                traciVehicle->changeRoute("edge1", 9999);
                //double u = (flow3+flow4)/l2;
                double u = flow2/l2;
                x = u*v2*3600;
                //x = rate-rate*(flow1/(flow1+flow2));
                speed = v_max/(1.0+alpha*pow(x/(lane2*cap2), beta));
                //traciVehicle->setSpeed(speed);
                //traciVehicle->slowDown(speed, 1);
                mac->setParametersForBitrate(BITRATES_80211P[b2]);
            }
        } else {
            Mac1609_4 *mac = FindModule<Mac1609_4*>::findSubModule(getParentModule());

            //if(flow1/l1*v1*3600 < 3904) {// J1 <= J2) {//choose route 1
            if( J1 <= J2) {//choose route 1
                traciVehicle->changeRoute("edge2", 9999);
                //double u = (flow1+flow2)/l1;
                double u = flow1/l1;
                x = u*v1*3600;
                //x = rate*(flow1/(flow1+flow2));
                speed = v_max/(1.0+alpha*pow(x/(lane1*cap1), beta));
                //traciVehicle->slowDown(speed, 1);
                traciVehicle->setSpeed(speed);
                mac->setParametersForBitrate(BITRATES_80211P[b1]);
            } else {//choose route 2
                traciVehicle->changeRoute("edge1", 9999);
                //double u = (flow3+flow4)/l2;
                double u = flow2/l2;
                x = u*v2*3600;
                //x = rate-rate*(flow1/(flow1+flow2));
                speed = v_max/(1.0+alpha*pow(x/(lane2*cap2), beta));
                //traciVehicle->slowDown(speed, 1);
                traciVehicle->setSpeed(speed);
                mac->setParametersForBitrate(BITRATES_80211P[b2]);
            }
        //double speed = v_max/(1.0+alpha*pow(x/cap, beta));
        //traciVehicle->setSpeed(speed);

        //int n = NUM_BITRATES_80211P-1;
        //unsigned int i = n;
        //if(J1 > J2) {//will choose route 2
        //    i = n-floor(fmax(0,(simTime().raw()/pow(10,12)-200.0))/floor(800.0/7.0));
        //    std::cerr<<i<<": "<<simTime()<<"!\n";
        //    mac->setParametersForBitrate(BITRATES_80211P[i]);
        //}
        //std::cout << c1 << ", " << BITRATES_80211P[i] << "\n";
        }
        std::cout << simTime() <<": "<< c1 <<" " << c2 <<" "<< t1<< " "<<t2 <<" " << x1*3600 << " " << x2*3600 << "\n";
        //std::cout<<"speed: "<<speed<<", x: "<<x<<"\n";
//// two route code end
/*
        std::cout << "link 1 #vehicle: " << flow1 <<"\n";
        std::cout << "link 2 #vehicle: " << flow2 <<"\n";
        std::cout << "link 3 #vehicle: " << flow3 <<"\n";
        std::cout << "link 4 #vehicle: " << flow4 <<"\n";
*/
        //int time1 = road1.getCurrentTravelTime() + road2.getCurrentTravelTime();
        //int time4 = road4.getCurrentTravelTime() + road3.getCurrentTravelTime();
        //std::string str = "time" + std::to_string(time1) + " " + std::to_string(time4);
        //std::string str = "flow" + std::to_string(flow1) + " " + std::to_string(flow4);
        //if(simTime()>10)
        //traciVehicle->changeRoute("edge1", 9999);

        //double flow1 = manager->drivingVehicleCount;

            //std::cout << road3.getCurrentTravelTime() << "\n";
    //std::cout<<hostSt.size()<<"\n";
    //std::cout << mobility->getManager()->getManagedHosts().size() << "\n";
    //for(auto & i : mobility->getManager()->getManagedHosts()) std::cout<<i.first<<"\n";

/*
    else {
        //std::cout << "node send info to RSU\n";
        WaveShortMessage* wsm = new WaveShortMessage();
        populateWSM(wsm);
        wsm->setWsmData(mobility->getRoadId().c_str());
        sendDown(wsm);
    }
    */
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
    //findHost()->getDisplayString().updateWith("r=16,green");

    //if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getWsmData(), 9999);
    std::string msg = wsm->getWsmData();
    //std::cout<<myId<<" receives "<<msg<<"\n";
    /*
    std::string road = mobility->getRoadId();
    if(road == "edge3" || road == "edge4") {
        Mac1609_4 *mac = FindModule<Mac1609_4*>::findSubModule(getParentModule());
        int n = NUM_BITRATES_80211P-1;
        unsigned int i = n;
        //if(J1 > J2) {//will choose route 2
            i = n-floor(fmax(0, (simTime().raw()/pow(10,12)-200.0))/floor(800.0/7.0));
            std::cerr<<i<<": "<<simTime()<<"!\n";
            mac->setParametersForBitrate(BITRATES_80211P[i]);
        //}
    }
    */
    if (mobility->getRoadId()[0] != ':' && msg.size() > 4) {
        if (msg.substr(0,4)=="req@") {
            std::string req = msg.substr(4);
            //std::cout << myId<< " receives req: "<< req << " from " <<wsm->getSenderAddress()<<"\n";
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
            //std::cout << simTime() << ", "<< getSimulation()->getWarmupPeriod() << "\n";
            //if (simTime() >= getSimulation()->getWarmupPeriod()) {
                BaseWaveApplLayer::hitMap[simTime()].insert(std::make_pair(myId, mobility->getRoadId()));
                //std::cerr << "cache hit\n";
                emit(throughputSignal, long(wsm->getBitLength()));
            //}
            //std::cout<<myId<<" receive data "<<msg.substr(8)<<":" << msg << "\n";
        } else {
            std::cout<<myId<<" receive data "<< msg << "\n";
        }
    }
    /*
    if (!sentMessage) {
        sentMessage = true;
        //repeat the received traffic update once in 2 seconds plus some random delay
        wsm->setSenderAddress(myId);
        wsm->setSerial(3);
        scheduleAt(simTime() + 2 + uniform(0.01,0.2), wsm->dup());
    }
    */
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
    //if (mobility->getSpeed() < 1) {
        if (simTime() - lastDroveAt >= 2 && sentMessage == false) {
            //findHost()->getDisplayString().updateWith("r=16,red");
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
    //}
    //else {
        lastDroveAt = simTime();
    //}
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
    //if (thpt1==0) thpt1 = c1;
    //else c1 = thpt1*0.9+c1*0.1;
    //if (thpt2==0) thpt2 = c2;
    //else c2 = thpt2*0.9+c2*0.1;
    //thpt1 = c1;
    //thpt2 = c2;
    //std::cout<<"c1: " <<c1 <<", "<<"c2: "<<c2<<"\n";
    c1 = fmin(150.0, 1.0/c1);//20
    c2 = fmin(150.0, 1.0/c2);//20
    return std::make_pair(c1, c2);
}
