/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "acceleration.hpp"
#include "collector.hpp"
#include <Eigen/Dense>
#include <cstdint>
#include <tuple>
#include <utility>
#include <iostream>
#include <string>
#include <thread>

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  std::map<std::string, std::string> commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (commandlineArguments.size()<=0) {
    std::cerr << argv[0] << " is a driver model returning heading and acceleration requests given a discrete path in local 2D coordinates" << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> [--id=<Identifier in case of simulated units>] [--verbose] [Module specific parameters....]" << std::endl;
    std::cerr << "Example: " << argv[0] << "--cid=111 --id=221 --maxSteering=25.0 --maxAcceleration=5.0 --maxDeceleration=5.0 [more...]" <<  std::endl;
    retCode = 1;
  } else {
    uint32_t const surfaceId=(commandlineArguments["surfaceId"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["surfaceId"])) : (0);
    uint32_t const speedId1=(commandlineArguments["speedId1"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["speedId1"])) : (0);
    uint32_t const speedId2=(commandlineArguments["speedId2"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["speedId2"])) : (0);
    uint32_t id = (commandlineArguments.count("id")>0)?(static_cast<uint32_t>(std::stoi(commandlineArguments["id"]))):(221);

    // Interface to a running OpenDaVINCI session
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::OD4Session od4_WheelSpeed{static_cast<uint16_t>(std::stoi(commandlineArguments["cidWheelSpeed"]))};


    Acceleration acceleration(od4);
    int gatheringTimeMs = (commandlineArguments.count("gatheringTimeMs")>0)?(std::stoi(commandlineArguments["gatheringTimeMs"])):(50);
    int separationTimeMs = (commandlineArguments.count("separationTimeMs")>0)?(std::stoi(commandlineArguments["separationTimeMs"])):(10);

    Collector collector(acceleration,gatheringTimeMs, separationTimeMs, 1);

    auto surfaceEnvelope{[senderStamp = surfaceId,&collector](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == senderStamp){
          collector.CollectSurfaces(envelope);
        }
      }
    };
    auto nextEnvelope{[&surfer = acceleration, speedId1, speedId2, surfaceId](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == speedId1 || envelope.senderStamp() == speedId2 || envelope.senderStamp() == surfaceId){
          surfer.nextContainer(envelope);
        }
      }
    };

    od4.dataTrigger(opendlv::logic::perception::GroundSurfaceArea::ID(),surfaceEnvelope);
    od4.dataTrigger(opendlv::logic::perception::GroundSurfaceProperty::ID(),nextEnvelope);
    od4od4_WheelSpeed.dataTrigger(opendlv::proxy::GroundSpeedReading::ID(),nextEnvelope);


    // Just sleep as this microservice is data driven.
    using namespace std::literals::chrono_literals;
    while (od4.isRunning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
      cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
      opendlv::system::SignalStatusMessage readySignal;
      readySignal.code(1);
      od4.send(readySignal, sampleTime, id);
    }
  }
  return retCode;
}
