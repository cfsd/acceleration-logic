/**
* Copyright (C) 2017 Chalmers Revere
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
* USA.
*/

#include <iostream>
#include <cmath>
#include "acceleration.hpp"
#include <chrono>

Acceleration::Acceleration(cluon::OD4Session &od4) :
 m_od4(od4)
{
 setUp();
}

Acceleration::~Acceleration()
{
}

void Acceleration::nextContainer(cluon::data::Envelope &a_container)
{
  if (a_container.dataType() == opendlv::logic::perception::GroundSurfaceProperty::ID() ) {
    Acceleration::stop();
  }
}


void Acceleration::receiveCombinedMessage(std::map<int,opendlv::logic::perception::GroundSurfaceArea> currentFrame, cluon::data::TimeStamp sampleTime){
  std::reverse_iterator<std::map<int,opendlv::logic::perception::GroundSurfaceArea>::iterator> it;
  it = currentFrame.rbegin();
  int I = 0;
  Eigen::MatrixXf localPath(currentFrame.size()*2,2);
  while(it != currentFrame.rend()){
    auto surfaceArea = it->second;
    float x1 = surfaceArea.x1(); //Unpack message
    float y1 = surfaceArea.y1();
    float x2 = surfaceArea.x2();
    float y2 = surfaceArea.y2();
    float x3 = surfaceArea.x3();
    float y3 = surfaceArea.y3();
    float x4 = surfaceArea.x4();
    float y4 = surfaceArea.y4();

    localPath(2*I,0)=(x1+x2)/2.0f;
    localPath(2*I,1)=(y1+y2)/2.0f;
    localPath(2*I+1,0)=(x3+x4)/2.0f;
    localPath(2*I+1,1)=(y3+y4)/2.0f;
    it++;
    I++;
  }
  Acceleration::leastSquare(localPath, sampleTime);
} // End of recieveCombinedMessage


void Acceleration::leastSquare(Eigen::MatrixXf localPath, cluon::data::TimeStamp sampleTime) {
  Eigen::VectorXf x = localPath.col(0);
  Eigen::VectorXf y = localPath.col(1);
  Eigen::RowVectorXf C;
  Eigen::MatrixXf X(x.size(),2);


  X.col(0) = Eigen::VectorXf::Ones(x.size());
  X.col(1) = x;

  C = (X.transpose()*X)*(X.inverse().transpose())*y;

  Acceleration::closestPoint(C, sampleTime);

  (void) sampleTime;
  (void) X;
  (void) x;
  (void) y;
}

void Acceleration::closestPoint(Eigen::RowVectorXf C, cluon::data::TimeStamp sampleTime) {

  float d = static_cast<float>(C(0)/sqrt( pow(C(1),2) + 1) );
  float steeringAngle = 0.0349f*d;

  opendlv::logic::action::AimPoint aimPoint;
  aimPoint.azimuthAngle(steeringAngle);

  m_od4.send(aimPoint,sampleTime,317);
}

void Acceleration::stop() {
  opendlv::proxy::SwitchStateReading message;
  cluon::data::TimeStamp sampleTime = cluon::time::now();
  message.state(1);
  m_od4.send(message,sampleTime,1403);
}

void Acceleration::setUp()
{
}

void Acceleration::tearDown()
{
}
