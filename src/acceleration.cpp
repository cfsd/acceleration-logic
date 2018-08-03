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

Acceleration::Acceleration(cluon::OD4Session &od4, float Kp,  float Ki, float targetSpeed, float accelerationLimit, float maxAmpSteer) :
 m_od4(od4)
 , m_steerError(0.0f)
 , m_speedError(0.0f)
 , m_Kp(Kp)
 , m_Ki(Ki)
 , m_lastSteerRequest()
 , m_lastSpeedRead()
 , m_groundSpeedReading()
 , m_groundSpeedReadingLeft()
 , m_groundSpeedReadingRight()
 , m_speedMutex()
 , m_od4Mutex()
 , m_targetSpeed(targetSpeed)
 , m_accelerationLimit(accelerationLimit)
 , m_maxAmpSteer(maxAmpSteer)
{
 setUp();
}

Acceleration::~Acceleration()
{
}

void Acceleration::nextContainer(cluon::data::Envelope &a_container)
{
  if (a_container.dataType() == opendlv::logic::perception::GroundSurfaceProperty::ID() ) {
    std::lock_guard<std::mutex> lockSpeed(m_speedMutex);
    m_targetSpeed = 0.0f;
    Acceleration::stop();
  }

  if (a_container.dataType() == opendlv::proxy::GroundSpeedReading::ID()) {
    std::lock_guard<std::mutex> lockSpeed(m_speedMutex);
    auto vehicleSpeed = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(a_container));
    if(a_container.senderStamp()==1504){
      m_groundSpeedReadingLeft = vehicleSpeed.groundSpeed();
    } else if (a_container.senderStamp()==1505){
      m_groundSpeedReadingRight = vehicleSpeed.groundSpeed();
    }
    m_groundSpeedReading = (m_groundSpeedReadingLeft + m_groundSpeedReadingRight)*0.5f;

    Acceleration::speedControl();
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

  std::cout << "first" << std::endl;

  std::cout << "X: \n" << X << std::endl;
  std::cout << "y: \n" << y << std::endl;

  C = ((X.transpose()*X).inverse())*X.transpose()*y;

  std::cout << "second" << std::endl;

  Acceleration::closestPoint(C, sampleTime);

}

void Acceleration::closestPoint(Eigen::RowVectorXf C, cluon::data::TimeStamp sampleTime) {

  float d = static_cast<float>(C(0)/sqrt( pow(C(1),2) + 1) );

  {
    std::lock_guard<std::mutex> lockSpeed(m_speedMutex);
    if (m_groundSpeedReading > 1) {
      float dt = static_cast<float>( (cluon::time::toMicroseconds(cluon::time::now()) - cluon::time::toMicroseconds(m_lastSteerRequest))*1e-6 );
      m_steerError += d*dt;
    }
  }

  float maxAmpSteer = static_cast<float>(m_maxAmpSteer*3.14/180);

  float steeringAngle = m_Kp*d + m_Ki*m_steerError;
  steeringAngle = (steeringAngle > maxAmpSteer) ? maxAmpSteer : steeringAngle;
  steeringAngle = (steeringAngle < -maxAmpSteer) ? -maxAmpSteer : steeringAngle;

  m_lastSteerRequest = cluon::time::now();

  opendlv::logic::action::AimPoint aimPoint;
  aimPoint.azimuthAngle(steeringAngle);

  m_od4.send(aimPoint,sampleTime,317);
}

void Acceleration::speedControl() {

  m_speedMutex.lock();
  float speedErr = m_targetSpeed - m_groundSpeedReading;
  m_speedMutex.unlock();


  if (speedErr > -3 && speedErr < 3) {
    float dt = static_cast<float>( (cluon::time::toMicroseconds(cluon::time::now()) - cluon::time::toMicroseconds(m_lastSpeedRead))*1e-6 );
    m_speedError += speedErr*dt;
    m_speedError = (m_speedError > 10) ? 10:m_speedError;
    m_speedError = (m_speedError < -10) ? -10 : m_speedError;
  }

  float accelerationRequest = speedErr + m_speedError*0.1f;

  accelerationRequest = std::min(std::max(accelerationRequest,-m_accelerationLimit),m_accelerationLimit);
  cluon::data::TimeStamp sampleTime = cluon::time::now();

  if (accelerationRequest >= 0) {
    opendlv::proxy::GroundAccelerationRequest message;
    message.groundAcceleration(accelerationRequest);

    std::lock_guard<std::mutex> lockOd4(m_od4Mutex);
    m_od4.send(message,sampleTime,317);
  } else {
    opendlv::proxy::GroundDecelerationRequest message;
    message.groundDeceleration(-accelerationRequest);

    std::lock_guard<std::mutex> lockOd4(m_od4Mutex);
    m_od4.send(message,sampleTime,317);
  }
}

void Acceleration::stop() {
  std::lock_guard<std::mutex> lockSpeed(m_speedMutex);
  if (std::abs(m_groundSpeedReading) < 1){
    opendlv::proxy::SwitchStateReading message;
    cluon::data::TimeStamp sampleTime = cluon::time::now();
    message.state(1);

    std::lock_guard<std::mutex> lockOd4(m_od4Mutex);
    m_od4.send(message,sampleTime,1403);
  }
}

void Acceleration::setUp()
{
  m_lastSteerRequest = cluon::time::now();
  m_lastSpeedRead = cluon::time::now();
}

void Acceleration::tearDown()
{
}
