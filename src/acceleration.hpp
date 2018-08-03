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

#ifndef CFSD18_COGNITION_ACCELERATION_HPP
#define CFSD18_COGNITION_ACCELERATION_HPP

#include <opendlv-standard-message-set.hpp>
#include <cluon-complete.hpp>
#include <Eigen/Dense>
#include <map>
#include <chrono>
#include <thread>

class Acceleration {
 public:
  Acceleration(cluon::OD4Session &od4);
  virtual ~Acceleration();
  void nextContainer(cluon::data::Envelope &);
  void receiveCombinedMessage(std::map<int,opendlv::logic::perception::GroundSurfaceArea>, cluon::data::TimeStamp);

 private:
  void setUp();
  void tearDown();
  void leastSquare(Eigen::MatrixXf, cluon::data::TimeStamp);
  void closestPoint(Eigen::RowVectorXf, cluon::data::TimeStamp);
  void stop();

 private:
   cluon::OD4Session &m_od4;

};

#endif
