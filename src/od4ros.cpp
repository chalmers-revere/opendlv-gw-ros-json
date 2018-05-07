/*
 * Copyright (C) 2018 Ola Benderius
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

#include <memory>

#include "cluon-complete.hpp"
#include "od4ros.hpp"

static std::unique_ptr<Od4Ros> g_od4ros;

extern "C" {
  static void start(uint32_t cid) {
    if (g_od4ros == nullptr) {
      g_od4ros.reset(new Od4Ros(cid));
    }
  }

  static void toOd4(std::string const &data) {
    if (g_od4ros != nullptr) {
      g_od4ros->toOd4(data);
    }
  }
}

Od4Ros::Od4Ros(uint16_t cid):
  m_od4Session()
{
  //PyObject *dataFromOd4Func = PyObject_GetAttrString(module, "dataFromOd4");

  auto onIncomingEnvelope([](cluon::data::Envelope &&envelope) {
      std::string data = cluon::serializeEnvelope(std::move(envelope));

      (void) data;
      //Pyton: Send JSON.
    });
  cluon::OD4Session od4{cid, onIncomingEnvelope};

  auto dataReceivedDelegate([&od4](std::string const &message) {
      std::stringstream sstr(message);
      while (sstr.good()) {
        auto tmp{cluon::extractEnvelope(sstr)};
        if (tmp.first) {
          cluon::data::Envelope env{tmp.second};
          env.sent(cluon::time::now());
          env.sampleTimeStamp(cluon::time::now());
          od4.send(std::move(env));
        }
      }
    });
  // Python: Set lambda as deligate.
  (void) dataReceivedDelegate;
}

Od4Ros::~Od4Ros() {
}
  
void Od4Ros::toOd4(std::string const &jsonData) {
  std::cout << jsonData << std::endl;
}
