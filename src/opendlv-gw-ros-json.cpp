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

#include <bitset>
#include <memory>
#include <iostream>
#include <sstream>

#include <Python.h>

#include "cluon-complete.hpp"

int32_t main(int32_t argc, char **argv)
{
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") || 0 == commandlineArguments.count("odvd")) {
    std::cerr << argv[0] << " is an OpenDLV to ROS interface for bidirectional transactions via JSON." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --odvd=<ODVD message list file> --verbose" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --odvd=messages.odvd" << std::endl;
    retCode = 1;
  } else {
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    uint16_t const CID = static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]));
    std::string const ODVD = commandlineArguments["odvd"];
    std::string const ADDRESS = "127.0.0.1";
    uint16_t const TCP_PORT = 9000;

    std::ifstream is(ODVD);
    std::stringstream buffer;
    buffer << is.rdbuf();

    cluon::EnvelopeConverter envConverter;
    envConverter.setMessageSpecification(std::string(buffer.str()));

    std::unique_ptr<cluon::TCPConnection> tcpConnection;

    auto onIncomingEnvelope([&tcpConnection, &envConverter](cluon::data::Envelope &&envelope) {
        if (tcpConnection != nullptr && tcpConnection->isRunning()) {
          std::string JSON = envConverter.getJSONFromEnvelope(envelope);
          tcpConnection->send(std::move(JSON));
        }
      });
    cluon::OD4Session od4{CID, onIncomingEnvelope};


    auto onIncomingTcpData([&od4, &envConverter](std::string &&data, std::chrono::system_clock::time_point &&) {

        uint32_t messageId;

        std::stringstream sstr;
        sstr.str(data);
        sstr.read(reinterpret_cast<char*>(&messageId), sizeof(uint32_t));

        std::string json = data.substr(3);

        std::string proto{envConverter.getProtoEncodedEnvelopeFromJSONWithoutTimeStamps(json, messageId, 0)};
        cluon::data::Envelope env;
        env.dataType(messageId)
           .senderStamp(0)
           .sampleTimeStamp(cluon::time::now())
           .serializedData(proto)
           .sent(cluon::time::now());
        
        od4.send(std::move(env));
      });

    tcpConnection.reset(new cluon::TCPConnection(ADDRESS, TCP_PORT, onIncomingTcpData, nullptr));

    Py_Initialize();
    if (VERBOSE) {
      std::cout << "Python is initialized." << std::endl;
    }

    PySys_SetArgv(argc, argv);
    
    std::string const PYTHON_SCRIPT = "od4ros";
    PyObject *name = PyString_FromString(PYTHON_SCRIPT.c_str());

    PyObject *module = PyImport_Import(name);
    Py_DECREF(name);

    if (module != nullptr) {
      std::string const START_FUNC_NAME = "start";
      PyObject *startRosFunc = PyObject_GetAttrString(module, START_FUNC_NAME.c_str());

      if (startRosFunc && PyCallable_Check(startRosFunc)) {
        PyObject *args = PyTuple_New(1);
        PyTuple_SetItem(args, 0, PyInt_FromLong(CID));

        PyObject *value = PyObject_CallObject(startRosFunc, args);
        Py_DECREF(args);
        if (value != nullptr) {
          Py_DECREF(value);
        } else {
          Py_DECREF(startRosFunc);
          Py_DECREF(module);
          PyErr_Print();
          std::cerr << "Could not call Python function '" << START_FUNC_NAME << "'." << std::endl;
          return 1;
        }
      } else {
        if (PyErr_Occurred()) {
          PyErr_Print();
        }
        std::cerr << "Could not find Python function '" << START_FUNC_NAME << "'." << std::endl;
      }
      Py_XDECREF(startRosFunc);
      Py_DECREF(module);
    } else {
      PyErr_Print();
      retCode = 1;
    }

    Py_Finalize();
  }
  return retCode;
}
