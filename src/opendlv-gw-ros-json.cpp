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
#include <iostream>
#include <sstream>

#include <Python.h>

#include "cluon-complete.hpp"

int32_t main(int32_t argc, char **argv)
{
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid")) {
    std::cerr << argv[0] << " is an OpenDLV to ROS interface for bidirectional transactions via JSON." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --verbose" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111" << std::endl;
    retCode = 1;
  } else {
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    uint16_t const CID = static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]));

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
      std::string const START_FUNC_NAME = "startRos";
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

  //  while (od4.isRunning()) {
  //    std::this_thread::sleep_for(std::chrono::seconds(1));
  //  }
  }
  return retCode;
}
