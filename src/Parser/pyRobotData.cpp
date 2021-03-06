#include <Parser/RobotData.h>

#include <boost/python.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>

BOOST_PYTHON_MODULE(apcRobot)
{
  namespace python = boost::python;
  {
    python::class_<InterProcessCommunication::RobotData , boost::noncopyable>("RobotData", python::no_init)
      .def("getInstance",&InterProcessCommunication::RobotData::getInstance,python::return_value_policy<python::reference_existing_object>() )
      .staticmethod("getInstance")
      .def("getBinItem",&InterProcessCommunication::RobotData::getBinItem)
      .def("setBinItem",&InterProcessCommunication::RobotData::setBinItem)
      .def("getWorkOrder",&InterProcessCommunication::RobotData::getWorkOrder)
      .def("setWorkOrder",&InterProcessCommunication::RobotData::setWorkOrder)
      ;
  }
}
