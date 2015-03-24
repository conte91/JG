#include <APC/APC.h>

#include <boost/python/module.hpp>
#include <boost/python/def.hpp>

BOOST_PYTHON_MODULE(pyAPC)
{
  namespace python = boost::python;
  {
    python::class_<APC::APC, boost::noncopyable>("APC", python::no_init)
      .def("main")
      .staticmethod("main")
      ;
  }
}
