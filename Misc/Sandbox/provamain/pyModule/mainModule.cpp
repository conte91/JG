#include <APC/APC.h>

#include <boost/python.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
char **list_to_argv_array(PyObject *lst)
{
  assert (PyList_Check(lst));       // use better error handling here
  size_t cnt = PyList_GET_SIZE(lst);
  char **ret = new char*[cnt + 1];
  for (size_t i = 0; i < cnt; i++) {
    PyObject *s = PyList_GET_ITEM(lst, i);
    assert (PyString_Check(s));     // likewise
    size_t len = PyString_GET_SIZE(s);
    char *copy = new char[len + 1];
    memcpy(copy, PyString_AS_STRING(s), len + 1);
    ret[i] = copy;
  }
  ret[cnt] = NULL;
  return ret;
}

int theBigAPCPythonMain(int argc, PyObject *lst){
  return APC::APC::main(argc, list_to_argv_array(lst));
}
BOOST_PYTHON_MODULE(pyAPC)
{
  namespace python = boost::python;
  {
    /*python::class_<APC::APC, boost::noncopyable>("APC", python::no_init)
      .def("main", &APC::APC::main)
      .staticmethod("main")
      ;*/
    python::def("theBigAPCPythonMain", &theBigAPCPythonMain);
  }
}
