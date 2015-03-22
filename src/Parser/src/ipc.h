#pragma once

#include <boost/python.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

namespace InterProcessCommunication{
    class RobotData {
        public:
            static RobotData& getInstance();
            struct VideoImage {
                cv::Mat rgb;
                cv::Mat depth;
            };
            std::string getBinItem(int row, int column, int item);
            void setBinItem(int row,int column,int item,const std::string& val);
            static VideoImage getImageFrame();
            int start();
            int stop();
        private:
            struct Bin {
                std::string object[5];
                bool dirty;
            };
            struct Shelf {
                Bin bins[12];
            };
            RobotData() {};
            ~RobotData() {};
            void operator=(RobotData const&); // Don't implement
            RobotData(RobotData const&);              // Don't Implement
    }
}
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
BOOST_PYTHON_MODULE(libapcRobot)
{
    namespace python = boost::python;
    {
        python::class_<RobotData, boost::noncopyable>("RobotData", python::no_init)
            .def("getInstance",&RobotData::getInstance,python::return_value_policy<python::reference_existing_object>() )
            .staticmethod("getInstance")
            .def("getBinItem",&RobotData::getBinItem)
            .def("setBinItem",&RobotData::setBinItem)
            .def("start",&RobotData::start)
            .def("stop",&RobotData::stop)
            .def("getImageFrame",&RobotData::getImageFrame)
            ;
    }
}
