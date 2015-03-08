#include <boost/python.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace std;

class RobotData {
    public:
        static RobotData& getInstance() {
            static RobotData    instance; // Guaranteed to be destroyed.
            return instance;
        }
        struct VideoImage {
            cv::Mat rgb;
            cv::Mat depth;
        } videoFrame;
        std::string getBinItem(int row, int column, int item){
            return this->shelf.bins[(row*4)+column].object[item];
        }
        void setBinItem(int row,int column,int item,const std::string& val){
            this->shelf.bins[(row*4)+column].object[item] = val;
        }
        //cv::Mat (const cv::Mat& orgImage)
        static VideoImage getImageFrame(){
            VideoImage frame;
            frame.rgb = cv::imread("a.jpg");
            frame.depth = cv::imread("a.jpg",CV_LOAD_IMAGE_GRAYSCALE);
            cv::imshow("rgb",cv::imread("a.jpg"));
            cv::imshow("bw",cv::imread("a.jpg",CV_LOAD_IMAGE_GRAYSCALE));
            return frame;
        }
        int start() {
            return 0;
        }
        int stop() {
            return 0;
        }
        /*
         * void start();      void stop();
         * const Image& getLastFrame();
         * LeapCamera(const std::string& id="LEAP");
         *
         * start()
         * stop()
         * con start() inizia ad acquisire immagini
         * con stop() finisce
         * (cioè si ferma, ma può riprendere con start())
         * e poi ha getLastImage() che ritorna l'ultimo frame letto
         * (fake)
         * di tipo Image
         * che si costruisce con due cv::Mat
         * una per il depth e una per l'RGB
         * end
         * posso pushare sul tuo repo?
         */

    private:
        struct Bin {
            std::string object[5];
        } bin;
        struct Shelf {
            Bin bins[12];
        } shelf;
        //RobotData() {};                   // Constructor? (the {} brackets) are needed here.

        // C++ 03
        // ========
        // Dont forget to declare these two. You want to make sure they
        // are unacceptable otherwise you may accidentally get copies of
        // your singleton appearing.
        //RobotData(RobotData const&);              // Don't Implement
        //void operator=(RobotData const&); // Don't implement

        // C++ 11
        // =======
        // We can use the better technique of deleting the methods
        // we don't want.
        //RobotData(RobotData const&)               = delete;
        //void operator=(RobotData const&)  = delete;
};

#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
BOOST_PYTHON_MODULE(apcRobot)
{
    namespace python = boost::python;
    {
        python::class_<RobotData>("RobotData")
            .def("getBinItem",&RobotData::getBinItem)
            .def("setBinItem",&RobotData::setBinItem)
            .def("start",&RobotData::start)
            .def("stop",&RobotData::stop)
            .def("getImageFrame",&RobotData::getImageFrame)
            ;
    }
}
