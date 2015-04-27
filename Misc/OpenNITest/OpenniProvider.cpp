#include <Camera/Image.h>
#include <pcl/io/openni_grabber.h>

pcl::Grabber;
Grabber* openniGrabber;                                               // OpenNI grabber that takes data from the device.
 
// This function is called every time the device has new data.
void
grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
{
	if (! viewer->wasStopped())
		viewer->showCloud(cloud);
 
	if (saveCloud)
	{
		stringstream stream;
		stream << "inputCloud" << filesSaved << ".pcd";
		string filename = stream.str();
		if (io::savePCDFile(filename, *cloud, true) == 0)
		{
			filesSaved++;
			cout << "Saved " << filename << "." << endl;
		}
		else PCL_ERROR("Problem saving %s.\n", filename.c_str());
 
		saveCloud = false;
	}
}
int main(){
		openniGrabber = new OpenNIGrabber();
		if (openniGrabber == 0)
			return -1;
		boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f =
			boost::bind(&grabberCallback, _1);
		openniGrabber->registerCallback(f);
}
