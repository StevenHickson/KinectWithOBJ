#define HAVE_OPENCV

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

#include "Microsoft_grabber.h"
/*#include <FaceTrackLib.h>
#include <KinectInteraction.h>
#include <NuiKinectFusionApi.h>
#include <NuiKinectFusionDepthProcessor.h>
#include <NuiKinectFusionVolume.h>*/

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace pcl;
using namespace cv;

class SimpleMicrosoftViewer
{
public:
	SimpleMicrosoftViewer () : viewer(new pcl::visualization::PCLVisualizer ("PCL Microsoft Viewer")), normals(new pcl::PointCloud<pcl::Normal>), sharedCloud(new pcl::PointCloud<pcl::PointXYZRGBA>), first(false), update(false) {}

	void cloud_cb_ (const boost::shared_ptr<const KinectData> &data)
	{
		/*if (!viewer.wasStopped())
		viewer.showCloud (cloud);*/
		// estimate normals
		imshow("image", data->image);
		imshow("depth", data->depth);
		waitKey(1);
		if(!data->cloud.empty()) {
			normalMutex.lock();
			copyPointCloud(data->cloud,*sharedCloud);
			PointCloud<PointXYZRGBA>::iterator pCloud = sharedCloud->begin();
			while(pCloud != sharedCloud->end()) {
				pCloud->x *= 150;
				pCloud->y = pCloud->y * 150 + 100;
				pCloud->z = pCloud->z * -150 + 350;
				++pCloud;
			}

			//sharedCloud = cloud;
			update = true;
			normalMutex.unlock();
		}
	}

	void run (string obj_file)
	{
		// create a new grabber for OpenNI devices
		pcl::Grabber* my_interface = new pcl::MicrosoftGrabber();

		// make callback function from member function
		boost::function<void (const boost::shared_ptr<const KinectData>&)> f =
			boost::bind (&SimpleMicrosoftViewer::cloud_cb_, this, _1);

		my_interface->registerCallback (f);

		//viewer.setBackgroundColor(0.0, 0.0, 0.5);

		//pcl::TextureMesh mesh; 
		//pcl::io::loadOBJFile(obj_file,mesh);
		//viewer->addTextureMesh(mesh);
		//vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		//transform->Scale(0.1f,0.1f,0.1f);
		viewer->addModelFromPLYFile(obj_file);

		my_interface->start ();
		Sleep(30);
		while (!viewer->wasStopped())
		{
			normalMutex.lock();
			if(update) {
				viewer->removePointCloud("original");
				viewer->addPointCloud(sharedCloud,"original");
				update = false;
			}
			viewer->spinOnce();
			normalMutex.unlock();
		}

		my_interface->stop ();
	}

	boost::shared_ptr<pcl::PointCloud<pcl::Normal> > normals;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > sharedCloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	bool first, update;
	boost::mutex normalMutex;
};

int
	main (int argc, char** argv)
{
	PointCloud<PointXYZ> depth;
	PointCloud<PointXYZRGB> color;
	PointCloud<PointXYZRGB> cloud;
	try {
		SimpleMicrosoftViewer v;
		v.run(argv[1]);
	} catch (pcl::PCLException e) {
		cout << e.detailedMessage() << endl;
	} catch (std::exception &e) {
		cout << e.what() << endl;
	}
	printf("Done\n");
	cin.get();
	return (0);
}