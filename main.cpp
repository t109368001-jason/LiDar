
#include <iostream>
#include <cmath>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <thread>
#include <pcl/io/vlp_grabber.h>
#include "include/function.h"
#include "include/boundary.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;
using namespace Eigen;

PCLVisualizer::Ptr viewer;
std::string filename;
uint64_t startTime;
uint64_t startTime2;

void keyboardEventOccurred(const KeyboardEvent& event, void* nothing)
{
  if (event.getKeySym() == "y" && event.keyDown()) // Flat shading
  {
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_SHADING,
                                        PCL_VISUALIZER_SHADING_FLAT, filename);
  }
  else if (event.getKeySym() == "t" && event.keyDown()) // Gouraud shading
  {
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_SHADING,
                                        PCL_VISUALIZER_SHADING_GOURAUD, filename);
  }
  else if (event.getKeySym() == "n" && event.keyDown()) // Phong shading
  {
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_SHADING,
                                        PCL_VISUALIZER_SHADING_PHONG, filename);
  }
}

void timer1()
{
	while(false)
	{
		if((clock() - startTime) > 1000000)
		{
			startTime = clock();
			std::cout << filename << std::endl;
		}
	}
}

typedef pcl::PointXYZI PointType;

void pcl_viewer()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcd_to_pointCloud(filename);
    myClass::Boundary boundary;
    double camera_Height = sqrt(3.0)*2.0;
    double camera_Width = sqrt(3.0)*2.0;
    double camera_Vertical_FOV = 60.0 * M_PI / 180.0;
    double camera_Horizontal_FOV = 180.0 * M_PI / 180.0;
    double object_X = 0;
    double object_Y = 0;
    double object_Height = sqrt(3.0)*2.0;
    double object_Width = sqrt(3.0)*2.0;
    double LiDar_Height_Offset = 0.0;
    double focal_Leftength_Offset = 5.0;
    Vector3f cameraAt = Vector3f(0.0, 0.0, 0.0);
    double LiDar_Horizontal_Offset = 0;

    cout << "Please input camera_Horizontal_FOV : "; 
    cin >> camera_Horizontal_FOV;

    camera_Horizontal_FOV = camera_Horizontal_FOV * M_PI / 180.0;

    boundary.setCameraParameter(cameraAt, "UL", camera_Height, camera_Width, camera_Vertical_FOV, camera_Horizontal_FOV);
    boundary.setLiDarParameter(LiDar_Height_Offset, LiDar_Horizontal_Offset, focal_Leftength_Offset);
    boundary.setBound(object_X, object_Y, object_Height, object_Width);

    boundary.division(cloud);
    
    cout << boundary.LiDar_Up_Angle * 180.0 / M_PI << endl;
    cout << boundary.LiDar_Down_Angle * 180.0 / M_PI << endl;
    cout << boundary.LiDar_Left_Angle * 180.0 / M_PI << endl;
    cout << boundary.LiDar_Right_Angle * 180.0 / M_PI << endl;

    viewer.reset(new PCLVisualizer);
    viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);
    //viewer->addCoordinateSystem( 3.0, "coordinate" );
    //viewer->addPolygonMesh(pcd_to_poissonMesh(filename), filename);
    //viewer->addPointCloud(pcd_to_pointCloud(filename), filename);
    //viewer->addPolygonMesh(stl_to_mesh(filename), filename);
    viewer->addPointCloud(cloud, filename);
    viewer->setCameraPosition( 0.0, -0.1, 0.0, 0.0, 0.0, 1.0, 0 );
    viewer->setCameraFieldOfView(60.0 * M_PI / 180.0);
    viewer->spin();
    /*
    while( !viewer->wasStopped() ){
        // Update Viewer
        viewer->spinOnce();

        LiDar_Horizontal_Offset++;

        cloud = pcd_to_pointCloud(filename);
        boundary.calculation("UL", camera_Height, camera_Width, camera_Vertical_FOV, camera_Horizontal_FOV, object_X, object_Y, object_Height, object_Width, LiDar_Height_Offset, focal_Leftength);

        cout << boundary.LiDar_Up_Angle / M_PI * 180.0 << endl;
        cout << boundary.LiDar_Down_Angle / M_PI * 180.0 << endl;
        cout << boundary.LiDar_Left_Angle / M_PI * 180.0 << endl;
        cout << boundary.LiDar_Right_Angle / M_PI * 180.0 << endl;

        for(size_t i = 0; i < cloud->points.size(); i++)
        {
            if(!boundary.pointIsInside(cloud->points[i].getVector3fMap(), cameraAt, LiDar_Horizontal_Offset))
            {
                cloud->points.erase(cloud->points.begin() + i);
                i--;
            }
        }


        if( !viewer->updatePointCloud( cloud, filename) ){
            viewer->addPointCloud( cloud, filename);
        }
    }
    viewer->spin();
    //*/
/*
    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;
    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) );
    viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
    viewer->initCameraParameters();
    viewer->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 );

    pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;

        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<PointType>( "intensity" ) );
        handler = color_handler;

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
        [ &cloud, &mutex ]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
            boost::mutex::scoped_lock lock( mutex );

            cloud = ptr;
        };// VLP Grabber
    boost::shared_ptr<pcl::VLPGrabber> grabber;
        std::cout << "Capture from PCAP..." << std::endl;
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( filename ) );

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

    // Start Grabber
    grabber->start();

    while( !viewer->wasStopped() ){
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock( mutex );
        if( lock.owns_lock() && cloud ){
            // Update Point Cloud
            handler->setInputCloud( cloud );
            if( !viewer->updatePointCloud( cloud, *handler, "cloud" ) ){
                viewer->addPointCloud( cloud, *handler, "cloud" );
            }
        }
    }

    // Stop Grabber
    grabber->stop();

    // Disconnect Callback Function
    if( connection.connected() ){
        connection.disconnect();
    }*/
}

int main(int argc, char * argv[])
{

    std::vector<int> file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    if (file_indices.empty())
    {
    PCL_ERROR ("Please provide file as argument\n");
    return 1;
    }
    filename = argv[file_indices[0]];

    std::thread t1(timer1);
    std::thread t2(pcl_viewer);
    t1.join();
    t2.join();
    while(1);
    return 0;
}
