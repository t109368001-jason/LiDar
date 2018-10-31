#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <thread>
#include <pcl/io/vlp_grabber.h>
#include "../include/function.h"
#include "../include/segmentation.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

PCLVisualizer::Ptr viewer;
std::string filename;
typedef pcl::PointXYZI PointType;

void pcl_viewer()
{
    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;
    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) );
    viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
    viewer->initCameraParameters();
    viewer->setSize(1600, 900);
    viewer->setCameraPosition( 0.0, 0.0, 20.0, 0.0, 1.0, 1.0, 0 );

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

    myClass::backgroundSegmentation<PointType> backgroundSegmentation;

    pcl::PointCloud<PointType>::Ptr background(new pcl::PointCloud<PointType>);
    pcl::io::loadPCDFile("../file/background_2018-10-29-18-58-59_Velodyne-VLP-16-Data.pcd", *background);

    double resolution;
    std::cerr << "Input resolution : ";
    cin >> resolution;

    backgroundSegmentation.setBackground(background, resolution);

    uint32_t seq = std::numeric_limits<uint32_t>::min();

    // Start Grabber
    grabber->start();
    pcl::PointCloud<PointType>::Ptr temp(new pcl::PointCloud<PointType>);
    while( !viewer->wasStopped() ){
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock( mutex );
        if( lock.owns_lock() && cloud ){
            // Update Point Cloud
            *temp = *cloud;
            temp = backgroundSegmentation.compute(temp);
            handler->setInputCloud( temp );
            if( !viewer->updatePointCloud( temp, *handler, "cloud" ) ){
                viewer->addPointCloud( temp, *handler, "cloud" );
            }


            if(seq <= cloud->header.seq)
            {
                if(seq != cloud->header.seq)
                {
                    std::stringstream ss;
                    ss << cloud->header.seq;
                    ss << ".png";
                    viewer->saveScreenshot("video/" + ss.str());    

                    double x_max = std::numeric_limits<double>::min();
                    double y_max = std::numeric_limits<double>::min();
                    double z_max = std::numeric_limits<double>::min();
                    for(auto it : temp->points)
                    {
                        x_max = std::fmax(it.x, x_max);
                        y_max = std::fmax(it.y, y_max);
                        z_max = std::fmax(it.z, z_max);
                    }
                    std::cerr << "frame_id : " << cloud->header.frame_id << std::endl;
                    std::cerr << "seq : " << cloud->header.seq << std::endl;
                    std::cerr << "stamp : " << cloud->header.stamp << std::endl;
                    std::cerr << "convert to millis : " << myFunction::millisecondToString(std::chrono::milliseconds(cloud->header.stamp*1000), false) << std::endl;
                    
                    std::cerr << "x_max : " << x_max << std::endl;
                    std::cerr << "y_max : " << y_max << std::endl;
                    std::cerr << "z_max : " << z_max << std::endl;  
                    seq = cloud->header.seq;
                }   
            }
            else
            {
                break;
            }
        }
    }

    // Stop Grabber
    grabber->stop();

    // Disconnect Callback Function
    if( connection.connected() ){
        connection.disconnect();
    }
}

int main(int argc, char * argv[])
{
  std::vector<int> file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcap");
  if (file_indices.empty())
  {
    PCL_ERROR ("Please provide file as argument\n");
    return 1;
  }
  filename = argv[file_indices[0]];

  pcl_viewer();

  return 0;
}
