#define HAVE_BOOST
#define HAVE_PCAP
#define HAVE_FAST_PCAP
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
#include "../3rdparty/VelodyneCapture/VelodyneCapture.h"
#include "../include/function.h"
#include "../include/lasers.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

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

typedef pcl::PointXYZI PointType;

int main(int argc, char * argv[])
{
    std::vector<int> file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcap");
    if (file_indices.empty())
    {
        PCL_ERROR ("Please provide file as argument\n");
        return 1;
    }
    filename = argv[file_indices[0]];


    velodyne::VLP16Capture vlp16;

    if(!vlp16.open(filename))
    {
        std::cout << std::endl << "Error : load " << filename << " failed" << std::endl;
        return false;
    }

    if(!vlp16.isOpen())
    {
        std::cout << std::endl << "Error : load " << filename << " failed" << std::endl;
        return false;
    }

    pcl::PointXYZ point;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud;

    std::ofstream ofs("test.csv");
    ofs << "distance" << '\t';
    ofs << "azimuth" << '\t';
    ofs << "vertical" << '\t';
    ofs << std::endl;

    while(vlp16.isRun())
    {
        std::vector<velodyne::Laser> lasers;
        vlp16 >> lasers;
        vlp16 >> lasers;
        vlp16 >> lasers;
        vlp16 >> lasers;
        if( lasers.empty() ){
            continue;
        }

        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

        std::sort(lasers.begin(), lasers.end(), [] (auto a, auto b) { return a.vertical > b.vertical; });
        std::sort(lasers.begin(), lasers.end(), [] (auto a, auto b) { return a.azimuth > b.azimuth; });
        
        myLasers::Lines lines;

        for( const velodyne::Laser& laser : lasers ){
            const double distance = static_cast<double>( laser.distance );
            const double azimuth  = laser.azimuth  * M_PI / 180.0;
            const double vertical = laser.vertical * M_PI / 180.0;
        
            point.x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
            point.y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
            point.z = static_cast<float>( ( distance * std::sin( vertical ) ) );
        
          if( point.x == 0.0f && point.y == 0.0f && point.z == 0.0f ){
              continue;
          }
          lines.add(laser);
        }
        lines.remove_duplicate();
        lines.sort();
        lines.linear_interpolation(5);
        for( const myLasers::Line& line : lines.lines){
          for( const velodyne::Laser& laser : line.lasers ){
            const double distance = static_cast<double>( laser.distance );
            const double azimuth  = laser.azimuth  * M_PI / 180.0;
            const double vertical = laser.vertical * M_PI / 180.0;
        
            point.x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
            point.y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
            point.z = static_cast<float>( ( distance * std::sin( vertical ) ) );
        
            //point.intensity = laser.intensity;

            if( point.x == 0.0f && point.y == 0.0f && point.z == 0.0f ){
                continue;
                point.x = std::numeric_limits<float>::quiet_NaN();
                point.y = std::numeric_limits<float>::quiet_NaN();
                point.z = std::numeric_limits<float>::quiet_NaN();
            }

            ofs << laser.distance << '\t';
            ofs << laser.azimuth << '\t';
            ofs << laser.vertical << '\t';
            ofs << std::endl;
        
            cloud->points.push_back(point);
          }
        }
        /*
        for( const velodyne::Laser& laser : lasers ){
            const double distance = static_cast<double>( laser.distance );
            const double azimuth  = laser.azimuth  * M_PI / 180.0;
            const double vertical = laser.vertical * M_PI / 180.0;
        
            point.x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
            point.y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
            point.z = static_cast<float>( ( distance * std::sin( vertical ) ) );
        
            //point.intensity = laser.intensity;

            if( point.x == 0.0f && point.y == 0.0f && point.z == 0.0f ){
                continue;
                point.x = std::numeric_limits<float>::quiet_NaN();
                point.y = std::numeric_limits<float>::quiet_NaN();
                point.z = std::numeric_limits<float>::quiet_NaN();
            }

            ofs << laser.distance << '\t';
            ofs << laser.azimuth << '\t';
            ofs << laser.vertical << '\t';
            ofs << std::endl;
        
            cloud->points.push_back(point);
        }*/
        cloud->width = static_cast<uint32_t>(cloud->points.size());
        cloud->height = 1;
        break;
    }

    //pcl::PCLPointCloud2 cloud_blob;
    //pcl::io::loadPCDFile(filename, cloud_blob);
    //pcl::fromPCLPointCloud2(cloud_blob, *cloud);
    viewer.reset(new PCLVisualizer);
    viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);
    //viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setCameraPosition( 0.00001, 0.0, 0.0, 0.0, 0.0, 1.0, 0 );
    viewer->setCameraFieldOfView(30.0 * M_PI / 180.0);
    //viewer->addPolygonMesh(mesh, filename);
    /*
    for(int i = 0 ; i < cloud->points.size(); i++) 
    {
      if((isnan(cloud->points[i].x))||(isnan(cloud->points[i].y))||(isnan(cloud->points[i].z)))
      {
        cloud->points.erase(cloud->points.begin() + i);
        i--;
        continue;
      }
    }
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(myFunction::XYZ_to_XYZRGB<pcl::PointXYZ>(cloud, false));
    //viewer->addPointCloud(cloud, filename);
    myFunction::showCloud(viewer, cloud_rgb, "rgb", 1);
    viewer->spin();
    return 0;
}
