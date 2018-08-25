
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
#include <pcl/common/common_headers.h>
#include <thread>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/time.h>
#include "../include/function.h"
#include "../include/boundary.h"
#include "../include/test.h"
#include "../include/args.hxx"

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;
using namespace Eigen;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

int fps = 2;
bool viewer_pause = false;
std::string filename;

void keyboardEventOccurred(const KeyboardEvent& event, void* nothing)
{
    switch(event.getKeyCode())
    {
        case ' ':
            viewer_pause = !viewer_pause;
            break;
        default:
            std::cerr << "Keyboard pressed: " << (int)(event.getKeyCode()) << std::endl;
    }
}

typedef pcl::PointXYZ PointT;

void pcl_viewer()
{
    myClass::Boundary boundary;
    pcl::console::TicToc tt;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    std::cerr << "Loading point cloud...", tt.tic();
    if(pcl::io::loadPCDFile (filename, *cloud) == -1)return;
    std::cerr << " >> Done: " << tt.toc() << " ms\n";
    std::cerr << "Point cloud: " << cloud->points.size() << " points" << std::endl << std::endl;

    Vector3f cameraAt = Vector3f(0.0, 0.0, 0.0);
    double camera_Height = sqrt(3.0)*2.0;
    double camera_Width = sqrt(3.0)*2.0;
    double camera_Vertical_FOV = 60.0 * M_PI / 180.0;
    double camera_Horizontal_FOV = 180.0 * M_PI / 180.0;
    double LiDar_Height_Offset = 0.0;
    double focal_Leftength_Offset = 5.0;
    double LiDar_Horizontal_Offset = 0;
    double object_X = 0;
    double object_Y = 0;
    double object_Height = sqrt(3.0)*2.0;
    double object_Width = sqrt(3.0)*2.0;

    cout << "Please input camera_Horizontal_FOV (0 ~ 180): "; 
    cin >> camera_Horizontal_FOV;
    camera_Horizontal_FOV = camera_Horizontal_FOV * M_PI / 180.0;

    boundary.setCameraParameter(cameraAt, "UL", camera_Height, camera_Width, camera_Vertical_FOV, camera_Horizontal_FOV);
    boundary.setLiDarParameter(LiDar_Height_Offset, LiDar_Horizontal_Offset, focal_Leftength_Offset);
    boundary.setBound(object_X, object_Y, object_Height, object_Width);

    std::cerr << "Dividing point cloud...", tt.tic();
    cloud = boundary.division<PointT>(cloud);
    std::cerr << " >> Done: " << tt.toc() << " ms\n";
    std::cerr << "Point cloud after division: " << cloud->points.size() << " points" << std::endl << std::endl;

/*
    std::cerr << "test...", tt.tic();
    for(int i = 0; i < 40; i++)
    {
        auto a = myFunction::getFarthestPoint(cloud);
    }
    std::cerr << " >> Done: " << tt.toc() << " ms\n";
    return;
    */
    viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    
    std::cerr << "XYZ to XYZRGB...", tt.tic();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_view = myFunction::XYZ_to_XYZRGB(cloud);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_view);
    std::cerr << " >> Done: " << tt.toc() << " ms\n";

    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_view, rgb, filename);

    viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);
    //viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, filename);
    viewer->setCameraPosition( 0.0, -0.1, 0.0, 0.0, 0.0, 1.0, 0 );
    viewer->setCameraFieldOfView(45.0 * M_PI / 180.0);
    viewer->spin();
}

int main(int argc, char * argv[])
{
    args::ArgumentParser parser("This is a test program.", "This goes after the options.");
    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
    args::Group group(parser, "This group is all required:", args::Group::Validators::All);

    args::ValueFlag<std::string> input(group, "CLOUD_IN", "input PointCloud path", {'i', "if"});

    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch (args::Help)
    {
        std::cout << parser;
        return 0;
    }
    catch (args::ParseError e)
    {
        std::cerr << e.what() << std::endl;
        parser.Help(std::cerr);
        return 1;
    }
    catch (args::ValidationError e)
    {
        std::cerr << e.what() << std::endl;
        parser.Help(std::cerr);
        return 1;
    }

    filename = args::get(input);

    pcl_viewer();

    return 0;
}
