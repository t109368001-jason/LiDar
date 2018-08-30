
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/console/time.h>
#include <boost/format.hpp>
#include "../include/args.hxx"
#include "../include/segmentation.h"
#include "../include/function.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;
using namespace Eigen;

class MicroStopwatch
{
    public:
        boost::posix_time::ptime tictic;
        boost::posix_time::ptime toctoc;
        void tic()
        {
            tictic = boost::posix_time::microsec_clock::local_time ();
        }
        int64_t toc()
        {
            toctoc = boost::posix_time::microsec_clock::local_time ();
            return (toctoc - tictic).total_microseconds();
        }
        std::string toc_string()
        {
            toctoc = boost::posix_time::microsec_clock::local_time ();
            return myFunction::commaFix((toctoc - tictic).total_microseconds());
        }
};

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

int fps = 2;
bool viewer_pause = false;

args::ArgumentParser parser("This is a test program.", "This goes after the options.");
args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
args::Group group(parser, "This group is all required:", args::Group::Validators::All);
args::ValueFlag<std::string> input(group, "CLOUD_IN", "input PointCloud path", {'i', "if"});


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
    MicroStopwatch tt;
    myClass::objectSegmentation<PointT> object_segmentation;
    myClass::backgroundSegmentation<PointT> background_segmentation;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
    std::string filename = args::get(input);
    std::string filename1 = "../file/123/2.pcd";
    std::string filename2 = "../file/123/3.pcd";

    ////////////////////////////////////////////////////////////////
    std::cerr << "Loading point cloud...", tt.tic();
    if(pcl::io::loadPCDFile (filename, *cloud) == -1)return;
    if(pcl::io::loadPCDFile (filename1, *cloud1) == -1)return;
    if(pcl::io::loadPCDFile (filename2, *cloud2) == -1)return;
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Point cloud: " << cloud->points.size() << " points\n";
    std::cerr << "Point cloud1: " << cloud1->points.size() << " points\n";
    std::cerr << "Point cloud2: " << cloud2->points.size() << " points\n";
    std::cerr << '\n';
    ////////////////////////////////////////////////////////////////*/

    ////////////////////////////////////////////////////////////////
    double division_Horizontal_FOV;
    cout << "Please input division_Horizontal_FOV (0 ~ 360): "; 
    cin >> division_Horizontal_FOV;
    division_Horizontal_FOV = division_Horizontal_FOV * M_PI / 180.0;

    std::cerr << "Point cloud object segmentation...", tt.tic();
    cloud = object_segmentation.easyDivision(cloud, division_Horizontal_FOV, M_PI);
    cloud1 = object_segmentation.easyDivision(cloud1, division_Horizontal_FOV, M_PI);
    cloud2 = object_segmentation.easyDivision(cloud2, division_Horizontal_FOV, M_PI);
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Point cloud after object segmentation: " << myFunction::commaFix(cloud->points.size()) << " points\n";
    std::cerr << "Point cloud1 after object segmentation: " << myFunction::commaFix(cloud1->points.size()) << " points\n";
    std::cerr << "Point cloud2 after object segmentation: " << myFunction::commaFix(cloud2->points.size()) << " points\n";
    std::cerr << '\n';
    ////////////////////////////////////////////////////////////////*/

    /*///////////////////////////////////////////////////////////////
    double temp;
    cout << "Please input background segmentation resolution: "; 
    cin >> temp;
    pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);
    background_segmentation.setBackground(cloud, temp);
    std::cerr << "test1...", tt.tic();
    for(int i = 0; i < 100; i++)
    {  
        cloud_temp = background_segmentation.compute(cloud1);
	}
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << (1-((double)cloud_temp->points.size()/(double)cloud1->points.size()))*100 << "%" << std::endl;
    std::cerr << '\n';
    return;
    //////////////////////////////////////////////////////////////////*/

    ////////////////////////////////////////////////////////////////
    double resolution;
    cout << "Please input background segmentation resolution: "; 
    cin >> resolution;
    std::cerr << "Point cloud background segmentation...", tt.tic();
    background_segmentation.setBackground(cloud, resolution);
    cloud1 = background_segmentation.compute(cloud1);
    cloud2 = background_segmentation.compute(cloud2);
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Point cloud1 after background segmentation: " << myFunction::commaFix(cloud1->points.size()) << " points\n";
    std::cerr << "Point cloud2 after background segmentation: " << myFunction::commaFix(cloud2->points.size()) << " points\n";
    std::cerr << '\n';
    ////////////////////////////////////////////////////////////////*/
    
    viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    
    ////////////////////////////////////////////////////////////////
    std::cerr << "XYZ to XYZRGB...", tt.tic();
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_view = myFunction::XYZ_to_XYZRGB<PointT>(cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_view = myFunction::fillColor<PointT>(cloud, 255, 0, 0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_view1 = myFunction::fillColor<PointT>(cloud1, 0, 255, 0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_view2 = myFunction::fillColor<PointT>(cloud2, 0, 0, 255);
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << '\n';
    ////////////////////////////////////////////////////////////////*/

    myFunction::showCloud(viewer, cloud_view, filename);
    myFunction::showCloud(viewer, cloud_view1, "cloud1");
    myFunction::showCloud(viewer, cloud_view2, "cloud2");

    //viewer->addPointCloud(cloud, filename);
    viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);
    //viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setCameraPosition( -0.1, 0.0, 0.0, 0.0, 0.0, 1.0, 0 );
    viewer->setCameraFieldOfView(45.0 * M_PI / 180.0);
    viewer->spin();
}

int main(int argc, char * argv[])
{
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

    pcl_viewer();

    return 0;
}
