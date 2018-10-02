#define CAPACITY 10
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/console/time.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include "../include/args.hxx"
#include "../include/segmentation.h"
#include "../include/function.h"
#include "../include/microStopwatch.h"
#include "../include/custom_frame.h"

typedef pcl::PointXYZRGB PointT;

int fps = 2;
bool viewer_pause = false;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
std::string cwd = std::string(getcwd(NULL, 0)) + '/';

args::ArgumentParser parser("This is a test program.", "This goes after the options.");
args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
args::Group group(parser, "This group is all required:", args::Group::Validators::All);
args::ValueFlag<std::string> inputBag(group, "CLOUD_IN", "input bag path", {'i', "if"});
args::ValueFlag<std::string> inputBagBackground(group, "CLOUD_IN", "input background cloud path", {'b', "bf"});

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);

void pcl_viewer()
{
    double firstFrameTimeStamp;

    std::vector<pcl::visualization::Camera> cameras;
    
    //rs2::pipeline pipe;
    myClass::MicroStopwatch tt("main");
    myClass::objectSegmentation<PointT> object_segmentation;
    myClass::backgroundSegmentation<PointT> background_segmentation;
    std::vector<boost::shared_ptr<myFrame::CustomFrame<PointT>>> customFrames;
    
    std::string backgroundCloudFile = args::get(inputBagBackground);
    std::string bagFile = args::get(inputBag);
    std::string bridgeFile = "/home/xian-jie/workspace/github/tmp.txt";
    std::string tmp_dir = "tmp/";
    std::string output_dir = "data/";
    
    if(!myFunction::fileExists(bridgeFile))
    {
        std::cerr << "bridge file not found" << std::endl;
        return;
    }
    if(!myFunction::fileExists(bagFile))
    {
        std::cerr << "bag file not found" << std::endl;
        return;
    }
    if(!myFunction::fileExists(tmp_dir))
    {
        mkdir(tmp_dir.c_str(), 0777);
    }
    if(!myFunction::fileExists(output_dir))
    {
        mkdir(output_dir.c_str(), 0777);
    }

    
    pcl::PointCloud<PointT>::Ptr backgroundCloud(new pcl::PointCloud<PointT>);
    ////////////////////////////////////////////////////////////////
    std::cerr << "Load bag file...", tt.tic();

    myFrame::getCustomFrames(bagFile, customFrames, bridgeFile, tmp_dir);

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    std::cerr << '\n';
    ////////////////////////////////////////////////////////////////*/

    ////////////////////////////////////////////////////////////////
    std::cerr << "save custom frames...", tt.tic();
    for(int i = 0; i < customFrames.size(); i++)
    {
        customFrames[i]->save(output_dir);
    }
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    std::cerr << '\n';
    ////////////////////////////////////////////////////////////////*/

    ////////////////////////////////////////////////////////////////
    std::cerr << "load custom frames...", tt.tic();
    myFrame::loadCustomFrames(output_dir, customFrames);
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    ////////////////////////////////////////////////////////////////*/
    return;
    ////////////////////////////////////////////////////////////////
    double w = 1280.0;
    double h = 720.0;
    std::cerr << "Point cloud object segmentation...", tt.tic();
    object_segmentation.setCameraParameter("CC", w, h, 89.7974 * M_PI / 180.0, 89.7974 * M_PI / 180.0);
    object_segmentation.setBound(0.0, 0.0, 640.0, 360.0);
    
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << '\n';
    ////////////////////////////////////////////////////////////////*/

    ////////////////////////////////////////////////////////////////
    std::cerr << "Loading point cloud...", tt.tic();
    if(pcl::io::loadPCDFile (backgroundCloudFile, *backgroundCloud) == -1)return;
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "backgroundCloud: " << backgroundCloud->points.size() << " points\n";
    std::cerr << '\n';
    ////////////////////////////////////////////////////////////////*/
    

    /*///////////////////////////////////////////////////////////////
    //double resolution;
    //cout << "Please input background segmentation resolution: "; 
    //cin >> resolution;
    std::cerr << "Point cloud background segmentation...", tt.tic();
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr backgroundCloud_view = myFunction::fillColor<PointT>(backgroundCloud, 255, 0, 0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr backgroundCloud_view = myFunction::XYZ_to_XYZRGB<PointT>(backgroundCloud);
    background_segmentation.setBackground(backgroundCloud);
    //myFunction::showCloud(viewer, backgroundCloud_view, "backgroundCloud");

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << '\n';
    ////////////////////////////////////////////////////////////////*/
    
    viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setSize(1280, 720);
    viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);
    //viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setCameraPosition( 0.0, 0.0, -0.0000001, 0.0, -1.0, 0.0, 0 );
    viewer->setCameraFieldOfView(60.0 * M_PI / 180.0);

    viewer->spinOnce();
    viewer->getCameras(cameras);

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp = object_segmentation.division(cameras[0], backgroundCloud, false);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr backgroundCloud_view = myFunction::fillColor<PointT>(backgroundCloud, 255, 0, 0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr backgroundCloud_view = myFunction::XYZ_to_XYZRGB<PointT>(backgroundCloud, false);

    myFunction::showCloud(viewer, backgroundCloud_view, "Cloud_view1");

    viewer->spinOnce();

    while( !viewer->wasStopped())
    {
        viewer->spinOnce();
        viewer->getCameras(cameras);

        myFunction::updateCloud(viewer, object_segmentation.division(cameras[0], backgroundCloud_view, false), "Cloud_view1");

        //myFunction::printCamera(cameras[0]);
        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
    }
    /*///////////////////////////////////////////////////////////////
    auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH)
                                .as<rs2::video_stream_profile>();
    auto i = depth_stream.get_intrinsics();
    float fov[2]; // X, Y fov
    rs2_fov(&i, fov);
    std::cerr << "fx" << fov[0] << std::endl;
    std::cerr << "fy" << fov[1] << std::endl;
    ////////////////////////////////////////////////////////////////*/

    /*///////////////////////////////////////////////////////////////
    double division_Horizontal_FOV;
    cout << "Please input division_Horizontal_FOV (0 ~ 360): "; 
    cin >> division_Horizontal_FOV;
    division_Horizontal_FOV = division_Horizontal_FOV * M_PI / 180.0;

    std::cerr << "Point cloud object segmentation...", tt.tic();
    object_segmentation.setCameraParameter("UL", 1280, 720, division_Horizontal_FOV);
    
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Point cloud after object segmentation: " << myFunction::commaFix(cloud->points.size()) << " points\n";
    std::cerr << '\n';
    ////////////////////////////////////////////////////////////////*/

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

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
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

