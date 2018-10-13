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

int fps = 30;
bool viewer_pause = false;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
std::string cwd = std::string(getcwd(NULL, 0)) + '/';

args::ArgumentParser parser("This is a test program.", "This goes after the options.");
args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
args::Group group(parser, "This group is all required:", args::Group::Validators::All);
args::ValueFlag<std::string> inputBag(group, "inputBag", "input bag path", {'i', "if"});
args::ValueFlag<std::string> inputBagBackground(group, "inputBagBackground", "input background cloud path", {'b', "bf"});
args::ValueFlag<std::string> yolo(group, "yolo", "yolo path", {'y', "yp"});
args::ValueFlag<std::string> output(group, "output", "output path", {'o', "od"});
args::ValueFlag<std::string> tmp(group, "tmp", "tmp path", {'t', "td"});

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);

void pcl_viewer()
{
    double firstFrameTimeStamp;

    //std::vector<pcl::visualization::Camera> cameras;
    
    myClass::MicroStopwatch tt("main");
    myClass::objectSegmentation<PointT> object_segmentation;
    myClass::backgroundSegmentation<PointT> background_segmentation;
    std::vector<boost::shared_ptr<myFrame::CustomFrame<PointT>>> customFrames;
    pcl::PointCloud<PointT>::Ptr backgroundCloud(new pcl::PointCloud<PointT>);
    
    std::string backgroundCloudFile = args::get(inputBagBackground);
    std::string bagFile = args::get(inputBag);
    std::string bridgeFile = args::get(yolo);
    std::string tmp_dir = args::get(tmp);
    std::string data_dir = args::get(output);

    ////////// check file exists //////////////////////////////////////////////////////
    if(!myFunction::fileExists(backgroundCloudFile))
    {
        std::cerr << "background cloud file not found" << std::endl;
        return;
    }
    if(!myFunction::fileExists(bagFile))
    {
        std::cerr << "bag file not found" << std::endl;
        return;
    }
    if(!myFunction::fileExists(bridgeFile))
    {
        std::cerr << "bridge file not found" << std::endl;
        return;
    }
    ////////////////////////////////////////////////////////////////*/

    ////////// create output directory //////////////////////////////////////////////////////
    if(!myFunction::fileExists(tmp_dir))
    {
        mkdir(tmp_dir.c_str(), 0777);       //make directory tmp_dir and set permission to 777
    }
    if(!myFunction::fileExists(data_dir))
    {
        mkdir(data_dir.c_str(), 0777);      //make directory data_dir and set  permission to 777
    }
    ////////////////////////////////////////////////////////////////*/

    if(pcl::io::loadPCDFile(backgroundCloudFile, *backgroundCloud) == -1)return;

    /*///////// load bag file //////////////////////////////////////////////////////
    std::cerr << "Load bag file...", tt.tic();

    myFrame::getCustomFrames(bagFile, customFrames, bridgeFile, tmp_dir);

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    std::cerr << '\n';
    ////////////////////////////////////////////////////////////////*/
    
    /*///////// save to data directory //////////////////////////////////////////////////////
    std::cerr << "save custom frames...", tt.tic();

    myFrame::saveCustomFrames(data_dir, customFrames);

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    std::cerr << '\n';
    return;
    ////////////////////////////////////////////////////////////////*/
    
    ////////// test //////////////////////////////////////////////////////

    std::ofstream ofs("noiseRemoval.csv", ios::ate);
    ofs << "resolution" << "," << "meanK" << "," << "StddevMulThresh" << "," << "FPS" << "," << "remain(\%)" << std::endl;

    background_segmentation.setBackground(backgroundCloud, 0.01, true);
    for(int i = 10; i < 100; i += 10)
    {
        std::vector<boost::shared_ptr<myFrame::CustomFrame<PointT>>> testCustomFrames;
        myFrame::loadCustomFrames(data_dir, testCustomFrames, false);

        myFrame::backgroundSegmentationCustomFrames(background_segmentation, testCustomFrames);
        double before = 0;
        for(int j = 0; j < testCustomFrames.size(); j++)
        {
            before += testCustomFrames[j]->entire_cloud->points.size();
        }
        
        std::cerr << "custom frames noise removal(meanK = " << i << ", StddevMulThresh = " << 0.01 << ")..."; tt.tic();
        myFrame::noiseRemovalCustomFrames(testCustomFrames, i, 0.01);
        std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    
        double after = 0;
        for(int j = 0; j < testCustomFrames.size(); j++)
        {
            after += testCustomFrames[j]->entire_cloud->points.size();
        }

        ofs << 0.01 << "," << i << "," << 0.01 << "," << testCustomFrames.size() / (tt.toc_pre() / 1000000.0) << "," << after/before << std::endl;
    }
    for(int i = 100; i < 1000; i += 100)
    {
        std::vector<boost::shared_ptr<myFrame::CustomFrame<PointT>>> testCustomFrames;
        myFrame::loadCustomFrames(data_dir, testCustomFrames, false);

        myFrame::backgroundSegmentationCustomFrames(background_segmentation, testCustomFrames);
        double before = 0;
        for(int j = 0; j < testCustomFrames.size(); j++)
        {
            before += testCustomFrames[j]->entire_cloud->points.size();
        }
        
        std::cerr << "custom frames noise removal(meanK = " << i << ", StddevMulThresh = " << 0.01 << ")..."; tt.tic();
        myFrame::noiseRemovalCustomFrames(testCustomFrames, i, 0.01);
        std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    
        double after = 0;
        for(int j = 0; j < testCustomFrames.size(); j++)
        {
            after += testCustomFrames[j]->entire_cloud->points.size();
        }

        ofs << 0.01 << "," << i << "," << 0.01 << "," << testCustomFrames.size() / (tt.toc_pre() / 1000000.0) << "," << after/before << std::endl;
    }
    ofs.close();
    return;
    ////////////////////////////////////////////////////////////////*/

    ////////// load from data directory //////////////////////////////////////////////////////
    std::cerr << "load custom frames...", tt.tic();

    myFrame::loadCustomFrames(data_dir, customFrames, false);

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    ////////////////////////////////////////////////////////////////*/

    ////////// custom frames background segmentation //////////////////////////////////////////////////////
    double resolution;
    cout << "Please input background segmentation resolution: "; 
    cin >> resolution;
    std::cerr << "Point cloud background segmentation...", tt.tic();
    background_segmentation.setBackground(backgroundCloud, resolution, true);

    myFrame::backgroundSegmentationCustomFrames(background_segmentation, customFrames);

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    ////////////////////////////////////////////////////////////////*/
    
    ////////// custom frames noise removal //////////////////////////////////////////////////////
    std::cerr << "custom frames noise removal...", tt.tic();

    myFrame::noiseRemovalCustomFrames(customFrames, 2000, 0.01);

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    ////////////////////////////////////////////////////////////////*/

    ////////// custom frames object segmentation //////////////////////////////////////////////////////
    std::cerr << "Custom frames object segmentation...", tt.tic();
    double w = 1280.0;
    double h = 720.0;
    object_segmentation.setCameraParameter("UL", w, h, 89.7974 * M_PI / 180.0, 69.4 * M_PI / 180.0, -0.03);
    
    myFrame::objectSegmentationCustomFrames(tmp_dir, object_segmentation, customFrames);

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    std::cerr << '\n';
    ////////////////////////////////////////////////////////////////*/

    /*///////// save to data directory //////////////////////////////////////////////////////
    std::cerr << "save custom frames...", tt.tic();

    myFrame::saveCustomFrames(data_dir, customFrames);

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    std::cerr << '\n';
    return;
    ////////////////////////////////////////////////////////////////*/
    
    /*///////// load from data directory //////////////////////////////////////////////////////
    std::cerr << "load custom frames...", tt.tic();

    myFrame::loadCustomFrames(data_dir, customFrames, true);

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    ////////////////////////////////////////////////////////////////*/

    viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setSize(1280, 720);
    viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);
    //viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setCameraPosition( 0.0, 0.0, -0.0000001, 0.0, -1.0, 0.0, 0 );
    viewer->setCameraFieldOfView(60.0 * M_PI / 180.0);

    int play_count = 0;

    std::sort(customFrames.begin(), customFrames.end(), [](const boost::shared_ptr<myFrame::CustomFrame<PointT>> &a, const boost::shared_ptr<myFrame::CustomFrame<PointT>> &b) { return a->file_name < b->file_name; });

    viewer << *customFrames[play_count];
    
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr backgroundCloud_view = myFunction::fillColor<PointT>(backgroundCloud, 255, 0, 0);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr backgroundCloud_view = myFunction::XYZ_to_XYZRGB<PointT>(backgroundCloud, false);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr backgroundCloud_view = myFunction::XYZ_to_XYZRGB<PointT>(backgroundCloud);

    //myFunction::showCloud(viewer, backgroundCloud_view, "backgroundCloud_view");
    
    viewer->spinOnce();

    while( !viewer->wasStopped())
    {
        viewer->spinOnce();
        if(!viewer_pause)
        {
            viewer -= *customFrames[play_count];
            if(++play_count == customFrames.size()) play_count = 0;
            viewer << *customFrames[play_count];
        }
        //boost::this_thread::sleep(boost::posix_time::milliseconds(16));
    }

    //std::chrono::milliseconds start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-std::chrono::time_point<std::chrono::system_clock>(std::chrono::milliseconds(int64_t(0))));
    
    /*///////////////////////////////////////////////////////////////
    auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH)
                                .as<rs2::video_stream_profile>();
    auto i = depth_stream.get_intrinsics();
    float fov[2]; // X, Y fov
    rs2_fov(&i, fov);
    std::cerr << "fx" << fov[0] << std::endl;
    std::cerr << "fy" << fov[1] << std::endl;
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
    if(event.keyDown())
    {
        switch(event.getKeyCode())
        {
            case ' ':
                viewer_pause = !viewer_pause;
                std::cerr << "viewer_pause: " << viewer_pause << std::endl;
                break;
            default:
                std::cerr << "Keyboard pressed: " << (int)(event.getKeyCode()) << std::endl;
        }
    }
}

