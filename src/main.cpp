#define CAPACITY 10
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/console/time.h>
#include <boost/format.hpp>
#include "../include/args.hxx"
#include "../include/segmentation.h"
#include "../include/function.h"
#include "../include/class.h"

typedef pcl::PointXYZ PointT;

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
    rs2::config cfg;
    rs2::pointcloud pc;
    rs2::points points;
    auto pipe = std::make_shared<rs2::pipeline>();
    
    //rs2::pipeline pipe;
    myClass::MicroStopwatch tt("main");
    myClass::objectSegmentation2<PointT> object_segmentation;
    myClass::backgroundSegmentation<PointT> background_segmentation;
    
    std::string backgroundCloudFile = args::get(inputBagBackground);
    std::string bagFile = args::get(inputBag);
    std::string bridgeFile = "/home/xian-jie/workspace/github/tmp.txt";
    
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

    cfg.enable_device_from_file(bagFile);
    std::chrono::milliseconds bagStartTime = myFunction::bagFileNameToMilliseconds(bagFile);

    pcl::PointCloud<PointT>::Ptr backgroundCloud(new pcl::PointCloud<PointT>);
    std::vector<boost::shared_ptr<myClass::CustomFrame<PointT>>> customFrames;

    rs2::pipeline_profile selection = pipe->start(cfg);
    
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
    std::cerr << "Load bag file...\n", tt.tic();
    auto device = pipe->get_active_profile().get_device();
    rs2::playback playback = device.as<rs2::playback>();
    playback.set_real_time(false);

    auto duration = playback.get_duration();
    int progress = 0;
    auto frameNumber = 0ULL;
    myClass::MicroStopwatch tt1("tt1");

    while (true) 
    {
        playback.resume();
        auto frameset = pipe->wait_for_frames();
        playback.pause();

        int posP = static_cast<int>(playback.get_position() * 100. / duration.count());
        
        if (frameset[0].get_frame_number() < frameNumber) {
            //std::cerr << "100%" << std::endl;
            break;
        }

        if(frameNumber == 0ULL)
        {
            bagStartTime -= std::chrono::milliseconds(int64_t(frameset.get_timestamp()));
        }

        boost::shared_ptr<myClass::CustomFrame<PointT>> customFrame(new myClass::CustomFrame<PointT>);

        if(customFrame->set(frameset, bagStartTime, bridgeFile))
        {
            customFrames.push_back(customFrame);
        }
        frameNumber = frameset[0].get_frame_number();
    }
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << customFrames.size() << std::endl;
    std::cerr << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    return;
    ////////////////////////////////////////////////////////////////*/

    ////////////////////////////////////////////////////////////////
    std::cerr << "Loading point cloud...", tt.tic();
    if(pcl::io::loadPCDFile (backgroundCloudFile, *backgroundCloud) == -1)return;
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "backgroundCloud: " << backgroundCloud->points.size() << " points\n";
    std::cerr << '\n';
    ////////////////////////////////////////////////////////////////*/
    
    /*///////////////////////////////////////////////////////////////
    double w = 1280.0;
    double h = 720.0;
    std::cerr << "Point cloud object segmentation...", tt.tic();
    object_segmentation.setCameraParameter("UL", w, h, 89.7974 * M_PI / 180.0, 89.7974 * M_PI / 180.0);
    object_segmentation.setBound(960.0, 180.0, 640.0, 360.0);
    backgroundCloud = object_segmentation.division(backgroundCloud, false);
    object_segmentation.printAngle();
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Point cloud after object segmentation: " << myFunction::commaFix(backgroundCloud->points.size()) << " points\n";
    std::cerr << '\n';
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

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr backgroundCloud_view = myFunction::fillColor<PointT>(backgroundCloud, 255, 0, 0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr backgroundCloud_view = myFunction::fillColor<PointT>(backgroundCloud, 255, 0, 0);

    myFunction::showCloud(viewer, backgroundCloud_view, "Cloud_view1");

    viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);
    //viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setCameraPosition( 0.0, 0.0, -0.0000001, 0.0, -1.0, 0.0, 0 );
    viewer->setCameraFieldOfView(60.0 * M_PI / 180.0);
    viewer->spinOnce();

    std::vector<pcl::visualization::Camera> cameras;

    while( !viewer->wasStopped())
    {
        viewer->spinOnce();
        viewer->getCameras(cameras);
        myFunction::printCamera(cameras[0]);

        double r;
        double phi;
        double theta;
        myFunction::XYZ_to_Sphere(r, phi, theta, cameras[0].view[0], cameras[0].view[1], cameras[0].view[2]);
        std::cerr << " - phi: " << phi*180.0/M_PI << std::endl;
        std::cerr << " - theta: " << theta*180.0/M_PI << std::endl;
        boost::this_thread::sleep(boost::posix_time::milliseconds(16));
        /*
        if (pipe->try_wait_for_frames(frames))
        {
            viewer->spinOnce();

            std::chrono::milliseconds currentTime = bagStartTime + std::chrono::milliseconds(int64_t(frames->get_timestamp() - firstFrameTimeStamp));
            if( !viewer->updateText(myFunction::millisecondToString(currentTime, false), 200, 500, "current time"))
            {
                viewer->addText(myFunction::millisecondToString(currentTime, false), 200, 500, "current time");
            }

    ////////////////////////////////////////////////////////////////
            rs2::video_frame vf = frames->get_color_frame();
            rs2::colorizer color_map;
            auto stream = frames->get_profile().stream_type();
            // Use the colorizer to get an rgb image for the depth stream
            if (vf.is<rs2::depth_frame>()) vf = color_map.process(*frames);

            // Write images to disk
            std::stringstream png_file;
            png_file << cwd << "tmp/" << myFunction::millisecondToString(currentTime) << ".png";
            stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                        vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());

            while(1)
            {
                std::ifstream ifs;
                ifs.open(bridgeFile);
                std::string line;
                if(ifs.is_open())
                {
                    std::getline(ifs, line);
                    ifs.close();
                }
                if(line == "")
                {
                    std::ofstream ofs;
                    ofs.open(bridgeFile);
                    ofs.write(png_file.str().c_str(), png_file.str().size());
                    ofs.close();
                    break;
                }
                else
                {
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                }
            }
    /////////////////////////////////////////////////////////////////

            auto depth = frames->get_depth_frame();
            points = pc.calculate(depth);

            pcl::PointCloud<PointT>::Ptr current = myFunction::points_to_pcl<PointT>(points);

            //current = background_segmentation.compute(current);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_view = myFunction::XYZ_to_XYZRGB<PointT>(current);
            myFunction::updateCloud(viewer, current_view, "current");
        }
        */
    }
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

