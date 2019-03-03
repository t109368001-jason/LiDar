#include <mutex>
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
#include "../3rdparty/args/args.hxx"
#include "../include/microStopwatch.h"
#include "../include/velodyne/velodyne.h"
#include "../include/function.h"
#include "../include/lasers.h"

std::mutex cloudMutex;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> inputCloud_1;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> inputCloud_2;

pcl::visualization::PCLVisualizer::Ptr viewer;
uint64_t startTime;
uint64_t startTime2;

bool viewerPause = false;
double totalTheta = 0.0;
pcl::PointXYZ pointOrigin;
pcl::PointXYZ pointMoved;

args::ArgumentParser parser("This is a test program.", "This goes after the options.");
args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});

args::Group requirementGroup(parser, "This group is all required:", args::Group::Validators::All);
    args::ValueFlag<std::string> inputPcapArg_1(requirementGroup, "inputPcap1", "input pcap1", {"p1"});
    args::ValueFlag<std::string> inputPcapArg_2(requirementGroup, "inputPcap2", "input pcap2", {"p2"});
    
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
    double deltaX = 0.0;
    double deltaY = 0.0;
    double deltaZ = 0.0;
    double deltaTheta = 0.0;

    if((event.getKeySym() == "Up")&&(event.keyDown()))
    {
        deltaY = 1.0;
        
    }
    else if((event.getKeySym() == "Down")&&(event.keyDown()))
    {
        deltaY = -1.0;
    }
    else if((event.getKeySym() == "Left")&&(event.keyDown()))
    {
        deltaX = -1.0;
    }
    else if((event.getKeySym() == "Right")&&(event.keyDown()))
    {
        deltaX = 1.0;
    } 
    else if((event.getKeySym() == "KP_8")&&(event.keyDown()))
    {
        deltaZ = 1.0;
    } 
    else if((event.getKeySym() == "KP_2")&&(event.keyDown()))
    {
        deltaZ = -1.0;
    } 
    else if((event.getKeySym() == "bracketleft")&&(event.keyDown()))
    {
        deltaTheta = -1.0 * M_PI / 180.0;
    } 
    else if((event.getKeySym() == "bracketright")&&(event.keyDown()))
    {
        deltaTheta = 1.0 * M_PI / 180.0;
    } 
    else if((event.getKeySym() == "space")&&(event.keyDown()))
    {
        viewerPause = !viewerPause;
        std::cout << "pause: " << ((viewerPause == true)? "ON" : "OFF") << std::endl;
    }
    else 
    {
        std::cout << "Keyboard pressed: " << ((event.isAltPressed())? "Alt + " : "") << ((event.isCtrlPressed())? "Ctrl + " : "") << ((event.isShiftPressed())? "Shift + " : "") << event.getKeySym() << std::endl;
    }

    if(event.isCtrlPressed()) {
        deltaX *= 10.0;
        deltaY *= 10.0;
        deltaZ *= 10.0;
        deltaTheta *= 5;
    }

    if(event.isShiftPressed()) {
        deltaX *= 100.0;
        deltaY *= 100.0;
        deltaZ *= 100.0;
        deltaTheta *= 10;
    }

    if(std::fabs(deltaX+deltaY+deltaZ+deltaTheta) > 0.0) 
    {
        if( cloudMutex.try_lock() ){
            for(pcl::PointXYZ &point : inputCloud_2->points) {

                double x_ = point.x + deltaX;
                double y_ = point.y + deltaY;
                double z_ = point.z + deltaZ;

                point.z = z_;
                point.y = x_*(-std::sin(deltaTheta)) + y_*std::cos(deltaTheta);
                point.x = x_*std::cos(deltaTheta) + y_*std::sin(deltaTheta);
            }
            totalTheta += deltaTheta;
            pointMoved = inputCloud_2->points[0];
        }
        cloudMutex.unlock();
        std::cout << "point 0 from (" << pointOrigin.x << ", " << pointOrigin.y << ", " << pointOrigin.z << ")";
        std::cout << "to (" << pointMoved.x << ", " << pointMoved.y << ", " << pointMoved.z << ")";
        std::cout << ", theta: " << totalTheta << std::endl;
    }
}

int main(int argc, char * argv[])
{
    try
    {
        parser.ParseCLI(argc, argv);
        
        myClass::MicroStopwatch tt;
        
        boost::filesystem::path pcapPath_1{args::get(inputPcapArg_1)};
        boost::filesystem::path pcapPath_2{args::get(inputPcapArg_2)};

        velodyne::VLP16 vlp16_1;
        velodyne::VLP16 vlp16_2;

        std::cout << pcapPath_1.stem().string() << std::endl;
        std::cout << pcapPath_2.string() << std::endl;

        if(!vlp16_1.open(pcapPath_1.string()))
        {
            std::cout << std::endl << "Error : load " << pcapPath_1.string() << " failed" << std::endl;
            return false;
        }

        if(!vlp16_2.open(pcapPath_2.string()))
        {
            std::cout << std::endl << "Error : load " << pcapPath_2.string() << " failed" << std::endl;
            return false;
        }

        while(vlp16_1.isRun())
        {
            vlp16_1.moveToNext();
            vlp16_1.moveToNext();
            vlp16_1.moveToNext();
            vlp16_1 >> inputCloud_1;
            break;
        }
        
        while(vlp16_2.isRun())
        {
            vlp16_2.moveToNext();
            vlp16_2.moveToNext();
            vlp16_2.moveToNext();
            vlp16_2.moveToNext();
            vlp16_2.moveToNext();
            std::lock_guard<std::mutex> lock( cloudMutex );
            vlp16_2 >> inputCloud_2;
            cloudMutex.unlock();
            break;
        }

        viewer.reset(new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ));
        viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);
        viewer->addCoordinateSystem( 3.0, "coordinate" );
        viewer->setCameraPosition( 0.0, 0.0, 10000.0, 0.0, 1.0, 0.0, 0 );

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_1(myFunction::XYZ_to_XYZRGB<pcl::PointXYZ>(inputCloud_1, false));
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_2(myFunction::XYZ_to_XYZRGB<pcl::PointXYZ>(inputCloud_2, false));

        myFunction::showCloud(viewer, cloud_1, "cloud_1");
        myFunction::showCloud(viewer, cloud_2, "cloud_2");

        pointOrigin = inputCloud_2->points[0];

        while( !viewer->wasStopped() ){
            viewer->spinOnce();

            if(viewerPause) continue;
            
            if(vlp16_1.isRun())
            {
                vlp16_1.moveToNext();
                vlp16_1 >> inputCloud_1;
            } else break;
            
            if(vlp16_2.isRun())
            {
                vlp16_2.moveToNext();
                std::lock_guard<std::mutex> lock( cloudMutex );
                vlp16_2 >> inputCloud_2;

                double theta = -0.15708;
                for(pcl::PointXYZ &point : inputCloud_2->points) 
                {
                    double x_ = point.x*std::cos(theta) + point.y*std::sin(theta);
                    double y_ = point.x*(-std::sin(theta)) + point.y*std::cos(theta);
                    double z_ = point.z;

                    point.x = x_ + -1042.3277129076;
                    point.y = y_ + -94.3663384411;
                    point.z = z_ + 37;
                }

                cloudMutex.unlock();
            } else break;


            cloud_1 = myFunction::XYZ_to_XYZRGB<pcl::PointXYZ>(inputCloud_1, false);
            cloud_2 = myFunction::XYZ_to_XYZRGB<pcl::PointXYZ>(inputCloud_2, false);
            myFunction::updateCloud(viewer, cloud_1, "cloud_1");
            myFunction::updateCloud(viewer, cloud_2, "cloud_2");
        }
        while(1);
    }
    catch (args::Help)
    {
        std::cout << parser;
        return 0;
    }
    catch (args::ParseError e)
    {
        std::cout << e.what() << std::endl;
        parser.Help(std::cout);
        return 1;
    }
    catch (args::ValidationError e)
    {
        std::cout << e.what() << std::endl;
        parser.Help(std::cout);
        return 1;
    }


    return 0;
}
