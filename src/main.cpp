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
#include <sys/stat.h>

struct Density
{
    int number;
    int times;
};

typedef pcl::PointXYZRGB PointT;

int fps = 30;
bool viewer_pause = false;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
std::string cwd = std::string(getcwd(NULL, 0)) + '/';

args::ArgumentParser parser("This is a test program.", "This goes after the options.");
args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
args::Group group(parser, "This group is all required:", args::Group::Validators::DontCare);
args::ValueFlag<std::string> inputBag(group, "CLOUD_IN", "input bag path", {'i', "if"});
args::ValueFlag<std::string> inputBagBackground(group, "CLOUD_IN", "input background cloud path", {'b', "bf"});
args::ValueFlag<std::string> yolo(group, "CLOUD_IN", "yolo path", {'y', "yp"});

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);

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
for(int t = 2; t < 21 ;t = t + 2){
    double len = 0.005;  //squre_size = len * shift_len;
    int shift_len = t;   //shift_len = len;
    std::string file_name = std::string("density_file_size") + std::to_string(int(len * 100 * shift_len)) + "_shift_len0" + std::to_string(int(len * 1000)) + ".csv";
    double x_max = -10, x_min = 10, y_max = -10, y_min = 10, z_max = -10, z_min = 10;
    myClass::MicroStopwatch tt("main");
    std::vector<std::vector<std::vector<std::vector<pcl::PointXYZRGB>>>> cube;
    std::vector<Density> density;
    std::string backgroundCloudFile = args::get(inputBagBackground);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if(pcl::io::loadPCDFile(backgroundCloudFile, *cloud) == -1)return 0;



    ////////// custom frames object segmentation //////////////////////////////////////////////////////
    std::cerr << "Check x,y,z max&min size...", tt.tic();
    
    for(int i = 0; i < cloud->points.size(); i++)
    {
        if(std::sqrt( cloud->points[i].x*cloud->points[i].x + cloud->points[i].y*cloud->points[i].y + cloud->points[i].z*cloud->points[i].z ) == 0)
        {
            continue;
        }
        
        x_max = (x_max < cloud->points[i].x)? cloud->points[i].x : x_max;
        y_max = (y_max < cloud->points[i].y)? cloud->points[i].y : y_max;
        z_max = (z_max < cloud->points[i].z)? cloud->points[i].z : z_max;
        x_min = (x_min > cloud->points[i].x)? cloud->points[i].x : x_min;
        y_min = (y_min > cloud->points[i].y)? cloud->points[i].y : y_min;
        z_min = (z_min > cloud->points[i].z)? cloud->points[i].z : z_min;
        
    }

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "x_max: " << x_max << "\n";
    std::cerr << "y_max: " << y_max << "\n";
    std::cerr << "z_max: " << z_max << "\n";
    std::cerr << "x_min: " << x_min << "\n";
    std::cerr << "y_min: " << y_min << "\n";
    std::cerr << "z_min: " << z_min << "\n";
    ////////////////////////////////////////////////////////////////*/


    ////////// custom frames object segmentation //////////////////////////////////////////////////////
    std::cerr << "1...", tt.tic();
    
    for(int i = 0; i < (std::ceil(z_max-z_min))/len; i++ )
    {
        std::vector<std::vector<std::vector<pcl::PointXYZRGB>>> y;
        for(int j = 0; j < (std::ceil(y_max-y_min))/len; j++)
        {
            std::vector<std::vector<pcl::PointXYZRGB>> x((std::ceil(x_max-x_min))/len);
            y.push_back(x); //line
        }
        cube.push_back(y);  //plane
    }
    
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    ////////////////////////////////////////////////////////////////*/

    ////////// custom frames object segmentation //////////////////////////////////////////////////////
    std::cerr << "2...", tt.tic();
    
    for(int i = 0; i < cloud->points.size(); i++)
    {
        if(std::sqrt( cloud->points[i].x*cloud->points[i].x + cloud->points[i].y*cloud->points[i].y + cloud->points[i].z*cloud->points[i].z ) == 0)
        {
            continue;
        }
        int z = std::floor((cloud->points[i].z-z_min)/len);
        int y = std::floor((cloud->points[i].y-y_min)/len);
        int x = std::floor((cloud->points[i].x-x_min)/len);
        cube[z][y][x].push_back(cloud->points[i]);
    }

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    ////////////////////////////////////////////////////////////////*/

    ////////// custom frames object segmentation //////////////////////////////////////////////////////
    std::cerr << "3...", tt.tic();

    int x_shift = 0, y_shift = 0, z_shift = 0;
    Density temp;
    bool add_density;
    for(z_shift = 0; z_shift < cube.size() - shift_len; z_shift++)
    {
        for(y_shift = 0; y_shift < cube[0].size() - shift_len; y_shift++)
        {
            for(x_shift = 0; x_shift < cube[0][0].size() - shift_len; x_shift++)
            {
                add_density = true;
                for(int i = 0; i < density.size(); i++)
                {
                    if(density[i].number == (cube[z_shift + shift_len][y_shift][x_shift].size() + cube[z_shift + shift_len][y_shift + shift_len][x_shift].size() + cube[z_shift + shift_len][y_shift][x_shift + shift_len].size() + cube[z_shift + shift_len][y_shift + shift_len][x_shift + shift_len].size()))
                    {
                        density[i].times++;
                        add_density = false;
                        break;
                    }
                }

                if(add_density == true)
                {
                    temp.number = (cube[z_shift + shift_len][y_shift][x_shift].size() + cube[z_shift + shift_len][y_shift + shift_len][x_shift].size() + cube[z_shift + shift_len][y_shift][x_shift + shift_len].size() + cube[z_shift + shift_len][y_shift + shift_len][x_shift + shift_len].size());
                    temp.times = 1;
                    density.push_back(temp);
                }
            }
        }
    }


    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << density.size() << "\n";
    ////////////////////////////////////////////////////////////////*/
    ////////// custom frames object segmentation //////////////////////////////////////////////////////
    std::cerr << "4...", tt.tic();
    int number = 0;
    for(int i = 0; i < density.size(); i++)
    {
        number = number + (density[i].number * density[i].times);
    }
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << number << '\n';
    ////////////////////////////////////////////////////////////////*/

    ////////// custom frames object segmentation //////////////////////////////////////////////////////
    std::cerr << "save file...", tt.tic();

    struct stat buffer;
    bool file_exists = (stat (file_name.c_str(), &buffer) == 0)? true : false;

    if (file_exists == false)
    {
        ofstream density_file (file_name);

        if (density_file.is_open())
        {
            density_file << "number,times\n";
            for(int i = 0; i < density.size(); i++)
            {
                density_file << density[i].number << ',' <<density[i].times << "\n";
            }
            density_file.close();
        }
        else std::cerr << "Unable to open file.\n";
    }
    else std::cerr << "\nthis file is exist!\n";

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << " >> Done: " << file_name << '\n';
    ////////////////////////////////////////////////////////////////*/

    /*///////// custom frames object segmentation //////////////////////////////////////////////////////
    std::cerr << "test file exists...", tt.tic();

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    ////////////////////////////////////////////////////////////////*/
}
return 0;
}


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
    std::string tmp_dir = "tmp/";
    std::string data_dir = "data/";

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

    /*///////// custom frames object segmentation //////////////////////////////////////////////////////
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
    
    ////////// load from data directory //////////////////////////////////////////////////////
    std::cerr << "load custom frames...", tt.tic();
    myFrame::loadCustomFrames(data_dir, customFrames, true);
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    ////////////////////////////////////////////////////////////////*/

    ////////// custom frames background segmentation //////////////////////////////////////////////////////
    double resolution;
    cout << "Please input background segmentation resolution: "; 
    cin >> resolution;
    std::cerr << "Point cloud background segmentation...", tt.tic();
    background_segmentation.setBackground(backgroundCloud, resolution);

    myFrame::backgroundSegmentationCustomFrames(background_segmentation, customFrames);

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

    ////////// sort from data directory //////////////////////////////////////////////////////
    std::cerr << "sort custom frames...", tt.tic();
    std::sort(customFrames.begin(), customFrames.end(), [](const boost::shared_ptr<myFrame::CustomFrame<PointT>> &a, const boost::shared_ptr<myFrame::CustomFrame<PointT>> &b) { return a->file_name < b->file_name; });
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    ////////////////////////////////////////////////////////////////*/

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
