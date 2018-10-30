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

class DimZ
{
public:
    int dim;
    int num;
    DimZ (int z)
    {
        this->dim = z;
        this->num = 1;
    }   

    bool add (int z)
    {
        if(z == this->dim)
        {
            this->num++;
            return true;
        }

        return false;
    }

    int get (int z, int cube_len) const
    {
        if(this->dim >= z && this->dim < (z + cube_len)) 
        {
            //std::cerr << "Zif~~~" << z << '\t' << this->dim << '\t' << this->num << '\n';
            return this->num;
        }
        //else std::cerr << "Zelse~~~" << z << '\t' << this->dim << '\t' << this->num << '\n';
        return 0;
    }
};

class DimY
{

public:

    int dim;
    std::vector<DimZ> subD;
    DimY(int y, int z)
    {
        this->dim = y;
        this->subD.push_back(DimZ(z));
    }

    bool add (int y,int z)
    {
        if(y == this->dim)
        {
            for(int i = 0; i < this->subD.size(); i++)
            {
                if (this->subD[i].add(z))
                {
                    return true;
                }
            }

            this->subD.push_back(DimZ(z));
            return true;
        }
        return false;
    }

    int get (int y, int z, int cube_len) const
    {
        int temp = 0;
        if(this->dim >= y && this->dim < (y + cube_len)) 
        {
            for(int i = 0; i < this->subD.size(); i++)
            {
                temp += this->subD[i].get(z, cube_len);
            }

            //std::cerr << "Yif~~~" << y << '\t' << this->dim << '\n';
        }
       // else std::cerr << "Yelse~~~" << y << '\t' << this->dim << '\n';
        return temp;
    }
};

class DimX
{

public:

    int dim;
    std::vector<DimY> subD;
    DimX(int x,int y, int z)
    {
        this->dim = x;
        this->subD.push_back(DimY(y, z));
    }

    bool add (int x, int y, int z)
    {
        if(x == this->dim)
        {
            for(int i = 0; i < this->subD.size(); i++)
            {
                if (this->subD[i].add(y, z))
                {
                    return true;
                }
            }
            
            this->subD.push_back(DimY(y, z));
            return true;
        }
        return false;
    }

    int get (int x, int y, int z, int cube_len) const
    {
        int temp = 0;
        if((this->dim >= x) && (this->dim < (x + cube_len))) 
        {
            for(int i = 0; i < this->subD.size(); i++)
            {
                temp += this->subD[i].get(y, z, cube_len);
            }
            //std::cerr << "Xif~~~" << x << '\t' << this->dim << '\n';
        }
        //else std::cerr << "Xelse~~~" << x << '\t' << this->dim <<  '\n';
        return temp;
    }

};

class Cubes
{
public:
    vector<DimX> subD;

    void add (int x, int y, int z)
    {
        for(int i = 0; i < this->subD.size(); i++)
        {
            if (this->subD[i].add(x, y, z))
            {
                return ;
            }
        }
            
        this->subD.push_back(DimX(x, y, z));
    }

    int get (int x, int y, int z, int cube_len) const
    {
        int temp = 0;
        for(int i = 0; i < this->subD.size(); i++)
        {
            temp += this->subD[i].get(x, y, z, cube_len);
        }
        
        return temp;
    }
    
    void dis ()
    {
        for (int i = 0; i < subD.size(); i++)
        {
            for(int j = 0; j < subD[i].subD.size(); j++)
            {
                for(int k = 0; k < subD[i].subD[j].subD.size(); k++)
                {
                    std::cerr << subD[i].dim << '\t' << subD[i].subD[j].dim << '\t' << subD[i].subD[j].subD[k].dim << '\t' << subD[i].subD[j].subD[k].num << '\n';
                }
            }
        }
    }
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

bool density_cal_part (const double &shift_len, const double &x_ran, const double &y_ran, const double &z_ran, const Cubes &cube, const int &cube_len, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    //squre_size = shift_len * cube_len;
    //cube_len = shift_len;
    std::string file_name = std::string("density_file_size") + std::to_string(int(shift_len * 100 * cube_len)) + "_cube_len0" + std::to_string(int(shift_len * 1000)) + ".csv";
    std::vector<Density> density;

    ////////// custom frames object segmentation //////////////////////////////////////////////////////
    int cube_size = 0;
    int x_shift = 0, y_shift = 0, z_shift = 0;
    Density temp;
    bool add_density;

    int x_rans = x_ran - cube_len;
    int y_rans = y_ran - cube_len;
    int z_rans = z_ran - cube_len;
    
    for(z_shift = 0; z_shift < z_rans; z_shift++)
    {
        for(y_shift = 0; y_shift < y_rans; y_shift++)
        {
            for(x_shift = 0; x_shift < x_rans; x_shift++)
            {
                add_density = true;
                cube_size = cube.get(x_shift, y_shift, z_shift, cube_len);
                //if(cube_size != 0) std::cerr << "cube_size = " << cube_size << '\n';
                //std::cerr << z_shift << '\t' << y_shift << '\t' << x_shift << '\t' << z_rans <<'\t' << y_rans << '\t' << x_rans << '\n';


                for(int i = 0; i < density.size(); i++)
                {
                    if(density[i].number == cube_size)
                    {
                        density[i].times++;
                        add_density = false;
                        break;
                    }
                }

                if(add_density == true)
                {
                    temp.number = cube_size;
                    temp.times = 1;
                    density.push_back(temp);
                }
            }
        }
    }

    ////////////////////////////////////////////////////////////////*/

    /*///////// custom frames object segmentation //////////////////////////////////////////////////////
    int number = 0;
    for(int i = 0; i < density.size(); i++)
    {
        number = number + (density[i].number * density[i].times);
    }
    ////////////////////////////////////////////////////////////////*/

    ////////// custom frames object segmentation //////////////////////////////////////////////////////
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
        else return false;
    }
    else return false;

    ////////////////////////////////////////////////////////////////*/
}

bool density_cal (const double &shift_len, const double &x_ran, const double &y_ran, const double &z_ran, const Cubes &cube, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const int &beg, const int &end)
{
    bool temp;
    for(int i = beg; i < end; i++)
    {
        temp = density_cal_part(shift_len, x_ran, y_ran, z_ran, cube, i * (0.01 / shift_len), cloud);
    }
    return temp;
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

    myClass::MicroStopwatch tt("main");
    std::string backgroundCloudFile = args::get(inputBagBackground);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    double shift_len = 0.005;
    double x_max = -10, x_min = 10, y_max = -10, y_min = 10, z_max = -10, z_min = 10;
    double x_ran, y_ran, z_ran;
    Cubes cube;

    if(pcl::io::loadPCDFile(backgroundCloudFile, *cloud) == -1)return 0;

    ////////// custom frames object segmentation //////////////////////////////////////////////////////
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

    x_ran = std::ceil((x_max-x_min) / shift_len);
    y_ran = std::ceil((y_max-y_min) / shift_len);
    z_ran = std::ceil((z_max-z_min) / shift_len);
    
    ////////////////////////////////////////////////////////////////*/

    ////////// custom frames object segmentation //////////////////////////////////////////////////////
    std::cerr << "1...", tt.tic();

    for(int i = 0; i < cloud->points.size(); i++)
    {
        if(std::sqrt( cloud->points[i].x*cloud->points[i].x + cloud->points[i].y*cloud->points[i].y + cloud->points[i].z*cloud->points[i].z ) == 0)
        {
            continue;
        }
        int z = std::floor((cloud->points[i].z-z_min)/shift_len);
        int y = std::floor((cloud->points[i].y-y_min)/shift_len);
        int x = std::floor((cloud->points[i].x-x_min)/shift_len);
        
        cube.add(x, y, z);
    }
    
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    //cube.dis();
    /*///////////////////////////////////////////////////////////////
    std::cerr << "test...", tt.tic();
	auto handle0 = std::async(std::launch::async, density_cal, shift_len, x_ran, y_ran, z_ran, cube, cloud, 1, 2);
    auto out0 = handle0.get();

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    return 0;
    ////////////////////////////////////////////////////////////////*/
    ////////// custom frames object segmentation //////////////////////////////////////////////////////
    std::cerr << "2...", tt.tic();
    
	auto handle0 = std::async(std::launch::async, density_cal, shift_len, x_ran, y_ran, z_ran, cube, cloud, 6, 7);
    auto handle1 = std::async(std::launch::async, density_cal, shift_len, x_ran, y_ran, z_ran, cube, cloud, 7, 8);
	auto handle2 = std::async(std::launch::async, density_cal, shift_len, x_ran, y_ran, z_ran, cube, cloud, 8, 9);
	auto handle3 = std::async(std::launch::async, density_cal, shift_len, x_ran, y_ran, z_ran, cube, cloud, 9, 10);
	auto handle4 = std::async(std::launch::async, density_cal, shift_len, x_ran, y_ran, z_ran, cube, cloud, 10, 11);
    auto out0 = handle0.get();
    auto out1 = handle1.get();
    auto out2 = handle2.get();
    auto out3 = handle3.get();
    auto out4 = handle4.get();
    

    if(out0 && out1  && out2 && out3 && out4 )std::cerr << "Error\n";

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    
    /*///////// custom frames object segmentation //////////////////////////////////////////////////////
    std::cerr << "test file exists...", tt.tic();

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    ////////////////////////////////////////////////////////////////*/
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

bool density_cal_part2 (const double &shift_len, const int &cube_len, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    //squre_size = shift_len * cube_len;
    //cube_len = shift_len;
    std::string file_name = std::string("density_file_size") + std::to_string(int(shift_len * 100 * cube_len)) + "_cube_len0" + std::to_string(int(shift_len * 1000)) + ".csv";
    double x_max = -10, x_min = 10, y_max = -10, y_min = 10, z_max = -10, z_min = 10;
    std::vector<std::vector<std::vector<std::vector<pcl::PointXYZRGB>>>> cube;
    std::vector<Density> density;

    ////////// custom frames object segmentation //////////////////////////////////////////////////////
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

    ////////////////////////////////////////////////////////////////*/


    ////////// custom frames object segmentation //////////////////////////////////////////////////////
    for(int i = 0; i < (std::ceil(z_max-z_min))/shift_len; i++ )
    {
        std::vector<std::vector<std::vector<pcl::PointXYZRGB>>> y;
        for(int j = 0; j < (std::ceil(y_max-y_min))/shift_len; j++)
        {
            std::vector<std::vector<pcl::PointXYZRGB>> x((std::ceil(x_max-x_min))/shift_len);
            y.push_back(x); //line
        }
        cube.push_back(y);  //plane
    }
    
    ////////////////////////////////////////////////////////////////*/

    ////////// custom frames object segmentation //////////////////////////////////////////////////////
    for(int i = 0; i < cloud->points.size(); i++)
    {
        if(std::sqrt( cloud->points[i].x*cloud->points[i].x + cloud->points[i].y*cloud->points[i].y + cloud->points[i].z*cloud->points[i].z ) == 0)
        {
            continue;
        }
        int z = std::floor((cloud->points[i].z-z_min)/shift_len);
        int y = std::floor((cloud->points[i].y-y_min)/shift_len);
        int x = std::floor((cloud->points[i].x-x_min)/shift_len);
        
        cube[z][y][x].push_back(cloud->points[i]);
    }

    ////////////////////////////////////////////////////////////////*/

    ////////// custom frames object segmentation //////////////////////////////////////////////////////
    int cube_size = 0;
    int x_shift = 0, y_shift = 0, z_shift = 0;
    Density temp;
    bool add_density;
    for(z_shift = 0; z_shift < cube.size() - cube_len; z_shift++)
    {
        for(y_shift = 0; y_shift < cube[0].size() - cube_len; y_shift++)
        {
            for(x_shift = 0; x_shift < cube[0][0].size() - cube_len; x_shift++)
            {
                add_density = true;

                for(int z = 0; z < cube_len; z++)
                    {
                        for(int y = 0; y < cube_len; y++)
                        {
                            for(int x = 0; x < cube_len; x++)
                            {
                                cube_size = cube_size + cube[z_shift + z][y_shift + y][x_shift + x].size();
                            }
                        }
                    }

                for(int i = 0; i < density.size(); i++)
                {
                    if(density[i].number == cube_size)
                    {
                        density[i].times++;
                        add_density = false;
                        break;
                    }
                }

                if(add_density == true)
                {
                    temp.number = cube_size;
                    temp.times = 1;
                    density.push_back(temp);
                }
            }
        }
    }

    ////////////////////////////////////////////////////////////////*/

    /*///////// custom frames object segmentation //////////////////////////////////////////////////////
    int number = 0;
    for(int i = 0; i < density.size(); i++)
    {
        number = number + (density[i].number * density[i].times);
    }
    ////////////////////////////////////////////////////////////////*/

    ////////// custom frames object segmentation //////////////////////////////////////////////////////


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
        else return false;
    }
    else return false;

    ////////////////////////////////////////////////////////////////*/
}
