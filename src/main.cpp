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
bool show_background = false;
bool show_background_status = false;
bool save_pcd = false;
bool real_time = true;
double point_size = 1.0;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
std::string cwd = std::string(getcwd(NULL, 0)) + '/';

args::ArgumentParser parser("This is a test program.", "This goes after the options.");
args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});

args::Group requirementGroup(parser, "This group is all required:", args::Group::Validators::All);
    args::Group inputGroup(requirementGroup, "This group is xor:", args::Group::Validators::AtMostOne);//Xor);
        args::Group inputBagGroup(inputGroup, "This group is all or none:", args::Group::Validators::All);
            args::ValueFlag<std::string> darknetTxtPath(inputBagGroup, "darknetTxtPath", "darknet txt path", {"darknet"});
            args::ValueFlag<std::string> inputBag(inputBagGroup, "inputBag", "input bag path", {"bag"});
        args::Group inputDataGroup(inputGroup, "This group is at most one:", args::Group::Validators::Xor);
            args::ValueFlag<std::string> inputAll(inputDataGroup, "inputAll", "input all cloud",{"ia", "input_all"});
            args::ValueFlag<std::string> inputOnlyFullCloud(inputDataGroup, "inputOnlyFullCloud", "input only full cloud", {"ifc", "input_full_cloud"});
            args::ValueFlag<std::string> inputOnlyObjectCloud(inputDataGroup, "inputOnlyObjectCloud", "input only objects cloud", {"ioc", "input_object_cloud"});
args::Group postProcessingGroup(parser, "This group is dont care:", args::Group::Validators::DontCare);
    args::ValueFlag<double> objectSeg(postProcessingGroup, "objectSeg", "Enable object segmentation", {"os", "object_segmentation"});
    args::Flag noiseRemoval(postProcessingGroup, "noiseRemoval", "Enable noise removal", {"nr", "noise_removal"});
    args::Flag postProcessing(postProcessingGroup, "postProcessing", "Enable post processing", {"pp", "post_processing"});
    args::Group backgroundSegGroup(postProcessingGroup, "This group is all or none:", args::Group::Validators::AllOrNone);
        args::ValueFlag<double> backgroundSeg(backgroundSegGroup, "backgroundSeg", "Enable background segmentation", {"bs", "background_segmentation"});
        args::ValueFlag<std::string> inputBagBackground(backgroundSegGroup, "inputBagBackground", "input background cloud path", {"bg", "background"});
args::Group outputGroup(parser, "This group is at most one:", args::Group::Validators::AtMostOne);
    args::ValueFlag<std::string> outputAll(outputGroup, "outputAll", "output all cloud", {"oa", "output_all"});
    args::ValueFlag<std::string> outputOnlyObjectCloud(outputGroup, "outputOnlyObjectCloud", "output only object cloud", {"ooc", "output_object_cloud"});
    args::ValueFlag<std::string> outputOnlyFullCloud(outputGroup, "outputOnlyFullCloud", "output only full cloud", {"ofc", "output_full_cloud"});
args::Group optionGroup(parser, "This group is at most one:", args::Group::Validators::DontCare);
    args::Flag pauseAtStart(optionGroup, "pauseAtStart", "pause at start", {"pas", "pause_at_start"});
    args::ValueFlag<std::string> saveAsVideo(optionGroup, "saveAsVideo", "save as video", {"sav", "save_as_video"});
    args::ValueFlag<double> test(optionGroup, "test", "test", {"test"});

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);

void pcl_viewer()
{
    double firstFrameTimeStamp;

    //std::vector<pcl::visualization::Camera> cameras;
    
    myClass::MicroStopwatch tt("main");
    myClass::MicroStopwatch test_tt("test");
    myClass::objectSegmentation<PointT> object_segmentation;
    myClass::backgroundSegmentation<PointT> background_segmentation;
    std::vector<boost::shared_ptr<myFrame::CustomFrame<PointT>>> customFrames;
    pcl::PointCloud<PointT>::Ptr backgroundCloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr backgroundCloud_view;
    std::string input_path;
    std::string bag_path = args::get(inputBag);
    std::string darknet_txt_path = args::get(darknetTxtPath);

    std::string background_cloud_path = args::get(inputBagBackground);

    std::string output_path = "";
    std::string tmp_path = "tmp/";

    bool input_full_cloud = (inputOnlyFullCloud|inputAll)&(!inputOnlyObjectCloud);
    bool input_objects_cloud = (inputOnlyObjectCloud|inputAll)&(!inputOnlyFullCloud);
    bool output_full_cloud = (outputOnlyFullCloud|outputAll)&(!outputOnlyObjectCloud);
    bool output_objects_cloud = (outputOnlyObjectCloud|outputAll)&(!outputOnlyFullCloud);
    bool show_full_cloud = (inputOnlyFullCloud & (!objectSeg));

    viewer_pause = args::get(pauseAtStart);

    std::cerr << args::get(test) << std::endl;
    std::cerr << args::get(test) << std::endl;
    std::cerr << args::get(test) << std::endl;
    
    std::stringstream auto_name;

#pragma region check_logic ///////////////////////////////////////
    if(!(input_full_cloud||inputBag))
    {
        if(objectSeg)
        {
            std::cerr << "Error: No full cloud to object sehmentation" << std::endl;
            return;
        }
        if(output_full_cloud)
        {
            std::cerr << "Error: No full cloud to save" << std::endl;
            return;
        }
        if(!input_objects_cloud)
        {
            std::cerr << "Error: Nothing to do" << std::endl;
            return;
        }
    }
    if(input_objects_cloud)
    {
        if(objectSeg)
        {
            std::cerr << "Error: Objects is already input" << std::endl;
            return;
        }
    }
    ////////////////////////////////////////////////////////////////*/
#pragma endregion check_logic

#pragma region check_file_and_path ///////////////////////////////////////
    if(outputOnlyObjectCloud)
    {
        output_path = args::get(outputOnlyObjectCloud) + '/';
        tmp_path = output_path + "/tmp/";
    }
    else if(outputOnlyFullCloud)
    {
        output_path = args::get(outputOnlyFullCloud) + '/';
        tmp_path = output_path + "/tmp/";
    }
    else if(outputAll)
    {
        output_path = args::get(outputAll) + '/';
        tmp_path = output_path + "/tmp/";
    }
    else
    {
        std::cerr << "No data output" << std::endl;
    }

    if(!myFunction::fileExists(output_path))
    {
        mkdir(output_path.c_str(), 0777);      //make directory data_dir and set  permission to 777
    }
    if(!myFunction::fileExists(tmp_path))
    {
        mkdir(tmp_path.c_str(), 0777);       //make directory tmp_dir and set permission to 777
    }

    if(inputBag)
    {
        if(!myFunction::fileExists(bag_path))
        {
            std::cerr << "bag file " << bag_path << " not found" << std::endl;
            return;
        }
        if(!myFunction::fileExists(darknet_txt_path))
        {
            std::cerr << "darknet txt file " << darknet_txt_path << " not found" << std::endl;
            return;
        }
    }
    else
    {
        if(inputOnlyFullCloud)
        {
            input_path = args::get(inputOnlyFullCloud) + '/';
        }
        else if(inputOnlyObjectCloud)
        {
            input_path = args::get(inputOnlyObjectCloud) + '/';
        }
        else if(inputAll)
        {
            input_path = args::get(inputAll) + '/';
        }
        else
        {
            std::cout << "Input error" << std::endl;
            return;
        }
        if(!myFunction::fileExists(input_path))
        {
            std::cerr << "input path not found" << std::endl;
            return;
        }
        if(output_path == "")
        {
            output_path = input_path;
            tmp_path = output_path + "/tmp/";
        }
    }

    if(backgroundSeg)
    {
        if(!myFunction::fileExists(background_cloud_path))
        {
            std::cerr << "background cloud file not found" << std::endl;
            return;
        }
    }

    if((output_path == input_path)&&(outputOnlyFullCloud||outputOnlyObjectCloud||outputAll))
    {
        std::cerr << "output path = input path, continue?(y / n):";
        char choise;
        cin >> choise;
        if((choise == 'n')&&(choise == 'N')) 
        {
            return;
        }
    }
    ////////////////////////////////////////////////////////////////*/
#pragma endregion check_file_and_path

#pragma region load ///////////////////////////////////////
    std::cerr << "Loading...", tt.tic();
    if(inputBag)
    {  
        boost::filesystem::path path_to_remove(tmp_path);
        for (boost::filesystem::directory_iterator end_dir_it, it(path_to_remove); it!=end_dir_it; ++it)
        {
            boost::filesystem::remove_all(it->path());
        }
        myFrame::getCustomFrames(bag_path, customFrames, darknet_txt_path, tmp_path, postProcessing);
        auto_name << "[if=";
        auto_name << boost::filesystem::path{bag_path}.filename().stem().string();
        auto_name << "]";
        if(postProcessing) auto_name << "[pp]";
    }
    else 
    {
        myFrame::loadCustomFrames(input_path, customFrames, !input_full_cloud, !input_objects_cloud);
        std::string temp = input_path.substr(input_path.find('=') + 1, input_path.find(']') - input_path.find('=') - 1);
        std::string pp = input_path.substr(input_path.find('[', 1) + 1, input_path.find(']', 1) - input_path.find('[', 1) - 1);
        auto_name << "[if=";
        auto_name << temp;
        auto_name << "]";
        auto_name << pp;
    }
    if(inputBagBackground)
    {
        if(pcl::io::loadPCDFile(background_cloud_path, *backgroundCloud) == -1)
        {
            std::cerr << "Load background_cloud_path error" << std::endl;
        }
        backgroundCloud_view = myFunction::XYZ_to_XYZRGB<PointT>(backgroundCloud, true);
        auto_name << "[bg=";
        auto_name << boost::filesystem::path{background_cloud_path}.filename().stem().string();
        auto_name << "]";
    }
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    std::cerr << '\n';
    ////////////////////////////////////////////////////////////////*/
#pragma endregion load

#pragma region post_processing ///////////////////////////////////////
    if(objectSeg && (inputOnlyFullCloud||inputBag))
    {    
        std::cerr << "Custom frames object segmentation...", tt.tic();

        double w = 1280.0;
        double h = 720.0;
        object_segmentation.setCameraParameter("UL", w, h, 89.7974 * M_PI / 180.0, 69.4 * M_PI / 180.0, -0.03);
    
        double scale = args::get(objectSeg);
        myFrame::objectSegmentationCustomFrames(tmp_path, object_segmentation, customFrames, scale);

        std::cerr << " >> Done: " << tt.toc_string() << " us\n";
        std::cerr << "Total frame : " << customFrames.size();
        std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
        std::cerr << '\n';
        auto_name << "[os=";
        auto_name << scale;
        auto_name << "]";
    }
    if(backgroundSeg)
    {
        std::cerr << "Custom frames background segmentation...", tt.tic();

        double resolution = args::get(backgroundSeg);

        background_segmentation.setBackground(backgroundCloud, resolution);

        if(objectSeg || input_objects_cloud)
        {
            myFrame::backgroundSegmentationCustomFrameYoloObjects(background_segmentation, customFrames);
        }
        else
        {
            myFrame::backgroundSegmentationCustomFrames(background_segmentation, customFrames);
        }
        std::cerr << " >> Done: " << tt.toc_string() << " us\n";
        std::cerr << "Total frame : " << customFrames.size();
        std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
        std::cerr << '\n';
        auto_name << "[bs=";
        auto_name << resolution;
        auto_name << "]";
    }
    if(noiseRemoval)
    {
        double percentP;
        double StddevMulThresh;
        cout << "Please input noise removal percentP: "; 
        cin >> percentP;
        if(percentP > 1.0) percentP = 1.0;
        if(percentP < 0.0) percentP = 0.0;
        cout << "Please input noise removal StddevMulThresh: "; 
        cin >> StddevMulThresh;

        std::cerr << "Custom frames noise removal...", tt.tic();

        myFrame::noiseRemovalCustomFrameYoloObjects(customFrames, percentP, StddevMulThresh);

        std::cerr << " >> Done: " << tt.toc_string() << " us\n";
        std::cerr << "Total frame : " << customFrames.size();
        std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
        std::cerr << '\n';
        auto_name << "[nr=";
        auto_name << percentP;
        auto_name << "_";
        auto_name << StddevMulThresh;
        auto_name << "]";
    }
    ////////////////////////////////////////////////////////////////*/
#pragma endregion post_processing

#pragma region save ///////////////////////////////////////
    if(outputOnlyFullCloud|outputOnlyObjectCloud|outputAll)
    {
        std::cerr << "Saving...", tt.tic();
        myFrame::saveCustomFrames(output_path, customFrames, true, !output_full_cloud, !output_objects_cloud);
        std::cerr << " >> Done: " << tt.toc_string() << " us\n";
        std::cerr << "Total frame : " << customFrames.size();
        std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
        std::cerr << '\n';
    }
    std::cerr << "save data to : " << auto_name.str() << std::endl;
    ////////////////////////////////////////////////////////////////*/
#pragma endregion save

    std::sort(customFrames.begin(), customFrames.end(), [](const boost::shared_ptr<myFrame::CustomFrame<PointT>> &a, const boost::shared_ptr<myFrame::CustomFrame<PointT>> &b) { return a->file_name < b->file_name; });

    viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setSize(1280, 720);
    viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);
    //viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor (0.0, 0.0, 0.0);
    viewer->setCameraPosition( 0.0, 0.0, -0.0000001, 0.0, -1.0, 0.0, 0 );
    viewer->setCameraFieldOfView(60.0 * M_PI / 180.0);

    if(saveAsVideo)
    {
        std::string video_path = "video/";
        if(!myFunction::fileExists(video_path))
        {
            mkdir(video_path.c_str(), 0777);       //make directory tmp_dir and set permission to 777
        }
        for(int i = 0; i < customFrames.size(); i++)
        {
            if(show_full_cloud)
            {
                customFrames[i]->showFullCloud(viewer, point_size);
            }
            else
            {
                customFrames[i]->show(viewer, point_size);
            }
            std::stringstream temp;
            temp << video_path << "video" << i << ".png";
            viewer->saveScreenshot(temp.str());
            if(show_full_cloud)
            {
                customFrames[i]->removeFullCloud(viewer);
            }
            else
            {
                customFrames[i]->remove(viewer);
            }
        }
        std::stringstream command;
        command << "ffmpeg -r 1/" << 1.0/30.0 << " -i " << video_path << "video%d.png -c:v mpeg4 -q:v 4 " << args::get(saveAsVideo) << ".mp4";
        system(command.str().c_str());
    }
    int play_count = 0;
    std::chrono::milliseconds start_time_stamp = customFrames[0]->time_stamp;
    std::chrono::milliseconds next_time_stamp;
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    if(show_full_cloud)
    {
        customFrames[play_count]->showFullCloud(viewer, point_size);
    }
    else
    {
        customFrames[play_count]->show(viewer, point_size);
    }
    next_time_stamp = customFrames[(play_count + 1)% customFrames.size()]->time_stamp;
    
    viewer->spinOnce();
    
    while( !viewer->wasStopped())
    {
        viewer->spinOnce();
        if(!viewer_pause)
        {
            if(save_pcd)
            {
                std::string back_file = "background/" + customFrames[play_count]->file_name + ".pcd";
                if(!myFunction::fileExists("background/"))
                {
                    mkdir("background/", 0777);       //make directory tmp_dir and set permission to 777
                }
                pcl::io::savePCDFileBinaryCompressed(back_file, *(customFrames[play_count]->full_cloud));
                std::cerr << "background saved : " << back_file << std::endl;
                save_pcd = false;
            }
            if(show_background != show_background_status)
            {
                if(show_background)
                {
                    myFunction::showCloud(viewer, backgroundCloud_view, "backgroundCloud_view", point_size);
                }
                else
                {
                    myFunction::removeCloud(viewer, "backgroundCloud_view");
                }
                show_background_status = show_background;
            }
            bool stay = ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count() < (next_time_stamp-start_time_stamp).count()) && real_time);
            if(stay) continue;
            if(show_full_cloud)
            {
                customFrames[play_count]->removeFullCloud(viewer);
                play_count = (play_count + 1)% customFrames.size();
                customFrames[play_count]->showFullCloud(viewer, point_size);
            }
            else
            {
                customFrames[play_count]->remove(viewer);
                play_count = (play_count + 1)% customFrames.size();
                customFrames[play_count]->show(viewer, point_size);
            }
            next_time_stamp = customFrames[(play_count + 1)% customFrames.size()]->time_stamp;
            if(play_count == 0)
            {
                start_time_stamp = customFrames[0]->time_stamp;
                start_time = std::chrono::steady_clock::now();
            }
        }
        else
        {
            start_time = std::chrono::steady_clock::now();
            start_time_stamp = customFrames[play_count]->time_stamp;
        }
        //boost::this_thread::sleep(boost::posix_time::milliseconds(16));
    }

    /*///////// load from data directory //////////////////////////////////////////////////////
    std::cerr << "load custom frames...", tt.tic();

    myFrame::loadCustomFrames(data_dir, customFrames, false);

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    ////////////////////////////////////////////////////////////////*/

    /*///////// custom frames background segmentation //////////////////////////////////////////////////////
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
    
    /*///////// custom frames noise removal //////////////////////////////////////////////////////
    int meanK;
    double StddevMulThresh;
    cout << "Please input noise removal meanK: "; 
    cin >> meanK;
    cout << "Please input noise removal StddevMulThresh: "; 
    cin >> StddevMulThresh;
    std::cerr << "custom frames noise removal...", tt.tic();

    myFrame::noiseRemovalCustomFrames(customFrames, meanK, StddevMulThresh);

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    ////////////////////////////////////////////////////////////////*/

    /*///////// custom frames object segmentation //////////////////////////////////////////////////////
    std::cerr << "Custom frames object segmentation...", tt.tic();
    double w = 1280.0;
    double h = 720.0;
    object_segmentation.setCameraParameter("UL", w, h, 89.7974 * M_PI / 180.0, 69.4 * M_PI / 180.0, -0.03);
    
    myFrame::objectSegmentationCustomFrames(tmp_dir, object_segmentation, customFrames, 1.5);

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    std::cerr << '\n';
    ////////////////////////////////////////////////////////////////*/

    /*///////// custom frames background segmentation //////////////////////////////////////////////////////
    double resolution;
    cout << "Please input background segmentation resolution: "; 
    cin >> resolution;
    std::cerr << "Point cloud background segmentation...", tt.tic();
    background_segmentation.setBackground(backgroundCloud, resolution, true);

    myFrame::backgroundSegmentationCustomFrameYoloObjects(background_segmentation, customFrames);

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
    ////////////////////////////////////////////////////////////////*/
    
    /*///////// custom frames noise removal //////////////////////////////////////////////////////
    double percentP;
    double StddevMulThresh;
    cout << "Please input noise removal percentP: "; 
    cin >> percentP;
    if(percentP > 1.0) percentP = 1.0;
    if(percentP < 0.0) percentP = 0.0;
    cout << "Please input noise removal StddevMulThresh: "; 
    cin >> StddevMulThresh;
    std::cerr << "custom frames noise removal...", tt.tic();

    myFrame::noiseRemovalCustomFrameYoloObjects(customFrames, percentP, StddevMulThresh);

    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    std::cerr << "Total frame : " << customFrames.size();
    std::cerr << "\tprocess speed : " << customFrames.size() / (tt.toc_pre() / 1000000.0) << " FPS" << std::endl;
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
    if((event.getKeySym() == "b")&&(event.keyDown())&&(event.isCtrlPressed()))
    {
        if(inputBagBackground)
        {
            show_background = !show_background;
            std::cerr << "Background: " << ((show_background == true)? "ON" : "OFF") << std::endl;
        }
    }
    else if((event.getKeySym() == "p")&&(event.keyDown())&&(event.isCtrlPressed()))
    {
        save_pcd = true;
    }
    else if((event.getKeySym() == "space")&&(event.keyDown())&&(event.isCtrlPressed()))
    {
        real_time = !real_time;
        std::cerr << "real_time: " << ((real_time == true)? "ON" : "OFF") << std::endl;
    }
    else if((event.getKeySym() == "space")&&(event.keyDown()))
    {
        viewer_pause = !viewer_pause;
        std::cerr << "viewer_pause: " << ((viewer_pause == true)? "ON" : "OFF") << std::endl;
    }
    else if((event.getKeySym() == "KP_Add")&&(event.keyDown()))
    {
        if(point_size < 10.0) point_size += 0.5;
    }
    else if((event.getKeySym() == "KP_Subtract")&&(event.keyDown()))
    {
        if(point_size > 1.0) point_size -= 0.5;
    }
    else if(event.keyDown())
    {
        std::cerr << "Keyboard pressed: \"" << event.getKeySym() << "\"" << std::endl;
    }
}

