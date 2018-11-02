#define HAVE_BOOST
#define HAVE_PCAP
#define HAVE_FAST_PCAP
#define VELOFRAME_USE_MULTITHREAD

#define TIMEZONE (+8)

#include <iostream>
#include "../3rdparty/args/args.hxx"
#include "../3rdparty/VelodyneCapture/VelodyneCapture.h"
#include "../include/veloFrame.h"
#include "../include/microStopwatch.h"

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

std::string filename;

args::ArgumentParser parser("This is a test program.", "This goes after the options.");
args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});

args::Group requirementGroup(parser, "This group is all required:", args::Group::Validators::All);
    args::ValueFlag<std::string> inputPcap(requirementGroup, "inputPcap", "input pcap", {"pcap"});
args::Group dontCareGroup(parser, "This group is dont care:", args::Group::Validators::DontCare);
    args::ValueFlag<std::string> inputBackground(dontCareGroup, "inputBackground", "input background", {"back"});

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
    VeloFrame::VeloFrames veloFrames;
    boost::filesystem::path pcapPath{args::get(inputPcap)};

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::io::loadPCDFile(args::get(inputBackground), *cloud);
    try
    {
        veloFrames.setPcapFile(pcapPath.string());
        veloFrames.setBackgroundSegmentationResolution(0.1);
        veloFrames.setBackgroundCloud(cloud);
        veloFrames.setNoiseRemovalPercentP(0.01);
        veloFrames.setNoiseRemovalStddevMulThresh(0.01);

        std::cerr << "Loading...", tt.tic();
        //veloFrames.load();
        veloFrames.load(pcapPath.parent_path().string());
        std::cerr << " >> Done: " << tt.toc_string() << " us\n";

        std::cerr << "Saving...", tt.tic();
        veloFrames.save(pcapPath.parent_path().string());
        std::cerr << " >> Done: " << tt.toc_string() << " us\n";
        
        std::cerr << "Background segmentation...", tt.tic();
        veloFrames.backgroundSegmentation();
        std::cerr << " >> Done: " << tt.toc_string() << " us\n";
        
        std::cerr << "Saving...", tt.tic();
        veloFrames.save(pcapPath.parent_path().string());
        std::cerr << " >> Done: " << tt.toc_string() << " us\n";
        
        std::cerr << "Noise removal...", tt.tic();
        veloFrames.noiseRemoval();
        std::cerr << " >> Done: " << tt.toc_string() << " us\n";
        
        std::cerr << "Saving...", tt.tic();
        veloFrames.save(pcapPath.parent_path().string());
        std::cerr << " >> Done: " << tt.toc_string() << " us\n";
        
        veloFrames.print();
    }
    catch(std::runtime_error &e)
    {
        std::cerr << red << "Error : " << reset << e.what() << std::endl;
    }

    return 0;
}
