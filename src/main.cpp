#define HAVE_BOOST
#define HAVE_PCAP
#define HAVE_FAST_PCAP

#define TIMEZONE (+8)

#include <iostream>
#include "../3rdparty/args/args.hxx"
#include "../3rdparty/VelodyneCapture/VelodyneCapture.h"
#include "../include/veloFrame.h"
#include "../include/microStopwatch.h"

std::string filename;

args::ArgumentParser parser("This is a test program.", "This goes after the options.");
args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});

args::Group requirementGroup(parser, "This group is all required:", args::Group::Validators::All);
    args::ValueFlag<std::string> inputPcap(parser, "inputPcap", "input pcap", {"pcap"});

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

    veloFrames.setPcapFile(pcapPath.string());

    std::cerr << "Loading...", tt.tic();
    veloFrames.load(pcapPath.parent_path().string());
    std::cerr << " >> Done: " << tt.toc_string() << " us\n";
    veloFrames.save(pcapPath.parent_path().string());
    
    veloFrames.print();

    return 0;
}
