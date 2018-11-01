#define HAVE_BOOST
#define HAVE_PCAP

#include <iostream>
#include "../3rdparty/args/args.hxx"
#include "../3rdparty/VelodyneCapture/VelodyneCapture.h"
#include "../include/veloFrame.h"

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

    VeloFrame::VeloFrames veloFrames;
    veloFrames.pcapFileName = args::get(inputPcap);
    veloFrames.loadFromPcap();

    std::cerr << veloFrames;
    std::cerr << std::numeric_limits<long long>::max() << std::endl;

    return 0;
}
