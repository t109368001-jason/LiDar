#define HAVE_BOOST
#define HAVE_PCAP
#define HAVE_FAST_PCAP
#define VELOFRAME_USE_MULTITHREAD
#define __CL_ENABLE_EXCEPTIONS

#define TIMEZONE (+8)

#define TEST_COUNT 1

#include <iostream>
#include <CL/cl.hpp>

#include "../3rdparty/args/args.hxx"
#include "../3rdparty/VelodyneCapture/VelodyneCapture.h"
#include "../include/veloFrame.h"
#include "../include/microStopwatch.h"

typedef double Type;

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
    args::ValueFlag<std::string> inputBackground(requirementGroup, "inputBackground", "input background", {"back"});
args::Group dontCareGroup(parser, "This group is dont care:", args::Group::Validators::DontCare);
    args::ValueFlag<double> backgroundSeg(dontCareGroup, "backgroundSeg", "Enable background segmentation", {"bs", "background_segmentation"});
    args::ValueFlagList<double> noiseRemoval(dontCareGroup, "noiseRemoval", "Enable noise removal", {"nr", "noise_removal"});
    args::Flag output(dontCareGroup, "output", "output", {'o', "output"});
    args::Flag outputAll(dontCareGroup, "outputAll", "output all", {"oa", "outputAll"});

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
    if(event.keyDown())
    {
        std::cout << "Keyboard pressed: " << ((event.isAltPressed())? "Alt + " : "") << ((event.isCtrlPressed())? "Ctrl + " : "") << ((event.isShiftPressed())? "Shift + " : "") << event.getKeySym() << std::endl;
    }
}

const char * helloStr  = "__kernel void "
		"hello(void) "
		"{ "
		"  "
		"} ";

int main(int argc, char * argv[])
{
    myClass::MicroStopwatch tt("main");
    VeloFrame::VeloFrames veloFrames;
    boost::shared_ptr<VeloFrame::VeloFrameViewer> veloFrameViewer;

	std::vector<cl::Platform> all_platforms;
    cl::Platform::get(&all_platforms);
    if(all_platforms.size()==0){
        std::cout<<" No platforms found. Check OpenCL installation!\n";
        exit(1);
    }

    // kernel calculates for each element C=A+B
    std::string kernel_code=
            "   void kernel simple_add(global const double* A, global const double* B, global double* C)"
            "   {"
            "       for(int i = 0; i < 10000; i++)"
            "       {"
            "           C[get_global_id(0)]=A[get_global_id(0)]+B[get_global_id(0)];"
            "       }"
            "   }";

    Type A[TEST_COUNT];
    Type B[TEST_COUNT];

    for(auto it : all_platforms)
    {
        std::cout << "Test...";tt.tic();
        cl::Platform default_platform=it;
        std::cout << "Using platform: "<<default_platform.getInfo<CL_PLATFORM_NAME>()<<"\n";
    
        //get default device of the default platform
        std::vector<cl::Device> all_devices;
        default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
        if(all_devices.size()==0){
            std::cout<<" No devices found. Check OpenCL installation!\n";
            exit(1);
        }
        cl::Device default_device=all_devices[0];
        std::cout<< "Using device: "<<default_device.getInfo<CL_DEVICE_NAME>()<<"\n";
    
    
        cl::Context context({default_device});
    
        cl::Program::Sources sources;
    
        sources.push_back({kernel_code.c_str(),kernel_code.length()});
    
        cl::Program program(context,sources);
        if(program.build({default_device})!=CL_SUCCESS){
            std::cout<<" Error building: "<<program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(default_device)<<"\n";
            exit(1);
        }
    
        // create buffers on the device
        cl::Buffer buffer_A(context,CL_MEM_READ_WRITE,sizeof(Type)*TEST_COUNT);
        cl::Buffer buffer_B(context,CL_MEM_READ_WRITE,sizeof(Type)*TEST_COUNT);
        cl::Buffer buffer_C(context,CL_MEM_READ_WRITE,sizeof(Type)*TEST_COUNT);
    
    
        for(int i = 0; i < TEST_COUNT; i++)
        {
            A[i] = rand();
            B[i] = rand();
        }

        //create queue to which we will push commands for the device.
        cl::CommandQueue queue(context,default_device);
    
        //write arrays A and B to the device
        queue.enqueueWriteBuffer(buffer_A,CL_TRUE,0,sizeof(Type)*TEST_COUNT,A);
        queue.enqueueWriteBuffer(buffer_B,CL_TRUE,0,sizeof(Type)*TEST_COUNT,B);
    
    
        //run the kernel
        /*cl::KernelFunctor simple_add(cl::Kernel(program,"simple_add"),queue,cl::NullRange,cl::NDRange(10),cl::NullRange);
        simple_add(buffer_A,buffer_B,buffer_C);*/
    
        //alternative way to run the kernel
        cl::Kernel kernel_add=cl::Kernel(program,"simple_add");
        kernel_add.setArg(0,buffer_A);
        kernel_add.setArg(1,buffer_B);
        kernel_add.setArg(2,buffer_C);
        queue.enqueueNDRangeKernel(kernel_add,cl::NullRange,cl::NDRange(TEST_COUNT),cl::NullRange);
        queue.finish();
        
    
        Type C[TEST_COUNT];
        //read result C from the device to array C
        queue.enqueueReadBuffer(buffer_C,CL_TRUE,0,sizeof(Type)*TEST_COUNT,C);
        std::cout << " >> Done: " << tt.toc_string() << " us\n";
    }

    std::cout << "Test...";tt.tic();
    Type D[TEST_COUNT];
    for(int i = 0; i < TEST_COUNT; i++)
    {
        for(int j = 0; j < 10000; j++)
        {
            D[i] = A[i] + B[i];
        }
    }
    std::cout << " >> Done: " << tt.toc_string() << " us\n";

    return 0;

    try
    {
        parser.ParseCLI(argc, argv);
        
        boost::filesystem::path pcapPath{args::get(inputPcap)};
        boost::filesystem::path backgroundPath{args::get(inputBackground)};

        veloFrames.setPcapFile(pcapPath);
        veloFrames.setBackgroundCloud(backgroundPath);
        if(backgroundSeg)
        {
            veloFrames.setBackgroundSegmentationResolution(args::get(backgroundSeg));
        }
        if(noiseRemoval)
        {
            veloFrames.setNoiseRemovalParameter(args::get(noiseRemoval));
        }
        veloFrames.setOffsetPoint(1000.0, 2000.0, 3000.0);

        std::cout << "Loading...";tt.tic();
        veloFrames.load(pcapPath.parent_path().string());
        std::cout << " >> Done: " << tt.toc_string() << " us\n";
        
        if(outputAll)
        {
            std::cout << "Saving...";tt.tic();
            veloFrames.save(pcapPath.parent_path().string());
            std::cout << " >> Done: " << tt.toc_string() << " us\n";
        }

        if(backgroundSeg)
        {
            std::cout << "Background segmentation...";tt.tic();
            veloFrames.backgroundSegmentation();
            std::cout << " >> Done: " << tt.toc_string() << " us\n";
        }

        if(outputAll)
        {
            std::cout << "Saving...";tt.tic();
            veloFrames.save(pcapPath.parent_path().string());
            std::cout << " >> Done: " << tt.toc_string() << " us\n";
        }

        if(noiseRemoval)
        {
            std::cout << "Noise removal...";tt.tic();
            veloFrames.noiseRemoval();
            std::cout << " >> Done: " << tt.toc_string() << " us\n";
        }

        if(output||outputAll)
        {
            std::cout << "Saving...";tt.tic();
            veloFrames.save(pcapPath.parent_path().string());
            std::cout << " >> Done: " << tt.toc_string() << " us\n";
        }

        std::cout << "Offset...";tt.tic();
        veloFrames.offset();
        std::cout << " >> Done: " << tt.toc_string() << " us\n";
        veloFrames.print();

        veloFrameViewer.reset(new VeloFrame::VeloFrameViewer);
        veloFrameViewer->registerKeyboardCallback(keyboardEventOccurred);
        veloFrameViewer->addFrames(veloFrames);
        veloFrameViewer->run();
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
    catch(VeloFrame::VeloFrameException &e)
    {
        std::cout << e.message() << std::endl;
        return 1;
    }

    return 0;
}
