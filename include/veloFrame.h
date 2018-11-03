#ifndef VELOFRAME_H_
#define VELOFRAME_H_

#ifndef TIMEZONE
    #define TIMEZONE 0
#endif

#include <iostream>
#include <chrono>
#include <typeinfo>
#include <future>
#include <boost/shared_ptr.hpp>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include "../3rdparty/VelodyneCapture/VelodyneCapture.h"
#include "../include/basic_function.h"
#include "../include/microStopwatch.h"
#include "../include/backgroundSegmentation.h"

namespace VeloFrame
{
    class VeloFrameException : public std::exception
    {
        private:
            std::string exceptionMessage;

        public:
            VeloFrameException() :
                exceptionMessage ("No message.") {}
            explicit VeloFrameException(std::string message) :
                exceptionMessage (std::move(message)) {}
            const char *what() const throw()
            {
                std::stringstream s;
                s << "VeloFrameException : " << this->exceptionMessage << std::endl;

                std::string wharString = s.str();
                return wharString.c_str();
            }
            std::string message() const
            {
                std::stringstream s;
                s << "\033[0;31m";      //red
                s << "VeloFrameException: ";
                s << "\033[0m";         //default color
                s << this->exceptionMessage;
                s << std::endl;
                return s.str();
            }
    };

    class VeloFrame
    {
        public:
            std::chrono::microseconds minTimestamp;
            std::chrono::microseconds midTimestamp;
            std::chrono::microseconds maxTimestamp;
            boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud;

            VeloFrame()
            {
                this->minTimestamp = std::chrono::microseconds(0);
                this->midTimestamp = std::chrono::microseconds(0);
                this->maxTimestamp = std::chrono::microseconds(0);
                this->cloud = NULL;
            }
            VeloFrame(const long long &minTimestamp, const long long &midTimestamp, const long long &maxTimestamp, const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud)
            {
                this->minTimestamp = std::chrono::microseconds(minTimestamp);
                this->midTimestamp = std::chrono::microseconds(midTimestamp);
                this->maxTimestamp = std::chrono::microseconds(maxTimestamp);
                this->cloud = cloud;
            }

            void setMinTimestamp(const long long &minTimestamp)
            {
                this->minTimestamp = std::chrono::microseconds(minTimestamp);
            }
            
            void setMidTimestamp(const long long &midTimestamp)
            {
                this->midTimestamp = std::chrono::microseconds(midTimestamp);
            }
            
            void setMaxTimestamp(const long long &maxTimestamp)
            {
                this->maxTimestamp = std::chrono::microseconds(maxTimestamp);
            }

            void setCloud(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud)
            {
                this->cloud = cloud;
            }
    };

    class VeloFrames
    {
        friend class VeloFrameViewer;
        protected:
            boost::filesystem::path pcapFilePath;
            boost::filesystem::path outputPath;
            boost::filesystem::path outputPathWithParameter;
            std::string parameterString;
            std::vector<boost::shared_ptr<VeloFrame>> frames;
            boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> backgroundCloud;
            bool isChanged;
            bool isLoaded;
            bool isSaved;
            bool isBackgroundSegmented;
            bool isNoiseRemoved;
            double backgroundSegmentationResolution;
            double noiseRemovalPercentP;
            double noiseRemovalStddevMulThresh;
            std::chrono::microseconds begTime;
            std::chrono::microseconds endTime;

        public:

            VeloFrames()
            {
                this->pcapFilePath = "";
                this->outputPath = "";
                this->parameterString = "";
                this->isChanged = false;
                this->isLoaded = false;
                this->isSaved = false;
                this->isBackgroundSegmented = false;
                this->isNoiseRemoved = false;
                this->backgroundSegmentationResolution = 0.0;
                this->noiseRemovalPercentP = 0.0;
                this->noiseRemovalStddevMulThresh = 0.0;
                this->begTime = std::chrono::microseconds(int64_t(0));
                this->endTime = std::chrono::microseconds(std::numeric_limits<int64_t>::max());
            }

            bool setPcapFile(const std::string &pcapFilePath)
            {
                boost::filesystem::path temp = boost::filesystem::path{pcapFilePath};
                if(!boost::filesystem::exists(temp))
                {                    
                    throw VeloFrameException(boost::filesystem::absolute(temp).string() + " not found");
                    return false;
                }
                this->pcapFilePath = boost::filesystem::canonical(temp);
                return true;
            }

            bool setBackgroundSegmentationResolution(const double &backgroundSegmentationResolutionCM)
            {
                if(backgroundSegmentationResolutionCM <= 0.0) throw VeloFrameException("background segmentation resolution should > 0");
                this->backgroundSegmentationResolution = backgroundSegmentationResolutionCM;
            }

            bool setNoiseRemovalPercentP(const double &noiseRemovalPercentP)
            {
                if((noiseRemovalPercentP <= 0.0)&&(noiseRemovalPercentP > 1.0)) throw VeloFrameException("noise removal percentP should > 0 and <= 1.0");
                this->noiseRemovalPercentP = noiseRemovalPercentP;
            }

            bool setNoiseRemovalStddevMulThresh(const double &noiseRemovalStddevMulThresh)
            {
                if((noiseRemovalStddevMulThresh <= 0.0)&&(noiseRemovalStddevMulThresh > 1.0)) throw VeloFrameException("noise removal stddevMulThresh should > 0 and <= 1.0");
                this->noiseRemovalStddevMulThresh = noiseRemovalStddevMulThresh;
            }

            bool setBackgroundCloud(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &backgroundCloud)
            {
                this->backgroundCloud = backgroundCloud;
            }

            bool setBegTime(const std::chrono::microseconds &begTime)
            {
                if(begTime < std::chrono::microseconds(int64_t(0))) throw VeloFrameException("Begin time can't < 0");
                this->begTime = begTime;
            }

            bool setEndTime(const std::chrono::microseconds &endTime)
            {
                if(endTime <= this->begTime) throw VeloFrameException("End time should > begin time");
                this->endTime = endTime;
            }

            bool resetParameterString()
            {
                std::stringstream ps;
                if(this->isBackgroundSegmented)
                {
                    ps << "bs_" << this->backgroundSegmentationResolution;
                }
                ps << '/';
                if(this->isNoiseRemoved)
                {
                    ps << "nr_" << this->noiseRemovalPercentP << "_" << this->noiseRemovalStddevMulThresh;
                }
                this->parameterString = ps.str();
            }

            bool load()
            {
                if(this->isLoaded) return false;
                this->loadFromPcap();
                this->isLoaded = true;
                return true;
            }

            bool load(const std::string &prefixPath)
            {
                bool result;

                if(this->frames.size() != 0)
                {
                    return false;
                }
                this->outputPath = prefixPath;
                this->outputPath.append(this->pcapFilePath.stem().string());
                this->resetParameterString();
                this->outputPathWithParameter = this->outputPath;
                this->outputPathWithParameter.append(this->parameterString);

                if(!boost::filesystem::exists(this->outputPathWithParameter)) this->outputPathWithParameter = this->outputPath;

                if(boost::filesystem::exists(this->outputPathWithParameter))
                {
                    result = this->loadFromFolder();
                }
                else
                {
                    result = this->loadFromPcap();
                }

                this->isLoaded = result;
                return result;
            }

            bool save(const std::string &prefixPath)
            {
                if(!this->isChanged) 
                {
                    std::cerr << std::endl << "No changes to save" << std::endl;
                    return false;
                }
                if(this->isSaved)
                {
                    std::cerr << std::endl << "No changes to save" << std::endl;
                    return false;
                }


                this->outputPath = prefixPath;
                this->outputPath.append(this->pcapFilePath.stem().string());
                this->resetParameterString();
                this->outputPathWithParameter = this->outputPath;
                this->outputPathWithParameter.append(this->parameterString);

                if(!boost::filesystem::exists(this->outputPath))
                {
                    boost::filesystem::create_directories(this->outputPath);
                }
                this->outputPath = boost::filesystem::canonical(this->outputPath);

                if(!boost::filesystem::exists(this->outputPathWithParameter))
                {
                    boost::filesystem::create_directories(this->outputPathWithParameter);
                }
                this->outputPathWithParameter = boost::filesystem::canonical(this->outputPathWithParameter);

                boost::filesystem::path configPath = this->outputPathWithParameter;
                configPath.append("config.txt");

                std::ofstream ofs(configPath.string());

                ofs << "pcap=" << this->pcapFilePath.string() << std::endl;
                if(this->backgroundCloud != NULL)
                {
                    boost::filesystem::path backgroundPath = this->outputPath;
                    backgroundPath.append("background.pcd");
                    pcl::io::savePCDFileBinaryCompressed(backgroundPath.string(), *(this->backgroundCloud));
                    ofs << "background=" << boost::filesystem::relative(this->outputPath, backgroundPath).string() << std::endl;
                }

                #ifdef VELOFRAME_USE_MULTITHREAD
                for(int i = 0; i < this->frames.size(); i++)
                {
                    boost::filesystem::path cloudPath = this->outputPathWithParameter;
                    cloudPath.append(myFunction::durationToString(this->frames[i]->minTimestamp) + ".pcd");
                    ofs << "cloud_" << i << "=" 
                        << this->frames[i]->minTimestamp.count() << "&"
                        << this->frames[i]->midTimestamp.count() << "&"
                        << this->frames[i]->maxTimestamp.count() << "&"
                        << boost::filesystem::relative(this->outputPath, cloudPath).string() << std::endl;
                }

                size_t divisionNumber = myFunction::getDivNum(this->frames.size());

                bool result = this->savePart(divisionNumber, this->frames.begin(), this->frames.end());
                #else
                for(int i = 0; i < this->frames.size(); i++)
                {
                    boost::filesystem::path cloudPath = this->outputPath;
                    cloudPath.append(durationToString(this->frames[i]->minTimestamp) + ".pcd");
                    pcl::io::savePCDFileBinaryCompressed(cloudPath.string(), *(this->frames[i]->cloud));
                    ofs << "cloud_" << i << "=" 
                        << this->frames[i]->minTimestamp.count() << "&"
                        << this->frames[i]->midTimestamp.count() << "&"
                        << this->frames[i]->maxTimestamp.count() << "&"
                        << boost::filesystem::relative(this->outputPath, cloudPath).string() << std::endl;
                }
                #endif

                this->isSaved = true;
                return true;
            }

	        template<typename RandomIt1>
            bool backgroundSegmentationPart(const size_t &divisionNumber, const myClass::backgroundSegmentation<pcl::PointXYZI> backgroundSegmentation, const RandomIt1 &beg, const RandomIt1 &end)
            {
                auto len = end - beg;

                if(len < divisionNumber)
                {
                    bool out = true;;
                    for(auto it = beg; it != end; ++it)
                    {
                        (*it)->cloud = backgroundSegmentation.compute((*it)->cloud);
                    }
                    return out;
                }

                auto mid = beg + len/2;
                auto handle = std::async(std::launch::async, &VeloFrames::backgroundSegmentationPart<RandomIt1>, this, divisionNumber, backgroundSegmentation, beg, mid);
                auto out1 = VeloFrames::backgroundSegmentationPart<RandomIt1>(divisionNumber, backgroundSegmentation, mid, end);
                auto out = handle.get();

                return out & out1;
            }

            bool backgroundSegmentation()
            {
                if(this->frames.size() == 0) throw VeloFrameException("veloFrame is empty");
                if(this->backgroundCloud == NULL) throw VeloFrameException("background cloud not set");
                if(this->backgroundSegmentationResolution <= 0) throw VeloFrameException("background segmentation resolution not set");

                #ifdef VELOFRAME_USE_MULTITHREAD
                myClass::backgroundSegmentation<pcl::PointXYZI> backgroundSegmentation;
                backgroundSegmentation.setBackground(this->backgroundCloud, this->backgroundSegmentationResolution);

                size_t divisionNumber = myFunction::getDivNum(this->frames.size());

                bool result = this->backgroundSegmentationPart(divisionNumber, backgroundSegmentation, this->frames.begin(), this->frames.end());
                #else
                myClass::backgroundSegmentation<pcl::PointXYZI> backgroundSegmentation;
                backgroundSegmentation.setBackground(this->backgroundCloud, this->backgroundSegmentationResolution);
                for(auto it : this->frames)
                {
                    it->cloud = backgroundSegmentation.compute(it->cloud);
                }
                #endif

                for(int i = 0; i < this->frames.size(); ++i)
                {
                    if(this->frames[i]->cloud->points.size() == 0)
                    {
                        this->frames.erase(this->frames.begin() + i);
                        --i;
                        continue;
                    }
                }

                this->isBackgroundSegmented = true;
                this->isChanged = true;
                this->isSaved = false;
            }

	        template<typename RandomIt1>
            bool noiseRemovalPart(const size_t &divisionNumber, const RandomIt1 &beg, const RandomIt1 &end)
            {
                auto len = end - beg;

                if(len < divisionNumber)
                {
                    bool out = true;;
                    for(auto it = beg; it != end; ++it)
                    {
                        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
                        sor.setInputCloud ((*it)->cloud);
                        sor.setMeanK ((*it)->cloud->points.size() * this->noiseRemovalPercentP);
                        sor.setStddevMulThresh (this->noiseRemovalStddevMulThresh);
                        sor.filter (*((*it)->cloud));
                    }
                    return out;
                }

                auto mid = beg + len/2;
                auto handle = std::async(std::launch::async, &VeloFrames::noiseRemovalPart<RandomIt1>, this, divisionNumber, beg, mid);
                auto out1 = VeloFrames::noiseRemovalPart<RandomIt1>(divisionNumber, mid, end);
                auto out = handle.get();

                return out & out1;
            }

            bool noiseRemoval()
            {
                
                #ifdef VELOFRAME_USE_MULTITHREAD
                size_t divisionNumber = myFunction::getDivNum(this->frames.size());

                bool result = this->noiseRemovalPart(divisionNumber, this->frames.begin(), this->frames.end());
                #else
                pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
                for(auto it : this->frames)
                {
                    sor.setInputCloud (it->cloud);
                    sor.setMeanK (it->cloud->points.size() * this->noiseRemovalPercentP);
                    sor.setStddevMulThresh (this->noiseRemovalStddevMulThresh);
                    sor.filter (*(it->cloud));
                }
                #endif

                for(int i = 0; i < this->frames.size(); ++i)
                {
                    if(this->frames[i]->cloud->points.size() == 0)
                    {
                        this->frames.erase(this->frames.begin() + i);
                        --i;
                        continue;
                    }
                }

                this->isNoiseRemoved = true;
                this->isChanged = true;
                this->isSaved = false;
            }

            void print(const bool showBasicInfo = true) const
            {
                std::cerr << "pcap : " << this->pcapFilePath.string() << std::endl;
                std::cerr << "size : " << this->frames.size() << std::endl;
                if(this->isSaved)
                {
                    std::cerr << "output : " << this->outputPathWithParameter.string() << std::endl;
                }
                if(!showBasicInfo)
                {
                    for(int i = 0; i < this->frames.size(); i++)
                    {
                        std::cerr << "\tcloud " << i << " : "<< std::endl;
                        std::cerr << "\t\tpoint size : " << this->frames[i]->cloud->points.size() << std::endl;
                        std::cerr << "\t\tminTimestamp : " << myFunction::durationToString(this->frames[i]->minTimestamp, false)
                                    << " (" << this->frames[i]->minTimestamp.count() << ")" << std::endl;
                        std::cerr << "\t\tmidTimestamp : " << myFunction::durationToString(this->frames[i]->midTimestamp, false)
                                    << " (" << this->frames[i]->midTimestamp.count() << ")" << std::endl;
                        std::cerr << "\t\tmaxTimestamp : " << myFunction::durationToString(this->frames[i]->maxTimestamp, false)
                                    << " (" << this->frames[i]->maxTimestamp.count() << ")" << std::endl;
                    }
                }
            }
        private:

	        template<typename RandomIt1, typename RandomIt2>
            bool loadFromPcapPart(const size_t &divisionNumber, const RandomIt1 &beg1, const RandomIt1 &end1, const RandomIt2 &beg2, const RandomIt2 &end2)
            {
                auto len1 = end1 - beg1;
                auto len2 = end2 - beg2;

                if(len1 < divisionNumber)
                {
                    bool out = true;
                    pcl::PointXYZI point;
                    auto it2 = beg2;
                    for(auto it1 = beg1; it1 != end1; ++it1,++it2)
                    {
                        long long minTimestamp = std::numeric_limits<long long>::max();
                        long long midTimestamp = 0;
                        long long maxTimestamp = std::numeric_limits<long long>::min();
                        (*it2).reset(new VeloFrame());
                        (*it2)->cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

                        for( const velodyne::Laser& laser : (**it1) ){
                            const double distance = static_cast<double>( laser.distance );
                            const double azimuth  = laser.azimuth  * M_PI / 180.0;
                            const double vertical = laser.vertical * M_PI / 180.0;
                        
                            minTimestamp = std::min(laser.time, minTimestamp);
                            maxTimestamp = std::max(laser.time, maxTimestamp);
                            midTimestamp = (maxTimestamp - minTimestamp)/2 + minTimestamp;


                            point.x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
                            point.y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
                            point.z = static_cast<float>( ( distance * std::sin( vertical ) ) );
                        
                            point.intensity = laser.intensity;

                            if( point.x == 0.0f && point.y == 0.0f && point.z == 0.0f ){
                                point.x = std::numeric_limits<float>::quiet_NaN();
                                point.y = std::numeric_limits<float>::quiet_NaN();
                                point.z = std::numeric_limits<float>::quiet_NaN();
                            }
                        
                            (*it2)->cloud->points.push_back(point);
                        }
                        (*it2)->minTimestamp = std::chrono::microseconds(minTimestamp);
                        (*it2)->midTimestamp = std::chrono::microseconds(midTimestamp);
                        (*it2)->maxTimestamp = std::chrono::microseconds(maxTimestamp);
                        (*it2)->cloud->width = (int) (*it2)->cloud->points.size();
                        (*it2)->cloud->height = 1;
                    }
                    return out;
                }

                auto mid1 = beg1 + len1/2;
                auto mid2 = beg2 + len1/2;
                auto handle = std::async(std::launch::async, &VeloFrames::loadFromPcapPart<RandomIt1, RandomIt2>, this, divisionNumber, beg1, mid1, beg2, mid2);
                auto out1 = VeloFrames::loadFromPcapPart<RandomIt1, RandomIt2>(divisionNumber, mid1, end1, mid2, end2);
                auto out = handle.get();

                return out & out1;
            }

            bool loadFromPcap()
            {
                velodyne::VLP16Capture vlp16;

                if(!vlp16.open(this->pcapFilePath.string()))
                {
                    std::cerr << std::endl << "Error : load " << this->pcapFilePath << " failed" << std::endl;
                    return false;
                }

                if(!vlp16.isOpen())
                {
                    std::cerr << std::endl << "Error : load " << this->pcapFilePath << " failed" << std::endl;
                    return false;
                }

                #ifdef VELOFRAME_USE_MULTITHREAD
                std::vector<boost::shared_ptr<std::vector<velodyne::Laser>>> f;

                while(vlp16.isRun())
                {
                    boost::shared_ptr<std::vector<velodyne::Laser>> lasers(new std::vector<velodyne::Laser>);
                    vlp16 >> *lasers;
                    if( (*lasers).empty() ){
                        continue;
                    }
                    f.push_back(lasers);
                }
                this->frames.resize(f.size());

                size_t divisionNumber = myFunction::getDivNum(f.size());

                bool result = this->loadFromPcapPart(divisionNumber, f.begin(), f.end(), this->frames.begin(), this->frames.end());
                #else
                boost::shared_ptr<VeloFrame> frame;
                pcl::PointXYZI point;
                while(vlp16.isRun())
                {
                    std::vector<velodyne::Laser> lasers;
                    vlp16 >> lasers;
                    if( lasers.empty() ){
                        continue;
                    }

                    long long minTimestamp = std::numeric_limits<long long>::max();
                    long long midTimestamp = 0;
                    long long maxTimestamp = std::numeric_limits<long long>::min();
                    frame.reset(new VeloFrame);
                    frame->cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

                    for( const velodyne::Laser& laser : lasers ){
                        const double distance = static_cast<double>( laser.distance );
                        const double azimuth  = laser.azimuth  * M_PI / 180.0;
                        const double vertical = laser.vertical * M_PI / 180.0;
                    
                        minTimestamp = std::min(laser.time, minTimestamp);
                        maxTimestamp = std::max(laser.time, maxTimestamp);
                        midTimestamp = (maxTimestamp - minTimestamp)/2 + minTimestamp;


                        point.x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
                        point.y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
                        point.z = static_cast<float>( ( distance * std::sin( vertical ) ) );
                    
                        point.intensity = laser.intensity;

                        if( point.x == 0.0f && point.y == 0.0f && point.z == 0.0f ){
                            point.x = std::numeric_limits<float>::quiet_NaN();
                            point.y = std::numeric_limits<float>::quiet_NaN();
                            point.z = std::numeric_limits<float>::quiet_NaN();
                        }
                    
                        frame->cloud->points.push_back(point);
                    }
                    frame->minTimestamp = std::chrono::microseconds(minTimestamp);
                    frame->midTimestamp = std::chrono::microseconds(midTimestamp);
                    frame->maxTimestamp = std::chrono::microseconds(maxTimestamp);
                    frame->cloud->width = (int) frame->cloud->points.size();
                    frame->cloud->height = 1;
                    this->frames.push_back(frame);
                }
                #endif
                this->isChanged = true;
                return true;
            }

	        template<typename RandomIt1, typename RandomIt2>
            bool loadFromFolderPart(const size_t &divisionNumber, const RandomIt1 &beg1, const RandomIt1 &end1, const RandomIt2 &beg2, const RandomIt2 &end2)
            {
                auto len1 = end1 - beg1;
                auto len2 = end2 - beg2;

                if(len1 < divisionNumber)
                {
                    bool out = true;
                    auto it2 = beg2;
                    for(auto it1 = beg1; it1 != end1; ++it1,++it2)
                    {
                        std::vector<std::string> ss;
                        boost::filesystem::path cloudPath = this->outputPath;
                        boost::split(ss, *it1, boost::is_any_of("&"));

                        (*it2).reset(new VeloFrame());
                        (*it2)->cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
                        (*it2)->setMinTimestamp(std::stoll(ss[0]));
                        (*it2)->setMidTimestamp(std::stoll(ss[1]));
                        (*it2)->setMaxTimestamp(std::stoll(ss[2]));
                        cloudPath.append(ss[3]);
                        pcl::io::loadPCDFile(cloudPath.string(), *((*it2)->cloud));
                    }
                    return out;
                }

                auto mid1 = beg1 + len1/2;
                auto mid2 = beg2 + len1/2;
                auto handle = std::async(std::launch::async, &VeloFrames::loadFromFolderPart<RandomIt1, RandomIt2>, this, divisionNumber, beg1, mid1, beg2, mid2);
                auto out1 = VeloFrames::loadFromFolderPart<RandomIt1, RandomIt2>(divisionNumber, mid1, end1, mid2, end2);
                auto out = handle.get();

                return out & out1;
            }

            bool loadFromFolder()
            {
                boost::filesystem::path configPath = this->outputPathWithParameter;
                configPath.append("config.txt");

                if(!boost::filesystem::exists(configPath))
                {
                    throw VeloFrameException(boost::filesystem::absolute(configPath).string() + " not found");
                    return false;
                }

                std::ifstream ifs(configPath.string());

                if(!ifs.is_open()) return false;

                std::string line;
                std::vector<std::string> ss1s;
                std::vector<std::string> f;

                while(!ifs.eof())
                {
                    std::getline(ifs, line);
                    boost::trim(line);
                    if(line == "") continue;
                    ss1s.push_back(line);
                }

                for(auto ss1 : ss1s)
                {
                    std::vector<std::string> ss2s;
                    boost::split(ss2s, ss1, boost::is_any_of("="));

                    if(ss2s[0] == "pcap") this->pcapFilePath = boost::filesystem::path{ss2s[1]};
                    else if(ss2s[0] == "background")
                    {
                        boost::filesystem::path backgroundPath = this->outputPath;
                        this->backgroundCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
                        backgroundPath.append(ss2s[1]);
                        pcl::io::loadPCDFile(backgroundPath.string(), *(this->backgroundCloud));
                    }
                    else
                    {
                        f.push_back(ss2s[1]);
                    }
                }
                this->frames.resize(f.size());

                #ifdef VELOFRAME_USE_MULTITHREAD

                size_t divisionNumber = myFunction::getDivNum(f.size());

                bool result = this->loadFromFolderPart(divisionNumber, f.begin(), f.end(), this->frames.begin(), this->frames.end());
                #else
                auto it2 = this->frames.begin();
                for(auto it1 : f)
                {
                    std::vector<std::string> ss;
                    boost::filesystem::path cloudPath = this->outputPath;
                    boost::split(ss, it1, boost::is_any_of("&"));

                    (*it2).reset(new VeloFrame());
                    (*it2)->cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
                    (*it2)->setMinTimestamp(std::stoll(ss[0]));
                    (*it2)->setMidTimestamp(std::stoll(ss[1]));
                    (*it2)->setMaxTimestamp(std::stoll(ss[2]));
                    cloudPath.append(ss[3]);
                    pcl::io::loadPCDFile(cloudPath.string(), *((*it2)->cloud));
                    it2++;
                }
                #endif

                std::sort(this->frames.begin(), this->frames.end(), [](const auto &a, const auto &b){ return a->minTimestamp < b->minTimestamp; });

                return true;
            }

	        template<typename RandomIt1>
            bool savePart(const size_t &divisionNumber, const RandomIt1 &beg, const RandomIt1 &end)
            {
                auto len = end - beg;

                if(len < divisionNumber)
                {
                    bool out = true;;
                    for(auto it = beg; it != end; ++it)
                    {
                        boost::filesystem::path cloudPath = this->outputPathWithParameter;
                        cloudPath.append(myFunction::durationToString((*it)->minTimestamp) + ".pcd");
                        pcl::io::savePCDFileBinaryCompressed(cloudPath.string(), *((*it)->cloud));
                    }
                    return out;
                }

                auto mid = beg + len/2;
                auto handle = std::async(std::launch::async, &VeloFrames::savePart<RandomIt1>, this, divisionNumber, beg, mid);
                auto out1 = VeloFrames::savePart<RandomIt1>(divisionNumber, mid, end);
                auto out = handle.get();

                return out & out1;
            }

    };

    class VeloFrameViewer : protected pcl::visualization::PCLVisualizer
    {
        private:
            bool pause;
            bool showBackground;
            bool realTime;
            bool startTimeReset;
            double playSpeedRate;
            double pointSize;
            std::vector<boost::shared_ptr<VeloFrame>> frames;
            std::queue<boost::shared_ptr<VeloFrame>> queue;
            std::chrono::steady_clock::time_point startTime;
            std::chrono::microseconds startTimestamp;
            boost::function<void (const pcl::visualization::KeyboardEvent&, void*)> externalKeyboardEventOccurred;

        public:
            VeloFrameViewer():pcl::visualization::PCLVisualizer()
            {
                this->pause = false;
                this->showBackground = false;
                this->realTime = true;
                this->playSpeedRate = 1.0;
                this->pointSize = 1.0;
                this->startTime = std::chrono::steady_clock::now();
                this->startTimestamp = std::chrono::microseconds(int64_t(0));
                this->addCoordinateSystem( 3.0, "coordinate" );
                this->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
                this->initCameraParameters();
                this->setCameraPosition( 0.0, 0.0, 2000.0, 0.0, 1.0, 0.0, 0 );

            }

            void addFrames(const VeloFrames &vf)
            {
                for(auto it : vf.frames)
                {
                    boost::shared_ptr<VeloFrame> temp(new VeloFrame);
                    *temp = *it;
                    this->frames.push_back(temp);
                }
                std::sort(this->frames.begin(), this->frames.end(), [](const auto &a, const auto &b){ return a->minTimestamp < b->minTimestamp; });
                for(int i = 0; i < (this->frames.size()-1); ++i)
                {
                    if(this->frames[i]->minTimestamp == this->frames[i+1]->minTimestamp)
                    {
                        this->frames.erase(this->frames.begin() + i);
                        --i;
                        continue;
                    }
                }
            }

            void run()
            {
                boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>> handler( new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>( "intensity" ) );
                std::chrono::microseconds nextTimestamp;

                for(auto it : this->frames)
                {
                    this->queue.push(it);
                }

                this->startTime = std::chrono::steady_clock::now();
                this->startTimestamp = this->queue.front()->minTimestamp - std::chrono::microseconds(int64_t(1000000/5));
                nextTimestamp = this->queue.front()->minTimestamp;
                while(!this->wasStopped())
                {
                    this->spinOnce();
                    
                    if(this->pause) continue;
                    bool stay = (((std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - this->startTime).count()*this->playSpeedRate) < (nextTimestamp-this->startTimestamp).count()) && this->realTime);

                    if(stay) continue;

                    auto it = this->queue.front();
                    this->queue.pop();
                    
                    handler->setInputCloud( it->cloud );
                    if( !this->updatePointCloud( it->cloud, *handler, "cloud" ) ){
                        this->addPointCloud( it->cloud, *handler, "cloud" );
                    }
                    this->queue.push(it);
                    
                    if(nextTimestamp > this->queue.front()->midTimestamp)
                    {
                        this->startTime = std::chrono::steady_clock::now();
                    }
                    nextTimestamp = this->queue.front()->minTimestamp;
                }
            }

            inline void registerKeyboardCallback (void (*callback) (const pcl::visualization::KeyboardEvent&, void*), void* cookie = NULL)
            {
                this->externalKeyboardEventOccurred = boost::bind(callback, _1, cookie);
                this->pcl::visualization::PCLVisualizer::registerKeyboardCallback(&VeloFrameViewer::keyboardEventOccurred, *this);
            }

            void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
            {
                if(event.isCtrlPressed())
                {
                    if((event.getKeySym() == "b")&&(event.keyDown()))
                    {
                            this->showBackground = !this->showBackground;
                            std::cerr << "showBackground: " << ((this->showBackground == true)? "ON" : "OFF") << std::endl;
                    }
                    else if((event.getKeySym() == "space")&&(event.keyDown()))
                    {
                        this->realTime = !this->realTime;
                        std::cerr << "realTime: " << ((this->realTime == true)? "ON" : "OFF") << std::endl;
                    }
                    else if((event.getKeySym() == "KP_Add")&&(event.keyDown()))
                    {
                        if(this->playSpeedRate < 10.0) this->playSpeedRate += 0.25;
                        this->startTimeReset = true;
                        std::cerr << "playSpeedRate: " << this->playSpeedRate << std::endl;
                    }
                    else if((event.getKeySym() == "KP_Subtract")&&(event.keyDown()))
                    {
                        if(this->playSpeedRate > 0.25) this->playSpeedRate -= 0.25;
                        this->startTimeReset = true;
                        std::cerr << "playSpeedRate: " << this->playSpeedRate << std::endl;
                    }
                    else
                    {
                        this->externalKeyboardEventOccurred(event, nothing);
                    }
                }
                else
                {
                    if((event.getKeySym() == "space")&&(event.keyDown()))
                    {
                        this->pause = !this->pause;
                        std::cerr << "viewer_pause: " << ((this->pause == true)? "ON" : "OFF") << std::endl;
                    }
                    else if((event.getKeySym() == "KP_Add")&&(event.keyDown()))
                    {
                        if(this->pointSize < 10.0) this->pointSize += 0.5;
                    }
                    else if((event.getKeySym() == "KP_Subtract")&&(event.keyDown()))
                    {
                        if(this->pointSize > 1.0) this->pointSize -= 0.5;
                    }
                    else
                    {
                        this->externalKeyboardEventOccurred(event, nothing);
                    }
                }

            }

    };
}

#endif