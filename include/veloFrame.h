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
#include "../3rdparty/VelodyneCapture/VelodyneCapture.h"
#include "../include/date.h"
#include "../include/microStopwatch.h"

namespace VeloFrame
{
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
            
    };

    class VeloFrames
    {
        private:
            boost::filesystem::path pcapFilePath;
            boost::filesystem::path outputPath;
            std::vector<boost::shared_ptr<VeloFrame>> frames;
            bool isChanged;
            bool isLoaded;
            bool isSaved;

        public:

            VeloFrames()
            {
                this->pcapFilePath = "";
                this->outputPath = "";
                this->isChanged = false;
                this->isLoaded = false;
                this->isSaved = false;
            }

            bool setPcapFile(std::string pcapFilePath)
            {
                boost::filesystem::path temp = boost::filesystem::path{pcapFilePath};
                if(!boost::filesystem::exists(temp))
                {
                    std::cerr << std::endl << "Error : " << boost::filesystem::absolute(temp).string() << " not found" << std::endl;
                }
                this->pcapFilePath = pcapFilePath;
                if(!boost::filesystem::exists(this->pcapFilePath)) return false;
                this->pcapFilePath = boost::filesystem::canonical(this->pcapFilePath);
            }

            bool load()
            {
                if(this->isLoaded) return false;
                this->loadFromPcap();
            }

            bool load(std::string prefixPath)
            {
                bool result;

                if(this->isLoaded) return false;
                this->outputPath = prefixPath + '/' + this->pcapFilePath.stem().string();

                if(boost::filesystem::exists(this->outputPath))
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

            bool save(std::string prefixPath)
            {
                if(!this->isChanged) 
                {
                    std::cerr << std::endl << "No changes to save" << std::endl;
                    return false;
                }

                this->outputPath = prefixPath + '/' + this->pcapFilePath.stem().string() + '/';
                if(!boost::filesystem::exists(this->outputPath))
                {
                    boost::filesystem::create_directories(this->outputPath);
                }
                this->outputPath = boost::filesystem::canonical(this->outputPath);

                boost::filesystem::path configPath = this->outputPath;
                configPath.append("config.txt");

                std::ofstream ofs(configPath.string());

                ofs << "pcap=" << this->pcapFilePath.string() << std::endl;
                for(int i = 0; i < this->frames.size(); i++)
                {
                    boost::filesystem::path cloudPath = this->outputPath;
                    cloudPath.append(durationToString(this->frames[i]->minTimestamp) + ".pcd");
                    ofs << "cloud_" << i << "=" 
                        << this->frames[i]->minTimestamp.count() << "&"
                        << this->frames[i]->midTimestamp.count() << "&"
                        << this->frames[i]->maxTimestamp.count() << "&"
                        << cloudPath.filename().string() << std::endl;
                    pcl::io::savePCDFileBinaryCompressed(cloudPath.string(), (*this->frames[i]->cloud));
                }

                this->isSaved = true;
            }

            void print(bool showBasicInfo = true)
            {
                std::cerr << "pcap : " << this->pcapFilePath.string() << std::endl;
                std::cerr << "size : " << this->frames.size() << std::endl;
                if(this->isSaved)
                {
                    std::cerr << "output : " << this->outputPath.string() << std::endl;
                }
                if(!showBasicInfo)
                {
                    for(int i = 0; i < this->frames.size(); i++)
                    {
                        std::cerr << "\tcloud " << i << " : "<< std::endl;
                        std::cerr << "\t\tpoint size : " << this->frames[i]->cloud->points.size() << std::endl;
                        std::cerr << "\t\tminTimestamp : " << this->durationToString(this->frames[i]->minTimestamp, false)
                                    << " (" << this->frames[i]->minTimestamp.count() << ")" << std::endl;
                        std::cerr << "\t\tmidTimestamp : " << this->durationToString(this->frames[i]->midTimestamp, false)
                                    << " (" << this->frames[i]->midTimestamp.count() << ")" << std::endl;
                        std::cerr << "\t\tmaxTimestamp : " << this->durationToString(this->frames[i]->maxTimestamp, false)
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

                size_t divisionNumber = std::ceil(f.size() / (std::thread::hardware_concurrency()+1));

                bool result = loadFromPcapPart(divisionNumber, f.begin(), f.end(), this->frames.begin(), this->frames.end());
                    
                this->isChanged = true;
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
                boost::filesystem::path configPath = this->outputPath;
                configPath.append("config.txt");

                std::ifstream ifs(configPath.string());

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

                    if(ss2s[0] == "pcap") this->pcapFilePath = boost::filesystem::path{ss2s[0]};
                    else
                    {
                        f.push_back(ss2s[1]);
                    }
                }

                this->frames.resize(f.size());

                size_t divisionNumber = std::ceil(f.size() / (std::thread::hardware_concurrency()+1));

                bool result = loadFromFolderPart(divisionNumber, f.begin(), f.end(), this->frames.begin(), this->frames.end());
                
                std::sort(this->frames.begin(), this->frames.end(), [](const auto &a, const auto &b){ return a->minTimestamp < b->minTimestamp; });
            }

            template<typename RandomIt>
            std::string durationToString(const RandomIt &duration, const bool isFileName = true)
            {
                std::ostringstream stream;
                std::chrono::time_point<std::chrono::system_clock> tp = std::chrono::time_point<std::chrono::system_clock>(duration);
                tp += std::chrono::hours(TIMEZONE);
                auto dp = date::floor<date::days>(tp);  // dp is a sys_days, which is a
                                                // type alias for a C::time_point
                auto date = date::year_month_day{dp};
                auto time = date::make_time(std::chrono::duration_cast<std::chrono::milliseconds>(tp-dp));
                stream << std::setfill('0') << std::setw(4) << date.year().operator int();
                if(!isFileName) stream << '/';
                stream << std::setfill('0') << std::setw(2) << date.month().operator unsigned int();
                if(!isFileName) stream << '/';
                stream << std::setfill('0') << std::setw(2) << date.day().operator unsigned int();
                if(!isFileName) stream << ' ';
                else stream << '_';
                stream << std::setfill('0') << std::setw(2) << time.hours().count();
                if(!isFileName) stream << ':';
                stream << std::setfill('0') << std::setw(2) << time.minutes().count();
                if(!isFileName) stream << ':';
                stream << std::setfill('0') << std::setw(2) << time.seconds().count();
                if(!isFileName) stream << '.';
                else stream << '_';
                stream << std::setfill('0') << std::setw(3) << time.subseconds().count();
                if(typeid(RandomIt) == typeid(std::chrono::microseconds))
                {
                    stream << std::setfill('0') << std::setw(3) << duration.count() % 1000;
                }
                if(typeid(RandomIt) == typeid(std::chrono::nanoseconds))
                {
                    stream << std::setfill('0') << std::setw(3) << duration.count() % 1000000;
                }
                return stream.str();
            }

    };
}

#endif