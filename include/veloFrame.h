#ifndef VELOFRAME_H_
#define VELOFRAME_H_

#include <iostream>
#include <chrono>
//#include <vector>
#include <boost/shared_ptr.hpp>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>
#include "../3rdparty/VelodyneCapture/VelodyneCapture.h"

namespace VeloFrame
{
    class VeloFrame
    {
        public:
            std::chrono::microseconds minTimestamp;
            std::chrono::microseconds maxTimestamp;
            std::chrono::microseconds avgTimestamp;
            boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud;

            VeloFrame();
            VeloFrame(long long &minTimestamp, long long &maxTimestamp, long long &avgTimestamp, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &cloud)
            {
                this->minTimestamp = std::chrono::microseconds(minTimestamp);
                this->maxTimestamp = std::chrono::microseconds(maxTimestamp);
                this->avgTimestamp = std::chrono::microseconds(avgTimestamp);
                this->cloud = cloud;
            }
    };

    class VeloFrames
    {
        public:
            std::string pcapFileName;
            std::vector<VeloFrame> frames;

            bool loadFromPcap()
            {
                velodyne::VLP16Capture vlp16;
                boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud;

                if(!vlp16.open(pcapFileName)) return false;

                if(!vlp16.isOpen()) return false;

                while(vlp16.isRun())
                {
                    pcl::PointXYZI point;
                    std::vector<velodyne::Laser> lasers;
                    vlp16 >> lasers;
                    if( lasers.empty() ){
                        continue;
                    }
                
                    /*
                    // Sort Laser Data ( 0 degree -> 359 degree, 0 id -> n id )
                    std::sort( lasers.begin(), lasers.end() );
                    */

                    
                
                    long long minTimestamp = std::numeric_limits<long long>::max();
                    long long maxTimestamp = std::numeric_limits<long long>::min();
                    long long avgTimestamp = 0;

                    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

                    for( const velodyne::Laser& laser : lasers ){
                        const double distance = static_cast<double>( laser.distance );
                        const double azimuth  = laser.azimuth  * M_PI / 180.0;
                        const double vertical = laser.vertical * M_PI / 180.0;
                    
                        minTimestamp = std::min(laser.time, minTimestamp);
                        maxTimestamp = std::max(laser.time, maxTimestamp);
                        avgTimestamp += laser.time;


                        point.x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
                        point.y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
                        point.z = static_cast<float>( ( distance * std::sin( vertical ) ) );
                    
                        point.intensity = laser.intensity;

                        if( point.x == 0.0f && point.y == 0.0f && point.z == 0.0f ){
                            point.x = std::numeric_limits<float>::quiet_NaN();
                            point.y = std::numeric_limits<float>::quiet_NaN();
                            point.z = std::numeric_limits<float>::quiet_NaN();
                        }
                    
                        cloud->points.push_back(point);
                    }

                    avgTimestamp /= lasers.size();

                    cloud->width = (int) cloud->points.size();
                    cloud->height = 1;

                    frames.push_back(VeloFrame(minTimestamp, maxTimestamp, avgTimestamp, cloud));
                }
            }

            friend std::ostream& operator << (std::ostream& out, VeloFrames &obj)
            {
                out << "pcap : " << obj.pcapFileName << std::endl;
                out << "size : " << obj.frames.size() << std::endl;
                for(int i = 0; i < obj.frames.size(); i++)
                {
                    out << "\tcloud " << i << " : "<< std::endl;
                    out << "\t\tpoint size : " << obj.frames[i].cloud->points.size() << std::endl;
                    out << "\t\tminTimestamp : " << obj.frames[i].minTimestamp.count() << std::endl;
                    out << "\t\tmaxTimestamp : " << obj.frames[i].maxTimestamp.count() << std::endl;
                    out << "\t\tavgTimestamp : " << obj.frames[i].avgTimestamp.count() << std::endl;
                }
            }
    };
}

#endif