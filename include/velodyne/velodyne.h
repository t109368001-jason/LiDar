#ifndef LASER_TYPES_H_
#define LASER_TYPES_H_

#define HAVE_BOOST
#define HAVE_PCAP
#define HAVE_FAST_PCAP

#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include "line_frame.h"

using namespace std;

namespace velodyne {

    class VLP16 : public VLP16Capture {
        public:
            int64_t frameNumber;

            VLP16() : VLP16Capture(), frameNumber(-1) {};

            const bool open( const string& filename ) {
                bool result = VelodyneCapture::open(filename);
                result &= VLP16Capture::isOpen();
                frameNumber = 0;

                return result;
            }

            void moveToNext() {
                if( this->mutex.try_lock() ){
                    if( !this->queue.empty() ){
                        this->queue.pop();
                    }
                    this->mutex.unlock();
                }
                while(this->isRun())
                {
                    if(this->queue.size() > 0) break;
                }
                frameNumber++;
            }

            void operator >> (std::vector<Laser>& lasers) {
                if( this->mutex.try_lock() ){
                    if( !this->queue.empty() ){
                        lasers = this->queue.front();
                    }
                    this->mutex.unlock();
                }
            }

            void operator >> (LineFrame& lineFrame) {
                vector<Laser> lasers;

                (*this) >> lasers;
                lineFrame << lasers;
            }

            void saveAsPNG(const std::string & filename) {
                LineFrame lineFrame;
                (*this) >> lineFrame;

                std::cout << frameNumber << std::endl;
                std::cout << lineFrame;
            }

            void operator >> (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud) {
                pcl::PointXYZ point;
                std::vector<Laser> lasers;

                (*this) >> lasers;

                cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

                for( const velodyne::Laser& laser : lasers ){
                    const double distance = static_cast<double>( laser.distance );
                    const double azimuth  = laser.azimuth  * CV_PI / 180.0;
                    const double vertical = laser.vertical * CV_PI / 180.0;
                
                    point.x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
                    point.y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
                    point.z = static_cast<float>( ( distance * std::sin( vertical ) ) );
                
                    if( point.x == 0.0f && point.y == 0.0f && point.z == 0.0f ){
                        continue;
                    }

                    cloud->points.push_back(point);
                }
                cloud->width = static_cast<uint32_t>(cloud->points.size());
                cloud->height = 1;
            }
    };
}

#endif