#ifndef CLASS_H_
#define CLASS_H_

#include "../include/function.h"
#include "../include/segmentation.h"

namespace myClass
{
    class YoloObject
    {
        public:
            double x;
            double y;
            double w;
            double h;
            std::string name;
    };
    template<typename PointT>
    class CustomFrame
    {
        public:
            std::chrono::milliseconds time;
            std::unordered_map<int, std::unordered_set<unsigned long long>> _framesMap;
            typename pcl::PointCloud<PointT>::Ptr cloud;
            rs2::video_frame *frameColor;
            CustomFrame()
            {
                time = std::chrono::milliseconds(0);
            }
            CustomFrame(rs2::frameset &frameset, std::chrono::milliseconds timestamp)
            {
                auto frameDepth = frameset.get_depth_frame();
                
                for (size_t i = 0; i < frameset.size(); i++) {
                    rs2::video_frame frameColor = frameset[i].as<rs2::video_frame>();

                    if (frameColor && (frameColor.get_profile().stream_type() == rs2_stream::RS2_STREAM_ANY)) {
                        if (frames_map_get_and_set(frameColor.get_profile().stream_type(), frameColor.get_frame_number())) {
                            continue;
                        }

                        rs2::colorizer color_map;
                        if (frameColor.get_profile().stream_type() == rs2_stream::RS2_STREAM_DEPTH) {
                            frameColor = color_map.process(frameColor);
                        }
                        this->frameColor = new rs2::video_frame(frameColor);
                    }
                }
                
                if (frameDepth) {
                    if (frames_map_get_and_set(rs2_stream::RS2_STREAM_ANY, frameDepth.get_frame_number())) {
                        return;
                    }
                    
                    rs2::pointcloud pc;
                    rs2::points points;
                    points = pc.calculate(frameDepth);

                    this->cloud = myFunction::points_to_pcl<PointT>(points);
                }
                this->time = timestamp;
            }
            bool frames_map_get_and_set(rs2_stream streamType, unsigned long long frameNumber)
            {
                if (_framesMap.find(streamType) == _framesMap.end()) {
                    _framesMap.emplace(streamType, std::unordered_set<unsigned long long>());
                }

                auto & set = _framesMap[streamType];
                bool result = (set.find(frameNumber) != set.end());

                if (!result) {
                    set.emplace(frameNumber);
                }

                return result;
            }
    };
}
#endif