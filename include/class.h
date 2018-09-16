#ifndef CLASS_H_
#define CLASS_H_

#include "../include/function.h"

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
            std::vector<YoloObject> yoloObjects;
            typename pcl::PointCloud<PointT>::Ptr cloud;
            rs2::frameset frameset;
            std::chrono::milliseconds time;
            CustomFrame()
            {
                time = std::chrono::milliseconds(0);
            }
            CustomFrame(rs2::frameset frameset, std::chrono::milliseconds startTime)
            {
                rs2::pointcloud pc;
                rs2::points points;
                rs2::colorizer color_map;
                auto vf = frameset.get_color_frame();
                
                auto stream = frameset.get_profile().stream_type();
                
                if (vf.is<rs2::depth_frame>()) vf = color_map.process(frameset);

                this->frameset = frameset;

                auto depth = frameset.get_depth_frame();
                points = pc.calculate(depth);
                this->cloud = myFunction::points_to_pcl<PointT>(points);

                std::unordered_map<int, std::unordered_set<unsigned long long>> _framesMap;
                if (_framesMap.find(rs2_stream::RS2_STREAM_ANY) == _framesMap.end()) {
                    _framesMap.emplace(rs2_stream::RS2_STREAM_ANY, std::unordered_set<unsigned long long>());
                }

                auto & set = _framesMap[rs2_stream::RS2_STREAM_ANY];
                bool result = (set.find(depth.get_frame_number()) != set.end());

                if (!result) {
                    set.emplace(depth.get_frame_number());
                }
            }
            bool getYoloObjects()
            {
                
            }
    };
}
#endif