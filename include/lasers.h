#ifndef LASERS_H_
#define LASERS_H_
#include "../3rdparty/VelodyneCapture/VelodyneCapture.h"

namespace myLasers {
    class Line {
        public:
            double azimuth;
            std::vector<velodyne::Laser> lasers;

            Line(velodyne::Laser laser) : azimuth(laser.azimuth) {
                lasers.push_back(laser);
            }

            void add(velodyne::Laser laser) {
                lasers.push_back(laser);
            }

            void remove_duplicate() {
                for(int i = 0; i < lasers.size(); i++) {
                    for(int j = 0; j < i; j++) {
                        if((lasers[i].distance == lasers[j].distance)&&(lasers[i].azimuth == lasers[j].azimuth)&&(lasers[i].vertical == lasers[j].vertical)) {
                            lasers.erase(lasers.begin() + i);
                            i--;
                            break;
                        }
                    }
                }
            }

            void sort() {
                std::sort(lasers.begin(), lasers.end(), [] (auto a, auto b) { return a.vertical < b.vertical; });
            }
    };

    class Lines {
        public:
            std::vector<Line> lines;

            void add(velodyne::Laser laser) {
                bool create = true;
                for(int i = 0; i < lines.size(); i++) {
                    if(lines[i].azimuth == laser.azimuth) {
                        lines[i].add(laser);
                        create = false;
                    }
                }
                if(create) {
                    Line line(laser);
                    lines.push_back(line);
                }
            }
            void remove_duplicate() {
                for(int i = 0; i < lines.size(); i++) {
                    lines[i].remove_duplicate();
                }
            }

            void sort() {
                std::sort(lines.begin(), lines.end(), [] (auto a, auto b) { return a.azimuth < b.azimuth; });
                for(int i = 0; i < lines.size(); i++) {
                    lines[i].sort();
                }
            }

            size_t size() {
                size_t num = 0;
                for( const myLasers::Line& line : lines){
                    num += line.lasers.size();
                }
                return num;
            }

            void linear_interpolation(int num) {
                for( myLasers::Line& line : lines){
                    std::vector<velodyne::Laser> lasers_new;
                    for( int i = 1; i < line.lasers.size(); i++ ){
                        for(int j = 0; j < num; j++) {
                            velodyne::Laser laser;
                            laser.azimuth = line.azimuth;
                            laser.distance = (line.lasers[i-1].distance - line.lasers[i].distance) * (j+1) / (num+1) + line.lasers[i].distance;
                            laser.vertical = (line.lasers[i-1].vertical - line.lasers[i].vertical) * (j+1) / (num+1) + line.lasers[i].vertical;
                            laser.intensity = (line.lasers[i-1].intensity - line.lasers[i].intensity) * (j+1) / (num+1) + line.lasers[i].intensity;
                            laser.time = (line.lasers[i-1].time - line.lasers[i].time) * (j+1) / (num+1) + line.lasers[i].time;
                            lasers_new.push_back(laser);
                        }
                    }
                    /*for(int i = 0; i < lasers_new.size(); i++) {
                        line.lasers.push_back(lasers_new[i]);
                    }*/
		            std::copy(lasers_new.begin(), lasers_new.end(), std::back_inserter(line.lasers));
                }
            }
    };
}

#endif