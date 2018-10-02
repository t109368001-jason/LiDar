#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

#include <iostream>
#include <future>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include "../include/function.h"

namespace myClass
{
#pragma region backgroundSegmentation

	template<typename PointT>
	class backgroundSegmentation
	{
		private:
			bool isSet;
			typename pcl::PointCloud<PointT>::Ptr background;
			pcl::octree::OctreePointCloudChangeDetector<PointT> *octree;
		public:
			void setBackground(const typename pcl::PointCloud<PointT>::Ptr &background, const double &resolution = 1)
			{
				this->background = background;
				
				this->octree = new pcl::octree::OctreePointCloudChangeDetector<PointT>(resolution);
				this->octree->setInputCloud(this->background);
				this->octree->addPointsFromInputCloud();
				this->octree->switchBuffers();

				this->isSet = true;
			}

			template<typename RandomIt, typename RandomIt2>
			RandomIt2 computePart(const int &division_num, const RandomIt2 &points, const RandomIt &beg, const RandomIt &end)
			{
				auto len = end - beg;

				if (len < division_num)
				{
					RandomIt2 out;
					for(auto it = beg; it != end; ++it)
					{
						if (static_cast<int> ((*it)->getSize ()) >= 0)
						{
							out.push_back(points[(*it)->getPointIndex()]);
						}
					}
					return out;
				}

				auto mid = beg + len/2;
				auto handle = std::async(std::launch::async, &backgroundSegmentation::computePart<RandomIt, RandomIt2>, this, division_num, points, beg, mid);
				auto out(computePart<RandomIt, RandomIt2>(division_num, points, mid, end));
				auto out1(handle.get());
				
				std::copy(out1.begin(),  out1.end(), std::back_inserter(out));

				return out;
			}

			typename pcl::PointCloud<PointT>::Ptr compute(const typename pcl::PointCloud<PointT>::Ptr &cloud)
			{
				if(!isSet)
				{
					std::cerr << "Background is not set" << endl;
					return cloud;
				}

				typename pcl::PointCloud<PointT>::Ptr temp(new typename pcl::PointCloud<PointT>);
				pcl::octree::OctreePointCloudChangeDetector<PointT> tree = *this->octree;
				std::vector<int> newPointIdxVector;
				
				tree.setInputCloud(cloud);
				tree.addPointsFromInputCloud();
				tree.getPointIndicesFromNewVoxels(newPointIdxVector);
				
				for(auto it = newPointIdxVector.begin(); it != newPointIdxVector.end(); ++it)
				{
					temp->points.push_back(cloud->points[*it]);
				}

				temp->width = (int) temp->points.size();
				temp->height = 1;
				
				return temp;
			}
			
	};

#pragma endregion backgroundSegmentation
	
	template<typename PointT = pcl::PointXYZ>
	class objectSegmentation
	{
			string camera_O;
			
			double camera_Width;
			double camera_Height;
			double camera_Depth;

			double segmentation_Up_Bound;
			double segmentation_Down_Bound;
			double segmentation_Left_Bound;
			double segmentation_Right_Bound;
			double segmentation_Scale_Fix;

			bool cameraParameterIsSet;
			bool BoundIsSet;

			bool keep_Inside;

			PointT camera_pos;
			PointT camera_view;
			PointT camera_focal;

		public:
			objectSegmentation()
			{
				this->BoundIsSet = false;
				this->cameraParameterIsSet = false;
			}

			objectSegmentation(const string &camera_O, const double &camera_Width, const double &camera_Height, const double &camera_FOV, double depth_Camera_FOV = 0)
			{
				setCameraParameter(camera_O, camera_Width, camera_Height, camera_FOV, depth_Camera_FOV);
			}

			bool setCameraParameter(const string &camera_O, const double &camera_Width, const double &camera_Height, const double &camera_FOV, double depth_Camera_FOV = 0)
			{
				this->camera_O = camera_O;
				this->camera_Width = camera_Width;
				this->camera_Height = camera_Height;
				this->camera_Depth = (camera_Width/2.0) / tan(camera_FOV/2.0);

				if(depth_Camera_FOV != 0)
				{
					this->segmentation_Scale_Fix = (2 * this->camera_Depth * tan(depth_Camera_FOV / 2.0)) / camera_Width;
				}
				else
				{
					this->segmentation_Scale_Fix = 1.0;
				}

				this->cameraParameterIsSet = true;
				this->BoundIsSet = false;				//須重新計算邊界
				return true;
			}

			//x and y is center
			bool setBound(const double &object_X, const double &object_Y, const double &object_Width, const double &object_Height)
			{
				if(!this->cameraParameterIsSet)
				{
					std::cerr << "Camera parameter is not set" << endl;
					return false;
				}
				double object_X_Temp = object_X;
				double object_Y_Temp = object_Y;

				if(this->camera_O == "UL")
				{
					object_X_Temp -= this->camera_Width / 2.0;
					object_Y_Temp -= this->camera_Height / 2.0;
				}

				this->segmentation_Left_Bound = (object_X_Temp - (object_Width/2.0)) * this->segmentation_Scale_Fix;
				this->segmentation_Right_Bound = (object_X_Temp + (object_Width/2.0)) * this->segmentation_Scale_Fix;
				this->segmentation_Up_Bound = (object_Y_Temp - (object_Height/2.0)) * this->segmentation_Scale_Fix;
				this->segmentation_Down_Bound = (object_Y_Temp + (object_Height/2.0)) * this->segmentation_Scale_Fix;
				
				this->BoundIsSet = true;
				return true;
			}

			void printAngle()
			{
				std::cerr << "segmentation_Left_Bound: " << this->segmentation_Left_Bound << std::endl;
				std::cerr << "segmentation_Right_Bound: " << this->segmentation_Right_Bound << std::endl;
				std::cerr << "segmentation_Up_Bound: " << this->segmentation_Up_Bound << std::endl;
				std::cerr << "segmentation_Down_Bound: " << this->segmentation_Down_Bound << std::endl;
			}

			typename pcl::PointCloud<PointT>::Ptr division(const typename pcl::PointCloud<PointT>::Ptr &input, const bool &keep_Inside = true)
			{
				if(!BoundIsSet)
				{
					std::cerr << "\nBound parameter is not set\n";
					return input;
				}

				typename pcl::PointCloud<PointT>::Ptr cloud(new typename pcl::PointCloud<PointT>);
				this->keep_Inside = keep_Inside;
				
				int division_num = std::ceil(input->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
				
				cloud->points = divisionPart(division_num, input->points.begin(), input->points.end());

				cloud->width = (int) cloud->points.size();
				cloud->height = 1;
				return cloud;
			}

			typename pcl::PointCloud<PointT>::Ptr division(const pcl::visualization::Camera camera, const typename pcl::PointCloud<PointT>::Ptr &input, const bool &keep_Inside = true)
			{
				if(!BoundIsSet)
				{
					std::cerr << "\nBound parameter is not set\n";
					return input;
				}

				typename pcl::PointCloud<PointT>::Ptr cloud(new typename pcl::PointCloud<PointT>);
				this->keep_Inside = keep_Inside;

				this->camera_pos.x = camera.pos[0];
				this->camera_pos.y = camera.pos[1];
				this->camera_pos.z = camera.pos[2];
				this->camera_view.x = camera.view[0];
				this->camera_view.y = camera.view[1];
				this->camera_view.z = camera.view[2];
				this->camera_focal.x = camera.focal[0];
				this->camera_focal.y = camera.focal[1];
				this->camera_focal.z = camera.focal[2];

				int division_num = std::ceil(input->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
				
				cloud->points = divisionPart(division_num, input->points.begin(), input->points.end());

				cloud->width = (int) cloud->points.size();
				cloud->height = 1;
				return cloud;
			}

		private:
			bool pointIsInside(const PointT &point)
			{
				if(std::fabs(point.x*point.y*point.z) == 0.0){ return false; }

				double X_Pojection = (point.x * this->camera_Depth) / point.z;
				double Y_Pojection = (point.y * this->camera_Depth) / point.z;

				if(X_Pojection < this->segmentation_Left_Bound) { return false; }
				if(X_Pojection > this->segmentation_Right_Bound) { return false; }
				if(Y_Pojection < this->segmentation_Up_Bound) { return false; }
				if(Y_Pojection > this->segmentation_Down_Bound) { return false; }

				return true;
			}

			bool pointIsInside2(const PointT &point)
			{
				PointT point_;
				if(std::fabs(point.x*point.y*point.z) == 0.0){ return false; }

				PointT camera_view_direction;
				camera_view_direction.x = this->camera_focal.x - this->camera_pos.x;
				camera_view_direction.y = this->camera_focal.y - this->camera_pos.y;
				camera_view_direction.z = this->camera_focal.z - this->camera_pos.z;

				point_.x = point.x - this->camera_pos.x;
				point_.y = point.y - this->camera_pos.y;
				point_.z = point.z - this->camera_pos.z;

				double rotation_theta_y = -(myFunction::getPhi(camera_view_direction.x, camera_view_direction.z) - M_PI_2);

				myFunction::rotateY(point_, rotation_theta_y);
				myFunction::rotateY(camera_view, rotation_theta_y);
				myFunction::rotateY(camera_view_direction, rotation_theta_y);

				double rotation_theta_x = -(myFunction::getTheta(camera_view_direction.x, camera_view_direction.z, -camera_view_direction.y) - M_PI_2);

				myFunction::rotateX(point_, rotation_theta_x);
				myFunction::rotateX(camera_view, rotation_theta_x);
				myFunction::rotateX(camera_view_direction, rotation_theta_x);
	
				double rotation_theta_z = -(myFunction::getTheta(camera_view.x, camera_view.z, -camera_view.y));

				myFunction::rotateZ(point_, rotation_theta_x);
				myFunction::rotateZ(camera_view, rotation_theta_x);
				myFunction::rotateZ(camera_view_direction, rotation_theta_x);

				double X_Pojection = (point_.x * this->camera_Depth) / point_.z;
				double Y_Pojection = (point_.y * this->camera_Depth) / point_.z;

				if(X_Pojection < this->segmentation_Left_Bound) { return false; }
				if(X_Pojection > this->segmentation_Right_Bound) { return false; }
				if(Y_Pojection < this->segmentation_Up_Bound) { return false; }
				if(Y_Pojection > this->segmentation_Down_Bound) { return false; }

				return true;
			}

			template<typename RandomIt>
			std::vector<PointT, Eigen::aligned_allocator<PointT>> divisionPart(const int &division_num, const RandomIt &beg, const RandomIt &end)
			{
				auto len = end - beg;

				if (len < division_num)
				{
					std::vector<PointT, Eigen::aligned_allocator<PointT>> out;
					for(auto it = beg; it != end; ++it)
					{
						if((!this->keep_Inside) ^ this->pointIsInside((*it)))
						{
							out.push_back(*it);
						}
					}
					return out;
				}

				auto mid = beg + len/2;
				auto handle = std::async(std::launch::async, &objectSegmentation::divisionPart<RandomIt>, this, division_num, beg, mid);
				auto out(divisionPart<RandomIt>(division_num, mid, end));
				auto out1(handle.get());
				
				std::copy(out1.begin(),  out1.end(), std::back_inserter(out));

				return out;
			}
	};
}

#endif