#ifndef BOUNDARY_H_
#define BOUNDARY_H_

#include <iostream>
#include <cmath>
#include <thread>
#include <numeric>
#include <future>
#include <type_traits>
#include <pcl/console/time.h>
#include "function.h"

using namespace std;
using namespace Eigen;

namespace myClass
{
	class Boundary
	{
	public:
		Vector3f cameraAt;
		string camera_O;
		double camera_Height;
		double camera_Width;
		double camera_Vertical_FOV;
		double camera_Horizontal_FOV;
		double camera_Vertical_Depth;
		double camera_Horizontal_Depth;
		bool cameraParameterIsSet;

		double LiDar_Height_Offset;
		double LiDar_Horizontal_Offset;
		double focal_Leftength_Offset;
		bool LiDarParameterIsSet;

		double LiDar_Up_Angle;
		double LiDar_Down_Angle;
		double LiDar_Left_Angle;
		double LiDar_Right_Angle;
		bool BoundIsSet;

		bool keep_Inside;
		int division_num;

		Boundary()
		{
			cameraParameterIsSet = false;
			LiDarParameterIsSet = false;
			BoundIsSet = false;
		}

		void setAllForwardDirection()
		{
			this->LiDar_Up_Angle = M_PI_2;
			this->LiDar_Down_Angle = -M_PI_2;
			this->LiDar_Left_Angle = -M_PI_2;
			this->LiDar_Right_Angle = M_PI_2;

			this->BoundIsSet = true;
		}

		void setCameraParameter(Vector3f cameraAt, string camera_O, double camera_Height, double camera_Width, double camera_Vertical_FOV, double camera_Horizontal_FOV)
		{
			this->cameraAt = cameraAt;
			this->camera_O = camera_O;
			this->camera_Height = camera_Height;
			this->camera_Width = camera_Width;
			this->camera_Vertical_FOV = camera_Vertical_FOV;
			this->camera_Horizontal_FOV = camera_Horizontal_FOV;
			this->camera_Vertical_Depth = (camera_Height/2.0) / tan(camera_Vertical_FOV/2.0);		//計算深度, 計算用
			this->camera_Horizontal_Depth = (camera_Width/2.0) / tan(camera_Horizontal_FOV/2.0);		//計算深度, 計算用

			cameraParameterIsSet = true;
			BoundIsSet = false;				//須重新計算邊界
		}

		void setLiDarParameter(double LiDar_Height_Offset, double LiDar_Horizontal_Offset = 0, double focal_Leftength_Offset = 0)
		{
			this->LiDar_Height_Offset = LiDar_Height_Offset;
			this->LiDar_Horizontal_Offset = LiDar_Horizontal_Offset;
			this->focal_Leftength_Offset = focal_Leftength_Offset;

			LiDarParameterIsSet = true;
			BoundIsSet = false;				//須重新計算邊界
		}

		void setBound(double object_X, double object_Y, double object_Height, double object_Width)
		{
			if(!cameraParameterIsSet)
			{
				cout << "Camera parameter is not set" << endl;
			}
			if(!LiDarParameterIsSet)
			{
				cout << "LiDar parameter is not set" << endl;
			}
			double object_X_Temp = object_X;
			double object_Y_Temp = object_Y;
			
			//YOLO原點修正成OpenGL原點
			if(this->camera_O == "UR")
			{
				object_X_Temp = this->camera_Width - object_X_Temp;
			}
			else if(this->camera_O == "DR")
			{
				object_X_Temp = this->camera_Width - object_X_Temp;
				object_Y_Temp = this->camera_Height - object_Y_Temp;
			}
			else if(this->camera_O == "DL")
			{
				object_Y_Temp = this->camera_Height - object_Y_Temp;
			}
			
			//將物件原點改為中心, 右邊為+x, 上面為+y
			double object_Up = -(object_Y_Temp - this->camera_Height/2);
			double object_Down = object_Up - object_Height;
			double object_Left = object_X_Temp - this->camera_Width/2;
			double object_Right = object_Left + object_Width;
			
			//將邊界轉換為角度
			double object_Up_Angle = atan(object_Up / this->camera_Vertical_Depth);
			double object_Down_Angle = atan(object_Down / this->camera_Vertical_Depth);
			double object_Left_Angle = atan(object_Left / this->camera_Horizontal_Depth);
			double object_Right_Angle = atan(object_Right / this->camera_Horizontal_Depth);
			
			//將邊界的角度以LiDar的高低差和最佳化焦距修正, 修正後的值在等於焦距時無誤差
			this->LiDar_Up_Angle = atan((this->focal_Leftength_Offset * tan(object_Up_Angle) + this->LiDar_Height_Offset) / this->focal_Leftength_Offset);
			this->LiDar_Down_Angle = atan((this->focal_Leftength_Offset * tan(object_Down_Angle) + this->LiDar_Height_Offset) / this->focal_Leftength_Offset);
			this->LiDar_Left_Angle = object_Left_Angle;
			this->LiDar_Right_Angle = object_Right_Angle;

			BoundIsSet = true;
		}

		template<typename PointT>
		typename pcl::PointCloud<PointT>::Ptr divisionPart(typename pcl::PointCloud<PointT>::Ptr cloud)
		{
			auto beg = cloud->points.begin();
			auto end = cloud->points.end();
			auto len = end - beg;

			if (len < this->division_num)
			{
				typename pcl::PointCloud<PointT>::Ptr cloud_out(new typename pcl::PointCloud<PointT>);
				for(auto it = beg; it != end; ++it)
				{
					if((!this->keep_Inside) ^ this->pointIsInside((*it).getVector3fMap()))
					{
						cloud_out->points.push_back(*it);
					}
				}
				return cloud_out;
			}

			auto mid = beg + len/2;
			
			typename pcl::PointCloud<PointT>::Ptr cloud1(new typename pcl::PointCloud<PointT>);
			typename pcl::PointCloud<PointT>::Ptr cloud2(new typename pcl::PointCloud<PointT>);
			
			for(auto it = beg; it != mid; ++it)
			{
				cloud1->points.push_back(*it);
			}
			for(auto it = mid; it != end; ++it)
			{
				cloud2->points.push_back(*it);
			}
			
			auto handle = std::async(std::launch::async, &Boundary::divisionPart<PointT>, this, cloud1);

			typename pcl::PointCloud<PointT>::Ptr cloud_out(divisionPart<PointT>(cloud2));

			typename pcl::PointCloud<PointT>::Ptr out1(handle.get());
			/*
			for(auto it = out1->points.begin(); it != out1->points.end(); ++it)
			{
				cloud_out->points.push_back(*it);
			}
			*/
			for(int i = 0; i < out1->points.size(); i++)
			{
				cloud_out->points.push_back(out1->points[i]);
			}
			cloud_out->width = (int) cloud_out->points.size();
			cloud_out->height = 1;
			return cloud_out;
		}

		template<typename PointT>
		typename pcl::PointCloud<PointT>::Ptr division(typename pcl::PointCloud<PointT>::Ptr input, bool keep_Inside = true)
		{
			this->keep_Inside = keep_Inside;
			this->division_num = std::ceil(input->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
			if(!BoundIsSet)
			{
				std::cerr << "\nBound parameter is not set\n";
				return input;
			}
			if(((LiDar_Up_Angle - LiDar_Down_Angle) == 0)||((LiDar_Right_Angle - LiDar_Left_Angle) == 0))
			{
				std::cerr << "\nInput angle valid\n";
				return input;
			}

			return divisionPart<PointT>(input);
		}

	private:
		bool pointIsInside(Vector3f point)
		{
			Vector3f camera_to_point;
			camera_to_point = -this->cameraAt + point;		//計算相機到點之向量

			//計算camera_to_point向量之水平及垂直角度
			double point_Vertical_Angle = acos(camera_to_point.dot(Eigen::Vector3f::UnitZ()) / camera_to_point.norm() / Eigen::Vector3f::UnitZ().norm());
			double point_Horizontal_Angle = atan(camera_to_point[1] / camera_to_point[0]);

			//修正水平角度, 因 -M_PI_2 <= atan <= M_PI_2
			if (camera_to_point[0] < 0.0)
				point_Horizontal_Angle += M_PI;
			
			//修正角度, 水平右正左負, 垂直上正下負
			point_Vertical_Angle = M_PI_2 - point_Vertical_Angle;
			point_Horizontal_Angle = M_PI_2 - point_Horizontal_Angle;

			//修正LiDar水平原點誤差
			point_Horizontal_Angle += this->LiDar_Horizontal_Offset;

			//修正數值範圍 -M_PI ~ M_PI
			if(point_Horizontal_Angle > M_PI) point_Horizontal_Angle -= M_PI * 2.0;
			if(point_Horizontal_Angle < -M_PI) point_Horizontal_Angle += M_PI * 2.0;

			//檢查是否超出邊界
			if(point_Vertical_Angle > this->LiDar_Up_Angle){ return false; }
			if(point_Vertical_Angle < this->LiDar_Down_Angle){ return false; }
			if(point_Horizontal_Angle > this->LiDar_Right_Angle){ return false; }
			if(point_Horizontal_Angle < this->LiDar_Left_Angle){ return false; }

			return true;
		}
	};
}
#endif