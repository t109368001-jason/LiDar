#ifndef BOUNDARY_H_
#define BOUNDARY_H_

#include <iostream>
#include <future>
#include <pcl/point_cloud.h>
#include "../include/function.h"

using namespace std;
using namespace Eigen;

namespace myClass
{
	template<typename PointT>
	class Boundary
	{
	public:
		PointT cameraAt;
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

		void setCameraParameter(PointT cameraAt, string camera_O, double camera_Height, double camera_Width, double camera_Vertical_FOV, double camera_Horizontal_FOV)
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

			//修正角度, 水平右0 左180 垂直上0 下180
			this->LiDar_Up_Angle = M_PI_2 - this->LiDar_Up_Angle;
			this->LiDar_Down_Angle = M_PI_2 - this->LiDar_Down_Angle;
			this->LiDar_Left_Angle = M_PI_2 - this->LiDar_Left_Angle;
			this->LiDar_Right_Angle = M_PI_2 - this->LiDar_Right_Angle;
			//修正LiDar水平原點誤差
			this->LiDar_Left_Angle -= this->LiDar_Horizontal_Offset;
			this->LiDar_Right_Angle -= this->LiDar_Horizontal_Offset;

			BoundIsSet = true;
		}

		template<typename RandomIt>
		std::vector<PointT, Eigen::aligned_allocator<PointT>> divisionPart(int division_num, RandomIt beg, RandomIt end)
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
			
			auto handle = std::async(std::launch::async, &Boundary::divisionPart<RandomIt>, this, division_num, beg, mid);

			auto out(divisionPart<RandomIt>(division_num, mid, end));

			auto out1(handle.get());
			
			std::copy(out1.begin(),  out1.end(), std::back_inserter(out));

			return out;
		}

		typename pcl::PointCloud<PointT>::Ptr division(typename pcl::PointCloud<PointT>::Ptr input, bool keep_Inside = true)
		{
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

			typename pcl::PointCloud<PointT>::Ptr cloud(new typename pcl::PointCloud<PointT>);
			this->keep_Inside = keep_Inside;
			
			int division_num = std::ceil(input->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
			
			cloud->points = divisionPart(division_num, input->points.begin(), input->points.end());

			cloud->width = (int) cloud->points.size();
			cloud->height = 1;
			return cloud;
		}

	private:
		bool pointIsInside(PointT point)
		{
			PointT camera_to_point;
			//修正角度, 水平右正左負, 垂直上正下負
			camera_to_point.x = -cameraAt.x + point.x;
			camera_to_point.y = -cameraAt.y + point.y;
			camera_to_point.z = -cameraAt.z + point.z;

			//計算camera_to_point向量之水平及垂直角度
			double point_Vertical_Angle = acos(camera_to_point.z / myFunction::norm(camera_to_point));
			double point_Horizontal_Angle = atan(camera_to_point.y / camera_to_point.x);

			//修正水平角度, 因 -M_PI_2 <= atan <= M_PI_2  >> 
			if (camera_to_point.x < 0.0)
			{
				if(camera_to_point.y < 0.0)
				{
					point_Horizontal_Angle -= M_PI;
				}
				else
				{
					point_Horizontal_Angle += M_PI;
				}
			}

			//檢查是否超出邊界
			if(point_Horizontal_Angle < this->LiDar_Right_Angle){ return false; }
			if(point_Horizontal_Angle > this->LiDar_Left_Angle){ return false; }
			if(point_Vertical_Angle < this->LiDar_Up_Angle){ return false; }
			if(point_Vertical_Angle > this->LiDar_Down_Angle){ return false; }

			return true;
		}
	};
}
#endif