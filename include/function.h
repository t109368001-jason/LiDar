#ifndef FUNCTION_H_
#define FUNCTION_H_

#include <iostream>
#include <future>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>

using namespace std;

namespace myFunction
{
	template<typename Type>
	std::string commaFix(Type input)
	{
        std::stringstream ss;
        ss.imbue(std::locale(""));
        ss << std::fixed << input;
        return ss.str();
	}
	template<typename RandomIt1, typename RandomIt2, typename PointT>
	int loadMultiPCDPart(int division_num, RandomIt1 beg1, RandomIt1 end1, RandomIt2 beg2, RandomIt2 end2)
	{
		auto len1 = end1 - beg1;
		auto len2 = end2 - beg2;

		if(len1 < division_num)
		{
			int out = 0;
			auto it2 = beg2;
			for(auto it1 = beg1; it1 != end1; ++it1,++it2)
			{
				typename pcl::PointCloud<PointT>::Ptr temp(new typename pcl::PointCloud<PointT>);
				*it2 = temp;
				pcl::io::loadPCDFile(*it1, **it2);
				out++;
			}
			return out;
		}

		auto mid1 = beg1 + len1/2;
		auto mid2 = beg2 + len1/2;

		auto handle = std::async(std::launch::async, loadMultiPCDPart<RandomIt1, RandomIt2, PointT>, division_num, beg1, mid1, beg2, mid2);

		auto out1 = loadMultiPCDPart<RandomIt1, RandomIt2, PointT>(division_num, mid1, end1, mid2, end2);

		auto out = handle.get();

		return out + out1;
	}
	template<typename PointT>
	int loadMultiPCD(std::string filename, std::vector<boost::shared_ptr<typename pcl::PointCloud<PointT>>> &clouds)
	{
		std::ifstream fs;
		fs.open(filename);
		std::string line;
		std::vector<std::string> lines;
		while(!fs.eof())
		{
			std::getline(fs, line);

			boost::trim(line);
			if(line == "") continue;
			lines.push_back(line);
		}
		clouds.resize(lines.size());

		int division_num = std::ceil(lines.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
		int num = loadMultiPCDPart<decltype(lines.begin()), decltype(clouds.begin()), PointT>(division_num, lines.begin(), lines.end(), clouds.begin(), clouds.end());
		return num;
	}
	//取得點雲中最近的兩個點距離
	template<class T>
	double getMinDistanceBetweenPoints(T cloud)
	{
		double minDistance = (cloud->points[0].getVector3fMap()-cloud->points[1].getVector3fMap()).norm();
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

		tree->setInputCloud(cloud);
		for(int i = 0; i < cloud->points.size(); i++)
		{
			std::vector<int> result(2);
			std::vector<float> result_sqr_distance(2);
			tree->nearestKSearch(cloud->points[i],2,result,result_sqr_distance);
			minDistance = std::fmin(std::sqrt(result_sqr_distance[1]), minDistance);
		}

		return minDistance;
	}
	//取得點雲中最遠的兩個點距離
	template<class T>
	double getMaxDistanceBetweenPoints(T cloud)
	{
		double maxDistance = 0;
		
		for(int i = 0; i < cloud->points.size(); i++)
		{
			for(int j = 0; j < cloud->points.size(); j++)
			{
				if(i == j)continue;
				maxDistance = std::fmax((cloud->points[i].getVector3fMap() - cloud->points[j].getVector3fMap()).norm(), maxDistance);
			}
		}
		
		return maxDistance;
	}

	double distance(double ax, double ay, double az, double bx = 0, double by = 0, double bz = 0)
	{
		return std::sqrt((ax-bx)*(ax-bx)+(ay-by)*(ay-by)+(az-bz)*(az-bz));
	}

	template<typename PointT>
	double distance(PointT p1, PointT p2 = PointT(0,0,0))
	{
		return distance(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
	}
	template<typename PointT>
	double norm(PointT p1)
	{
		return std::sqrt(p1.x*p1.x+p1.y*p1.y+p1.z*p1.z);
	}

	//取得點雲中距離point最遠的點
	template<typename RandomIt, typename PointT>
	PointT getNearOrFarthestPointPart(int division_num, bool Nearest, PointT point, RandomIt beg, RandomIt end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
			double current;
			PointT out;
			for(auto it = beg; it != end; ++it)
			{
				double temp = distance(point.x, point.y, point.z, (*it).x, (*it).y, (*it).z);
				if((temp > current) ^ Nearest)
				{
					current = temp;
					out = *it;
				}
			}
			return out;
		}
		auto mid = beg + len/2;

		auto handle = std::async(std::launch::async, getNearOrFarthestPointPart<RandomIt, PointT>, division_num, Nearest, point, beg, mid);

		auto out = getNearOrFarthestPointPart<RandomIt, PointT>(division_num, Nearest, point, mid, end);

		auto out1 = handle.get();

		if((distance(point.x, point.y, point.z, out1.x, out1.y, out1.z) > distance(point.x, point.y, point.z, out.x, out.y, out.z)) ^ Nearest)
		{
			return out1;
		}

		return out;
	}

	template<typename PointT>
	PointT getNearOrFarthestPoint(typename pcl::PointCloud<PointT>::Ptr cloud, bool Nearest = true, PointT point = PointT(0,0,0))
	{
		int division_num = std::ceil(cloud->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
		
		return getNearOrFarthestPointPart<decltype(cloud->points.begin()), PointT>(division_num, Nearest, point, cloud->points.begin(), cloud->points.end());
	}

	template<typename RandomIt>
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> XYZ_to_XYZRGBPart(int division_num, double min_Distance, double div, RandomIt beg, RandomIt end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
			std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> out;
			for(auto it = beg; it != end; ++it)
			{
				pcl::PointXYZRGB point;
				point.x = (*it).x;
				point.y = (*it).y;
				point.z = (*it).z;
				uint8_t r;
				r = ((std::sqrt(point.x*point.x+point.y*point.y+point.z*point.z) - min_Distance) * 255.0 / div);
				uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(r) << 8 | static_cast<uint32_t>(r));
				point.rgb = *reinterpret_cast<float*>(&rgb);
				out.push_back (point);
			}
			return out;
		}
		auto mid = beg + len/2;

		auto handle = std::async(std::launch::async, XYZ_to_XYZRGBPart<RandomIt>, division_num, min_Distance, div, beg, mid);

		auto out = XYZ_to_XYZRGBPart(division_num, min_Distance, div, mid, end);

		auto out1 = handle.get();

		std::copy(out1.begin(), out1.end(), std::back_inserter(out));

		return out;
	}

	template<typename PointT>
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr XYZ_to_XYZRGB(typename pcl::PointCloud<PointT>::Ptr cloud_in)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);

		double min_Distance = distance<PointT>(getNearOrFarthestPoint<PointT>(cloud_in));
		double max_Distance = distance<PointT>(getNearOrFarthestPoint<PointT>(cloud_in, false));

		double div = min_Distance - max_Distance;
		double division_num = std::ceil(cloud_in->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
		
		cloud_out->points = XYZ_to_XYZRGBPart(division_num, min_Distance, div, cloud_in->points.begin(), cloud_in->points.end());

		cloud_out->width = (int) cloud_out->points.size();
		cloud_out->height = 1;

		return cloud_out;
	}

	pcl::PolygonMesh stl_to_mesh(std::string filename)
	{
		pcl::PolygonMesh mesh;
		pcl::io::loadPolygonFileSTL(filename, mesh);

		return mesh;
	}

	void pcd_to_poissonMesh(std::string filename, pcl::PolygonMesh &poission)
	{
		string ply_filename = filename.substr(0,filename.find_last_of('.'))+"_poission.ply";

		if (pcl::io::loadPLYFile(ply_filename, poission) == -1)
		{
			pcl::PCDReader reader;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

			reader.read (filename, *cloud);

			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
			pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud(cloud);
			n.setInputCloud(cloud);
			n.setSearchMethod(tree);
			n.setKSearch(30);
			n.compute(*normals);

			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
			pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

			pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
			tree2->setInputCloud(cloud_with_normals);

			pcl::Poisson<pcl::PointNormal> poisson;
			poisson.setDepth(8);
			poisson.setSolverDivide(8);
			poisson.setIsoDivide(8);
			poisson.setPointWeight(4.0f);
			poisson.setInputCloud(cloud_with_normals);

			poisson.reconstruct(poission);
			pcl::io::savePLYFile(ply_filename, poission);
		}
	}

	pcl::PolygonMesh pcd_to_poissonMesh(std::string filename)
	{
		string ply_filename = filename.substr(0,filename.find_last_of('.'))+"_poission.ply";
		pcl::PolygonMesh poission;
		if (pcl::io::loadPLYFile(ply_filename, poission) == -1)
		{
			pcl::PCDReader reader;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

			reader.read (filename, *cloud);

			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
			pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud(cloud);
			n.setInputCloud(cloud);
			n.setSearchMethod(tree);
			n.setKSearch(30);
			n.compute(*normals);

			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
			pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

			pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
			tree2->setInputCloud(cloud_with_normals);

			pcl::Poisson<pcl::PointNormal> poisson;
			poisson.setDepth(8);
			poisson.setSolverDivide(8);
			poisson.setIsoDivide(8);
			poisson.setPointWeight(4.0f);
			poisson.setInputCloud(cloud_with_normals);

			poisson.reconstruct(poission);
			pcl::io::savePLYFile(ply_filename, poission);
		}
		return poission;
	}
}
#endif