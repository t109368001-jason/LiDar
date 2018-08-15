
#include <pcl/io/pcd_io.h>
#include <pcl/surface/poisson.h>

void pcd_to_poissonMesh(std::string filename, pcl::PolygonMesh &poission);
pcl::PolygonMesh pcd_to_poissonMesh(std::string filename);

pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_to_pointCloud(std::string filename)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile(filename, cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);

	return cloud;
} 

pcl::PolygonMesh stl_to_mesh(std::string filename)
{
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(filename, mesh);

	return mesh;
}

void pcd_to_poissonMesh(std::string filename, pcl::PolygonMesh &poission)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile(filename, cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);

	pcl::CentroidPoint<pcl::PointXYZ> centroid;
	pcl::PointXYZ cent;
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		centroid.add(cloud->points[i]);
	}
	centroid.get(cent);
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x -= cent.x;
		cloud->points[i].y -= cent.y;
		cloud->points[i].z -= cent.z;
	}

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
}
pcl::PolygonMesh pcd_to_poissonMesh(std::string filename)
{
    pcl::PolygonMesh poission;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile(filename, cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);

	pcl::CentroidPoint<pcl::PointXYZ> centroid;
	pcl::PointXYZ cent;
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		centroid.add(cloud->points[i]);
	}
	centroid.get(cent);
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x -= cent.x;
		cloud->points[i].y -= cent.y;
		cloud->points[i].z -= cent.z;
	}

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
    return poission;
}
