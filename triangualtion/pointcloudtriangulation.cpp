#include "pointcloudtriangulation.h"

PointCloudTriangulation::PointCloudTriangulation()
{
    pcl::PointCloud<pcl::PointNormal>::Ptr tmp_cloud  (new pcl::PointCloud<pcl::PointNormal>);
    cloud = tmp_cloud;
}

PointCloudTriangulation::PointCloudTriangulation(std::string file_name)
{    
    loadCloudFromFile(file_name);
}

PointCloudTriangulation::PointCloudTriangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
    this->cloud = cloud;
}

void PointCloudTriangulation::addFilter(std::string name, PCLFilter *algorithm)
{
    algorithms[name]= algorithm;
}

void PointCloudTriangulation::surfaceSmoothing(){
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    mls.setInputCloud(cloud);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);

    mls.process(*normalCloud);
    //cloud = normalCloud;
    std::cout<<"Surface smoothing compleated"<<endl;
}

void PointCloudTriangulation::reconstruct(){
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud (normalCloud);

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (DEFAULT_SEARCH_RADIUS);

    // Set typical values for the parameters
    gp3.setMu (DEFAULT_MULTIPLIER);
    gp3.setMaximumNearestNeighbors (DEFAULT_MAX_NEAREST_NEIGHBORS);
    gp3.setMaximumSurfaceAngle(DEFAULT_MAX_SURFACE_ANGLE); // 45 degrees
    gp3.setMinimumAngle(DEFAULT_MIN_ANGLE); // 10 degrees
    gp3.setMaximumAngle(DEFAULT_MAX_ANGLE); // 120 degrees
    gp3.setNormalConsistency(true);

    // Get result
    gp3.setInputCloud (normalCloud);
    gp3.setSearchMethod (tree);
    gp3.reconstruct (triangles);
    std::cout<<"triangulation done"
}

void PointCloudTriangulation::saveTriangulation(std::string file_name){
    pcl::io::saveVTKFile(file_name.c_str(), triangles);
}

void PointCloudTriangulation::applyFilter(std::string algorithm_name)
{
    std::cout<<"Applying "<< algorithm_name << std::endl;
    algorithms[algorithm_name]->apply(this->cloud);
}

PCLFilter* PointCloudTriangulation::getFilter(std::string algorithm_name)
{
    return algorithms[algorithm_name];
}

pcl::PointCloud<pcl::PointNormal>::Ptr PointCloudTriangulation::getCloud()
{
    return cloud;
}

pcl::PolygonMesh PointCloudTriangulation::getTriangulation()
{
    return triangles;
}

void PointCloudTriangulation::loadCloudFromFile(std::string file_name)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    if((pcl::io::loadPCDFile(file_name, cloud_blob))==-1){
        PCL_ERROR ("Error reading file '$s'\n", file_name.c_str());
        exit(EXIT_FAILURE);
    }
    
    pcl::fromPCLPointCloud2(cloud_blob, *tmp_cloud);

    std::cout<<"Points readed: "<< tmp_cloud->width*tmp_cloud->height << std::endl;

    //normal estimation
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//    tree->setInputCloud(tmp_cloud);
//    n.setInputCloud(tmp_cloud);
//    n.setSearchMethod(tree);
//    n.setKSearch(20);
//    n.compute(*normals);

//    pcl::concatenateFields(*tmp_cloud, *normals, *cloud);
//    cloud = tmp_cloud + normals

//    std::cout<<"Normal estimation done"<<std::endl;
//    std::cout<<"Points after normal estimation: "<< cloud->width*cloud->height<<std::endl;
}
