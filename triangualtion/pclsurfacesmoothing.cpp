#include "pclsurfacesmoothing.h"

PCLSurfaceSmoothing::PCLSurfaceSmoothing()
{
    parameters[PARAM_SEARCH_RADIUS] = DEFAULT_SEARCH_RADIUS;
    parameters[PARAM_POLYNOMIAL_FIT] = (double)DEFAULT_POLYNOMIAL_FIT;
    parameters[PARAM_SEARCH_RADIUS] = (double) DEFAULT_COMPUTE_NORMALS;
}

void PCLSurfaceSmoothing::apply(pcl::PointCloud<pcl::PointNormal>::Ptr cloud){

    mls.setComputeNormals ((bool)parameters[PARAM_COMPUTE_NORMALS]);
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);

    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialFit ((bool)parameters[PARAM_POLYNOMIAL_FIT]);
    mls.setSearchMethod (tree);
    mls.setSearchRadius ((float)parameters[PARAM_SEARCH_RADIUS]);
    //pcl::PointCloud<pcl::PointNormal> c;
    // Reconstruct
    //mls.process(c);

}

void PCLSurfaceSmoothing::setParameter(int code, double value){
    if(code >= 0 && code <= 2)
        parameters[code] = value;
}

double PCLSurfaceSmoothing::getParameter(int code){
    if(code >= 0 && code <= 2)
        return parameters[code];
    else
        return -1.0;
}

