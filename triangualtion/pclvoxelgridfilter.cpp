#include "pclvoxelgridfilter.h"

PCLVoxelGridFilter::PCLVoxelGridFilter()
{
    params[PARAM_LEAF_SIZE_X] = (double) DEFAULT_LEAF_SIZE;
    params[PARAM_LEAF_SIZE_Y] = (double) DEFAULT_LEAF_SIZE;
    params[PARAM_LEAF_SIZE_Z] = (double) DEFAULT_LEAF_SIZE;
}

void PCLVoxelGridFilter::apply(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::cout<<"Number of points before the Voxel Grid Filtering: "<<cloud->width*cloud->height<<std::endl;
    voxel_grid_filter.setInputCloud(cloud);
    voxel_grid_filter.setLeafSize((float)params[PARAM_LEAF_SIZE_X],
                                  (float)params[PARAM_LEAF_SIZE_Y], 
                                  (float)params[PARAM_LEAF_SIZE_Z]);
    voxel_grid_filter.filter(*cloud);
    std::cout<<"Number of points after the Voxel Grid Filtering: "<<cloud->width*cloud->height<<std::endl;
}

//void PCLVoxelGridFilter::setParameter(int param_code, std::string value){
//    if(isValidParam(param_code)){
//        params[param_code] = std::stof(value);
//    }else{
//        std::cerr<<"Error in the parameter name"<<std::endl;
//    }
//}

void PCLVoxelGridFilter::setParameter(int param_code, double value){
    if(isValidParam(param_code)){
        params[param_code] = value;
    }else{
        std::cerr<<"Error in the parameter name"<<std::endl;
    }
}

double PCLVoxelGridFilter::getParameter(int param_code){
    if(isValidParam(param_code))
        return (double)params[param_code];
    else
        return -1.0;
}


bool PCLVoxelGridFilter::isValidParam(int param_name){
    if(param_name == PARAM_LEAF_SIZE_X ||
       param_name == PARAM_LEAF_SIZE_Y ||
       param_name == PARAM_LEAF_SIZE_Z)
        return true;
    else
        return false;
}
