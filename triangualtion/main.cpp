#include <iostream>
#include <cstdlib>
#include <string>
#include "pointcloudtriangulation.h"
#include "pclvoxelgridfilter.h"
#include "pclstatisticaloutlierremoval.h"

using namespace std;

void help(int, char**);

int main(int argc,char **argv)
{
    help(argc, argv);
    
    PointCloudTriangulation *triangulation = new PointCloudTriangulation();

    triangulation->loadCloudFromFile(argv[1]);

    PCLFilter* voxel_grid_filter = new PCLVoxelGridFilter();
    PCLFilter* gaussian_noise_filter = new PCLStatisticalOutlierRemoval();

    triangulation->addFilter("voxel grid filter", voxel_grid_filter);
    triangulation->addFilter("gaussian noise filter", gaussian_noise_filter);
    //triangulation.addAlgorithm("surface smoothing", surface_smoothing);

    triangulation->applyFilter("voxel grid filter");
    triangulation->applyFilter("gaussian noise filter");

    triangulation->surfaceSmoothing();
    triangulation->reconstruct();

    triangulation->saveTriangulation(argv[2]);

    return 0;
}

void help(int argc, char **argv){
    if(argc == 3)
        return;
    cerr<<"Error, usage:"<<endl<<argv[0]<<" <pcdfile>"<<" <outfile>.vtk"<<endl;
    exit(EXIT_FAILURE);
}
