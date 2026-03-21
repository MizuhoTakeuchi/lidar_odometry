#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

template class pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::VoxelGridCovariance<pcl::PointXYZ>;
