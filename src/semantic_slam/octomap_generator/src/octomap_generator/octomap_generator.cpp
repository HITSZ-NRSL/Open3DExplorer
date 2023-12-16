#include <octomap_generator/octomap_generator.h>
#include <semantics_point_type/semantics_point_type.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <cmath>
#include <sstream>
#include <cstring> // For std::memcpy

template<class CLOUD, class OCTREE>
OctomapGenerator<CLOUD, OCTREE>::OctomapGenerator(): octomap_(0.05), max_range_(1.), raycast_range_(1.){}

template<class CLOUD, class OCTREE>
OctomapGenerator<CLOUD, OCTREE>::~OctomapGenerator(){}

template<class CLOUD, class OCTREE>
void OctomapGenerator<CLOUD, OCTREE>::setUseSemanticColor(bool use)
{
  octomap_.setUseSemanticColor(use);
}

template<>
void OctomapGenerator<PCLColor, ColorOcTree>::setUseSemanticColor(bool use){}

template<class CLOUD, class OCTREE>
bool OctomapGenerator<CLOUD, OCTREE>::isUseSemanticColor()
{
  return octomap_.isUseSemanticColor();
}

template<>
bool OctomapGenerator<PCLColor, ColorOcTree>::isUseSemanticColor(){return false;}

float perception_range = 8.5;
template<class CLOUD, class OCTREE>
void OctomapGenerator<CLOUD, OCTREE>::insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensorToWorld)
{
  // Voxel filter to down sample the point cloud
  // Create the filtering object
  pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2 ());
  // Perform voxel filter
  float voxel_flt_size = octomap_.getResolution();
  voxel_flt_size = voxel_flt_size * 0.5;
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (voxel_flt_size, voxel_flt_size, voxel_flt_size);
  sor.filter (*cloud_filtered);
  // std::cout << "filtered cloud size:" << cloud_filtered->width * cloud_filtered->height << std::endl;
  // Convert to PCL pointcloud
  CLOUD pcl_cloud;
  pcl::fromPCLPointCloud2(*cloud_filtered, pcl_cloud);
  //std::cout << "Voxel filtered cloud size: "<< pcl_cloud.size() << std::endl;
  // Transform coordinate
  // pcl::transformPointCloud(pcl_cloud, pcl_cloud, sensorToWorld);
  
  octomap::point3d origin(static_cast<float>(sensorToWorld(0,3)),static_cast<float>(sensorToWorld(1,3)),static_cast<float>(sensorToWorld(2,3)));
  // std::cout << "origin : " << origin.x() << " " << origin.y() << " " << origin.z() << " " << std::endl;

  // 将点云转换到世界坐标系下
  for (size_t i = 0; i < pcl_cloud.points.size (); ++i)
  {
    if(pcl_cloud[i].z <= 0){
      std::cout << "pcl_cloud " << i << " : " << pcl_cloud[i].z << std::endl;
      pcl_cloud[i].z = perception_range;    // 将相机坐标系下深度为0的点换成10
    }

    Eigen::Matrix<float, 3, 1> pt (pcl_cloud[i].x, pcl_cloud[i].y, pcl_cloud[i].z);
    pcl_cloud[i].x = static_cast<float> (sensorToWorld (0, 0) * pt.coeffRef (0) + sensorToWorld (0, 1) * pt.coeffRef (1) + sensorToWorld (0, 2) * pt.coeffRef (2) + sensorToWorld (0, 3));
    pcl_cloud[i].y = static_cast<float> (sensorToWorld (1, 0) * pt.coeffRef (0) + sensorToWorld (1, 1) * pt.coeffRef (1) + sensorToWorld (1, 2) * pt.coeffRef (2) + sensorToWorld (1, 3));
    pcl_cloud[i].z = static_cast<float> (sensorToWorld (2, 0) * pt.coeffRef (0) + sensorToWorld (2, 1) * pt.coeffRef (1) + sensorToWorld (2, 2) * pt.coeffRef (2) + sensorToWorld (2, 3));

    // 大于等于perception_range的点统一投影到perception_range处，这一部分点统一被视为free并进行光线投射;小于perception_range的点则根据实际情况进行光线投射
    // if(sqrt((pcl_cloud[i].x - origin.x())*(pcl_cloud[i].x - origin.x()) + (pcl_cloud[i].y - origin.y())*(pcl_cloud[i].y - origin.y()) + (pcl_cloud[i].z - origin.z())*(pcl_cloud[i].z - origin.z())) >= perception_range){
    //   // std::cout << "before pcl_cloud " << i << " : " << pcl_cloud[i].x << " " << pcl_cloud[i].y << " " << pcl_cloud[i].z << " " << std::endl;
    //   Eigen::Vector3f dir_vec;
    //   dir_vec << (pcl_cloud[i].x - origin.x()), (pcl_cloud[i].y - origin.y()), (pcl_cloud[i].z - origin.z());
    //   Eigen::Vector3f dir_vec_offset = dir_vec.normalized()*perception_range;
    //   pcl_cloud[i].x = origin.x() + dir_vec_offset.x();    // 
    //   pcl_cloud[i].y = origin.y() + dir_vec_offset.y();    // 
    //   pcl_cloud[i].z = origin.z() + dir_vec_offset.z();    // 
    //   // std::cout << "afetr pcl_cloud " << i << " : " << pcl_cloud[i].x << " " << pcl_cloud[i].y << " " << pcl_cloud[i].z << " " << std::endl;
    // }

    // std::cout << sqrt((pcl_cloud[i].x - origin.x())*(pcl_cloud[i].x - origin.x()) + (pcl_cloud[i].y - origin.y())*(pcl_cloud[i].y - origin.y()) + (pcl_cloud[i].z - origin.z())*(pcl_cloud[i].z - origin.z())) << std::endl;
  }

  //tf::Vector3 originTf = sensorToWorldTf.getOrigin();
  //octomap::point3d origin(originTf[0], originTf[1], originTf[2]);
  octomap::Pointcloud raycast_cloud; // Point cloud to be inserted with ray casting
  int endpoint_count = 0; // total number of endpoints inserted
  for(typename CLOUD::const_iterator it = pcl_cloud.begin(); it != pcl_cloud.end(); ++it)
  {
    // Check if the point is invalid
    if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
    {
      float dist = sqrt((it->x - origin.x())*(it->x - origin.x()) + (it->y - origin.y())*(it->y - origin.y()) + (it->z - origin.z())*(it->z - origin.z()));
      // Check if the point is in max_range
      if(dist < max_range_)
      {
        // 消除天空导致的错误深度点云
        if((it->r >= 0 && it->r <= 10) && (it->g >= 225 && it->g <= 235) && (it->b >= 225 && it->b <= 235)){
          std::cout << "semantic color: " << int(it->r) << " " << int(it->g) << " " << int(it->b) << " " << std::endl;  
          octomap_.updateNode(it->x, it->y, it->z, false, false); // 超过raycast_range_小于max_range_的部分，即超过并投影到perception_range上的那一部分点。设为free
          // octomap_.updateInnerOccupancy();
          continue;
        }

        // Check if the point is in the ray casting range, 造成空洞  && dist >= 5 for terrain
        if(dist <= raycast_range_ && dist >= 0) // Add to a point cloud and do ray casting later all together
        {
          raycast_cloud.push_back(it->x, it->y, it->z);
        }
        else// otherwise update the occupancy of node and transfer the point to the raycast range
        {
          octomap::point3d direction = (octomap::point3d(it->x, it->y, it->z) - origin).normalized();
          octomap::point3d new_end = origin + direction * (raycast_range_ + octomap_.getResolution()*2);
          raycast_cloud.push_back(new_end);

          // updateNode只更新node的状态，而不进行光线投射
          // 考虑空闲栅格的时候free the end point 
          // octomap_.updateNode(it->x, it->y, it->z, false, false); // 超过raycast_range_小于max_range_的部分，即超过并投影到perception_range上的那一部分点。设为free
          octomap_.updateNode(it->x, it->y, it->z, true, false);    // !!! occupied the end point
        }
        
        // octomap_.updateNode(it->x, it->y, it->z, true, false);    // !!! occupied the end point

        endpoint_count++;
      }
      else{   // 可消除较远距离->这一部分在点云中以及截断了
        if(it->z > origin.z()){
          octomap::point3d direction = (octomap::point3d(it->x, it->y, it->z) - origin).normalized();
          octomap::point3d new_end = origin + direction * (raycast_range_ + octomap_.getResolution()*2);
          raycast_cloud.push_back(new_end);

          octomap_.updateNode(it->x, it->y, it->z, false, false); // 超过raycast_range_小于max_range_的部分，即超过并投影到perception_range上的那一部分点。设为free
          endpoint_count++;
        }        
      }
    }
  }
  // Do ray casting for points in raycast_range_
  if(raycast_cloud.size() > 0)
    octomap_.insertPointCloud(raycast_cloud, origin, raycast_range_, false, true);  // use lazy_eval, run updateInnerOccupancy() when done, use discretize to downsample cloud
  
  // octomap_.updateInnerOccupancy();

  //** Update colors and semantics, differs between templates
  updateColorAndSemantics(&pcl_cloud);
  
  // updates inner node occupancy and colors
  if(endpoint_count > 0)
    octomap_.updateInnerOccupancy();
}

template<>
void OctomapGenerator<PCLColor, ColorOcTree>::updateColorAndSemantics(PCLColor* pcl_cloud)
{
  for(PCLColor::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++)
  {
    if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
    {
      octomap_.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
      // octomap_.averageNodeColor(it->x, it->y, it->z, 0xff, 0x00, 0x00);
    }
  }
  octomap::ColorOcTreeNode* node = octomap_.search(pcl_cloud->begin()->x, pcl_cloud->begin()->y, pcl_cloud->begin()->z);
  //std::cout << "Example octree node: " << std::endl;
  //std::cout << "Color: " << node->getColor()<< std::endl;
}

template<>
void OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>::updateColorAndSemantics(PCLSemanticsMax* pcl_cloud)
{
  for(PCLSemanticsMax::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++)
  {
    if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
    {
      // octomap_.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
      // octomap_.averageNodeColor(it->x, it->y, it->z, 0xff, 0x00, 0x00);
        // Get semantics
        octomap::SemanticsMax sem;
        uint32_t rgb;

        
        // use color as semantic color in filed 4:5
        sem.semantic_color.r = it->r;
        sem.semantic_color.g = it->g;
        sem.semantic_color.b = it->b;        
        
        // //// semantic color in filed 5:6
        // std::memcpy(&rgb, &it->semantic_color, sizeof(uint32_t));
        // sem.semantic_color.r = (rgb >> 16) & 0x0000ff;
        // sem.semantic_color.g = (rgb >> 8)  & 0x0000ff;
        // sem.semantic_color.b = (rgb)       & 0x0000ff;
        
        // std::cout << "Semantics color.rgb : " << unsigned(sem.semantic_color.r)  << " " << unsigned(sem.semantic_color.g) << " " << unsigned(sem.semantic_color.b )<< std::endl;
        
        sem.confidence = it->confidence;
        octomap_.updateNodeSemantics(it->x, it->y, it->z, sem);
        // std::cout << "Semantics color: " << sem.semantic_color << std::endl;
    }
  }
  SemanticsOcTreeNodeMax* node = octomap_.search(pcl_cloud->begin()->x, pcl_cloud->begin()->y, pcl_cloud->begin()->z);
    // std::cout << "Example octree node: " << std::endl;
    // std::cout << "Color: " << node->getColor()<< std::endl;
    // std::cout << "Semantics: " << node->getSemantics() << std::endl;
}

template<>
void OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>::updateColorAndSemantics(PCLSemanticsBayesian* pcl_cloud)
{
  for(PCLSemanticsBayesian::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++)
  {
    if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
    {
      // octomap_.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
        // Get semantics
        octomap::SemanticsBayesian sem;
        for(int i = 0; i < 3; i++)
        {
          uint32_t rgb;
          std::memcpy(&rgb, &it->data_sem[i], sizeof(uint32_t));
          sem.data[i].color.r = (rgb >> 16) & 0x0000ff;
          sem.data[i].color.g = (rgb >> 8)  & 0x0000ff;
          sem.data[i].color.b = (rgb)       & 0x0000ff;
          sem.data[i].confidence = it->data_conf[i];
        }
        octomap_.updateNodeSemantics(it->x, it->y, it->z, sem);

    }
  }
    SemanticsOcTreeNodeBayesian* node = octomap_.search(pcl_cloud->begin()->x, pcl_cloud->begin()->y, pcl_cloud->begin()->z);
    //std::cout << "Example octree node: " << std::endl;
    //std::cout << "Color: " << node->getColor()<< std::endl;
    //std::cout << "Semantics: " << node->getSemantics() << std::endl;
}

template<class CLOUD, class OCTREE>
bool OctomapGenerator<CLOUD, OCTREE>::save(const char* filename) const
{
  std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);
  if (outfile.is_open()){
    std::cout << "Writing octomap to " << filename << std::endl;
    octomap_.write(outfile);
    outfile.close();
    std::cout << "Color tree written " << filename << std::endl;
    return true;
  }
  else {
    std::cout << "Could not open " << filename  << " for writing" << std::endl;
    return false;
  }
}


//Explicit template instantiation
template class OctomapGenerator<PCLColor, ColorOcTree>;
template class OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>;
template class OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>;
