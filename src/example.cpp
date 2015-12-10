#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <time.h>

// #include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
ros::Publisher marker_arr_pub;
ros::Publisher pub_inliers_;
ros::Publisher pub_coefficients_;
boost::mutex mutex_;
float ransac_dist_thres_ = 0.01; //5cm
int ransac_min_inliers_ = 50;
int ransac_min_trial_ = 3;
int ransac_model_min_points_ = 30; // 2* ransac_min_inliers__
float cluster_tolerance_ = 0.15; //30cm
int cluster_min_size_ = 30;
int base_scan_total_size = 1040;

  void applyRecursiveRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, std::vector<pcl::PointIndices::Ptr>& output_inliers, std::vector<pcl::ModelCoefficients::Ptr>& output_coefficients){
    pcl::PointCloud<pcl::PointXYZ>::Ptr rest_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *rest_cloud = *input;
    int counter = 0;
    while (true) {
      ++counter;
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_LINE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (ransac_dist_thres_);
      seg.setInputCloud (rest_cloud);
      seg.segment (*inliers, *coefficients);

      //prepare for next loop
      pcl::PointCloud<pcl::PointXYZ>::Ptr next_rest_cloud (new pcl::PointCloud<pcl::PointXYZ>);

      if (inliers->indices.size() >= ransac_min_inliers_) {
        output_inliers.push_back(inliers);
        output_coefficients.push_back(coefficients);
        pcl::ExtractIndices<pcl::PointXYZ> ex;
        ex.setInputCloud (rest_cloud);
        ex.setIndices (inliers);
        ex.setNegative (true);
        ex.setKeepOrganized(true);
        ex.filter(*next_rest_cloud);
      }
      else{
                next_rest_cloud = rest_cloud;
        if (ransac_min_trial_ <= counter) {
          return;
        }
        
      } 
 

    if (next_rest_cloud->points.size() < ransac_model_min_points_) {
      return;
    }
    rest_cloud = next_rest_cloud;
    }
}

  Eigen::Vector3d* calculate_inliner_polars(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,const pcl::PointIndices::Ptr& inliers,const pcl::ModelCoefficients::Ptr& coefficients)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_relay (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> ex;
  ex.setInputCloud (cloud);
  ex.setIndices (inliers);
  ex.setNegative (false);
  ex.setKeepOrganized(true);
  ex.filter(*cloud_relay);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_LINE);
  proj.setInputCloud (cloud_relay);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  Eigen::Vector3d* res (new Eigen::Vector3d[2]);
  size_t indices_size = inliers->indices.size();
  res[0] = Eigen::Vector3d (cloud_projected->points[inliers->indices[0]].x, cloud_projected->points[inliers->indices[0]].y, cloud_projected->points[inliers->indices[0]].z);
  res[1] = Eigen::Vector3d (cloud_projected->points[inliers->indices[indices_size - 1]].x, cloud_projected->points[inliers->indices[indices_size -1]].y, cloud_projected->points[inliers->indices[indices_size - 1]].z);

  return res;
}

visualization_msgs::Marker::Ptr make_line_marker(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& inliers, const pcl::ModelCoefficients::Ptr& coefficients,int count){

  Eigen::Vector3d* polars= calculate_inliner_polars(cloud, inliers, coefficients);
  geometry_msgs::Point p0;
  geometry_msgs::Point p1;
  p0.x = polars[0](0); p0.y = polars[0](1); p0.z = polars[0](2);
  p1.x = polars[1](0); p1.y = polars[1](1); p1.z = polars[1](2);

  visualization_msgs::Marker::Ptr marker (new visualization_msgs::Marker);
  marker->header = pcl_conversions::fromPCL(cloud->header);
  marker->ns = "line_segment";
  marker->id = count;
  marker->type = visualization_msgs::Marker::LINE_LIST;

  marker->points.push_back(p0);
  marker->points.push_back(p1);
  marker->scale.x = 0.05;
  float red = ((float) rand() / (RAND_MAX));
  float green =  ((float) rand() / (RAND_MAX));
  float blue =  ((float) rand() / (RAND_MAX));

  marker->color.a = 1.0;
  marker->color.r = red;
  marker->color.g = green;
  marker->color.b = blue;

  return marker;
}

  
  





  // Eigen::Vector3d* res (new Eigen::Vector3d[2]);
  // Eigen::Vector3d point_on_line (*coefficients[0], *coefficients[1], *coefficients[2]);
  
  


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  boost::mutex::scoped_lock lock(mutex_);

  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); 
  //  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  //  pcl::PCLPointCloud2 cloud_filtered;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointIndices::Ptr> all_inliers;
  std::vector<pcl::ModelCoefficients::Ptr> all_coefficients;
  

  // Convert to PCL data type
  pcl::fromROSMsg(*cloud_msg, *cloud);
  
  std::cout<<cloud->points.size()<<std::endl;
  //Extract cloud clusters
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_); //100mm
  ec.setMinClusterSize (cluster_min_size_);
  //  ec.setMaxClusterSize (10000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
  
  pcl::ExtractIndices<pcl::PointXYZ> ex;
  ex.setInputCloud(cloud);
  ex.setKeepOrganized(true);
  ex.setNegative(false);

  //For every cluster calculate line segment
  for (size_t i = 0; i < cluster_indices.size(); i++){
    pcl::PointIndices::Ptr indices (new pcl::PointIndices ());
    *indices = cluster_indices[i];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ex.setIndices (indices);
    ex.filter (*cluster_cloud);
    std::vector<pcl::PointIndices::Ptr> inliers;  
    std::vector<pcl::ModelCoefficients::Ptr> coefficients;
    applyRecursiveRANSAC(cluster_cloud, inliers, coefficients);
    for (size_t j = 0; j < inliers.size(); j++){
      all_inliers.push_back(inliers[j]);
    }
    for (size_t k = 0; k < coefficients.size(); k++){
      all_coefficients.push_back(coefficients[k]);
    }
  }

  //make colorized cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  colored_cloud->header = cloud->header;
  colored_cloud->points.resize(cloud->points.size());
  for (size_t i = 0; i < cloud->points.size(); i++){
    colored_cloud->points[i].x = cloud->points[i].x;
    colored_cloud->points[i].y = cloud->points[i].y;
    colored_cloud->points[i].z = cloud->points[i].z;
  }

  for (size_t i = 0; i < all_inliers.size(); i++){
    uint red = rand() % 256;
    uint green = rand() % 256;
    uint blue = rand() % 256;
    
    for(size_t j = 0; j < all_inliers[i]->indices.size(); j++){
      colored_cloud->points[all_inliers[i]->indices[j]].r = red;
      colored_cloud->points[all_inliers[i]->indices[j]].g = green;
      colored_cloud->points[all_inliers[i]->indices[j]].b = blue;
    }
  }

  //make line marker message
  visualization_msgs::MarkerArray marker_array;
  for(size_t i = 0; i < all_inliers.size(); i++)
    {
      visualization_msgs::Marker::Ptr marker;
      marker = make_line_marker(cloud, all_inliers[i], all_coefficients[i], i);
      marker_array.markers.push_back(*marker);
    }

      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*colored_cloud, output);

  // Publish the data
      pub.publish (output);
      marker_arr_pub.publish (marker_array);
}

    
int
main (int argc, char** argv)
{

  srand((unsigned int)time(NULL));
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  marker_arr_pub = nh.advertise<visualization_msgs::MarkerArray> ("markers_output", 1);
  // pub_inliers_ = nh.advertise<jsk_recognition_msgs::ClusterPointIndices>("/line_segmentation/output_indices", 1);
  // pub_coefficients_ = nh.advertise<jsk_recognition_msgs::ModelCoefficientsArray>("/line_segmentation/output_coefficnets", 1);

  // Spin
  ros::Rate loop_rate(5);
  while(ros::ok()){
    ros::spinOnce ();
    loop_rate.sleep();
  }
}
