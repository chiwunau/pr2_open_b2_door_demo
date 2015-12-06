#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>

// #include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
ros::Publisher pub_inliers_;
ros::Publisher pub_coefficients_;
int min_inliers_ = 10;
int min_trial_ = 5;
int min_points_ = 100;

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
      seg.setDistanceThreshold (0.005);
      seg.setInputCloud (rest_cloud);
      seg.segment (*inliers, *coefficients);

      if (inliers->indices.size() >= min_inliers_) {
        output_inliers.push_back(inliers);
        output_coefficients.push_back(coefficients);
      }
      else{
        if (min_trial_ <= counter) {
          return;
        }
      }

    //prepare for next loop
    pcl::PointCloud<pcl::PointXYZ>::Ptr next_rest_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud (rest_cloud);
    ex.setIndices (inliers);
    ex.setNegative (true);
    ex.filter(*next_rest_cloud);

    if (next_rest_cloud->points.size() < min_points_) {
      return;
    }
    rest_cloud = next_rest_cloud;
    }
  }


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); 
  //  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  //  pcl::PCLPointCloud2 cloud_filtered;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointIndices::Ptr> all_inliers;
  std::vector<pcl::ModelCoefficients::Ptr> all_coefficients;
  

  // Convert to PCL data type
  pcl::fromROSMsg(*cloud_msg, *cloud);

  //Extract cloud clusters
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.1);
  ec.setMinClusterSize (20);
  ec.setMaxClusterSize (10000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
  
  pcl::ExtractIndices<pcl::PointXYZ> ex;
  ex.setInputCloud(cloud);
  ex.setNegative(false);

  //For every cluster calcuate line segment
  for (size_t i = 0; i < cluster_indices.size(); i++){
    pcl::PointIndices::Ptr indices (new pcl::PointIndices ());
    *indices = cluster_indices[i];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ex.setIndices (indices);
    ex.filter (*cluster_cloud);
    std::vector<pcl::PointIndices::Ptr> inliers;  
    std::vector<pcl::ModelCoefficients::Ptr> coefficients;
    applyRecursiveRANSAC(cluster_cloud, inliers, coefficients);
    for (size_t i = 0; i < inliers.size(); i++){
      all_inliers.push_back(inliers[i]);
    }
    for (size_t i = 0; i < coefficients.size(); i++){
      all_coefficients.push_back(coefficients[i]);
    }
  }

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  // *output_cloud = *cloud;
    
    // std::vector<pcl::ModelCoefficients::Ptr> coefficients;
    // std::vector<pcl::PointIndices::Ptr> inliers;
    
    // pcl::SACSegmentation<pcl::PointXYZ> seg;
    // seg.setOptimizeCoefficients (true);
    // seg.setModelType (pcl::SACMODEL_LINE);
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setDistanceThreshold (0.005);
    // seg.setInputCloud (cloud->makeShared ());
    // seg.segment (*inliers, *coefficients);
    
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud (cloud);
    // extract.setIndices (inliers);
    // extract.setNegative (false);
    // extract.filter(*cloud_p);


  // ros_indices_output.header = cloud_msg->header;
  // ros_coefficients_output = cloud_msg->header;
  
  // ros_indices_output.cluster_indices = pcl_conversions::convertToROSPointIndices(inliners, cloud_msg->header);
  // ros_coefficients_output.coefficients = pcl_conversions::convertToROSModelCoefficients(coefficients, cloud_msg->header);
  // pub_inliers_.publish(ros_indices_output);
  // pub_coefficients_.publish(ros_coefficients_output);

  // perform the actual filtering
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloudPtr);
  // sor.setLeafSize (0.1, 0.1, 0.1);
  // sor.filter (cloud_filtered);

  //  sensor_msgs::PointCloud2 output;
  //pcl::toROSMsg(*cloud_p, output);
  // pcl_conversions::moveFromPCL(cloud_filtered, output);

  // // Publish the data
  //  pub.publish (output);
}



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  // pub_inliers_ = nh.advertise<jsk_recognition_msgs::ClusterPointIndices>("/line_segmentation/output_indices", 1);
  // pub_coefficients_ = nh.advertise<jsk_recognition_msgs::ModelCoefficientsArray>("/line_segmentation/output_coefficnets", 1);

  // Spin
  ros::spin ();
}
