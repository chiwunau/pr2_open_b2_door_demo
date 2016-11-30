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
#include <pcl/common/angles.h>

// #include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
ros::Publisher marker_arr_pub;
ros::Publisher refined_marker_arr_pub;
ros::Publisher pub_inliers_;
ros::Publisher pub_coefficients_;
boost::mutex mutex_;
float ransac_dist_thres_ = 0.03; //5cm
// float ransac_dist_thres_ = 0.05; //5cm
int ransac_min_inliers_ = 20;
int ransac_min_trial_ = 5;
// int ransac_min_trial_ = 5;
// int ransac_model_min_points_ = 30; // 2* ransac_min_inliers__
int ransac_model_min_points_ = 30; // 2* ransac_min_inliers__
// float cluster_tolerance_ = 0.15; //30cm
float cluster_tolerance_ = 0.1; //30cm
int cluster_min_size_ = 30;
int base_scan_total_size = 1040;


void applyRecursiveRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, std::vector<pcl::PointIndices::Ptr>& output_inliers, std::vector<pcl::ModelCoefficients::Ptr>& output_coefficients){
  pcl::PointCloud<pcl::PointXYZ>::Ptr rest_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndices::Ptr nonnan_indices (new pcl::PointIndices);
  *rest_cloud = *input;
  int counter = 0;
  while (true) {
    ++counter;
    nonnan_indices->indices.clear();
    for (size_t i = 0; i < rest_cloud->points.size(); i++) {
      pcl::PointXYZ p = rest_cloud->points[i];
      if (!isnan(p.x) && !isnan(p.y) && !isnan(p.z)) {
        nonnan_indices->indices.push_back(i);
      }
    }
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (ransac_dist_thres_);
    seg.setInputCloud (rest_cloud);
    seg.setIndices (nonnan_indices);
    seg.segment (*inliers, *coefficients);
    // std::cout<<"max_iteration:"<<seg.getMaxIterations()<<std::endl;

    // std::cout<<"inliers_in_ransac:"<<inliers->indices.size()<<std::endl;
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
      // std::cout<<counter<<std::endl;
      // std::cout<<"cloud size:"<<next_rest_cloud->points.size()<<std::endl;
    }
    else{
      next_rest_cloud = rest_cloud;
      if (ransac_min_trial_ <= counter) {
        return;
      }
    }

    nonnan_indices->indices.clear();
    for (size_t i = 0; i < next_rest_cloud->points.size(); i++) {
      pcl::PointXYZ p = next_rest_cloud->points[i];
      if (!isnan(p.x) && !isnan(p.y) && !isnan(p.z)) {
        nonnan_indices->indices.push_back(i);
      }
    }
    // std::cout<<"left_cloud_size:"<<nonnan_indices->indices.size()<<std::endl;;
    if (nonnan_indices->indices.size() < ransac_model_min_points_) {
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
  // std::cout<<"cloud_size"<<cloud_relay->points.size()<<std::endl;
  // std::cout<<"proj_cloud_size"<<cloud_projected->points.size()<<std::endl;
  int indices_size = inliers->indices.size();
  // std::cout<<"start"<<inliers->indices[0]<<std::endl;
  // std::cout<<"end"<<inliers->indices[indices_size - 1]<<std::endl;
  

  //   std::cout<<"-----------------------------------"<<std::endl;
  // for (size_t i = 0; i < inliers->indices.size(); i++){
  //   std::cout<<inliers->indices[i]<<std::endl;
  // }
  // std::cout<<"-----------------------------------"<<std::endl;
    
  // std::vector<int>::iterator iter = std::min_element(inliers->indices.begin(), inliers->indices[indices_size -1]);
  // size_t index = std::distance(inliers->indices.begin(), iter);
  // std::cout<<"start_a"<<index<<std::endl;
  // res[0] = Eigen::Vector3d (cloud_projected->points[index].x, cloud_projected->points[index].y, cloud_projected->points[index].z);
  // iter = std::max_element(inliers->indices.begin(), inliers->indices.end());
  // index = std::distance(inliers->indices.begin(), iter);
  // std::cout<<"end_a"<<index<<std::endl;
  // res[1] = Eigen::Vector3d (cloud_projected->points[index].x, cloud_projected->points[index].y, cloud_projected->points[index].z);

  res[0] = Eigen::Vector3d (cloud->points[inliers->indices[0]].x, cloud->points[inliers->indices[0]].y, cloud->points[inliers->indices[0]].z);
  res[1] = Eigen::Vector3d (cloud->points[inliers->indices[indices_size - 1]].x, cloud->points[inliers->indices[indices_size -1]].y, cloud->points[inliers->indices[indices_size - 1]].z);

  // res[0] = Eigen::Vector3d (cloud_projected->points[inliers->indices[0]].x, cloud_projected->points[inliers->indices[0]].y, cloud_projected->points[inliers->indices[0]].z);
  // res[1] = Eigen::Vector3d (cloud_projected->points[inliers->indices[indices_size - 1]].x, cloud_projected->points[inliers->indices[indices_size -1]].y, cloud_projected->points[inliers->indices[indices_size - 1]].z);

  return res;
}

visualization_msgs::Marker::Ptr make_line_marker(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& inliers, const pcl::ModelCoefficients::Ptr& coefficients,int count, bool refined){

  Eigen::Vector3d* polars= calculate_inliner_polars(cloud, inliers, coefficients);
  geometry_msgs::Point p0;
  geometry_msgs::Point p1;
  p0.x = polars[0](0); p0.y = polars[0](1); p0.z = polars[0](2);
  p1.x = polars[1](0); p1.y = polars[1](1); p1.z = polars[1](2);

  visualization_msgs::Marker::Ptr marker (new visualization_msgs::Marker);
  marker->header = pcl_conversions::fromPCL(cloud->header);
  if (refined){
    marker->ns = "refined_line_segment";}
  else{
    marker->ns = "line_segment";}
  marker->id = count;
  marker->type = visualization_msgs::Marker::LINE_LIST;

  marker->points.push_back(p0);
  marker->points.push_back(p1);
  marker->scale.x = 0.05;
  float red = ((float) rand() / (RAND_MAX));
  float green =  ((float) rand() / (RAND_MAX));
  float blue =  ((float) rand() / (RAND_MAX));


  if (refined){
    marker->color.a = 1.0;
    marker->color.r = red;
    marker->color.g = green;
    marker->color.b = blue;
  }
  else{
    marker->color.a = 1.0;
    marker->color.r = red;
    marker->color.g = green;
    marker->color.b = blue;
  }
  

  return marker;
}


  // Eigen::Vector3d* res (new Eigen::Vector3d[2]);
  // Eigen::Vector3d point_on_line (*coefficients[0], *coefficients[1], *coefficients[2]);

typedef struct Group {
  std::vector<int> members;
}Group;

void refineModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, std::vector<pcl::PointIndices::Ptr>& input_inliers, std::vector<pcl::ModelCoefficients::Ptr>& input_coefficients, std::vector<pcl::PointIndices::Ptr>& output_inliers, std::vector<pcl::ModelCoefficients::Ptr>& output_coefficients){

  std::vector< Group > groups;
  // std::vector<int>* members (new std::vector<int>);


  std::vector<int> each_begin_index;
  std::vector<int> sorted_begin_index;
  std::vector<int> sorted_inliers_index;
  for (size_t i = 0; i < input_inliers.size(); i++){
    each_begin_index.push_back(input_inliers[i]->indices[0]);
    std::cout<<"("<<input_inliers[i]->indices[0]<<","<<input_inliers[i]->indices[input_inliers[i]->indices.size() -1]<<") ";
  }
  std::cout<<std::endl;

  std::copy(each_begin_index.begin(), each_begin_index.end(), std::back_inserter(sorted_begin_index));
  std::sort(sorted_begin_index.begin(), sorted_begin_index.end());
  int pos;
  for (size_t i = 0; i < sorted_begin_index.size(); i++){
    pos = find(each_begin_index.begin(), each_begin_index.end(), sorted_begin_index[i]) - each_begin_index.begin();
    sorted_inliers_index.push_back(pos);
  }

  for (size_t i = 0; i < sorted_inliers_index.size(); i++){
    each_begin_index.push_back(input_inliers[i]->indices[0]);
    std::cout<<sorted_inliers_index[i]<<" ";
  }


  Group* g;  
  for (size_t idx = 0; idx < input_inliers.size(); idx++){
    int i = sorted_inliers_index[idx];
    if (groups.size() == 0){
      g = new Group;
      // g->members = new std::vector<int>;
      g->members.push_back(i);
      groups.push_back(*g);}
    else{
      bool found_group = false;
      for (size_t j = 0; j < groups.size(); j++){
        // std::cout<<"member_size"<<ms.size()<<std::endl;
        for (size_t k = 0; k < groups[j].members.size(); k++){
          // if (k != 0 && k == ms.size()){
          //   break;}
          Eigen::Vector3f vm;
          Eigen::Vector3f vx;
          // vm << input_coefficients[group[j].members[k]]->values[3], input_coefficients[group[j].members[k]]->values[4], input_coefficients[group[j].members[k]]->values[5];
          // vx << input_coefficients[i]->values[3], input_coefficients[i]->values[4], input_coefficients[i]->values[5];
          vm << input_coefficients[groups[j].members[k]]->values[3], input_coefficients[groups[j].members[k]]->values[4], 0.0; //z = 0
          vx << input_coefficients[i]->values[3], input_coefficients[i]->values[4], 0.0; //z = 0;

          double rad =  vm.dot(vx) / sqrt(vm.squaredNorm() * vx.squaredNorm());
          // if (rad < -1.0) rad = -1.0;
          // if (rad > 1.0) rad = 1.0;
          float ang = pcl::rad2deg(float(acos(rad)));
          // std::cout<<ang<<std::endl;

          if ( std::abs(ang) < 10 || std::abs(ang) > 170){
            Eigen::Vector3d* m_polars = calculate_inliner_polars(input_cloud, input_inliers[groups[j].members[k]], input_coefficients[groups[j].members[k]]);
            Eigen::Vector3d* nl_polars = calculate_inliner_polars(input_cloud, input_inliers[i], input_coefficients[i]);

            double min_dist = -1.0;
            Eigen::Vector3d diff;
            for (size_t mp = 0; mp < 2; mp++){
              for (size_t nlp = 0; nlp < 2; nlp++){
                std::cout<<"mp:"<<mp<<"  nlp:"<< nlp<<std::endl;
                // (m_polars[mp] - nl_polars[nlp]).norm();
                diff = m_polars[mp] - nl_polars[nlp];
                if (diff.norm() < min_dist || min_dist < 0){
                  min_dist = diff.norm();
                  std::cout<<diff[0]<<" "<< diff[1]<<" "<<diff[2]<<std::endl;
                }
              }
            }
            std::cout<<"min_distance: "<<i<<" "<<j<<" "<<k<<" "<<min_dist<<std::endl;
            if  (min_dist <= 0.05){
              groups[j].members.push_back(i);
              found_group = true;
              break;}
          }
        }
        if (found_group == true){
          break;}
      }
      std::cout<<"---------------"<<std::endl;
      if (found_group == false){
        g = new Group;
        std::cout<<"NEW_GROUP"<<std::endl;
        // g->members = new std::vector<int>;
        g->members.push_back(i);
        groups.push_back(*g);
      }
    }
  }
  std::cout<<"refined_group_size"<<groups.size()<<std::endl;
  for (size_t i=0; i < groups.size(); i++){
    pcl::PointIndices::Ptr group_inliers (new pcl::PointIndices);
    for (size_t j=0; j < groups[i].members.size(); j++){
      for (size_t k=0; k < input_inliers[groups[i].members.at(j)]->indices.size(); k++){
        group_inliers->indices.push_back(input_inliers[groups[i].members.at(j)]->indices[k]);
          }
    }

    // std::cout<<"-----------------------------------"<<std::endl;
    // for (size_t p = 0; p < group_inliers->indices.size(); p++){
    //   std::cout<<group_inliers->indices[p]<<std::endl;
    // }
    // std::cout<<"-----------------------------------"<<std::endl;
    

      // std::cout<<"group_inliers_size:"<<group_inliers->indices.size()<<std::endl;
      pcl::PointIndices::Ptr sac_inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr sac_coefficients (new pcl::ModelCoefficients);
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_LINE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.03);
      seg.setInputCloud (input_cloud);
      seg.setIndices (group_inliers);
      seg.segment (*sac_inliers, *sac_coefficients);

      // std::cout<<"refined_inliers_size:"<<sac_inliers->indices.size()<<std::endl;
      if (sac_inliers->indices.size() > 0){
        output_inliers.push_back(sac_inliers);
        output_coefficients.push_back(sac_coefficients);}
  }
}



void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  std::cout<<"cloud callback"<<std::endl;
  boost::mutex::scoped_lock lock(mutex_);

  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); 
  //  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  //  pcl::PCLPointCloud2 cloud_filtered;

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointIndices::Ptr> all_inliers;
  std::vector<pcl::ModelCoefficients::Ptr> all_coefficients;
  std::vector<pcl::PointIndices::Ptr> refined_all_inliers;
  std::vector<pcl::ModelCoefficients::Ptr> refined_all_coefficients;

  // Convert to PCL data type
  pcl::fromROSMsg(*cloud_msg, *cloud);
  pcl::PointIndices::Ptr nonnan_indices (new pcl::PointIndices);
  for (size_t i = 0; i < cloud->points.size(); i++) {
    pcl::PointXYZ p = cloud->points[i];
    if (!isnan(p.x) && !isnan(p.y) && !isnan(p.z)) {
      nonnan_indices->indices.push_back(i);
    }
  }

  //Extract cloud clusters
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  std::vector<pcl::PointIndices> pre_cluster_indices;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_); //100mm
  ec.setMinClusterSize (cluster_min_size_);
  ec.setMaxClusterSize (1000000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.setIndices (nonnan_indices);
  ec.extract (pre_cluster_indices);

  //For every cluster calculate line segment
  for (size_t i = 0; i < pre_cluster_indices.size(); i++){
    // std::cout<<pre_cluster_indices[i].indices.size()<<std::endl;
    size_t indices_size = pre_cluster_indices[i].indices.size();
      // std::cout<<pre_cluster_indices[i].indices.size()<<std::endl;

    int split_step = 80;
    for (size_t j = 0; j <= indices_size /split_step; j++){
      size_t tail;
      if (indices_size / split_step < 1){
        tail = indices_size;}
      else if (j == indices_size / split_step){
        break;}
      else if (j == indices_size /split_step - 1){
        tail = indices_size;}
      else{
        tail = (j + 1) * split_step;
      }
      // std::cout<<tail<<std::endl;
      pcl::PointIndices splited_indices;
      for (size_t k = j * split_step; k < tail; k++){
        // std::cout<<pre_cluster_indices[i].indices[k]<<std::endl;
        splited_indices.indices.push_back(pre_cluster_indices[i].indices[k]);
      }
      cluster_indices.push_back(splited_indices);
    }
  }
  // std::cout<<pre_cluster_indices.size()<<std::endl;
  // std::cout<<cluster_indices.size()<<std::endl;

  for (size_t i = 0; i < cluster_indices.size(); i++){
    pcl::PointIndices::Ptr indices (new pcl::PointIndices ());
    *indices = cluster_indices[i];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud(cloud);
    ex.setKeepOrganized(true);
    ex.setNegative(false);
    ex.setIndices (indices);
    ex.filter (*cluster_cloud);
    // std::cout<<cluster_cloud->points.size()<<std::endl;
    std::vector<pcl::PointIndices::Ptr> inliers;
    std::vector<pcl::ModelCoefficients::Ptr> coefficients;
    applyRecursiveRANSAC(cluster_cloud, inliers, coefficients);
    // std::cout<<"inliers_size:"<<inliers.size()<<std::endl;
    for (size_t j = 0; j < inliers.size(); j++){
      all_inliers.push_back(inliers[j]);
    }
    for (size_t k = 0; k < coefficients.size(); k++){
      all_coefficients.push_back(coefficients[k]);
    }
  }

  refineModel(cloud, all_inliers, all_coefficients, refined_all_inliers, refined_all_coefficients);

  // std::cout<<"all_inliers_size:"<<all_inliers.size()<<std::endl;
  //make colorized cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  colored_cloud->header = cloud->header;
  colored_cloud->points.resize(cloud->points.size());
  for (size_t i = 0; i < cloud->points.size(); i++){
    colored_cloud->points[i].x = cloud->points[i].x;
    colored_cloud->points[i].y = cloud->points[i].y;
    colored_cloud->points[i].z = cloud->points[i].z;
  }

  for (size_t i = 0; i < cluster_indices.size(); i++){
    uint red = rand() % 256;
    uint green = rand() % 256;
    uint blue = rand() % 256;
    
    for(size_t j = 0; j < cluster_indices[i].indices.size(); j++){
      colored_cloud->points[cluster_indices[i].indices[j]].r = red;
      colored_cloud->points[cluster_indices[i].indices[j]].g = green;
      colored_cloud->points[cluster_indices[i].indices[j]].b = blue;
    }
  }

  std::cout<<"refined_inliers_size"<<refined_all_inliers.size()<<std::endl;
  // for (size_t i = 0; i < refined_all_inliers.size(); i++){
  //   uint red = rand() % 256;
  //   uint green = rand() % 256;
  //   uint blue = rand() % 256;
    
  //   for(size_t j = 0; j < refined_all_inliers[i]->indices.size(); j++){
  //     colored_cloud->points[refined_all_inliers[i]->indices[j]].r = red;
  //     colored_cloud->points[refined_all_inliers[i]->indices[j]].g = green;
  //     colored_cloud->points[refined_all_inliers[i]->indices[j]].b = blue;
  //   }
  // }


  //make line marker message
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::MarkerArray refined_marker_array;
  // std::cout<<"number of line:"<<all_inliers.size()<<std::endl;
  for(size_t i = 0; i < all_inliers.size(); i++)
    {
      visualization_msgs::Marker::Ptr marker;
      marker = make_line_marker(cloud, all_inliers[i], all_coefficients[i], i, false);
      marker_array.markers.push_back(*marker);
    }

  for(size_t i = 0; i < refined_all_inliers.size(); i++)
    {
      visualization_msgs::Marker::Ptr marker;
      marker = make_line_marker(cloud, refined_all_inliers[i], refined_all_coefficients[i], i, true);
      refined_marker_array.markers.push_back(*marker);
    }

      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*colored_cloud, output);

  // Publish the data
      pub.publish (output);
      marker_arr_pub.publish (marker_array);
      refined_marker_arr_pub.publish (refined_marker_array);

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
  pub = nh.advertise<sensor_msgs::PointCloud2> ("colored_cloud", 1);
  marker_arr_pub = nh.advertise<visualization_msgs::MarkerArray> ("markers_output", 1);
  refined_marker_arr_pub = nh.advertise<visualization_msgs::MarkerArray> ("refined_markers_output", 1);

  // pub_inliers_ = nh.advertise<jsk_recognition_msgs::ClusterPointIndices>("/line_segmentation/output_indices", 1);
  // pub_coefficients_ = nh.advertise<jsk_recognition_msgs::ModelCoefficientsArray>("/line_segmentation/output_coefficnets", 1);

  // Spin
  ros::Rate loop_rate(10);
  while(ros::ok()){
    ros::spinOnce ();
    loop_rate.sleep();
  }
}
