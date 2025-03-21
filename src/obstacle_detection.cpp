#include <obstacle_detection.hpp>
#include <pluginlib/class_list_macros.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/common/time.h>
// #include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/extract_indices.h>
// #include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

namespace obstacle_detection
{

Detector::~Detector()
{
    stop_thread_.store(true);
    conf_cond_.notify_one();
    queue_cond_.notify_one();
    if (process_thread_.joinable())
    {
        process_thread_.join();
    }
}

void Detector::onInit()
{
    ROS_DEBUG("Detector constructor");

    ros::NodeHandle &nh = getMTNodeHandle();
    ros::NodeHandle &private_nh = getMTPrivateNodeHandle();

    // Set up dynamic reconfigure
    server_ = std::make_shared<dynamic_reconfigure::Server<livox_ros_driver2::DetectorConfig>>(private_nh);
    dynamic_reconfigure::Server<livox_ros_driver2::DetectorConfig>::CallbackType f;
    f = boost::bind(&Detector::configCallback, this, _1, _2);
    server_->setCallback(f);

    // Subscribers
    sub_point_cloud_ = nh.subscribe("input_point_cloud", 1, &Detector::pointCloudCallback, this);

    // Publishers
    test_pub_ = nh.advertise<sensor_msgs::PointCloud2>("ground", 1);
    test_pub2_ = nh.advertise<sensor_msgs::PointCloud2>("above_ground", 1);
    test_pub3_ = nh.advertise<sensor_msgs::PointCloud2>("obstacles", 1);
    test_pub4_ = nh.advertise<sensor_msgs::PointCloud2>("obstacles_rgb", 1);
    pub_jsk_bboxes_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("obstacles_bbox", 1);

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("livox_frame","/rviz_visual_markers"));
    visual_tools2_.reset(new rviz_visual_tools::RvizVisualTools("livox_frame","/rviz_visual_markers2"));

    process_thread_ = std::thread(&Detector::process, this);
};

void Detector::configCallback(livox_ros_driver2::DetectorConfig &config, uint32_t level)
{
    std::unique_lock<std::mutex> lock(conf_mutex_);    
    enable_ = config.enable;
    num_scans_ = config.num_scans;
    max_range_ = config.max_range;
    voxel_size_ = config.voxel_size;
    min_pts_voxel_ = config.min_pts_voxel;
    max_iterations_ = config.max_iterations;
    distance_threshold_ = config.distance_threshold;
    eps_angle_ = config.eps_angle;
    min_plane_points_ = config.min_plane_points;
    max_height_ = config.max_height;
    model_variance_threshold_ = config.model_variance_threshold;
    max_obstacle_height_ = config.max_obstacle_height;
    cluster_tolerance_ = config.cluster_tolerance;
    min_cluster_size_ = config.min_cluster_size;
    max_cluster_size_ = config.max_cluster_size;

    if (config.use_imu != use_imu_)
    {
        use_imu_ = config.use_imu;
        if (!use_imu_)
        {
            sub_imu_.shutdown();
            std::lock_guard<std::mutex> lock(imu_mutex_);
            perpendicular_to_ground_ = Eigen::Vector3f{0.0, 0.0, 1.0};
        }
        else
        {
            sub_imu_ = getMTNodeHandle().subscribe("input_imu", 1, &Detector::imuCallback, this);
        }

        if (enable_)
        {
            lock.unlock();
            conf_cond_.notify_one();
        }    
    }    
}

void Detector::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    ROS_DEBUG("imuCallback");
    // Get the vector that is perpendicular to the ground
    Eigen::Quaternionf q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        perpendicular_to_ground_ = q.toRotationMatrix().col(2);
    }
}

void Detector::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    ROS_DEBUG("pointCloudCallback");

    if (need_pcl_.load())
    {
        ROS_DEBUG("Getting point cloud");
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            scan_queue_.push(cloud);
        }
        queue_cond_.notify_one();
    }
    else
    {
        ROS_DEBUG("Ignoring point cloud");
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Detector::accumulate_scans(const uint num_scans, const double max_range)
{
    ROS_DEBUG("accumulate_scans");

    // Pointer for final point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud{new pcl::PointCloud<pcl::PointXYZI>};

    // Empty queue
    std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> empty;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        std::swap(scan_queue_, empty);
    }

    need_pcl_.store(true);
    
    // Append point clouds as they come in
    uint i = 0;
    while (i < num_scans && ros::ok())
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cond_.wait(lock, [this] { return !scan_queue_.empty() || !ros::ok() || stop_thread_.load(); });

        if (!ros::ok() || stop_thread_.load())
        {
            return final_cloud;
        }

        if (i == 0)
        {
            final_cloud->header = scan_queue_.front()->header;
        }

        // Filter out points that are too far away
        for (const pcl::PointXYZI &point : scan_queue_.front()->points)
        {
            if (point.getVector3fMap().norm() < max_range)
            {
                final_cloud->push_back(point);
            }
        }

        scan_queue_.pop();
        i++;
    }

    need_pcl_.store(false);

    return final_cloud;
}

void Detector::process()
{
    ROS_DEBUG("process");

    bool have_transform{false};

    {        
        const std::string resolved_topic{getNodeHandle().resolveName("input_point_cloud", true)};
        sensor_msgs::PointCloud2ConstPtr init_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(resolved_topic);
        if (init_msg)
        {
            // lookup the transform between the frame "aristos_base_footprint" and the frame of the point cloud
            tf2_ros::Buffer tf_buffer;
            tf2_ros::TransformListener tf_listener(tf_buffer);
            geometry_msgs::TransformStamped transformStamped;
            try
            {
                transformStamped = tf_buffer.lookupTransform("aristos_base_footprint", init_msg->header.frame_id, ros::Time(0), ros::Duration(5.0));
                base_to_lidar_ = Eigen::Isometry3f(tf2::transformToEigen(transformStamped).cast<float>());
                have_transform = true;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
            }
        }
    }    

    pcl::StopWatch watch;

    // PCL Visualizer
    // pcl::visualization::PCLVisualizer::Ptr viewer;
    // viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // viewer->setBackgroundColor(0, 0, 0);

    while (ros::ok()) 
    {
        // visual_tools_->deleteAllMarkers();
        // visual_tools2_->deleteAllMarkers();
        std::unique_lock<std::mutex> lock(conf_mutex_);
        conf_cond_.wait(lock, [this] { return enable_ || !ros::ok() || stop_thread_.load(); });

        if (!ros::ok() || stop_thread_.load())
        {
            return;
        }

        // Get a local copy of the dynamic reconfigure parameters
        const uint num_scans{num_scans_};
        const double max_range{max_range_};
        const double voxel_size{voxel_size_};
        const uint min_pts_voxel{min_pts_voxel_};

        const uint max_iterations{max_iterations_};
        const double distance_threshold{distance_threshold_};
        const double eps_angle{eps_angle_};
        const uint min_plane_points{min_plane_points_};
        const double max_height{max_height_};
        const double model_variance_threshold{model_variance_threshold_};

        const double max_obstacle_height{max_obstacle_height_};
        const double cluster_tolerance{cluster_tolerance_};
        const uint min_cluster_size{min_cluster_size_};
        const uint max_cluster_size{max_cluster_size_};

        lock.unlock();
        
        // Accumulate scans
        watch.reset();
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud{accumulate_scans(num_scans, max_range)};
        std::cout << "Accumulate: " << watch.getTime() << std::endl;

        // Voxel downsampling
        watch.reset();
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(voxel_size, voxel_size, voxel_size);
        vg.setMinimumPointsNumberPerVoxel(min_pts_voxel);
        vg.filter(*cloud);
        std::cout << "Voxel: " << watch.getTime() << std::endl;

        // Ground segmentation
        watch.reset();
        
        // Get a copy of the perpendicular_to_ground vector
        const Eigen::Vector3f perpendicular_to_ground{[this](){
            std::lock_guard<std::mutex> lock(imu_mutex_);
            return perpendicular_to_ground_;
        }()};

        // Clip points above ground plane
        pcl::PointIndices::Ptr clip_indices(new pcl::PointIndices);
        if (have_transform)
        {
            Eigen::Vector4f clip_plane(-perpendicular_to_ground[0], -perpendicular_to_ground[1], -perpendicular_to_ground[2], max_height);
            clip_plane = base_to_lidar_.matrix().transpose() * clip_plane;
            pcl::PlaneClipper3D<pcl::PointXYZI> clipper(clip_plane);
            clipper.clipPointCloud3D(*cloud, clip_indices->indices);
        }
       
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setAxis(perpendicular_to_ground);
        seg.setOptimizeCoefficients(true);        
        seg.setDistanceThreshold(distance_threshold);        
        seg.setEpsAngle(eps_angle);
        seg.setMaxIterations(max_iterations);
        seg.setInputCloud(cloud);
        if (!clip_indices->indices.empty())
        {
            seg.setIndices(clip_indices);
        }
        seg.segment(*inliers, *coefficients);
        std::cout << "Ground: " << watch.getTime() << std::endl;

        if (inliers->indices.size() < min_plane_points || seg.getModel()->computeVariance() > model_variance_threshold)
        {
            ROS_WARN("Not enough points to segment ground or variance too high");
            continue;
        }

        // Extract ground
        watch.reset();
        // Ground = Inliers of plane segmentation + points below the fitted plane
        pcl::PointIndices::Ptr new_clip_indices(new pcl::PointIndices);
        const Eigen::Vector4f new_clip_plane(-coefficients->values[0], -coefficients->values[1], -coefficients->values[2], -coefficients->values[3]);
        pcl::PlaneClipper3D<pcl::PointXYZI> new_clipper(new_clip_plane);
        new_clipper.clipPointCloud3D(*cloud, new_clip_indices->indices, clip_indices->indices);

        // Join new_clip_indices with inliers without duplicates
        inliers->indices.insert(inliers->indices.end(), new_clip_indices->indices.begin(), new_clip_indices->indices.end());
        std::sort(inliers->indices.begin(), inliers->indices.end());
        inliers->indices.erase(std::unique(inliers->indices.begin(), inliers->indices.end()), inliers->indices.end());
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*ground);
        std::cout << "Extract: " << watch.getTime() << std::endl;

        // Extract not ground
        watch.reset();
        pcl::PointCloud<pcl::PointXYZI>::Ptr not_ground(new pcl::PointCloud<pcl::PointXYZI>);
        extract.setNegative(true);
        extract.filter(*not_ground);
        std::cout << "Extract not ground: " << watch.getTime() << std::endl;
        
        // Clip points that are above a certain height
        watch.reset();
        pcl::PointIndices::Ptr high_clip_indices(new pcl::PointIndices);
        const Eigen::Vector4f high_clip_plane(-coefficients->values[0], -coefficients->values[1], -coefficients->values[2], -coefficients->values[3] + max_obstacle_height);
        pcl::PlaneClipper3D<pcl::PointXYZI> high_clipper(high_clip_plane);
        high_clipper.clipPointCloud3D(*not_ground, high_clip_indices->indices);

        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZI>);
        extract.setInputCloud(not_ground);
        extract.setIndices(high_clip_indices);
        extract.setNegative(false);
        extract.filter(*obstacles);
        std::cout << "Clip tall obstacles: " << watch.getTime() << std::endl;

        // Perform Euclidean Clustering
        watch.reset();
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(obstacles);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(cluster_tolerance);
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(obstacles);
        ec.extract(cluster_indices);
        std::cout << "Euclidean clustering: " << watch.getTime() << std::endl;

        // Cluster Bounding Boxes
        pcl::PointCloud<pcl::PointXYZI>::Ptr line_projected(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr plane_projected(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pca_projected_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::ProjectInliers<pcl::PointXYZI> line_proj;
        line_proj.setModelType(pcl::SACMODEL_LINE);
        line_proj.setInputCloud(obstacles);

        pcl::ProjectInliers<pcl::PointXYZI> plane_proj;
        plane_proj.setModelType(pcl::SACMODEL_PLANE);
        plane_proj.setInputCloud(obstacles);

        pcl::PCA<pcl::PointXYZI> pca;

        Eigen::Vector4f min_pt, max_pt;

        // The six coefficients of the line are given by a point on the line and the direction of the line as: [point_on_line.x point_on_line.y point_on_line.z line_direction.x line_direction.y line_direction.z] 
        pcl::ModelCoefficients::Ptr line_coefficients(new pcl::ModelCoefficients());
        line_coefficients->values.resize(6);
        line_coefficients->values[3] = coefficients->values[0];
        line_coefficients->values[4] = coefficients->values[1];
        line_coefficients->values[5] = coefficients->values[2];

        // The four coefficients of the plane are its Hessian Normal form: [normal_x normal_y normal_z d] 
        pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());
        plane_coefficients->values.resize(4);
        plane_coefficients->values[0] = coefficients->values[0];
        plane_coefficients->values[1] = coefficients->values[1];
        plane_coefficients->values[2] = coefficients->values[2];

        jsk_recognition_msgs::BoundingBoxArray jsk_bboxes;
        jsk_bboxes.header = pcl_conversions::fromPCL(obstacles->header);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr lines_cloud{new pcl::PointCloud<pcl::PointXYZRGB>};
        lines_cloud->header= obstacles->header;

        for (const auto &c_indices : cluster_indices) 
        {
            // Compute the cluster centroid
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*obstacles, c_indices, centroid);

            // Project the cluster points on a line perpendicular to the ground plane that passes through the centroid
            line_coefficients->values[0] = centroid[0];
            line_coefficients->values[1] = centroid[1];
            line_coefficients->values[2] = centroid[2];           

            pcl::PointIndices::Ptr c_indices_ptr(new pcl::PointIndices(c_indices));
            line_proj.setIndices(c_indices_ptr);
            line_proj.setModelCoefficients(line_coefficients);
            line_proj.filter(*line_projected);

            pcl::getMinMax3D(*line_projected, min_pt, max_pt);
            const double box_height = (max_pt.head<3>() - min_pt.head<3>()).norm();

            // Project the cluster points on a plane parallel to the ground plane that passes through the centroid
            plane_coefficients->values[3] = -(coefficients->values[0]*centroid[0] + coefficients->values[1]*centroid[1] + coefficients->values[2]*centroid[2]);
            
            plane_proj.setIndices(c_indices_ptr);
            plane_proj.setModelCoefficients(plane_coefficients);
            plane_proj.filter(*plane_projected);

            // Compute principal directions & Transform the original cloud to PCA coordinates            
            pca.setInputCloud(plane_projected);
            pca.project(*plane_projected, *pca_projected_cloud);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_line_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
            extract.setInputCloud(obstacles);
            extract.setIndices(c_indices_ptr);
            extract.setNegative(false);
            extract.filter(*plane_projected);
            pcl::copyPointCloud(*plane_projected, *cloud_line_rgb);

            int red = rand() % 255;
            int green = rand() % 255;
            int blue = rand() % 255;
            
            for (int i = 0; i < cloud_line_rgb->points.size(); i++)
            {
                cloud_line_rgb->points.at(i).r = red;
                cloud_line_rgb->points.at(i).g = green;
                cloud_line_rgb->points.at(i).b = blue;
            }

            *lines_cloud += *cloud_line_rgb;
            

            pca.project(*plane_projected, *pca_projected_cloud);
            Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
            eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));

            // // Scale for the arrows
            // const double arrow_scale = 1.5;

            // // Eigenvector 1 (principal direction)
            // geometry_msgs::Point start_point, end_point;
            // start_point.x = centroid[0];
            // start_point.y = centroid[1];
            // start_point.z = centroid[2];
            // end_point.x = centroid[0] + eigen_vectors(0, 0) * arrow_scale;
            // end_point.y = centroid[1] + eigen_vectors(1, 0) * arrow_scale;
            // end_point.z = centroid[2] + eigen_vectors(2, 0) * arrow_scale;
            // visual_tools_->publishArrow(start_point, end_point, rviz_visual_tools::colors::RED, rviz_visual_tools::scales::MEDIUM);
            // // addLine needs PointXYZ
            // pcl::PointXYZ start_point_pcl, end_point_pcl;
            // start_point_pcl.x = start_point.x;
            // start_point_pcl.y = start_point.y;
            // start_point_pcl.z = start_point.z;
            // end_point_pcl.x = end_point.x;
            // end_point_pcl.y = end_point.y;
            // end_point_pcl.z = end_point.z;
            // viewer->addLine(start_point_pcl, end_point_pcl, 1.0, 0.0, 0.0, "line" + std::to_string(rand()));

            // // Eigenvector 2
            // end_point.x = centroid[0] + eigen_vectors(0, 1) * arrow_scale;
            // end_point.y = centroid[1] + eigen_vectors(1, 1) * arrow_scale;
            // end_point.z = centroid[2] + eigen_vectors(2, 1) * arrow_scale;
            // visual_tools_->publishArrow(start_point, end_point, rviz_visual_tools::colors::GREEN, rviz_visual_tools::scales::MEDIUM);
            // start_point_pcl.x = start_point.x;
            // start_point_pcl.y = start_point.y;
            // start_point_pcl.z = start_point.z;
            // end_point_pcl.x = end_point.x;
            // end_point_pcl.y = end_point.y;
            // end_point_pcl.z = end_point.z;
            // viewer->addLine(start_point_pcl, end_point_pcl, 0.0, 1.0, 0.0, "line" + std::to_string(rand()));
            // // Eigenvector 3
            // end_point.x = centroid[0] + eigen_vectors(0, 2) * arrow_scale;
            // end_point.y = centroid[1] + eigen_vectors(1, 2) * arrow_scale;
            // end_point.z = centroid[2] + eigen_vectors(2, 2) * arrow_scale;
            // visual_tools_->publishArrow(start_point, end_point, rviz_visual_tools::colors::BLUE, rviz_visual_tools::scales::MEDIUM);
            // start_point_pcl.x = start_point.x;
            // start_point_pcl.y = start_point.y;
            // start_point_pcl.z = start_point.z;
            // end_point_pcl.x = end_point.x;
            // end_point_pcl.y = end_point.y;
            // end_point_pcl.z = end_point.z;
            // viewer->addLine(start_point_pcl, end_point_pcl, 0.0, 0.0, 1.0, "line" + std::to_string(rand()));

            // // Trigger RViz update
            // visual_tools_->trigger();

            // Get the minimum and maximum points of the transformed cloud.
            pcl::getMinMax3D(*pca_projected_cloud, min_pt, max_pt);
            const Eigen::Vector3f meanDiagonal = 0.5f * (max_pt.head<3>() + min_pt.head<3>());

            // Final transform
            // Convert rotation matrix to quaternion
            const Eigen::Quaternionf quaternion(eigen_vectors);  
            const Eigen::Vector3f position = eigen_vectors * meanDiagonal + centroid.head<3>();
            const Eigen::Vector3f dimension((max_pt(0) - min_pt(0)), (max_pt(1) - min_pt(1)), box_height);
            
            jsk_recognition_msgs::BoundingBox jsk_bbox;
            jsk_bbox.header = jsk_bboxes.header;
            jsk_bbox.pose.position.x = position(0);
            jsk_bbox.pose.position.y = position(1);
            jsk_bbox.pose.position.z = position(2);
            jsk_bbox.pose.orientation.w = quaternion.w();
            jsk_bbox.pose.orientation.x = quaternion.x();
            jsk_bbox.pose.orientation.y = quaternion.y();
            jsk_bbox.pose.orientation.z = quaternion.z();
            jsk_bbox.dimensions.x = dimension(0);
            jsk_bbox.dimensions.y = dimension(1);
            jsk_bbox.dimensions.z = dimension(2);
            jsk_bbox.value = 1.0f;
            // jsk_bbox.label = id;
            jsk_bboxes.boxes.push_back(jsk_bbox);
        }

        pub_jsk_bboxes_.publish(jsk_bboxes);

        sensor_msgs::PointCloud2::Ptr output{new sensor_msgs::PointCloud2()};
        pcl::toROSMsg(*ground, *output);
        test_pub_.publish(output);

        sensor_msgs::PointCloud2::Ptr output2{new sensor_msgs::PointCloud2()};
        pcl::toROSMsg(*not_ground, *output2);
        test_pub2_.publish(output2);

        sensor_msgs::PointCloud2::Ptr output3{new sensor_msgs::PointCloud2()};
        pcl::toROSMsg(*obstacles, *output3);
        test_pub3_.publish(output3);

        sensor_msgs::PointCloud2::Ptr output4{new sensor_msgs::PointCloud2()};
        pcl::toROSMsg(*lines_cloud, *output4);
        test_pub4_.publish(output4);

        

        // // visualize obstacles and bounding boxes
        // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(lines_cloud);
        // viewer->addPointCloud<pcl::PointXYZRGB> (lines_cloud, rgb, "Obstacles");
        // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Obstacles");
        // for (const auto& box: jsk_bboxes.boxes)
        // {
        //     Eigen::Vector3f position(box.pose.position.x, box.pose.position.y, box.pose.position.z);
        //     Eigen::Quaternionf quaternion(box.pose.orientation.w, box.pose.orientation.x, box.pose.orientation.y, box.pose.orientation.z);
        //     // Eigen::Quaternionf quaternion(1,0,0,0);
        //     Eigen::Vector3f dimension(box.dimensions.x, box.dimensions.y, box.dimensions.z);
        //     std::string name = "box" + std::to_string(rand());
        //     viewer->addCube(position, quaternion, dimension(0), dimension(1), dimension(2), name);
        //     viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name);
        //     viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, name);
        // }
        // viewer->spinOnce(100);

          
    }
}

} // namespace obstacle_detection

PLUGINLIB_EXPORT_CLASS(obstacle_detection::Detector, nodelet::Nodelet)