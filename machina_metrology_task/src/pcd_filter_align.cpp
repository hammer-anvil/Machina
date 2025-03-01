#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>
#include <fstream>
#include <sstream>
#include <filesystem>

class PointCloudProcessor : public rclcpp::Node {
public:
    PointCloudProcessor() : Node("pcd_filter_align") {
        // Declare and get parameters
        this->declare_parameter("input_dir", "");
        this->declare_parameter("output_file_path", "");
        this->declare_parameter("leaf_size", 0.01);
        this->declare_parameter("outlier_mean_k", 50);
        this->declare_parameter("outlier_std_dev", 1.0);
        this->declare_parameter("use_calibration", true);
        this->declare_parameter("use_icp", true);
        this->declare_parameter("max_icp_iterations", 50);
        this->declare_parameter("icp_transform_epsilon", 1e-6);
        this->declare_parameter("icp_fitness_epsilon", 1e-6);
        this->declare_parameter("max_correspondence_distance", 0.05);
        this->declare_parameter("publish_aligned_clouds", true);
        this->declare_parameter("verbose", true);
        
        // Get parameters
        input_dir_ = this->get_parameter("input_dir").as_string();
        output_dir_ = this->get_parameter("output_file_path").as_string();
        leaf_size_ = this->get_parameter("leaf_size").as_double();
        outlier_mean_k_ = this->get_parameter("outlier_mean_k").as_int();
        outlier_std_dev_ = this->get_parameter("outlier_std_dev").as_double();
        use_calibration_ = this->get_parameter("use_calibration").as_bool();
        use_icp_ = this->get_parameter("use_icp").as_bool();
        max_icp_iterations_ = this->get_parameter("max_icp_iterations").as_int();
        icp_transform_epsilon_ = this->get_parameter("icp_transform_epsilon").as_double();
        icp_fitness_epsilon_ = this->get_parameter("icp_fitness_epsilon").as_double();
        max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();
        publish_aligned_clouds_ = this->get_parameter("publish_aligned_clouds").as_bool();
        verbose_ = this->get_parameter("verbose").as_bool();

        // Set the complete output file path (directory + filename)
        output_file_path_ = output_dir_ + "aligned_scan.csv";

        // Log parameters if verbose
        if (verbose_) {
            RCLCPP_INFO(this->get_logger(), "Input directory: %s", input_dir_.c_str());
            RCLCPP_INFO(this->get_logger(), "Output directory: %s", output_dir_.c_str());
            RCLCPP_INFO(this->get_logger(), "Output file path: %s", output_file_path_.c_str());
            RCLCPP_INFO(this->get_logger(), "ICP parameters - Max iterations: %d, Epsilon: %f, Fitness epsilon: %f, Max correspondence: %f", 
                max_icp_iterations_, icp_transform_epsilon_, icp_fitness_epsilon_, max_correspondence_distance_);
        }

        // Publishers for visualization
        raw_pub1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/raw_point_cloud1", 10);
        raw_pub2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/raw_point_cloud2", 10);
        aligned_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_point_cloud", 10);

        // TF Broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        publish_static_tf();

        // Service for triggering the filtering and alignment
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "/filter_and_align",
            std::bind(&PointCloudProcessor::filter_and_align_service, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        // Load raw point clouds and start publishing them
        load_raw_point_clouds();
        
        // Timer to publish point clouds at 1 Hz
        timer_ = this->create_wall_timer(std::chrono::seconds(1), 
            std::bind(&PointCloudProcessor::publish_clouds, this));
        
        RCLCPP_INFO(this->get_logger(), 
            "Point Cloud Processor node initialized and ready. Call the 'filter_and_align' service to process the point clouds.");
    }

private:
    // Parameters
    std::string input_dir_;
    std::string output_dir_;
    std::string output_file_path_;
    double leaf_size_;
    int outlier_mean_k_;
    double outlier_std_dev_;
    bool use_calibration_;
    bool use_icp_;
    int max_icp_iterations_;
    double icp_transform_epsilon_;
    double icp_fitness_epsilon_;
    double max_correspondence_distance_;
    bool publish_aligned_clouds_;
    bool verbose_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_, cloud2_, aligned_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered1_, filtered2_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    void publish_static_tf() {
        timer_tf_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = this->get_clock()->now();
            transform.header.frame_id = "world";
            transform.child_frame_id = "robot_base";
            transform.transform.translation.x = 0.0;
            transform.transform.translation.y = 0.0;
            transform.transform.translation.z = 0.0;
            transform.transform.rotation.x = 0.0;
            transform.transform.rotation.y = 0.0;
            transform.transform.rotation.z = 0.0;
            transform.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(transform);
            if (verbose_) {
                RCLCPP_INFO_ONCE(this->get_logger(), "Broadcasting static TF: world -> robot_base");
            }
        });
    }

    void load_raw_point_clouds() {
        cloud1_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        cloud2_.reset(new pcl::PointCloud<pcl::PointXYZ>);

        // Form paths for the input files
        std::string scan1_path = input_dir_ + "/noisy_scan1.csv";
        std::string scan2_path = input_dir_ + "/noisy_scan2.csv";

        // Load original point clouds
        if (!load_point_cloud(scan1_path, cloud1_) ||
            !load_point_cloud(scan2_path, cloud2_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load one or both scans.");
            return;
        }

        if (verbose_) {
            RCLCPP_INFO(this->get_logger(), "Loaded %zu points from scan1, %zu points from scan2", 
                cloud1_->size(), cloud2_->size());
        }
    }

    void filter_and_align_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // Unused parameter
        
        RCLCPP_INFO(this->get_logger(), "Processing service called.");
        
        if (!cloud1_ || !cloud2_ || cloud1_->empty() || cloud2_->empty()) {
            response->success = false;
            response->message = "Raw point clouds are not loaded or are empty.";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        try {
            // Apply filtering
            filtered1_ = apply_filters(cloud1_);
            filtered2_ = apply_filters(cloud2_);

            if (verbose_) {
                RCLCPP_INFO(this->get_logger(), "Filtered cloud sizes: scan1 = %zu, scan2 = %zu", 
                    filtered1_->size(), filtered2_->size());
            }

            // Align using ICP if enabled
            if (use_icp_) {
                aligned_cloud_ = align_point_clouds(filtered1_, filtered2_);
            } else {
                aligned_cloud_ = filtered2_;
                if (verbose_) {
                    RCLCPP_INFO(this->get_logger(), "ICP alignment skipped as per configuration.");
                }
            }

            // Save the aligned point cloud
            save_aligned_cloud();
            
            response->success = true;
            response->message = "Point clouds successfully filtered and aligned.";
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Error during processing: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    bool load_point_cloud(const std::string &file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
            return false;
        }

        std::string line;
        std::getline(file, line); // Skip header

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string x_str, y_str, z_str;
            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            std::getline(ss, z_str, ',');

            pcl::PointXYZ point;
            point.x = std::stof(x_str);
            point.y = std::stof(y_str);
            point.z = std::stof(z_str);
            cloud->points.push_back(point);
        }
        file.close();

        return !cloud->empty();
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_filters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        // Apply Statistical Outlier Removal (SOR)
        pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(outlier_mean_k_);
        sor.setStddevMulThresh(outlier_std_dev_);
        sor.filter(*sor_filtered);

        // Apply Moving Average Filtering (MAF)
        pcl::PointCloud<pcl::PointXYZ>::Ptr maf_filtered = apply_moving_average_filter(sor_filtered, 10);

        return maf_filtered;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_moving_average_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int k) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t i = 0; i < cloud->points.size(); ++i) {
            float sum_x = 0, sum_y = 0, sum_z = 0;
            int count = 0;

            for (int j = std::max(0, (int)i - k); j < std::min((int)cloud->points.size(), (int)i + k); ++j) {
                sum_x += cloud->points[j].x;
                sum_y += cloud->points[j].y;
                sum_z += cloud->points[j].z;
                count++;
            }

            pcl::PointXYZ smoothed_point;
            smoothed_point.x = sum_x / count;
            smoothed_point.y = sum_y / count;
            smoothed_point.z = sum_z / count;
            filtered_cloud->points.push_back(smoothed_point);
        }

        return filtered_cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr align_point_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud2);
        icp.setInputTarget(cloud1);
        icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
        icp.setTransformationEpsilon(icp_transform_epsilon_);
        icp.setEuclideanFitnessEpsilon(icp_fitness_epsilon_);
        icp.setMaximumIterations(max_icp_iterations_);
        icp.align(*aligned_cloud);

        if (verbose_) {
            if (icp.hasConverged()) {
                RCLCPP_INFO(this->get_logger(), "ICP converged with fitness score: %f", icp.getFitnessScore());
            } else {
                RCLCPP_WARN(this->get_logger(), "ICP did not converge!");
            }
        }

        return aligned_cloud;
    }
    
    void save_aligned_cloud() {
        if (!aligned_cloud_ || aligned_cloud_->empty()) {
            RCLCPP_ERROR(this->get_logger(), "No aligned point cloud available to save!");
            return;
        }

        // Create output directory if it doesn't exist
        std::filesystem::path output_path(output_file_path_);
        std::filesystem::create_directories(output_path.parent_path());

        std::ofstream file(output_file_path_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", output_file_path_.c_str());
            return;
        }

        // Write header
        file << "x,y,z" << std::endl;

        // Write points
        for (const auto& point : aligned_cloud_->points) {
            file << point.x << "," << point.y << "," << point.z << std::endl;
        }

        file.close();
        if (verbose_) {
            RCLCPP_INFO(this->get_logger(), "Saved aligned point cloud to %s", output_file_path_.c_str());
        }
    }

    void publish_clouds() {
        sensor_msgs::msg::PointCloud2 raw_msg1, raw_msg2;
        
        // Always publish raw point clouds if available
        if (cloud1_ && !cloud1_->empty() && cloud2_ && !cloud2_->empty()) {
            pcl::toROSMsg(*cloud1_, raw_msg1);
            pcl::toROSMsg(*cloud2_, raw_msg2);
            
            // Set correct frame for visualization
            raw_msg1.header.frame_id = raw_msg2.header.frame_id = "robot_base";
            raw_msg1.header.stamp = raw_msg2.header.stamp = this->get_clock()->now();
            
            raw_pub1_->publish(raw_msg1);
            raw_pub2_->publish(raw_msg2);
            
            if (verbose_) {
                RCLCPP_INFO_ONCE(this->get_logger(), "Publishing raw point clouds...");
            }
        }
        
        // Publish aligned point cloud if available
        if (aligned_cloud_ && !aligned_cloud_->empty()) {
            sensor_msgs::msg::PointCloud2 aligned_msg;
            pcl::toROSMsg(*aligned_cloud_, aligned_msg);
            aligned_msg.header.frame_id = "robot_base";
            aligned_msg.header.stamp = this->get_clock()->now();
            aligned_pub_->publish(aligned_msg);
            
            if (verbose_) {
                RCLCPP_INFO_ONCE(this->get_logger(), "Publishing aligned point cloud...");
            }
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pub1_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pub2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_tf_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}