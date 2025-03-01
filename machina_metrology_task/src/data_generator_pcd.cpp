#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <random>
#include <fstream>

class PointCloudGenerator : public rclcpp::Node {
public:
    PointCloudGenerator() : Node("point_cloud_generator") {
        generate_point_clouds();
    }

private:
    void generate_point_clouds() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        cloud1->width = cloud2->width = 1000;
        cloud1->height = cloud2->height = 1;
        cloud1->is_dense = cloud2->is_dense = false;
        cloud1->points.resize(cloud1->width * cloud1->height);
        cloud2->points.resize(cloud2->width * cloud2->height);

        std::default_random_engine generator;
        std::normal_distribution<float> noise_dist(0.0, 0.02);
        std::uniform_real_distribution<float> pos_dist(-1.0, 1.0);

        std::string file_path1 = "/ros2_ws/src/machina_metrology_task/data/pcd/noisy_scan1.csv";
        std::string file_path2 = "/ros2_ws/src/machina_metrology_task/data/pcd/noisy_scan2.csv";
        std::ofstream file1(file_path1);
        std::ofstream file2(file_path2);

        if (!file1.is_open() || !file2.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open one of the files");
            return;
        }

        file1 << "x,y,z\n";
        file2 << "x,y,z\n";

        for (size_t i = 0; i < cloud1->points.size(); ++i) {
            float x = pos_dist(generator) + noise_dist(generator);
            float y = pos_dist(generator) + noise_dist(generator);
            float z = pos_dist(generator) + noise_dist(generator);

            cloud1->points[i].x = x;
            cloud1->points[i].y = y;
            cloud1->points[i].z = z;
            file1 << x << "," << y << "," << z << "\n";

            // Apply a small transformation to create an overlapping scan
            cloud2->points[i].x = x + 0.05; // Slight translation
            cloud2->points[i].y = y + 0.02;
            cloud2->points[i].z = z;
            file2 << cloud2->points[i].x << "," << cloud2->points[i].y << "," << cloud2->points[i].z << "\n";
        }
        
        file1.close();
        file2.close();

        RCLCPP_INFO(this->get_logger(), "Generated two overlapping noisy scans and saved to files");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudGenerator>();
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
