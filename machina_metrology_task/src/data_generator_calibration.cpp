#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <fstream>
#include <random>
#include <cmath>
#include <sys/stat.h>

const std::string SAVE_DIR = "/ros2_ws/src/machina_metrology_task/data/calibration/";

struct Pose {
    double x, y, z;
    double roll, pitch, yaw;
};

Eigen::Matrix4d poseToMatrix(const Pose& pose) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd rollAngle(pose.roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pose.pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(pose.yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rotationMatrix = (yawAngle * pitchAngle * rollAngle).matrix();
    matrix.block<3,3>(0,0) = rotationMatrix;
    matrix(0,3) = pose.x;
    matrix(1,3) = pose.y;
    matrix(2,3) = pose.z;
    return matrix;
}

std::vector<Pose> generateFlangePoses(int num_poses) {
    std::vector<Pose> flange_poses;
    for (int i = 0; i < num_poses; i++) {
        Pose flange = {0.1 * i, 0.05 * i, 0.15, 0.0, M_PI/6, 0.0};
        flange_poses.push_back(flange);
    }
    return flange_poses;
}

std::vector<Pose> generateScannerPoses(const std::vector<Pose>& flange_poses, const Eigen::Matrix4d& T_ground_truth) {
    std::vector<Pose> scanner_poses;
    for (const auto& flange : flange_poses) {
        Eigen::Matrix4d T_flange = poseToMatrix(flange);
        Eigen::Matrix4d T_scanner = T_ground_truth * T_flange;
        Pose scanner;
        scanner.x = T_scanner(0,3);
        scanner.y = T_scanner(1,3);
        scanner.z = T_scanner(2,3);
        Eigen::Vector3d euler = T_scanner.block<3,3>(0,0).eulerAngles(2, 1, 0);
        scanner.roll = euler[2];
        scanner.pitch = euler[1];
        scanner.yaw = euler[0];
        scanner_poses.push_back(scanner);
    }
    return scanner_poses;
}

void savePosesToFile(const std::string& filename, const std::vector<Pose>& poses) {
    std::ofstream file(SAVE_DIR + filename);
    file << "X,Y,Z,Roll,Pitch,Yaw\n";
    for (const auto& pose : poses) {
        file << pose.x << "," << pose.y << "," << pose.z << ","
             << pose.roll << "," << pose.pitch << "," << pose.yaw << "\n";
    }
}

int main() {
    Pose ground_truth_pose = {0.10, 0.05, 0.15, 0.0, M_PI/6, 0.0};
    Eigen::Matrix4d T_ground_truth = poseToMatrix(ground_truth_pose);
    auto flange_poses = generateFlangePoses(10);
    auto scanner_poses = generateScannerPoses(flange_poses, T_ground_truth);
    savePosesToFile("flange_poses.csv", flange_poses);
    savePosesToFile("scanner_poses.csv", scanner_poses);
    std::cout << "Data generation complete with known transformations. Files saved in: " << SAVE_DIR << std::endl;
    return 0;
}
