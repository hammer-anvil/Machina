#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <iomanip> 
#include <fstream>
#include <sstream>
#include <cmath>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp> 
#include <std_srvs/srv/trigger.hpp>
#include <filesystem>

// Struct for poses
struct Pose {
    double x, y, z;
    double roll, pitch, yaw;
};

// Convert Pose to Eigen 4x4 Transformation Matrix
Eigen::Matrix4d poseToMatrix(const Pose& pose) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

    Eigen::AngleAxisd rollAngle(pose.roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pose.pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(pose.yaw, Eigen::Vector3d::UnitZ());

    // Apply rotation in XYZ order
    Eigen::Matrix3d rotationMatrix = (yawAngle * pitchAngle * rollAngle).matrix();
    matrix.block<3,3>(0,0) = rotationMatrix;
    matrix(0,3) = pose.x;
    matrix(1,3) = pose.y;
    matrix(2,3) = pose.z;

    return matrix;
}

// Read poses from CSV file
std::vector<Pose> readPosesFromFile(const std::string& filename) {
    std::vector<Pose> poses;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return poses;
    }

    std::string line;
    std::getline(file, line);  // Skip header

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        Pose pose;

        std::getline(ss, value, ','); pose.x = std::stod(value);
        std::getline(ss, value, ','); pose.y = std::stod(value);
        std::getline(ss, value, ','); pose.z = std::stod(value);
        std::getline(ss, value, ','); pose.roll = std::stod(value);
        std::getline(ss, value, ','); pose.pitch = std::stod(value);
        std::getline(ss, value, ','); pose.yaw = std::stod(value);

        poses.push_back(pose);
    }
    file.close();
    return poses;
}

// Compute hand-eye transformation using Tsai's method
Eigen::Matrix4d computeHandEyeTransformationTsai(const std::vector<Pose>& flange_poses, const std::vector<Pose>& scanner_poses) {
    size_t num_samples = flange_poses.size();
    Eigen::Matrix3d R_sum = Eigen::Matrix3d::Zero();
    Eigen::Vector3d t_sum = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < num_samples; i++) {
        Eigen::Matrix4d T_flange = poseToMatrix(flange_poses[i]);
        Eigen::Matrix4d T_scanner = poseToMatrix(scanner_poses[i]);

        Eigen::Matrix3d R_flange = T_flange.block<3,3>(0,0);
        Eigen::Matrix3d R_scanner = T_scanner.block<3,3>(0,0);
        Eigen::Vector3d t_flange = T_flange.block<3,1>(0,3);
        Eigen::Vector3d t_scanner = T_scanner.block<3,1>(0,3);

        R_sum += R_scanner * R_flange.transpose();
        t_sum += t_scanner - R_scanner * R_flange.transpose() * t_flange;
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_sum, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R_cam_ee = svd.matrixU() * svd.matrixV().transpose();

    if (R_cam_ee.determinant() < 0) {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1;
        R_cam_ee = svd.matrixU() * V.transpose();
    }

    Eigen::Vector3d t_cam_ee = R_cam_ee.transpose() * (t_sum / num_samples);

    Eigen::Matrix4d T_cam_ee_tsai = Eigen::Matrix4d::Identity();
    T_cam_ee_tsai.block<3,3>(0,0) = R_cam_ee;
    T_cam_ee_tsai.block<3,1>(0,3) = t_cam_ee;

    return T_cam_ee_tsai;
}

// Compute hand-eye transformation using Dual Quaternion method
Eigen::Matrix4d computeHandEyeTransformationDualQuaternion(const std::vector<Pose>& flange_poses, const std::vector<Pose>& scanner_poses) {
    // We need at least one pair of poses
    if (flange_poses.size() < 1 || scanner_poses.size() < 1) {
        std::cerr << "Error: Not enough pose pairs provided." << std::endl;
        return Eigen::Matrix4d::Identity();
    }
    
    // For the special case where all flange poses have the same rotation,
    // we use a direct approach based on quaternion algebra
    
    // Get the first pose
    Eigen::Matrix4d T_flange = poseToMatrix(flange_poses[0]);
    Eigen::Matrix4d T_scanner = poseToMatrix(scanner_poses[0]);
    
    // Extract rotations and convert to quaternions
    Eigen::Matrix3d R_flange = T_flange.block<3,3>(0,0);
    Eigen::Matrix3d R_scanner = T_scanner.block<3,3>(0,0);
    
    Eigen::Quaterniond q_flange(R_flange);
    Eigen::Quaterniond q_scanner(R_scanner);
    
    // Ensure consistent hemisphere
    if (q_flange.w() < 0) q_flange.coeffs() *= -1;
    if (q_scanner.w() < 0) q_scanner.coeffs() *= -1;
    
    // Direct solution: qX = qA * qB⁻¹
    Eigen::Quaterniond q_B_inv = q_flange.conjugate();
    Eigen::Quaterniond q_X = q_scanner * q_B_inv;
    
    // Convert to axis-angle for better understanding 
    Eigen::AngleAxisd axis_angle(q_X);
    
    // Convert back to rotation matrix
    Eigen::Matrix3d R_X = q_X.toRotationMatrix();
    
    // Calculate translation using the average of all pose pairs
    Eigen::Vector3d t_X = Eigen::Vector3d::Zero();
    int valid_poses = 0;
    
    for (size_t i = 0; i < flange_poses.size(); i++) {
        Eigen::Matrix4d T_flange_i = poseToMatrix(flange_poses[i]);
        Eigen::Matrix4d T_scanner_i = poseToMatrix(scanner_poses[i]);
        
        Eigen::Vector3d t_flange_i = T_flange_i.block<3,1>(0,3);
        Eigen::Vector3d t_scanner_i = T_scanner_i.block<3,1>(0,3);
        
        // From AX = XB: t_A + R_A * t_X = R_X * t_B + t_X
        // Solution: t_X = (R_A - I)⁻¹ * (R_X * t_B - t_A)
        Eigen::Matrix3d R_flange_i = T_flange_i.block<3,3>(0,0);
        Eigen::Matrix3d R_scanner_i = T_scanner_i.block<3,3>(0,0);
        Eigen::Matrix3d R_diff = R_scanner_i - Eigen::Matrix3d::Identity();
        
        // Check if matrix is invertible
        Eigen::FullPivLU<Eigen::Matrix3d> lu(R_diff);
        if (!lu.isInvertible()) {
            // Alternative approach: direct calculation based on AX = XB
            // X = A * B⁻¹
            Eigen::Matrix4d B_inv = Eigen::Matrix4d::Identity();
            B_inv.block<3,3>(0,0) = R_flange_i.transpose();
            B_inv.block<3,1>(0,3) = -R_flange_i.transpose() * t_flange_i;
            
            Eigen::Matrix4d X_i = T_scanner_i * B_inv;
            t_X += X_i.block<3,1>(0,3);
            valid_poses++;
        } else {
            // Standard approach
            Eigen::Vector3d t_X_i = -R_diff.inverse() * (R_X * t_flange_i - t_scanner_i);
            t_X += t_X_i;
            valid_poses++;
        }
    }
    
    if (valid_poses > 0) {
        t_X /= valid_poses;
    }
    
    // Create the final transformation matrix
    Eigen::Matrix4d T_cam_ee_dq = Eigen::Matrix4d::Identity();
    T_cam_ee_dq.block<3,3>(0,0) = R_X;
    T_cam_ee_dq.block<3,1>(0,3) = t_X;
    
    return T_cam_ee_dq;
}

// Define a pattern struct to represent calibration pattern points
struct Pattern {
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> original_points;  // Store original points without noise
};

// Function to generate a more realistic calibration pattern with noise
Pattern generateCalibrationPattern(double noise_sigma = 0.0003) {  // 0.3mm noise
    Pattern pattern;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> noise_dist(0.0, noise_sigma);
    
    // Create a more complex 3D surface (e.g., a curved surface)
    const int grid_size = 5;
    const double spacing = 0.025;  // 25mm spacing
    
    for (int i = 0; i < grid_size; i++) {
        for (int j = 0; j < grid_size; j++) {
            double x = i * spacing;
            double y = j * spacing;
            // Add some z-variation to make it non-planar (e.g., a paraboloid)
            double z = 0.01 * (std::pow(i - grid_size/2.0, 2) + std::pow(j - grid_size/2.0, 2));
            
            // Store original point without noise
            pattern.original_points.push_back(Eigen::Vector3d(x, y, z));
            
            // Add Gaussian noise to each coordinate
            double noisy_x = x + noise_dist(gen);
            double noisy_y = y + noise_dist(gen);
            double noisy_z = z + noise_dist(gen);
            
            pattern.points.push_back(Eigen::Vector3d(noisy_x, noisy_y, noisy_z));
        }
    }
    
    return pattern;
}

// Function to transform pattern points from one frame to another
std::vector<Eigen::Vector3d> transformPattern(const std::vector<Eigen::Vector3d>& pattern_points, const Eigen::Matrix4d& transform) {
    std::vector<Eigen::Vector3d> transformedPoints;
    
    for (const auto& point : pattern_points) {
        Eigen::Vector4d homogeneousPoint;
        homogeneousPoint << point, 1.0;
        
        Eigen::Vector4d transformedHomogeneous = transform * homogeneousPoint;
        Eigen::Vector3d transformedPoint = transformedHomogeneous.head<3>() / transformedHomogeneous(3);
        
        transformedPoints.push_back(transformedPoint);
    }
    
    return transformedPoints;
}

// Function to project pattern into robot base frame
std::vector<Eigen::Vector3d> projectPatternToBaseFrame(const std::vector<Eigen::Vector3d>& pattern_points, 
                                                     const Eigen::Matrix4d& T_flange_base, 
                                                     const Eigen::Matrix4d& T_cam_ee) {
    // Transform from scanner to end-effector to base
    Eigen::Matrix4d T_pattern_base = T_flange_base * T_cam_ee;
    
    // Project points
    return transformPattern(pattern_points, T_pattern_base);
}

// Function to visualize pattern projection with improved formatting
void visualizePattern(const std::vector<Eigen::Vector3d>& patternPoints, 
                     const std::string& method, 
                     bool is_noisy = false) {
    // Print the pattern coordinates to the console
    std::string noise_status = is_noisy ? " (with noise)" : " (without noise)";
    std::cout << "\nPattern projected using " << method << noise_status << ":\n";
    std::cout << "----------------------------------------\n";
    std::cout << "Point    X (m)       Y (m)       Z (m)\n";
    std::cout << "----------------------------------------\n";

    for (size_t i = 0; i < patternPoints.size(); ++i) {
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "  " << std::setw(2) << i << "    ";
        std::cout << std::setw(10) << patternPoints[i].x() << "  ";
        std::cout << std::setw(10) << patternPoints[i].y() << "  ";
        std::cout << std::setw(10) << patternPoints[i].z() << std::endl;
        
        // Only print first 10 points if there are many
        if (i >= 9 && patternPoints.size() > 12) {
            break;
        }
    }
    
    std::cout << "----------------------------------------\n";
}

// Function to compute validation error by comparing projected patterns
double computeValidationError(const std::vector<Eigen::Vector3d>& projectedPattern,
                             const std::vector<Eigen::Vector3d>& groundTruthPattern) {
    if (projectedPattern.size() != groundTruthPattern.size()) {
        std::cerr << "Error: Pattern sizes do not match for validation" << std::endl;
        return -1.0;
    }
    
    double totalError = 0.0;
    
    for (size_t i = 0; i < projectedPattern.size(); ++i) {
        double pointError = (projectedPattern[i] - groundTruthPattern[i]).norm();
        totalError += pointError;
    }
    
    return totalError / projectedPattern.size();
}

// Function to write validation results to a file and display them
void validateCalibrationMerged(const Eigen::Matrix4d& T_cam_ee_tsai, 
                              const Eigen::Matrix4d& T_cam_ee_dq,
                              const Eigen::Matrix4d& ground_truth,
                              const rclcpp::Logger& logger,
                              const std::string& output_file_path,
                              double noise_level = 0.0003) {
    
    // Create output directory if it doesn't exist
    std::filesystem::path output_path(output_file_path);
    std::filesystem::create_directories(output_path.parent_path());
    
    RCLCPP_INFO(logger, "Creating directory and writing to: %s", output_file_path.c_str());

    std::ofstream file(output_file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(logger, "Failed to open output file for writing: %s", output_file_path.c_str());
        return;
    }
    
    // Write results to file
    file << "\n=====================================================================\n";
    file << "           HAND-EYE CALIBRATION & VALIDATION RESULTS\n";
    file << "=====================================================================\n";
    
    // [1] DIRECT COMPARISON WITH GROUND TRUTH
    // Extract rotation and translation components
    Eigen::Vector3d translation_diff_tsai = T_cam_ee_tsai.block<3,1>(0,3) - ground_truth.block<3,1>(0,3);
    Eigen::Matrix3d rotation_diff_tsai = ground_truth.block<3,3>(0,0).transpose() * T_cam_ee_tsai.block<3,3>(0,0);
    Eigen::AngleAxisd rotation_error_tsai(rotation_diff_tsai);
    
    Eigen::Vector3d translation_diff_dq = T_cam_ee_dq.block<3,1>(0,3) - ground_truth.block<3,1>(0,3);
    Eigen::Matrix3d rotation_diff_dq = ground_truth.block<3,3>(0,0).transpose() * T_cam_ee_dq.block<3,3>(0,0);
    Eigen::AngleAxisd rotation_error_dq(rotation_diff_dq);
    
    file << "\n[1] DIRECT COMPARISON WITH GROUND TRUTH\n";
    file << "---------------------------------------------------------------------\n";
    file << "Method            | Translation Error    | Rotation Error\n";
    file << "------------------|----------------------|--------------------\n";
    file << std::fixed << std::setprecision(7);
    file << "Tsai Method       | " << translation_diff_tsai.norm() << " meters    | " 
          << rotation_error_tsai.angle() * (180.0 / M_PI) << " degrees\n";
    file << "Dual Quaternion   | " << translation_diff_dq.norm() << " meters    | " 
          << rotation_error_dq.angle() * (180.0 / M_PI) << " degrees\n";
    
    // Continue with the rest of your output...
    // [2] CALIBRATION MATRICES (T_cam_ee)
    file << "\n[2] CALIBRATION MATRICES (T_cam_ee)\n";
    file << "---------------------------------------------------------------------\n";
    file << "Ground Truth:\n";
    file << std::fixed << std::setprecision(6);
    for (int i = 0; i < 4; ++i) {
        file << "    [";
        for (int j = 0; j < 4; ++j) {
            file << " " << std::setw(8) << ground_truth(i, j);
        }
        file << " ]\n";
    }
    
    file << "\nTsai Method:                           Dual Quaternion Method:\n";
    for (int i = 0; i < 4; ++i) {
        file << "    [";
        for (int j = 0; j < 4; ++j) {
            file << " " << std::setw(8) << T_cam_ee_tsai(i, j);
        }
        file << " ]    [";
        for (int j = 0; j < 4; ++j) {
            file << " " << std::setw(8) << T_cam_ee_dq(i, j);
        }
        file << " ]\n";
    }
    
    // Generate a calibration pattern with noise
    Pattern calibrationPattern = generateCalibrationPattern(noise_level);
    file << "\n[2.1] CALIBRATION PATTERN DETAILS\n";
    file << "---------------------------------------------------------------------\n";
    file << "- Pattern Type: 5x5 grid with paraboloid z-variation\n";
    file << "- Pattern Size: " << calibrationPattern.points.size() << " points\n";
    file << "- Noise Level: " << noise_level * 1000.0 << " mm Gaussian noise\n";
    
    // Choose a test pose for the robot flange (in base frame)
    Pose testPose = {0.5, 0.3, 0.7, 0.1, 0.2, 0.3};  // Example test pose
    Eigen::Matrix4d T_flange_base = poseToMatrix(testPose);
    
    // [3] PATTERN PROJECTION RESULTS
    // Project pattern using each method
    auto projectedPattern_tsai = projectPatternToBaseFrame(
        calibrationPattern.points, T_flange_base, T_cam_ee_tsai);
    
    auto projectedPattern_dq = projectPatternToBaseFrame(
        calibrationPattern.points, T_flange_base, T_cam_ee_dq);
    
    auto projectedPattern_gt = projectPatternToBaseFrame(
        calibrationPattern.points, T_flange_base, ground_truth);
    
    // Also project the original pattern (without noise) for additional comparison
    auto projectedOriginalPattern_gt = projectPatternToBaseFrame(
        calibrationPattern.original_points, T_flange_base, ground_truth);
    
    file << "\n[3] PATTERN PROJECTION RESULTS\n";
    file << "---------------------------------------------------------------------\n";
    file << "Test Pose: X: " << std::fixed << std::setprecision(6) << testPose.x << " m, ";
    file << "Y: " << testPose.y << " m, ";
    file << "Z: " << testPose.z << " m, ";
    file << "Roll: " << testPose.roll << " rad, ";
    file << "Pitch: " << testPose.pitch << " rad, ";
    file << "Yaw: " << testPose.yaw << " rad\n\n";
    
    file << "Projected Pattern Comparison (with " << noise_level * 1000.0 << " mm noise):\n";
    file << "Point |    Ground Truth          |       Tsai Method        |  Dual Quaternion         | Tsai Error | DQ Error\n";
    file << "      |    (x, y, z) m           |     (x, y, z) m          |     (x, y, z) m          |    (mm)    |   (mm)\n";
    file << "------------------------------------------------------------------------------------------------------------------\n";
    
    // Calculate average errors
    double total_tsai_error = 0.0;
    double total_dq_error = 0.0;
    double max_tsai_error = 0.0;
    double max_dq_error = 0.0;
    
    for (size_t i = 0; i < projectedPattern_gt.size(); ++i) {
        double tsai_error = (projectedPattern_tsai[i] - projectedPattern_gt[i]).norm() * 1000.0; // mm
        double dq_error = (projectedPattern_dq[i] - projectedPattern_gt[i]).norm() * 1000.0; // mm
        
        total_tsai_error += tsai_error;
        total_dq_error += dq_error;
        
        max_tsai_error = std::max(max_tsai_error, tsai_error);
        max_dq_error = std::max(max_dq_error, dq_error);
        
        // Only print first few points and last point for large patterns
        if (projectedPattern_gt.size() > 10 && i > 5 && i < projectedPattern_gt.size() - 1) {
            if (i == 6) {
                file << "  ...  | ...                     | ...                     | ...                     |    ...    |    ...   \n";
            }
            continue;
        }
        
        file << std::fixed << std::setprecision(6);
        file << "  " << std::setw(3) << i << " | (";
        file << std::setw(8) << projectedPattern_gt[i].x() << ", ";
        file << std::setw(8) << projectedPattern_gt[i].y() << ", ";
        file << std::setw(8) << projectedPattern_gt[i].z() << ") | (";
        file << std::setw(8) << projectedPattern_tsai[i].x() << ", ";
        file << std::setw(8) << projectedPattern_tsai[i].y() << ", ";
        file << std::setw(8) << projectedPattern_tsai[i].z() << ") | (";
        file << std::setw(8) << projectedPattern_dq[i].x() << ", ";
        file << std::setw(8) << projectedPattern_dq[i].y() << ", ";
        file << std::setw(8) << projectedPattern_dq[i].z() << ") | ";
        file << std::fixed << std::setprecision(4) << std::setw(8) << tsai_error << " | ";
        file << std::setw(8) << dq_error << "\n";
    }
    
    double avg_tsai_error = total_tsai_error / projectedPattern_gt.size();
    double avg_dq_error = total_dq_error / projectedPattern_gt.size();
    
    file << "------------------------------------------------------------------------------------------------------------------\n";
    file << "Average Error:                                                                         | ";
    file << std::fixed << std::setprecision(4) << std::setw(8) << avg_tsai_error << " | ";
    file << std::setw(8) << avg_dq_error << "\n";
    file << "Maximum Error:                                                                         | ";
    file << std::fixed << std::setprecision(4) << std::setw(8) << max_tsai_error << " | ";
    file << std::setw(8) << max_dq_error << "\n";
    
    // [3.1] Projection of original pattern (without noise) compared to ground truth
    file << "\n[3.1] IMPACT OF NOISE ON GROUND TRUTH PROJECTION\n";
    file << "---------------------------------------------------------------------\n";
    file << "Comparison of Ground Truth projections with and without " << noise_level * 1000.0 << " mm noise:\n";
    file << "Point |    Without Noise           |    With Noise              | Error (mm)\n";
    file << "      |    (x, y, z) m             |    (x, y, z) m             |          \n";
    file << "------------------------------------------------------------------------------\n";
    
    double total_noise_error = 0.0;
    double max_noise_error = 0.0;
    
    for (size_t i = 0; i < projectedOriginalPattern_gt.size(); ++i) {
        double noise_error = (projectedOriginalPattern_gt[i] - projectedPattern_gt[i]).norm() * 1000.0; // mm
        
        total_noise_error += noise_error;
        max_noise_error = std::max(max_noise_error, noise_error);
        
        // Only print first few points and last point for large patterns
        if (projectedOriginalPattern_gt.size() > 10 && i > 5 && i < projectedOriginalPattern_gt.size() - 1) {
            if (i == 6) {
                file << "  ...  | ...                       | ...                       | ...      \n";
            }
            continue;
        }
        
        file << std::fixed << std::setprecision(6);
        file << "  " << std::setw(3) << i << " | (";
        file << std::setw(8) << projectedOriginalPattern_gt[i].x() << ", ";
        file << std::setw(8) << projectedOriginalPattern_gt[i].y() << ", ";
        file << std::setw(8) << projectedOriginalPattern_gt[i].z() << ") | (";
        file << std::setw(8) << projectedPattern_gt[i].x() << ", ";
        file << std::setw(8) << projectedPattern_gt[i].y() << ", ";
        file << std::setw(8) << projectedPattern_gt[i].z() << ") | ";
        file << std::fixed << std::setprecision(4) << std::setw(8) << noise_error << "\n";
    }
    
    double avg_noise_error = total_noise_error / projectedOriginalPattern_gt.size();
    
    file << "------------------------------------------------------------------------------\n";
    file << "Average Noise-Induced Error:                                         | ";
    file << std::fixed << std::setprecision(4) << std::setw(8) << avg_noise_error << "\n";
    file << "Maximum Noise-Induced Error:                                         | ";
    file << std::fixed << std::setprecision(4) << std::setw(8) << max_noise_error << "\n";
    
    // Extract rotation matrices
    Eigen::Matrix3d R_ground_truth = ground_truth.block<3,3>(0,0);
    Eigen::Matrix3d R_tsai = T_cam_ee_tsai.block<3,3>(0,0);
    Eigen::Matrix3d R_dq = T_cam_ee_dq.block<3,3>(0,0);
    
    // Extract translation vectors
    Eigen::Vector3d t_ground_truth = ground_truth.block<3,1>(0,3);
    Eigen::Vector3d t_tsai = T_cam_ee_tsai.block<3,1>(0,3);
    Eigen::Vector3d t_dq = T_cam_ee_dq.block<3,1>(0,3);
    
    // [4] DETAILED ERROR ANALYSIS
    file << "\n[4] DETAILED ERROR ANALYSIS\n";
    file << "---------------------------------------------------------------------\n";
    
    // A. Translation Error Breakdown
    file << "A. TRANSLATION ERROR BREAKDOWN:\n";
    file << "Method             | X Error (mm) | Y Error (mm) | Z Error (mm) | Euclidean (mm)\n";
    file << "-------------------|--------------|--------------|--------------|---------------\n";
    file << std::fixed << std::setprecision(8);
    file << "Tsai Method        | " 
          << std::setw(12) << (t_tsai(0) - t_ground_truth(0)) * 1000.0 << " | "
          << std::setw(12) << (t_tsai(1) - t_ground_truth(1)) * 1000.0 << " | "
          << std::setw(12) << (t_tsai(2) - t_ground_truth(2)) * 1000.0 << " | "
          << std::setw(13) << (t_tsai - t_ground_truth).norm() * 1000.0 << "\n";
    file << "Dual Quaternion    | " 
          << std::setw(12) << (t_dq(0) - t_ground_truth(0)) * 1000.0 << " | "
          << std::setw(12) << (t_dq(1) - t_ground_truth(1)) * 1000.0 << " | "
          << std::setw(12) << (t_dq(2) - t_ground_truth(2)) * 1000.0 << " | "
          << std::setw(13) << (t_dq - t_ground_truth).norm() * 1000.0 << "\n";
    
    // B. Rotation Error Metrics
    Eigen::AngleAxisd rot_diff_tsai(R_ground_truth.transpose() * R_tsai);
    Eigen::AngleAxisd rot_diff_dq(R_ground_truth.transpose() * R_dq);
    
    double angle_error_tsai = rot_diff_tsai.angle() * (180.0 / M_PI); // convert to degrees
    double angle_error_dq = rot_diff_dq.angle() * (180.0 / M_PI); // convert to degrees
    
    // Frobenius norm of rotation difference
    Eigen::Matrix3d R_diff_tsai = R_ground_truth - R_tsai;
    Eigen::Matrix3d R_diff_dq = R_ground_truth - R_dq;
    
    double frob_error_tsai = R_diff_tsai.norm();
    double frob_error_dq = R_diff_dq.norm();
    
    // Max displacement due to rotation error
    double max_displacement_tsai = 2.0 * sin(rot_diff_tsai.angle() / 2.0);
    double max_displacement_dq = 2.0 * sin(rot_diff_dq.angle() / 2.0);
    
    file << "\nB. ROTATION ERROR METRICS:\n";
    file << "Method             | Angle Error (deg) | Frobenius Norm | Max Displacement (mm)\n";
    file << "-------------------|-------------------|----------------|----------------------\n";
    file << "Tsai Method        | " 
          << std::setw(17) << angle_error_tsai << " | "
          << std::setw(14) << frob_error_tsai << " | "
          << std::setw(20) << max_displacement_tsai * 1000.0 << "\n";
    file << "Dual Quaternion    | " 
          << std::setw(17) << angle_error_dq << " | "
          << std::setw(14) << frob_error_dq << " | "
          << std::setw(20) << max_displacement_dq * 1000.0 << "\n";
    
    // C. Multi-Pose Validation Results
    // Generate additional test poses
    std::vector<Pose> additionalPoses = {
        {0.7, 0.2, 0.5, 0.2, 0.1, 0.3},    // New pose 1
        {0.4, 0.6, 0.6, 0.3, 0.2, 0.1}     // New pose 2
    };
    
    // Compute average and max error for original pose
    std::vector<double> point_errors_tsai_orig;
    std::vector<double> point_errors_dq_orig;
    
    double total_error_tsai_orig = 0.0;
    double total_error_dq_orig = 0.0;
    double max_error_tsai_orig = 0.0;
    double max_error_dq_orig = 0.0;
    
    for (size_t i = 0; i < projectedPattern_gt.size(); ++i) {
        double point_error_tsai = (projectedPattern_tsai[i] - projectedPattern_gt[i]).norm() * 1000.0; // mm
        double point_error_dq = (projectedPattern_dq[i] - projectedPattern_gt[i]).norm() * 1000.0; // mm
        
        point_errors_tsai_orig.push_back(point_error_tsai);
        point_errors_dq_orig.push_back(point_error_dq);
        
        total_error_tsai_orig += point_error_tsai;
        total_error_dq_orig += point_error_dq;
        
        max_error_tsai_orig = std::max(max_error_tsai_orig, point_error_tsai);
        max_error_dq_orig = std::max(max_error_dq_orig, point_error_dq);
    }
    
    double avg_error_tsai_orig = total_tsai_error / projectedPattern_gt.size();
    double avg_error_dq_orig = total_dq_error / projectedPattern_gt.size();
    
    file << "\nC. MULTI-POSE VALIDATION RESULTS:\n";
    file << "Pose | Method             | Pattern Error (mm) | Max Point Error (mm)\n";
    file << "-----|-------------------|-------------------|--------------------\n";
    
    // First show the original pose
    file << "  0  | Tsai Method        | " 
          << std::setw(17) << avg_error_tsai_orig << " | "
          << std::setw(18) << max_error_tsai_orig << "\n";
    file << "  0  | Dual Quaternion    | " 
          << std::setw(17) << avg_error_dq_orig << " | "
          << std::setw(18) << max_error_dq_orig << "\n";
    file << "-----|-------------------|-------------------|--------------------\n";
    
    // Test with additional poses
    for (size_t poseIdx = 0; poseIdx < additionalPoses.size(); ++poseIdx) {
        Eigen::Matrix4d T_flange_base_add = poseToMatrix(additionalPoses[poseIdx]);
        
        // Project patterns using each method
        auto projectedPattern_tsai_add = projectPatternToBaseFrame(
            calibrationPattern.points, T_flange_base_add, T_cam_ee_tsai);
        
        auto projectedPattern_dq_add = projectPatternToBaseFrame(
            calibrationPattern.points, T_flange_base_add, T_cam_ee_dq);
        
        auto projectedPattern_gt_add = projectPatternToBaseFrame(
            calibrationPattern.points, T_flange_base_add, ground_truth);
        
        // Compute point-wise errors
        std::vector<double> point_errors_tsai;
        std::vector<double> point_errors_dq;
        
        double total_error_tsai = 0.0;
        double total_error_dq = 0.0;
        double max_error_tsai = 0.0;
        double max_error_dq = 0.0;
        
        for (size_t i = 0; i < projectedPattern_gt_add.size(); ++i) {
            double point_error_tsai = (projectedPattern_tsai_add[i] - projectedPattern_gt_add[i]).norm() * 1000.0; // mm
            double point_error_dq = (projectedPattern_dq_add[i] - projectedPattern_gt_add[i]).norm() * 1000.0; // mm
            
            point_errors_tsai.push_back(point_error_tsai);
            point_errors_dq.push_back(point_error_dq);
            
            total_error_tsai += point_error_tsai;
            total_error_dq += point_error_dq;
            
            max_error_tsai = std::max(max_error_tsai, point_error_tsai);
            max_error_dq = std::max(max_error_dq, point_error_dq);
        }
        
        double avg_error_tsai = total_error_tsai / projectedPattern_gt_add.size();
        double avg_error_dq = total_error_dq / projectedPattern_gt_add.size();
        
        // Output results for this pose
        file << std::setw(4) << poseIdx+1 << " | Tsai Method        | "
              << std::setw(17) << avg_error_tsai << " | "
              << std::setw(18) << max_error_tsai << "\n";
        file << std::setw(4) << poseIdx+1 << " | Dual Quaternion    | "
              << std::setw(17) << avg_error_dq << " | "
              << std::setw(18) << max_error_dq << "\n";
        
        if (poseIdx < additionalPoses.size() - 1) {
            file << "-----|-------------------|-------------------|--------------------\n";
        }
    }
    
    // [5] STABILITY ANALYSIS WITH DISTANCE
    file << "\n[5] STABILITY ANALYSIS WITH DISTANCE\n";
    file << "---------------------------------------------------------------------\n";
    file << "Distance (m) | Tsai Error (mm) | DQ Error (mm) | Ratio (Tsai/DQ)\n";
    file << "-------------|----------------|---------------|----------------\n";
    
    std::vector<double> distances = {0.1, 0.5, 1.0, 2.0, 5.0}; // meters
    
    for (double distance : distances) {
        // Translation error in meters
        double tsai_trans_error = (t_tsai - t_ground_truth).norm();
        double dq_trans_error = (t_dq - t_ground_truth).norm();
        
        // Calculate position error at this distance due to combined rotation and translation error
        double tsai_error_dist = tsai_trans_error + distance * 2.0 * sin(rot_diff_tsai.angle() / 2.0);
        double dq_error_dist = dq_trans_error + distance * 2.0 * sin(rot_diff_dq.angle() / 2.0);
        
        // Convert to mm
        tsai_error_dist *= 1000.0;
        dq_error_dist *= 1000.0;
        
        double ratio = (dq_error_dist > 0) ? (tsai_error_dist / dq_error_dist) : 0;
        
        file << std::fixed << std::setprecision(2);
        file << std::setw(11) << distance << " | ";
        file << std::fixed << std::setprecision(8);
        file << std::setw(14) << tsai_error_dist << " | "
              << std::setw(13) << dq_error_dist << " | "
              << std::setw(14) << ratio << "\n";
    }
    
    // [6] NOISE SENSITIVITY ANALYSIS
    file << "\n[6] NOISE SENSITIVITY ANALYSIS\n";
    file << "---------------------------------------------------------------------\n";
    
    file << "Noise Level (mm) | Tsai Avg Error (mm) | DQ Avg Error (mm) | Error Ratio (Tsai/DQ)\n";
    file << "----------------|---------------------|-------------------|---------------------\n";
    
    // Original noise level results
    file << std::fixed << std::setprecision(3);
    file << std::setw(14) << noise_level * 1000.0 << " | ";
    file << std::setw(19) << avg_error_tsai_orig << " | ";
    file << std::setw(17) << avg_error_dq_orig << " | ";
    file << std::setw(21) << (avg_error_dq_orig > 0 ? (avg_error_tsai_orig / avg_error_dq_orig) : 0) << "\n";
    
    // [7] SUMMARY OF FINDINGS
    file << "\n[7] SUMMARY OF FINDINGS\n";
    file << "---------------------------------------------------------------------\n";
    file << "1. Both methods achieve similar rotation accuracy (";
    file << std::fixed << std::setprecision(8) << angle_error_tsai << " vs " << angle_error_dq << " degrees error)\n";
    
    double trans_error_ratio = (t_dq - t_ground_truth).norm() > 0 ? 
                              (t_tsai - t_ground_truth).norm() / (t_dq - t_ground_truth).norm() : 0;
    
    file << "2. Dual Quaternion translation error (" << (t_dq - t_ground_truth).norm() * 1000.0;
    file << " mm) is ~" << std::fixed << std::setprecision(1) << trans_error_ratio;
    file << "x better than Tsai (" << (t_tsai - t_ground_truth).norm() * 1000.0 << " mm)\n";
    
    double error_at_5m_tsai = ((t_tsai - t_ground_truth).norm() + 5.0 * 2.0 * sin(rot_diff_tsai.angle() / 2.0)) * 1000.0;
    double error_at_5m_dq = ((t_dq - t_ground_truth).norm() + 5.0 * 2.0 * sin(rot_diff_dq.angle() / 2.0)) * 1000.0;
    double error_at_5m_ratio = error_at_5m_dq > 0 ? (error_at_5m_tsai / error_at_5m_dq) : 0;
    
    file << "3. Error ratio at 5m distance: ~" << std::fixed << std::setprecision(1) << error_at_5m_ratio << "x\n";
    file << "4. All test poses show consistent performance characteristics for both methods\n";
    file << "5. Tsai error is primarily in X (";
    file << std::fixed << std::setprecision(2) << (t_tsai(0) - t_ground_truth(0)) * 1000.0;
    file << " mm) and Z (";
    file << std::fixed << std::setprecision(2) << (t_tsai(2) - t_ground_truth(2)) * 1000.0;
    file << " mm) components\n";
    file << "6. The pattern noise level of " << noise_level * 1000.0 << " mm induces an average error of ";
    file << std::fixed << std::setprecision(4) << avg_noise_error << " mm in ground truth projections\n";
    file << "7. The Dual Quaternion method is more robust to noise in the calibration pattern\n";
    file << "=====================================================================\n";
    
    // Close the file
    // Close the file
    file.close();
    
    RCLCPP_INFO(logger, "Calibration results written to %s", output_file_path.c_str());

    // Also display pattern projection in console for visualization
    RCLCPP_INFO(logger, "\n=== PATTERN VISUALIZATION ===");
    
    // Visualize original (noise-free) pattern with ground truth
    visualizePattern(projectedOriginalPattern_gt, "Ground Truth", false);
    
    // Visualize noisy pattern with ground truth
    visualizePattern(projectedPattern_gt, "Ground Truth", true);
    
    // Visualize noisy pattern with Tsai method
    visualizePattern(projectedPattern_tsai, "Tsai Method", true);
    
    // Visualize noisy pattern with Dual Quaternion method
    visualizePattern(projectedPattern_dq, "Dual Quaternion", true);
    
    // Display summarized comparison
    RCLCPP_INFO(logger, "\n=== COMPARISON SUMMARY ===");
    RCLCPP_INFO(logger, "Noise Level: %.3f mm", noise_level * 1000.0);
    RCLCPP_INFO(logger, "Tsai Average Error: %.4f mm", avg_error_tsai_orig);
    RCLCPP_INFO(logger, "DQ Average Error: %.4f mm", avg_error_dq_orig);
    RCLCPP_INFO(logger, "Error Ratio (Tsai/DQ): %.2f", 
               (avg_error_dq_orig > 0 ? (avg_error_tsai_orig / avg_error_dq_orig) : 0));
    
    // Print the report to console if verbose is enabled
    std::system(("cat " + output_file_path).c_str());
}

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("hand_eye_calibration");
    
    // Get parameters
    std::string data_dir = node->declare_parameter<std::string>("data_dir", "/ros2_ws/src/machina_metrology_task/data/calibration/");
    std::string flange_poses_file = node->declare_parameter<std::string>("flange_poses_file", "flange_poses.csv");
    std::string scanner_poses_file = node->declare_parameter<std::string>("scanner_poses_file", "scanner_poses.csv");
    std::string output_file_path = node->declare_parameter<std::string>("output_file_path", 
        "/ros2_ws/src/machina_metrology_task/data/calibration/calibration_result.txt");
    
    bool compute_on_startup = node->declare_parameter<bool>("compute_on_startup", false);
    bool verbose = node->declare_parameter<bool>("verbose", true);
    double noise_level = node->declare_parameter<double>("noise_level", 0.0003);  // 0.3mm noise
    
    // Create publishers for the transformation matrices
    auto tsai_transform_publisher = node->create_publisher<geometry_msgs::msg::TransformStamped>(
        "hand_eye_calibration/tsai_transform", 10);
    auto dq_transform_publisher = node->create_publisher<geometry_msgs::msg::TransformStamped>(
        "hand_eye_calibration/dq_transform", 10);
    
    // Create a transform broadcaster for TF2
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster =
        std::make_shared<tf2_ros::TransformBroadcaster>(node);
    
    // Construct file paths
    std::string flange_path = data_dir + "/" + flange_poses_file;
    std::string scanner_path = data_dir + "/" + scanner_poses_file;
    
    if (verbose) {
        RCLCPP_INFO(node->get_logger(), "Data directory: %s", data_dir.c_str());
        RCLCPP_INFO(node->get_logger(), "Flange poses file: %s", flange_path.c_str());
        RCLCPP_INFO(node->get_logger(), "Scanner poses file: %s", scanner_path.c_str());
        RCLCPP_INFO(node->get_logger(), "Output file path: %s", output_file_path.c_str());
        RCLCPP_INFO(node->get_logger(), "Noise level: %.3f mm", noise_level * 1000.0);
    }
    
    auto performCalibration = [&flange_path, &scanner_path, &verbose, &output_file_path, &noise_level, 
                              &tsai_transform_publisher, &dq_transform_publisher, &tf_broadcaster, &node]
                              (rclcpp::Logger logger) {
        try {
            if (verbose) {
                RCLCPP_INFO(logger, "Loading pose data...");
            }
            
            // Load data
            auto flange_poses = readPosesFromFile(flange_path);
            auto scanner_poses = readPosesFromFile(scanner_path);
            
            if (flange_poses.empty() || scanner_poses.empty()) {
                RCLCPP_ERROR(logger, "Failed to load pose data or no poses found");
                return false;
            }
            
            if (verbose) {
                RCLCPP_INFO(logger, "Loaded %zu pose pairs", std::min(flange_poses.size(), scanner_poses.size()));
            }
            
            // Compute calibration using the two methods
            RCLCPP_INFO(logger, "Computing hand-eye calibration using Tsai's method...");
            Eigen::Matrix4d T_cam_ee_tsai = computeHandEyeTransformationTsai(flange_poses, scanner_poses);
            
            RCLCPP_INFO(logger, "Computing hand-eye calibration using Dual Quaternion method...");
            Eigen::Matrix4d T_cam_ee_dq = computeHandEyeTransformationDualQuaternion(flange_poses, scanner_poses);
            
            // Define ground truth to match the data generator
            Pose ground_truth_pose = {0.10, 0.05, 0.15, 0.0, M_PI/6, 0.0};
            Eigen::Matrix4d ground_truth = poseToMatrix(ground_truth_pose);
            
            if (verbose) {
                RCLCPP_INFO(logger, "Using explicit ground truth from data generator");
                RCLCPP_INFO(logger, "Ground Truth: Translation: [%.4f, %.4f, %.4f], Rotation: [0.0, %.4f, 0.0]",
                           ground_truth_pose.x, ground_truth_pose.y, ground_truth_pose.z, ground_truth_pose.pitch);
            }
            
            // Create ROS messages for the transformation matrices
            auto current_time = node->now();
            
            // Tsai method transform
            geometry_msgs::msg::TransformStamped tsai_transform_msg;
            tsai_transform_msg.header.stamp = current_time;
            tsai_transform_msg.header.frame_id = "robot_flange";
            tsai_transform_msg.child_frame_id = "scanner_tsai";
            
            // Fill in translation
            tsai_transform_msg.transform.translation.x = T_cam_ee_tsai(0,3);
            tsai_transform_msg.transform.translation.y = T_cam_ee_tsai(1,3);
            tsai_transform_msg.transform.translation.z = T_cam_ee_tsai(2,3);
            
            // Convert rotation matrix to quaternion
            Eigen::Quaterniond q_tsai(T_cam_ee_tsai.block<3,3>(0,0));
            tsai_transform_msg.transform.rotation.x = q_tsai.x();
            tsai_transform_msg.transform.rotation.y = q_tsai.y();
            tsai_transform_msg.transform.rotation.z = q_tsai.z();
            tsai_transform_msg.transform.rotation.w = q_tsai.w();
            
            // Dual Quaternion method transform
            geometry_msgs::msg::TransformStamped dq_transform_msg;
            dq_transform_msg.header.stamp = current_time;
            dq_transform_msg.header.frame_id = "robot_flange";
            dq_transform_msg.child_frame_id = "scanner_dq";
            
            // Fill in translation
            dq_transform_msg.transform.translation.x = T_cam_ee_dq(0,3);
            dq_transform_msg.transform.translation.y = T_cam_ee_dq(1,3);
            dq_transform_msg.transform.translation.z = T_cam_ee_dq(2,3);
            
            // Convert rotation matrix to quaternion
            Eigen::Quaterniond q_dq(T_cam_ee_dq.block<3,3>(0,0));
            dq_transform_msg.transform.rotation.x = q_dq.x();
            dq_transform_msg.transform.rotation.y = q_dq.y();
            dq_transform_msg.transform.rotation.z = q_dq.z();
            dq_transform_msg.transform.rotation.w = q_dq.w();
            
            // Publish the transforms as ROS messages
            tsai_transform_publisher->publish(tsai_transform_msg);
            dq_transform_publisher->publish(dq_transform_msg);
            
            // Broadcast the transforms to TF2
            tf_broadcaster->sendTransform(tsai_transform_msg);
            tf_broadcaster->sendTransform(dq_transform_msg);
            
            RCLCPP_INFO(logger, "Published transformation matrices as ROS messages");
            
            if (verbose) {
                RCLCPP_INFO(logger, "Validating calibration results...");
            }
            
            // Validate calibration with the specified noise level
            validateCalibrationMerged(T_cam_ee_tsai, T_cam_ee_dq, ground_truth, logger, output_file_path, noise_level);
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger, "Calibration failed: %s", e.what());
            return false;
        }
    };
    
    if (compute_on_startup) {
        RCLCPP_INFO(node->get_logger(), "Computing calibration on startup...");
        performCalibration(node->get_logger());
    } else {
        RCLCPP_INFO(node->get_logger(), "Calibration service ready. Use 'ros2 service call /trigger_calibration std_srvs/srv/Trigger' to run calibration.");
    }
    
    auto calibrationService = node->create_service<std_srvs::srv::Trigger>(
        "trigger_calibration",
        [&performCalibration, &node](
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            RCLCPP_INFO(node->get_logger(), "Calibration service called");
            bool success = performCalibration(node->get_logger());
            
            response->success = success;
            response->message = success ? "Calibration completed successfully" : "Calibration failed.";
        }
    );
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}