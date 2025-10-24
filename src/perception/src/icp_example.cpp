#include <cmath>
#include <iostream>
#include <random>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr createTargetCloud() {
  const int steps_per_axis = 25;
  const float spacing = 0.05f;
  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->points.reserve(steps_per_axis * steps_per_axis);
  const float offset = 0.5f * spacing * static_cast<float>(steps_per_axis - 1);
  for (int i = 0; i < steps_per_axis; ++i) {
    for (int j = 0; j < steps_per_axis; ++j) {
      const float x = -offset + spacing * static_cast<float>(i);
      const float y = -offset + spacing * static_cast<float>(j);
      const float z = 0.1f * std::sin(3.0f * x) * std::cos(3.0f * y);
      cloud->points.emplace_back(x, y, z);
    }
  }
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

void addGaussianNoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  std::mt19937 rng(42u);
  std::normal_distribution<float> distribution(0.0f, 0.002f);
  for (auto& point : cloud->points) {
    point.x += distribution(rng);
    point.y += distribution(rng);
    point.z += distribution(rng);
  }
}

}

int main() {
  auto target_cloud = createTargetCloud();

  Eigen::Affine3f ground_truth = Eigen::Affine3f::Identity();
  ground_truth.translation() << 0.45f, -0.2f, 0.05f;
  ground_truth.rotate(Eigen::AngleAxisf(0.35f, Eigen::Vector3f::UnitZ()));

  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*target_cloud, *source_cloud, ground_truth);

  addGaussianNoise(source_cloud);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source_cloud);
  icp.setInputTarget(target_cloud);
  icp.setMaximumIterations(80);
  icp.setTransformationEpsilon(1e-9f);
  icp.setEuclideanFitnessEpsilon(1e-6f);
  icp.setMaxCorrespondenceDistance(0.5f);

  Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
  initial_guess(0, 3) = 0.1f;
  initial_guess(1, 3) = -0.05f;
  initial_guess.block<3, 3>(0, 0) = (Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitZ())).toRotationMatrix();

  pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
  icp.align(aligned_cloud, initial_guess);

  if (!icp.hasConverged()) {
    std::cerr << "ICP failed to converge." << std::endl;
    return 1;
  }

  const Eigen::Matrix4f estimated = icp.getFinalTransformation();
  const Eigen::Matrix4f source_to_target = ground_truth.inverse().matrix();
  const Eigen::Matrix4f residual = estimated * ground_truth.matrix();

  std::cout << "ICP converged. Fitness score: " << icp.getFitnessScore() << std::endl;
  std::cout << "\nEstimated source-to-target transformation:\n" << estimated << std::endl;
  std::cout << "\nGround truth source-to-target transformation:\n" << source_to_target << std::endl;
  std::cout << "\nResidual (should be close to identity):\n" << residual << std::endl;

  return 0;
}
