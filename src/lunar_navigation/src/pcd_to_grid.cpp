#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

struct Options
{
  fs::path input = fs::path(LUNAR_WORKSPACE_ROOT) / "maps/site/map.pcd";
  fs::path output = fs::path(LUNAR_WORKSPACE_ROOT) / "maps/site/map";
  double resolution = 0.05;
  double ground_z = std::numeric_limits<double>::quiet_NaN();
  double min_height = 0.10;
  double max_height = 0.65;
  double ground_tolerance = 0.04;
  double margin = 0.5;
};

void help(const Options & defaults)
{
  std::cout << "Offline flat-ground PCD to Nav2 map.yaml/map.pgm (PCD is unchanged).\n"
            << "Usage: pcd_to_grid [--input FILE] [--output PREFIX] [options]\n"
            << "  --input FILE          Default: " << defaults.input << '\n'
            << "  --output PREFIX       Default: " << defaults.output << '\n'
            << "  --resolution METRES   Cell size (default 0.05)\n"
            << "  --ground-z METRES     Horizontal ground override; default fits a plane\n"
            << "  --min-height METRES   Obstacle height above ground (default 0.10)\n"
            << "  --max-height METRES   Obstacle height limit (default 0.65)\n"
            << "  --ground-tolerance M  Ground support band (default 0.04)\n"
            << "  --margin METRES       Unknown border (default 0.5)\n";
}

Options parse(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string key(argv[i]);
    if (key == "--help" || key == "-h") {
      help(options);
      std::exit(0);
    }
    if (i + 1 >= argc) {
      throw std::runtime_error("Missing value for " + key);
    }
    const std::string value(argv[++i]);
    if (key == "--input") {options.input = value; continue;}
    if (key == "--output") {options.output = value; continue;}
    std::size_t end = 0;
    const double number = std::stod(value, &end);
    if (end != value.size() || !std::isfinite(number)) {
      throw std::runtime_error("Expected a finite number for " + key);
    }
    if (key == "--resolution") {options.resolution = number;}
    else if (key == "--ground-z") {options.ground_z = number;}
    else if (key == "--min-height") {options.min_height = number;}
    else if (key == "--max-height") {options.max_height = number;}
    else if (key == "--ground-tolerance") {options.ground_tolerance = number;}
    else if (key == "--margin") {options.margin = number;}
    else {throw std::runtime_error("Unknown argument: " + key);}
  }
  if (options.resolution <= 0 || options.ground_tolerance <= 0 || options.margin < 0 ||
    options.min_height <= options.ground_tolerance || options.max_height < options.min_height)
  {
    throw std::runtime_error(
            "Require resolution/tolerance > 0, margin >= 0, max-height >= min-height > tolerance");
  }
  return options;
}

std::array<double, 4> ground_plane(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const Options & options)
{
  if (std::isfinite(options.ground_z)) {
    return {0.0, 0.0, 1.0, -options.ground_z};
  }
  // The mapping-start IMU is near (0,0,0). Fit observed ground below it, not the ceiling.
  auto nearby = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const auto & point : cloud) {
    if (pcl::isFinite(point) && point.z < 0 &&
      point.x * point.x + point.y * point.y <= 9.0f)
    {
      nearby->push_back(point);
    }
  }
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(nearby);
  voxel.setLeafSize(options.resolution, options.resolution, options.resolution);
  auto sample = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  voxel.filter(*sample);
  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setAxis(Eigen::Vector3f::UnitZ());
  segmentation.setEpsAngle(10.0 * std::acos(-1.0) / 180.0);
  segmentation.setDistanceThreshold(options.ground_tolerance);
  segmentation.setMaxIterations(200);
  segmentation.setInputCloud(sample);
  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  segmentation.segment(inliers, coefficients);
  if (inliers.indices.size() < 3 || coefficients.values.size() != 4) {
    throw std::runtime_error(
            "Cannot fit ground below the starting IMU within 3m; provide --ground-z METRES");
  }
  std::array<double, 4> plane;
  for (std::size_t i = 0; i < plane.size(); ++i) {
    plane[i] = coefficients.values[i];
  }
  if (plane[2] < 0) {
    for (auto & value : plane) {value = -value;}
  }
  return plane;
}

int main(int argc, char ** argv)
{
  try {
    const Options options = parse(argc, argv);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(options.input.string(), cloud) < 0 || cloud.empty()) {
      throw std::runtime_error("Cannot read a nonempty PCD: " + options.input.string());
    }
    const auto plane = ground_plane(cloud, options);
    auto classify = [&](const pcl::PointXYZ & point) -> int {
        if (!pcl::isFinite(point)) {return -1;}
        // Height above the fitted plane, without moving/rotating map XY coordinates.
        const double h = (plane[0]*point.x + plane[1]*point.y +
          plane[2]*point.z + plane[3]) / plane[2];
        if (std::abs(h) <= options.ground_tolerance) {return 254;}
        if (h >= options.min_height && h <= options.max_height) {return 0;}
        return -1;
      };
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = min_x;
    double max_x = -min_x;
    double max_y = -min_x;
    for (const auto & point : cloud) {
      if (classify(point) < 0) {continue;}
      min_x = std::min(min_x, double(point.x));
      min_y = std::min(min_y, double(point.y));
      max_x = std::max(max_x, double(point.x));
      max_y = std::max(max_y, double(point.y));
    }
    if (!std::isfinite(min_x)) {
      throw std::runtime_error("No ground/obstacle observations within the chosen height bands");
    }
    const double padding = std::ceil(options.margin / options.resolution);
    const double first_x = std::floor(min_x / options.resolution) - padding;
    const double first_y = std::floor(min_y / options.resolution) - padding;
    const double cols = std::floor(max_x / options.resolution) + padding - first_x + 1;
    const double rows = std::floor(max_y / options.resolution) + padding - first_y + 1;
    // Outlier coordinates must not turn a small map into an unbounded allocation.
    if (!std::isfinite(cols * rows) || cols * rows > 100000000.0) {
      throw std::runtime_error(
              "Grid exceeds 100M cells; remove outliers or increase --resolution");
    }
    const auto width = static_cast<std::size_t>(cols);
    const auto height = static_cast<std::size_t>(rows);
    std::vector<std::uint8_t> image(width * height, 205);
    for (const auto & point : cloud) {
      const int value = classify(point);
      if (value < 0) {continue;}
      const auto x = static_cast<std::size_t>(
        std::floor(point.x / options.resolution) - first_x);
      const auto y = static_cast<std::size_t>(
        std::floor(point.y / options.resolution) - first_y);
      auto & cell = image[(height - 1 - y) * width + x];
      if (value == 0 || cell != 0) {cell = static_cast<std::uint8_t>(value);}
    }
    fs::create_directories(fs::absolute(options.output).parent_path());
    const fs::path pgm(options.output.string() + ".pgm");
    const fs::path yaml(options.output.string() + ".yaml");
    std::ofstream image_file(pgm, std::ios::binary);
    image_file << "P5\n" << width << ' ' << height << "\n255\n";
    image_file.write(reinterpret_cast<const char *>(image.data()), image.size());
    std::ofstream yaml_file(yaml);
    yaml_file << std::setprecision(17)
              << "image: " << std::quoted(pgm.filename().string()) << '\n'
              << "mode: trinary\nresolution: " << options.resolution << '\n'
              << "origin: [" << first_x * options.resolution << ", "
              << first_y * options.resolution << ", 0.0]\n"
              << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
    image_file.close();
    yaml_file.close();
    if (!image_file || !yaml_file) {throw std::runtime_error("Failed to write map output");}
    std::cout << "Ground z at map origin: " << -plane[3]/plane[2] << " m\n"
              << "Grid " << width << 'x' << height << " at " << options.resolution << " m/cell\n"
              << "Free: " << std::count(image.begin(), image.end(), 254)
              << ", occupied: " << std::count(image.begin(), image.end(), 0)
              << ", unknown: " << std::count(image.begin(), image.end(), 205) << '\n'
              << "Saved " << fs::absolute(yaml) << " and " << fs::absolute(pgm) << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "pcd_to_grid: " << error.what() << '\n';
    return 1;
  }
}
