#include "data_loader/waymo/utility.h"

#include <zlib.h>

namespace data_loader {
namespace waymo {

absl::StatusOr<std::string> Decompress(const std::string &compressed) {
  if (compressed.empty()) {
    return "";
  }

  // 1. Prepare Buffer
  // Use a safer, incremental expansion strategy.
  // We start with a reasonably large buffer (2MB) to avoid resizing for most frames.
  size_t buffer_size = 2 * 1024 * 1024; 
  std::string decompressed;
  decompressed.resize(buffer_size);

  // 2. Setup Zlib Stream
  z_stream strm;
  strm.zalloc = Z_NULL;
  strm.zfree = Z_NULL;
  strm.opaque = Z_NULL;
  strm.avail_in = compressed.size();
  strm.next_in = (Bytef *)compressed.data();
  strm.avail_out = decompressed.size();
  strm.next_out = (Bytef *)decompressed.data();

  // Use 15 + 32 to enable zlib and gzip decoding with automatic header detection
  if (inflateInit2(&strm, 15 + 32) != Z_OK) {
    return absl::InternalError("Failed to initialize zlib");
  }

  // 3. Decompression Loop (Standard Z_NO_FLUSH approach)
  // We loop until the stream is done or an error occurs.
  int ret;
  do {
    // If output buffer is full, resize it
    if (strm.avail_out == 0) {
      size_t old_size = decompressed.size();
      size_t new_size = old_size * 2;
      
      // Safety Cap (256MB)
      if (new_size > 256 * 1024 * 1024) {
        inflateEnd(&strm);
        return absl::ResourceExhaustedError("Decompressed data exceeds 256MB limit");
      }

      decompressed.resize(new_size);
      
      // Crucial: Update zlib's output pointer. 
      // Resizing std::string invalidates previous pointers.
      strm.next_out = (Bytef *)(decompressed.data() + old_size);
      strm.avail_out = new_size - old_size;
    }

    ret = inflate(&strm, Z_NO_FLUSH);

    if (ret == Z_STREAM_ERROR || ret == Z_NEED_DICT || ret == Z_DATA_ERROR || ret == Z_MEM_ERROR) {
      inflateEnd(&strm);
      return absl::DataLossError("Zlib decompression error: " + std::to_string(ret));
    }

  } while (ret != Z_STREAM_END);

  // 4. Final Cleanup
  // Resize to the actual amount of data written
  if (decompressed.size() != strm.total_out) {
      decompressed.resize(strm.total_out);
  }
  
  inflateEnd(&strm);
  return decompressed;
}

absl::StatusOr<pcl::PointCloud<pcl::PointXYZI>::Ptr>
ToPointCloud(const ::waymo::open_dataset::Laser &laser,
             const ::waymo::open_dataset::LaserCalibration &calibration) {

  if (laser.name() != calibration.name()) {
    return absl::InvalidArgumentError(
        "Laser and calibration have different name");
  }

  auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  if (!laser.has_ri_return1()) {
    return point_cloud;
  }

  const auto decompressed_data =
      Decompress(laser.ri_return1().range_image_compressed());

  if (!decompressed_data.ok()) {
    return decompressed_data.status();
  }

  if (decompressed_data->empty()) {
    std::cout << "Decompressed data is empty\n";
  }

  ::waymo::open_dataset::MatrixFloat matrix;
  if (!matrix.ParseFromString(*decompressed_data)) {
    return absl::InternalError("Failed to parse decompressed data into matrix");
  }

  const auto height = matrix.shape().dims(0);
  const auto width = matrix.shape().dims(1);
  const auto channels =
      (matrix.shape().dims_size() > 2) ? matrix.shape().dims(2) : 1;
      const auto& tf = calibration.extrinsic().transform(); // 4x4 Row-Major flattened

      

  auto get_idx = [&](int r, int c, int ch) {
    return (r * width + c) * channels + ch;
  };

  int valid_points = 0;

  for (int r = 0; r < height; ++r) {
    // Waymo stores beam inclinations in the calibration proto
    // Check bounds of beam_inclinations
    if (r >= calibration.beam_inclinations_size()) {
      continue;
    }

    const float inclination = calibration.beam_inclinations(r);
    const float cos_incl = std::cos(inclination);
    const float sin_incl = std::sin(inclination);

    for (int c = 0; c < width; ++c) {
      float range = matrix.data(get_idx(r, c, 0));

      // Filter invalid points (range < 0 or is_in_nlz set)
      if (range <= 0.0f) {
        continue;
      }
      if (channels > 3 && matrix.data(get_idx(r, c, 3)) > 0.0f) {
        continue;
      }

      valid_points++;


      // Approximate Azimuth: Map column 0..Width to [-PI, PI]
      // Note: For precise projection, one should use the 'range_image_pose'
      // to correct for vehicle motion (rolling shutter) and exact sensor frame.
      float azimuth = (static_cast<float>(c) / width) * 2.0f * M_PI - M_PI;
      float x_s = range * cos_incl * std::cos(azimuth);
      float y_s = range * cos_incl * std::sin(azimuth);
      float z_s = range * sin_incl;

      pcl::PointXYZI point;
      point.x = tf[0] * x_s + tf[1] * y_s + tf[2] * z_s + tf[3];
      point.y = tf[4] * x_s + tf[5] * y_s + tf[6] * z_s + tf[7];
      point.z = tf[8] * x_s + tf[9] * y_s + tf[10] * z_s + tf[11];

      point.intensity = (channels > 1) ? matrix.data(get_idx(r, c, 1)) : 0.0f;

      point_cloud->points.push_back(point);
    }
  }

  point_cloud->width = point_cloud->points.size();
  point_cloud->height = 1;
  point_cloud->is_dense = false;

  return point_cloud;
}

} // namespace waymo
} // namespace data_loader
