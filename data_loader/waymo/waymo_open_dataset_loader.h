#ifndef DATA_LOADER_WAYMO_OPEN_DATASET_LOADER_H_
#define DATA_LOADER_WAYMO_OPEN_DATASET_LOADER_H_

#include <fstream>

#include "absl/status/statusor.h"
#include "waymo_open_dataset/dataset.pb.h"

namespace data_loader {
namespace waymo {
class WaymoOpenDatasetLoader {
public:
  WaymoOpenDatasetLoader() = default;
  ~WaymoOpenDatasetLoader() = default;

  absl::Status ReadTfRecord(const std::string &file_path);
  absl::StatusOr<::waymo::open_dataset::Frame> NextFrame();

private:
  std::unique_ptr<std::ifstream> file_stream_;
};

} // namespace waymo
} // namespace data_loader

#endif
