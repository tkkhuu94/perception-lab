#include <filesystem>

#include "absl/strings/str_format.h"

#include "data_loader/waymo_open_dataset_loader.h"

namespace data_loader {

absl::Status
WaymoOpenDatasetLoader::ReadTfRecord(const std::string &file_path) {
  if (!std::filesystem::exists(file_path)) {
    return absl::NotFoundError(
        absl::StrFormat("File does not exist %s", file_path));
  }

  file_stream_ = std::make_unique<std::ifstream>(file_path, std::ios::binary);
  if (!file_stream_->is_open()) {
    return absl::InternalError(absl::StrFormat("Failed to open %s", file_path));
  }

  return absl::OkStatus();
}

absl::StatusOr<waymo::open_dataset::Frame> WaymoOpenDatasetLoader::NextFrame() {
  if (file_stream_ == nullptr) {
    return absl::UnavailableError(
        "Loader is not initialized with any TfRecord file");
  }

  if (!file_stream_->good()) {
    return absl::InternalError("File stream is in bad state");
  }

  if (file_stream_->peek() == EOF) {
    return absl::OutOfRangeError("End of file reached");
  }

  uint64_t length;
  uint32_t masked_crc_len;

  // Read length header
  file_stream_->read(reinterpret_cast<char *>(&length), sizeof(length));
  if (file_stream_->gcount() == 0) {
    return absl::OutOfRangeError("EOF");
  }

  if (file_stream_->gcount() != sizeof(length)) {
    return absl::DataLossError("Failed to read length header");
  }

  // Read CRC of length
  file_stream_->read(reinterpret_cast<char *>(&masked_crc_len),
                     sizeof(masked_crc_len));

  // Read payload
  std::vector<char> buffer(length);
  file_stream_->read(buffer.data(), length);
  if (file_stream_->gcount() != static_cast<std::streamsize>(length)) {
    return absl::DataLossError("Failed to read full payload");
  }

  // Read CRC of data
  uint32_t masked_crc_data;
  file_stream_->read(reinterpret_cast<char *>(&masked_crc_data),
                     sizeof(masked_crc_data));

  // Parse Proto
  waymo::open_dataset::Frame frame;
  if (!frame.ParseFromArray(buffer.data(), length)) {
    return absl::DataLossError("Failed to parse Frame proto");
  }

  return frame;
}

} // namespace data_loader
