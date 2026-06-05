#include "FileLogWriter.hpp"

#include <logger/LogParameters.hpp>

#include <easylogging++.h>

#include <filesystem>
#include <stdexcept>

namespace logger {

FileLogWriter::FileLogWriter(const std::string &outputPath,
                             const logger::LogParameters &parameters) {
  open(outputPath, parameters);
}

void FileLogWriter::open(const std::string &outputPath,
                         const logger::LogParameters &parameters) {
  if (outputPath.empty()) {
    throw std::runtime_error("Log output path is empty");
  }

  const std::filesystem::path path{outputPath};
  if (const auto parent = path.parent_path(); !parent.empty()) {
    std::filesystem::create_directories(parent);
  }

  _outputStream.open(outputPath);
  if (!_outputStream) {
    throw std::runtime_error("Failed to open log output: " + outputPath);
  }

  LOG(INFO) << "Logger CSV output: " << outputPath;
  _outputStream << "Time (sec),";
  for (const auto &param : parameters.parameters()) {
    _outputStream << param.description() << "(" << param.unit() << ") "
                  << param.name() << ",";
  }
  _outputStream << std::endl;
}

void FileLogWriter::onLogMessage(std::chrono::milliseconds timePoint,
                                 const std::vector<double> &values) {
  _outputStream << (timePoint.count() / 1000.0) << ",";

  for (const auto value : values) {
    _outputStream << value << ",";
  }
  _outputStream << std::endl;
}

} // namespace logger
