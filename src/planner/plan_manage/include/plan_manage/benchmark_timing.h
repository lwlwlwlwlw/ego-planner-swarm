#ifndef EGO_PLANNER_BENCHMARK_TIMING_H
#define EGO_PLANNER_BENCHMARK_TIMING_H

// Optional, ROS-independent timing for one complete online planning decision.
// The caller supplies simulation time; elapsed time always uses steady_clock.
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <locale>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unistd.h>

namespace ego_planner
{
namespace benchmark
{

inline std::int64_t steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

[[noreturn]] inline void timingFailure(const std::string &path)
{
  const int error = errno;
  std::fprintf(stderr, "SWARM_BENCHMARK_TIMING_ERROR: %s: %s\n",
               path.c_str(), std::strerror(error));
  std::fflush(stderr);
  // A requested benchmark must never continue with incomplete timing output.
  std::_Exit(EXIT_FAILURE);
}

class TimingWriter
{
public:
  explicit TimingWriter(const std::string &path) : path_(path)
  {
    // Each agent owns a fresh file in a run directory created by the runner.
    // Refuse old runs and duplicate agents rather than append or truncate them.
    fd_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (fd_ < 0)
      timingFailure(path_);
  }

  ~TimingWriter() { ::close(fd_); }

  void append(const std::string &line)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t offset = 0;
    while (offset < line.size())
    {
      const ssize_t written = ::write(fd_, line.data() + offset, line.size() - offset);
      if (written < 0 && errno == EINTR)
        continue;
      if (written <= 0)
      {
        if (written == 0)
          errno = EIO;
        timingFailure(path_);
      }
      offset += static_cast<std::size_t>(written);
    }
    // POSIX write is unbuffered: a completed event is immediately readable.
  }

  static TimingWriter *get(const std::string &directory, int agent_id)
  {
    static std::mutex registry_mutex;
    static std::map<std::string, std::unique_ptr<TimingWriter>> writers;
    const std::string path = directory + "/timing_uav" + std::to_string(agent_id) + ".jsonl";
    std::lock_guard<std::mutex> lock(registry_mutex);
    auto found = writers.find(path);
    if (found == writers.end())
      found = writers.emplace(path, std::unique_ptr<TimingWriter>(new TimingWriter(path))).first;
    return found->second.get();
  }

private:
  int fd_ = -1;
  std::string path_;
  std::mutex mutex_;
};

class ExcludeVisualization;

enum class RecordType { Decision, TargetSetup, WarmupTargetSetup };

class DecisionTiming
{
public:
  template <typename WorldTime>
  DecisionTiming(int agent_id, WorldTime world_time,
                 RecordType record_type = RecordType::Decision)
      : agent_id_(agent_id), record_type_(record_type)
  {
    const char *directory = std::getenv("SWARM_BENCHMARK_TIMING_DIR");
    if (directory == nullptr || directory[0] == '\0' || active() != nullptr)
      return;

    // File creation and registry locking precede the measured decision.
    writer_ = TimingWriter::get(directory, agent_id);
    world_time_s_ = world_time();
    start_ns_ = steadyNowNs();
    active() = this;
  }

  ~DecisionTiming()
  {
    if (writer_ == nullptr)
      return;

    const std::int64_t end_ns = steadyNowNs();
    active() = nullptr;
    // Serialization, locking and disk writes all happen after end_ns.
    static std::atomic<std::uint64_t> sequence{0};
    std::ostringstream line;
    line.imbue(std::locale::classic());
    line << "{\"event_id\":\"ego-uav" << agent_id_ << '-' << ::getpid()
         << '-' << start_ns_ << '-' << sequence.fetch_add(1) << "\","
         << "\"world_time_s\":" << std::setprecision(17) << world_time_s_
         << ",\"record_type\":\"" << (record_type_ == RecordType::Decision ? "decision" : "target_setup") << '"'
         << ",\"phase\":\"" << (record_type_ == RecordType::WarmupTargetSetup ? "warmup" : "online")
         << "\",\"agent_ids\":[" << agent_id_ << ']'
         << ",\"start_ns\":" << start_ns_ << ",\"end_ns\":" << end_ns
         << ",\"excluded_ns\":" << excluded_ns_
         << ",\"duration_ns\":" << end_ns - start_ns_ - excluded_ns_
         << ",\"success\":" << (success_ ? "true" : "false")
         << ",\"device\":\"cpu\",\"synchronized\":true}\n";
    writer_->append(line.str());
  }

  void setSuccess() { success_ = true; }

  DecisionTiming(const DecisionTiming &) = delete;
  DecisionTiming &operator=(const DecisionTiming &) = delete;

private:
  friend class ExcludeVisualization;

  static DecisionTiming *&active()
  {
    static thread_local DecisionTiming *current = nullptr;
    return current;
  }

  TimingWriter *writer_ = nullptr;
  int agent_id_;
  RecordType record_type_;
  double world_time_s_ = 0.0;
  std::int64_t start_ns_ = 0;
  std::int64_t excluded_ns_ = 0;
  std::int64_t exclusion_start_ns_ = 0;
  unsigned exclusion_depth_ = 0;
  bool success_ = false;
};

// Pause only for explicit rendering. Nested exclusions form a union of time
// intervals, so the same rendering time can never be subtracted twice.
class ExcludeVisualization
{
public:
  ExcludeVisualization() : timing_(DecisionTiming::active())
  {
    if (timing_ != nullptr && timing_->exclusion_depth_++ == 0)
      timing_->exclusion_start_ns_ = steadyNowNs();
  }

  ~ExcludeVisualization()
  {
    if (timing_ != nullptr && --timing_->exclusion_depth_ == 0)
      timing_->excluded_ns_ += steadyNowNs() - timing_->exclusion_start_ns_;
  }

  ExcludeVisualization(const ExcludeVisualization &) = delete;
  ExcludeVisualization &operator=(const ExcludeVisualization &) = delete;

private:
  DecisionTiming *timing_;
};

} // namespace benchmark
} // namespace ego_planner

#endif // EGO_PLANNER_BENCHMARK_TIMING_H
