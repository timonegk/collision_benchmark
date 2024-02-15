#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <numeric>

class BenchmarkLogger {
 public:
  explicit BenchmarkLogger(const std::string& file_name) {
    data_file_.open(file_name, std::ios::app);
    data_file_ << "trial,found_ik,solve_time,position_error,orientation_error\n";
  }

  ~BenchmarkLogger() {
    data_file_.close();
  }

  void log_data(int trial, bool found_ik, long solve_time, double position_error, double orientation_error) {
    data_file_ << std::boolalpha << trial << "," << found_ik << "," << solve_time << ","
               << position_error << "," << orientation_error << "\n";
    solve_times_.push_back(solve_time);
    success_.push_back(found_ik);
    count_++;
  }

  double get_average_solve_time() {
    return std::accumulate(solve_times_.begin(), solve_times_.end(), 0.0) / solve_times_.size();
  }

  double get_success_rate() {
    return std::accumulate(success_.begin(), success_.end(), 0.0) / success_.size();
  }

  double get_average_successful_solve_time() {
    double sum = 0;
    int count = 0;
    for (int i = 0; i < count_; i++) {
      if (success_[i]) {
        sum += solve_times_[i];
        count++;
      }
    }
    return sum / count;
  }

 private:
  std::ofstream data_file_;
  std::vector<long> solve_times_;
  int count_ = 0;
  std::vector<bool> success_;
};