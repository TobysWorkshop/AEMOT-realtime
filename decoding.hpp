// OpenCV
#include <eigen3/Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <fstream>
#include <iomanip>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include "parameters_decoding.h"
#include "yaml-cpp/yaml.h"
#pragma once

class Decoding
{

public:
  /**
   * Create a Kalman filter with the specified matrices.
   *   F - System dynamics matrix
   *   C - Output matrix
   *   Q - Process noise covariance
   *   R - Measurement noise covariance
   *   P - Estimate error covariance
   */
  Decoding(double dt, const std::string &data_name, const std::string &input_event_path, const double &contrast_threshold, int id);

  /**
   * Create a blank estimator.
   */
  Decoding();

  /**
   * Initialize the filter with a guess for initial states.
   */
  void init(double ts, std::string output_decode_path);

  /**
   * Update the estimated state based on measured values. The
   * time step is assumed to remain constant.
   */

  double get_comms_range();

  /**
   * Return the current state and time.
   */
  // Eigen::VectorXd state() { return x_hat; };
  // double time() { return t; };
  // double P_lambda() { return P(4, 4); };
  // Eigen::MatrixXd P_x() { return P.block(0, 0, 2, 2); };
  void high_pass_decoding(const double &ts, const int &p, const double &track_x, const double &track_y, const double &track_vx, const double &track_vy);
  void get_msg(const double &ts, const int &p, const double &track_x, const double &track_y, const double &track_vx, const double &track_vy);
  void process_peak(std::pair<double, double> &peak_, const double &timestamp, const double &value);

  ///////////////////////////////////////////
  void open_G_file();
  ///////////////////////////////////////////

private:
  int id;
  Eigen::MatrixXd F_test;

  std::vector<double> demod_;
  std::vector<double> demod_t_;
  std::vector<int> output_bin_;
  double base_frequency;
  double unit_T;
  bool initialised_;
  cv::Mat log_intensity_state_;
  cv::Mat ts_array_;
  double t_next_publish_;
  double t_next_log_intensity_update_;
  int camera_height;
  int camera_width;
  double contrast_threshold_on_user_defined_ = 0.1;
  double contrast_threshold_off_user_defined_ = -0.1;
  double publish_framerate_ = 60;

  double t_stop_right_limit;
  double t_stop_left_limit;
  // double t_stop_left_limit_slide;

  // Below are the variables used for finding the beginning of the binary message.
  bool found_start = false;
  double t2;
  double t_start;
  double t_end;
  int t_end_i;
  bool new_event = false;
  bool found_near_start = false;
  double t_stop;
  int t_stop_i;

  std::string buffer_string;
  std::ofstream output_msg_;

  // add TODO:
  double hp_dt_ = 0;
  double hp_t_prev_ = 0;
  double G_ = 0;
  double hp_ = 0;



  bool flag_save_DI = 1;
  bool flag_save_G = 1;

  // Not sure
  double peak = 0;
  std::deque<int> decode_binary;
  std::deque<double> raw_signal;
  std::deque<double> raw_signal_t;
  int decimal = 0;
  int count_one = 0;
  int last_peak = 1;
  double threshold, unitT, unitT_07;
  int windowSize;
  std::pair<double, double> peak_ = {0, 0};
  double last_decode_peak_t_ = 0;
  double last_last_decode_peak_t_ = 0;
  double last_letter_end_t, current_letter_end_t;
  int grid_size = 4;
  int wrong_parity_count = 0;
  double last_correct_msg_ts = 0;
  bool flag_reset_lose_track = 0;
  bool flag_full_tracker;
  double last_G_ = 1e6; // any large value
  double last_ts_ = 0;
  std::ofstream DI_txt_, G_txt_, G_txt_peak_, G_txt_sg_;
  std::string output_decode_path;
  double stop_bit_t_ = -1;
  double start_bit_t_ = -1;
  double alpha_decoding;
  double contrast_threshold;
  double comms_range;
  Eigen::MatrixXd x_hat_track;

  // peaks
  double lastPeakTimestamp_ = 0.0;
  std::deque<double> signal_;
  std::deque<double> smoothedSignal_;
  std::deque<double> smoothedSignal_t_;
  std::deque<double> localWindow;


  ///////////////////////////////////////////
  std::string G_filepath = "";
  ///////////////////////////////////////////

};