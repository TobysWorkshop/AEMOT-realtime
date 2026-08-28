#ifndef PARAMETERS_H
#define PARAMETERS_H
#include "yaml-cpp/yaml.h"
#include <string>

struct Parameters
{
  int width;
  int height;
  int data_format;
  double dist_threshold;
  int n_target;
  int n_state;
  int m;
  int publish_framerate;
  double dt;
  double contrast_threshold;
  int alpha;
  int use_gyro_flag;
  int save_video_flag;
  int save_image_flag;
  int disp_covariance_flag;
  int ref_image_ts_flag;
  double process_ts_start;
  double process_ts_end;
  int event_num_start;
  double event_num_end;
  int print_TTC_flag;
  int compute_TTC_flag;
  int save_track_flag;
  int ring_buffer_len;
  std::string input_data_name;
  std::string input_folder_path;
  double var_x;
  double var_y;
  double var_vx;
  double var_vy;
  double var_lambda_1;
  double var_lambda_2;
  double var_theta;
  double var_q;
  double q_x;
  double q_y;
  double q_vx;
  double q_vy;
  double q_lambda_1;
  double q_lambda_2;
  double q_theta;
  double q_q;
  std::vector<double> position_x_init;
  std::vector<double> position_y_init;
  double lambda_init;
  int select_target_flag;

  int accumulator_count_thresh;
  double accumulator_time_thresh;

  // Parameters for detector
  int SAE_operation_rate;
  int SAE_ksize;
  double SAE_alpha;
  double SAE_angle_thresh;
  double SAE_min_contributions;
  double SAE_detection_threshold;
  double SAE_min_active_pixels;
  int MAX_NUMBER_OF_TRACKS;
  double SAE_recency_window;
  double event_rate_threshold;
  int f_show_candidates;
  int f_evaluate;

  double detector_dist_threshold;
  double detector_dt_threshold;
  double evaluate_ts_age;
  double evaluate_dt_terminate;
  double evaluate_low_activity_factor;

  int pool_size;
  int use_dt_detector;
  double rate_per_area;

  int show_display;
};

struct DisplayParams
{
  int save_video_flag;
  int save_image_flag;
  int disp_covariance_flag;
};

Parameters loadParametersFromYAML(const std::string &yaml_file_path);
DisplayParams loadDispParametersFromYAML(const std::string &yaml_file_path);

#endif // PARAMETERS_H