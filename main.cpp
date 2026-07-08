
// This file is licensed under the Apache License, Version 2.0,
// with additional restrictions under the Commons Clause.
// See the LICENSE file for more details.

#include "yaml-cpp/yaml.h"
#include <eigen3/Eigen/Dense>
#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <math.h>
// #include <opencv4/opencv2/core/core.hpp>
// #include <opencv4/opencv2/highgui/highgui.hpp>
// #include <opencv4/opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>
#include <queue>
#include <chrono>

#include "kalman.hpp"
#include "parameters.h"
#include "decoding.hpp"
#include "TrackManager.hpp"
#include "SAEdetector.hpp"

// Global variables
double TTC, contrast_threshold, first_ts_flag;
int n_target, n_state, image_count, image_id, start_event_ts;
double t1;
double t_next_publish = 0;
int EventId = 0;
double ts_kf_last = 0;
double gyro_ts = 0;
double gyro_ts_last = 0;
double next_save_traj_ts = 0;
double dt_save_traj = 0.0001; // Output lower rate trajectory for plotting // TODO: add to config
double alpha_dist = 1.4;
bool first_gyro_flag = true;
bool use_gyro_flag;
Eigen::MatrixXd x_hat_track; // current target track
Eigen::MatrixXd P_track;     // current target P
// Choose two target ID to compute time-to-contact; default id=0 and id=1
int id_ttc_left = 0;
int id_ttc_right = 1;

std::queue<std::pair<int, std::pair<int, int>>> same_ts_e_buffer;
cv::Mat log_intensity_state, ts_array;
Eigen::MatrixXd F, C, R, P, Q, A, P_x, x0, x_hat;
std::ofstream output_txt_;
std::string input_event_path, output_image_path;
cv::Point selectedPoint(-1, -1);
std::vector<cv::Point> selectedPoints;
std::vector<cv::Point> selectedPointsSpeed;
cv::VideoWriter writer_ref, writer;


double ts_global = 0;
//--------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------//

//--------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------//

void onMouse(int event, int x, int y, int flags, void *userdata)
{
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    selectedPoint = cv::Point(x, y);
  }
}
void high_pass(double ts, int x, int y, int p, int &alpha);
void high_pass_global(double ts, int &alpha);
void init(Parameters &params);
void read_events(std::stringstream &ss, const int data_format, double &ts,
                 int &c, int &r, int &p);
void display(const DisplayParams &dispParams, Parameters &params,
             const double &ts, std::vector<cv::Mat> &output_video,
             std::vector<cv::Mat> &output_video_ref,
             std::vector<double> &image_ref_ts, int p, std::vector<KalmanFilter *> current_tracks);
void display_for_select();
void load_gyro(std::ifstream &my_gyro, TrackManager &track_manager);

int main(int argc, char *argv[])
{

  std::string data_name = argv[2];
  Parameters params;
  DisplayParams dispParams;

  try
  {
    std::string file_config_path = "../configs/" + data_name + ".yaml";
    params = loadParametersFromYAML(file_config_path);
    dispParams = loadDispParametersFromYAML(file_config_path);
  }
  catch (const std::exception &ex)
  {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1;
  }

  log_intensity_state = cv::Mat::zeros(params.height, params.width, CV_64FC1);
  ts_array = cv::Mat::zeros(params.height, params.width, CV_64FC1);
  use_gyro_flag = params.use_gyro_flag;

  contrast_threshold = params.contrast_threshold;
  n_target = params.n_target;
  n_state = params.n_state;
  input_event_path = params.input_folder_path + params.input_data_name;
  std::ofstream track_output_txt_, TTC_txt_, blob_measurements_txt_;
  double distance = 0;
  int id = 0;
  std::string input_file_name = input_event_path + ".csv";

  // Read image timestamp
  std::vector<double> image_ref_ts;
  if (params.ref_image_ts_flag)
  {
    std::ifstream ImageTimefile(params.input_folder_path + "image_ts.txt");
    if (ImageTimefile)
    {
      std::string line;
      while (std::getline(ImageTimefile, line))
      {
        std::stringstream ss(line);
        std::string value;
        if (std::getline(ss, value, ','))
        {
          double timestamp = std::stod(value);
          image_ref_ts.push_back(timestamp);
        }
      }
      ImageTimefile.close();
    }
    else
    {
      std::cout << "Failed to open the image timestamp file." << std::endl;
      return 1;
    }
  }

  // Save video with tracks
  cv::namedWindow("Video");
  cv::setMouseCallback("Video", onMouse);
  std::vector<cv::Mat> output_video;
  std::vector<cv::Mat> output_video_ref;
  std::string output_ref_video_path = input_event_path + "_video_ref.avi";
  std::string output_video_path = input_event_path + "_video.avi";
  if (params.save_video_flag)
  {
    writer.open(output_video_path,
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 40.0,
                cv::Size(params.width, params.height));

    if (params.ref_image_ts_flag)
    {
      writer_ref.open(output_ref_video_path,
                      cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 40.0,
                      cv::Size(params.width - 1, params.height - 1));
    }
  }

  // Display reference image
  if (params.ref_image_ts_flag)
  {
    cv::namedWindow("Video_ref", cv::WINDOW_NORMAL);
  }

  if (params.save_image_flag)
  {
    std::filesystem::path path(input_event_path + "/output_img");
    std::filesystem::create_directories(path);
    output_image_path = path.string();
  }

  // if (params.save_track_flag)
  // {
  //   std::string output_info_path = input_event_path + "_traj.csv";
  //   track_output_txt_.open(output_info_path);
  // }
  std::map<int, std::ofstream> track_output_files;

  if (params.compute_TTC_flag)
  {
    std::string output_TTC_path = input_event_path + "_ttc.txt";
    TTC_txt_.open(output_TTC_path);
  }

  // Load gyro
  std::string input_gyro_name = params.input_folder_path + "gyro.csv";
  std::ifstream my_gyro(input_gyro_name);
  if (use_gyro_flag)
  {
    if (!my_gyro.is_open())
    {
      std::cout << input_gyro_name << std::endl;
      throw std::runtime_error("Could not open file");
    }

    std::string gyro_line;
    std::getline(my_gyro, gyro_line); // skip header line
  }

  // Load raw events
  std::ifstream myFileFilter(input_file_name);
  if (!myFileFilter.is_open())
  {
    std::cout << input_file_name << std::endl;
    std::filesystem::path p(input_file_name);
    std::cout << "File exists: " << std::filesystem::exists(p) << std::endl;
    throw std::runtime_error("Could not open file");
  }

  std::string line;
  int c, r, p;
  double ts;
  Eigen::Vector3d e(3); // Event measurement

  // Skip the header
  std::getline(myFileFilter, line);

  if (params.event_num_start > 0)
  {
    // start from certain event data
    while (std::getline(myFileFilter, line))
    {
      EventId++;
      if (EventId >= params.event_num_start)
      {
        std::stringstream ss(line);
        read_events(ss, params.data_format, ts, c, r, p);
        break;
      }
    }
  }
  else
  {
    while (std::getline(myFileFilter, line))
    {
      std::stringstream ss(line);
      read_events(ss, params.data_format, ts, c, r, p);
      if (ts > params.process_ts_start)
      {
        break;
      }
    }
  }

  //--------------------------------------------------------------------------------------------------------//
  //---------------                              SELECT TARGETS                              ---------------//
  //--------------------------------------------------------------------------------------------------------//

  SAEdetector* detector = nullptr;
  double detection_event_count = 0;


  // Initialise targets from config file
  if (params.select_target_flag == 0)
  {
    // make sure config is correct
    if (params.position_x_init.size() != params.position_y_init.size())
    {
      throw std::runtime_error("Error loading initial positions: number of x and y coordinates do not match");
    }
    // Load init points
    for (int i = 0; i < params.position_x_init.size(); i++)
    {
      selectedPoint.x = params.position_x_init[i];
      selectedPoint.y = params.position_y_init[i];
      selectedPoints.push_back(selectedPoint);
    }
  }
  // Initialise targets from the display - click to select
  else if (params.select_target_flag == 1)
  {
    while (std::getline(myFileFilter, line))
    {
      EventId++;
      std::stringstream ss(line);
      read_events(ss, params.data_format, ts, c, r, p);

      e << c, r, ts;
      high_pass(ts, c, r, p, params.alpha);

      if (params.publish_framerate > 0 && ts > t_next_publish)
      {
        high_pass_global(ts, params.alpha);
        t_next_publish = ts + (1.0 / params.publish_framerate);
        image_count += 1;
      }
      if (image_count >= 4)
      {
        std::cout << ts << std::endl;
        display_for_select();
        std::cout << "Number of target: " << n_target << "\n";
        image_count += 1;
        break;
      }
    }
  }
  // Set up the detector
  else if (params.select_target_flag == 2)
  {
    detector = new SAEdetector(params.height, 
                               params.width, 
                               params.SAE_ksize, 
                               params.SAE_alpha, 
                               params.SAE_min_contributions, 
                               params.SAE_min_active_pixels, 
                               params.SAE_detection_threshold,
                               params.detector_dt_threshold); 
  }
  else 
  {
    throw std::runtime_error("Undefined params.select_target_flag");
  }



  //--------------------------------------------------------------------------------------------------------//
  //---------------                           SETUP TRACK MANAGER                            ---------------//
  //--------------------------------------------------------------------------------------------------------//
  bool flag_decoding = 0;
  if (data_name.substr(0, 5) == "comms")
  {
    flag_decoding = 1;
  }
  std::cout << "flag_decoding: " << flag_decoding << std::endl;

  //--------------------------------------------------------------------------------------------------------//
  //--------------------------------------------------------------------------------------------------------//
  // Initialise filter parameters
  init(params);

  // Create Track Manager
  TrackManager track_manager(params.dt, F, C, Q, R, P, A, x0, params.ring_buffer_len, n_state);
  track_manager.update_default_dist_threshold(params.dist_threshold);
  track_manager.update_frame_dimensions(params.height, params.width);
  track_manager.update_event_rate_threshold(params.event_rate_threshold);

  track_manager.store_parameters(
    params.evaluate_ts_age,
    params.evaluate_dt_terminate,
    params.evaluate_low_activity_factor
  );

  int double_track_evaluation_counter = 0;

  // Add selected points if predefine points or select points from display
  if (params.select_target_flag == 0 | params.select_target_flag == 1)
  {
    std::cout << "Selected points are: \n";
    for (int j = 0; j < selectedPoints.size(); j++)
    {
      const cv::Point &point = selectedPoints[j];
      std::cout << "(" << point.x << ", " << point.y << ")\n";
    }
    track_manager.addSelectedPoints(selectedPoints, ts);

  }

  if (flag_decoding == 1)
    {
      track_manager.initialiseDecoder(params.dt, data_name, input_event_path, contrast_threshold);
      track_manager.setDecodeFlag(flag_decoding);
      track_manager.storeDecodeVariables(params.dt, data_name, input_event_path, contrast_threshold);
    }



    


  //--------------------------------------------------------------------------------------------------------//
  //--------------------------------------------------------------------------------------------------------//
  //--------------------------------------------------------------------------------------------------------//

  // std::string blob_file_name = input_event_path + "_measurements.csv";
  // std::string blob_file_name = "/Users/angus/Desktop/aeb_data/general_motion/track_positions.csv";
  // std::string blob_file_name = "/Users/angus/Desktop/aeb_data/spinning_fast_exp2/track_positions.csv";

  // std::string blob_file_name = input_event_path + "_events.csv";
  // blob_measurements_txt_.open(blob_file_name);

  //--------------------------------------------------------------------------------------------------------//
  //--------------------------------------------------------------------------------------------------------//
  //--------------------------------------------------------------------------------------------------------//


  //--------------------------------------------------------------------------------------------------------//
  //--------------------------------------------------------------------------------------------------------//
  // Display video after selecting points
  display(dispParams, params, ts, output_video, output_video_ref,
          image_ref_ts, p, track_manager.getTracks());


  double t_last = -1;
  bool f_associated_this_ts = 0;
  bool f_write_position = 0;


  while (std::getline(myFileFilter, line))
  {
    EventId++;
    std::stringstream ss(line);
    read_events(ss, params.data_format, ts, c, r, p);

    if (params.event_num_end > 0)
    {
      if (EventId > params.event_num_end)
      {
        std::cout << "End Event Id: " << params.event_num_end << std::endl;
        break;
      }
    }
    else if (params.process_ts_end > 0)
    {
      if (ts > params.process_ts_end)
      {
        std::cout << "End time: " << params.process_ts_end << std::endl;
        break;
      }
    }
    e << c, r, ts;
    if (t_last == -1){
      t_last = ts;
    }

    high_pass(ts, c, r, p, params.alpha);

    // If we get a new timestamp, check if any of the events on this timestamp have been associated and set write flag if they have
    // Reset f_associated_this_ts flag for the new timestamp.
    if (ts - t_last > 0){
      if (f_associated_this_ts){
        f_write_position = 1;
      }
      else {
        f_write_position = 0;
      }
      f_associated_this_ts = 0;
    }


    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//

    // if ((ts - t_last > 0) & (f_write_position)){
    //   blob_measurements_txt_ << std::fixed << std::setprecision(12) << t_last;
    //   for (int id = 0; id < track_manager.len(); id++){
    //     blob_measurements_txt_ << "," << track_manager.getTrack(id)->state()(0) << "," << track_manager.getTrack(id)->state()(1);
    //   }
    //   blob_measurements_txt_ << std::endl;
    // }

    //     // TEMP - write position of associated track
    // if (f_event_associated){
    //   blob_measurements_txt_ << std::fixed << std::setprecision(12);
    //   blob_measurements_txt_ << id << "," << ts;
    //   for (int i = 0; i < 4; ++i){
    //     blob_measurements_txt_ << "," << x_hat_track(0, i);
    //   }
    //   blob_measurements_txt_ << ",";
    //   for (int i = 0; i < 4; ++i) {
    //       for (int j = 0; j < 4; ++j) {
    //           blob_measurements_txt_ << P_track(i, j);
    //           if (!(i == 3 && j == 3)) blob_measurements_txt_ << ','; // space separator, no trailing space at end
    //       }
    //   }
    //   blob_measurements_txt_ << std::endl;
    // }
    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//



    


    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//
    bool f_event_associated = 0;

    if (track_manager.len() > 0)
    {
      //////////////////////////////////////////////////////////////////////
      // comms functions TODO: only works for single LED now
      if (flag_decoding)
      {
        // Check comms range for each track
        for (int i = 0; i < track_manager.len(); i++)
        {
          x_hat_track = track_manager.getTrack(i)->state();
          double led_distance = sqrt(pow((c - x_hat_track(0)), 2) + pow((r - x_hat_track(1)), 2));
          double comms_range = track_manager.getTrack(i)->decoder->get_comms_range();

          if (led_distance < comms_range)
          {
            x_hat_track = track_manager.getTrack(i)->state();
            track_manager.getTrack(i)->decoder->get_msg(ts, p, x_hat_track(0), x_hat_track(1), x_hat_track(2), x_hat_track(3));

          }
        }
      }

      //--------------------------------------------------------------------------------------------------------//
      //--------------------------------------------------------------------------------------------------------//

      // Event data association - find the closet target id
      double dist_min = 1e6; // Any large number

      for (int i = 0; i < track_manager.len(); i++)
      {
        // TODO: need to create a new x_hat_track everytime?? - how to speed up - this is for every events
        x_hat_track = track_manager.getTrack(i)->state();
        distance = sqrt(pow((c - x_hat_track(0)), 2) + pow((r - x_hat_track(1)), 2));

        if (distance < dist_min)
        {
          dist_min = distance;
          id = i;
        }
      }

      //--------------------------------------------------------------------------------------------------------//
      //--------------------------------------------------------------------------------------------------------// 
      x_hat_track = track_manager.getTrack(id)->state();
      P_track = track_manager.getTrack(id)->P_x();
      double ts_last_for_gamma = track_manager.getTrack(id)->get_ts_last_for_gamma();
      double dist_threshold_track = track_manager.getTrack(id)->get_dist_threshold();

      // Compute distance to the closest track
      distance = sqrt(pow((c - x_hat_track(0)), 2) + pow((r - x_hat_track(1)), 2));

      if ((P_track(0, 0) < 3) && (P_track(1, 1) < 3))
      {
        double gamma = std::exp(-alpha_dist * (ts - ts_last_for_gamma));

        // Update distance threshold
        if (x_hat_track(4) > x_hat_track(5))
        {
          dist_threshold_track = gamma * dist_threshold_track + (1 - gamma) * 2.5 * x_hat_track(4);
        }
        else
        {
          dist_threshold_track = gamma * dist_threshold_track + (1 - gamma) * 2.5 * x_hat_track(5);
        }

        // Data association distance threshold lower bound
        if (dist_threshold_track < params.dist_threshold)
        {
          dist_threshold_track = params.dist_threshold;
        }

        track_manager.getTrack(id)->update_distance_threshold(dist_threshold_track);
      }


      //--------------------------------------------------------------------------------------------------------//
      //--------------------------------------------------------------------------------------------------------//

      if (distance < dist_threshold_track)
      {
        f_event_associated = 1; // set flag that event has been associated to a track
        track_manager.getTrack(id)->update(e, 0, p);
      }
    }
    
    f_associated_this_ts = (f_associated_this_ts | f_event_associated);


    //------------------------------------------------------------------------------------------------//
    //------------------------------------------------------------------------------------------------//
    // Detection - Not associated to any tracks 
    if ((params.select_target_flag == 2) & 
        (f_event_associated == 0)) 
    { 
      detector->addEvent({(double) c, (double) r, ts, (double) p});
      detection_event_count++;
    }

    if ((params.select_target_flag == 2) & 
        (f_event_associated == 0) &  // event is not associated to track
        (track_manager.len() < params.MAX_NUMBER_OF_TRACKS) & // upper limit on number of tracks (not currently doing anything)
        (track_manager.len() < params.n_target) &  // when we have less than the n_targets 
        (distance > params.detector_dist_threshold || track_manager.len() == 0) & // if the distance from any existing tracks is large enough
        (detection_event_count > params.SAE_operation_rate)) // when some events have passed.
    { 

        detection_event_count = 0;
      
        // int detector_output = detector->performDetection({(double) c, (double) r, ts, (double) p});
        int detector_output = detector->performDetection_dt({(double) c, (double) r, ts, (double) p});

        if (detector_output == 1)
        {
          track_manager.createNewTrack({(double) c, (double) r, ts});
        }
      // }
    }


    //------------------------------------------------------------------------------------------------//
    // ------------------------------------------------------------------------------------------------//
    // Evaluate tracks 
    std::vector<int> deleted_IDs, deleted_IDs_temp;

    // Evaluate double up tracks
    double_track_evaluation_counter++;
    if ((track_manager.len() > 0) & (params.f_evaluate == 1) & ((double_track_evaluation_counter > 100) & (track_manager.len() > 1)))    {
      double_track_evaluation_counter = 0; 
      deleted_IDs_temp = track_manager.evaluateDoubleTracks();
      if (deleted_IDs_temp.size() > 0){
        deleted_IDs.insert(deleted_IDs.end(), deleted_IDs_temp.begin(), deleted_IDs_temp.end());
      }
    }

    // Perform full evaluation
    else if ((track_manager.len() > 0) & (params.f_evaluate == 1)){
      deleted_IDs_temp = track_manager.evaluateTracks(ts);
      if (deleted_IDs_temp.size() > 0){
        deleted_IDs.insert(deleted_IDs.end(), deleted_IDs_temp.begin(), deleted_IDs_temp.end());
      }
    }

    // Only evaluated by position (delete when leaves the frame)
    else if ((track_manager.len() > 0) & (params.f_evaluate == 0)){
      deleted_IDs_temp = track_manager.evaluateTracksPosition();
      if (deleted_IDs_temp.size() > 0){
        deleted_IDs.insert(deleted_IDs.end(), deleted_IDs_temp.begin(), deleted_IDs_temp.end());
      }
    }

    if (deleted_IDs.size() > 0) {
      // Clear ts buffer 
      while(!same_ts_e_buffer.empty()){
        same_ts_e_buffer.pop();
      }
    }   

    if (f_event_associated){
      // std::cout << std::setprecision(0) << std::fixed;
      // std::cout << c << "," << r << "," << p << "," << int(ts*1e6) << std::endl;

      blob_measurements_txt_ << std::setprecision(0) << std::fixed;
      blob_measurements_txt_ << c << "," << r << "," << p << "," << int(ts*1e6) << std::endl;

    }

    

    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//

    // Global update high pass filter and display
    if (params.publish_framerate > 0 && ts > t_next_publish)
    {
      std::cout << ts << std::endl;
      high_pass_global(ts, params.alpha);
      display(dispParams, params, ts, output_video, output_video_ref,
              image_ref_ts, p, track_manager.getTracks());
      t_next_publish = ts + (1.0 / params.publish_framerate);    
    }

    
    
    if (track_manager.len() > 0)
    {
      // If the ID is not one of the deleted tracks
      if (std::find(deleted_IDs.begin(), deleted_IDs.end(), id) == deleted_IDs.end()){

        // If we have deleted a track and current ID > deleted ID, we need to minus 1 from ID
        bool is_larger = std::all_of(deleted_IDs.begin(), deleted_IDs.end(), [id](int elem) { return id >= elem;});
        if ((deleted_IDs.size()>0) & (is_larger)){
          id--;
        }

        if (id < track_manager.len()){
          track_manager.getTrack(id)->update_ts_last_for_gamma(ts);
        }
      }

      ts_kf_last = ts;

    }


    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//

    t_last = ts;

    // // TEMP - write position of associated track
    // if (f_event_associated){
    //   blob_measurements_txt_ << std::fixed << std::setprecision(12);
    //   blob_measurements_txt_ << id << "," << ts;
    //   for (int i = 0; i < 4; ++i){
    //     blob_measurements_txt_ << "," << x_hat_track(0, i);
    //   }
    //   blob_measurements_txt_ << ",";
    //   for (int i = 0; i < 4; ++i) {
    //       for (int j = 0; j < 4; ++j) {
    //           blob_measurements_txt_ << P_track(i, j);
    //           if (!(i == 3 && j == 3)) blob_measurements_txt_ << ','; // space separator, no trailing space at end
    //       }
    //   }
    //   blob_measurements_txt_ << std::endl;
    // }
    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------------------//




  }

  writer.release();
  return 0;
}

// High-pass Filter
void high_pass(double ts, int x, int y, int p, int &alpha)
{
  log_intensity_state.at<double>(y, x) =
      exp(-alpha * (ts - ts_array.at<double>(y, x))) *
      log_intensity_state.at<double>(y, x);
  log_intensity_state.at<double>(y, x) +=
      (p > 0) ? -contrast_threshold : contrast_threshold;
  ts_array.at<double>(y, x) = ts;
};

void high_pass_global(double ts, int &alpha)
{
  cv::Mat beta;
  cv::exp(-alpha * (ts - ts_array), beta);
  log_intensity_state = log_intensity_state.mul(beta);
  ts_array.setTo(ts);
};

// View image
void display(const DisplayParams &dispParams, Parameters &params,
             const double &ts, std::vector<cv::Mat> &output_video,
             std::vector<cv::Mat> &output_video_ref,
             std::vector<double> &image_ref_ts, int p, std::vector<KalmanFilter *> current_tracks)
{
  cv::Mat image, ref_image;
  cv::exp(log_intensity_state, image);
  double minVal = 1.4;
  double maxVal = 0.4;
  image = (image - minVal) / (maxVal - minVal);
  cv::Mat img(image.rows, image.cols, CV_64FC1, (char *)image.data);
  img.convertTo(img, CV_8U, 255.0 / 1.0);
  cv::Mat cimg;
  cv::cvtColor(img, cimg, cv::COLOR_GRAY2RGB);

  // IMU colour data plot
  std::vector<cv::Scalar> colors = {
      cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0),
      cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255),
      cv::Scalar(0, 0, 128), cv::Scalar(0, 128, 0), cv::Scalar(128, 0, 0),
      cv::Scalar(128, 128, 0), cv::Scalar(128, 0, 128), cv::Scalar(0, 128, 128)};
  std::vector<cv::Scalar> colors_dark = {
      cv::Scalar(0, 0, 128), cv::Scalar(0, 128, 0), cv::Scalar(128, 0, 0),
      cv::Scalar(128, 128, 0), cv::Scalar(128, 0, 128), cv::Scalar(0, 128, 128)};

  for (int i = 0; i < current_tracks.size(); i++)
  {
    x_hat_track = current_tracks[i]->state();
    P_track = current_tracks[i]->P_x();

    // (180.0 / 3.14) =
    double rotationAngle = x_hat_track(6) * 57.295;

    rotationAngle = std::fmod(rotationAngle, 360.0);
    if (rotationAngle < 0.0)
    {
      rotationAngle += 360.0;
    }

    // HACK: take abs of size parameter
    if (current_tracks[i]->validated == 1){
      cv::ellipse(cimg, cv::Point(x_hat_track(0), x_hat_track(1)),
                  cv::Size(abs(x_hat_track(4)), abs(x_hat_track(5))),
                  rotationAngle, 0, 360, cv::Scalar(0,0,255), 2, cv::LINE_AA);      
    }
    else if (params.f_show_candidates) {
      cv::ellipse(cimg, cv::Point(x_hat_track(0), x_hat_track(1)),
                  cv::Size(abs(x_hat_track(4)), abs(x_hat_track(5))),
                  rotationAngle, 0, 360, cv::Scalar(255,0,0), 2, cv::LINE_AA);      
    }


    // if (current_tracks[i]->validated == 1){
    //   cv::ellipse(cimg, cv::Point(x_hat_track(0), x_hat_track(1)),
    //               cv::Size(x_hat_track(4), x_hat_track(5)),
    //               rotationAngle, 0, 360, cv::Scalar(0,0,255), 2, cv::LINE_AA);      
    // }
    // else if (params.f_show_candidates) {
    //   cv::ellipse(cimg, cv::Point(x_hat_track(0), x_hat_track(1)),
    //               cv::Size(x_hat_track(4), x_hat_track(5)),
    //               rotationAngle, 0, 360, cv::Scalar(255,0,0), 2, cv::LINE_AA);      
    // }

    // cv::ellipse(cimg, cv::Point(x_hat_track(0), x_hat_track(1)),
    //             cv::Size(x_hat_track(4), x_hat_track(5)),
    //             rotationAngle, 0, 360, colors[i % colors.size()], 2, cv::LINE_AA);

    if (dispParams.disp_covariance_flag)
    {
      cv::ellipse(
          cimg, cv::Point(x_hat_track(0), x_hat_track(1)),
          cv::Size(50 * P_track(0, 0), 50 * P_track(1, 1)),
          x_hat_track(6) * (180.0 / M_PI), 0, 360, cv::Scalar(255, 0, 0), 1,
          cv::LINE_AA);
    }
  }

  // Plot image
  cv::imshow("Video", cimg);
  if (params.ref_image_ts_flag)
  {
    double event_ts = ts * 1e6 + t1;
    auto it = std::lower_bound(image_ref_ts.begin(), image_ref_ts.end(), event_ts);
    if (it == image_ref_ts.begin())
    {
      image_id = 1;
    }
    else if (it == image_ref_ts.end())
    {
      image_id = image_ref_ts.size() - 1;
    }
    else
    {
      image_id = std::distance(image_ref_ts.begin(), it);
    }

    image_id = image_id + 1; // please sync image id here
    std::string ref_image_path = params.input_folder_path + "/reproject_rgb/" +
                                 std::to_string(image_id) + ".png";

    ref_image = cv::imread(ref_image_path);
    // ref_image = ref_image * 2.5; // Option: make image brighter
    if (!ref_image.empty())
    {
      cv::imshow("Video_ref", ref_image);
    }
    else
    {
      std::cerr << "Error loading reference image: " << ref_image_path
                << std::endl;
    }
  }

  // Key input
  int keyPressed = cv::waitKey(30);
  if (keyPressed == 27)
  {
    // Select init points by press "Esc"
    for (int i = 0; i < n_target; i++)
    {
      std::cout << "Press ESC to start selecting points, and press Enter when "
                   "finished.\n";
      std::cout << "Waiting to click to select a point ... \n";
      selectedPoint = cv::Point(-1, -1); // Reset selected point
      while ((selectedPoint.x == -1 && selectedPoint.y == -1) &&
             keyPressed != 13)
      {
        keyPressed = cv::waitKey(1);
      }
      if (keyPressed == 13)
      {
        // 13 is the ASCII code for "Enter" key
        std::cout << "Selection completed.\n";
        std::cout << "Event Id: " << EventId << "\n";

        // TODO: re-init targets
        std::cout << x_hat << std::endl;
        params.var_x = 2;
        params.var_y = 2;
        init(params);
        break;
      }
      selectedPoints.push_back(
          selectedPoint); // Add selected point to the vector
      std::cout << "Selected point: (" << selectedPoint.x << ", "
                << selectedPoint.y << ")\n";
      std::cout << "Event Id: " << EventId << "\n";
      std::cout << "Time: " << ts << "\n";

      x_hat(i, 0) = selectedPoint.x; // Store selected point x-coordinate
      x_hat(i, 1) = selectedPoint.y; // Store selected point y-coordinate
      x_hat(i, 2) = 0;
      x_hat(i, 3) = 0;
      x_hat(i, 4) = 2;
      x_hat(i, 5) = 2;
    }
  }
  else if (keyPressed == 116 ||
           keyPressed ==
               84)
  { // Pressed 'T' key for display current time and location
    std::cout << "ts: " << ts << " (second)\n";
    std::cout << "Event Id: " << EventId << "\n";
    std::cout << "Current location x: (" << x_hat(0, 0) << ", " << x_hat(0, 1)
              << ")\n";
    std::cout << "Current lambda: (" << x_hat(0, 4) << ")\n";
    std::cout << "Current P_x: (" << P_x(0, 0) << ", " << P_x(1, 1) << ")\n";
  }
  cv::waitKey(1);

  // Save video
  if (dispParams.save_video_flag)
  {
    writer.write(cimg);

    // output_video.push_back(cimg);
    if (params.ref_image_ts_flag)
    {
      writer_ref.write(ref_image);
      // output_video_ref.push_back(ref_image);
    }
  }
  // Save image
  if (dispParams.save_image_flag)
  {
    std::string output_image_name =
        output_image_path + "/image" + std::to_string(image_count) + ".png";
    cv::imwrite(output_image_name, cimg);

    if (params.ref_image_ts_flag)
    {
      std::string output_image_ref_name =
          output_image_path + "/image" + std::to_string(image_count) + "_ref.png";
      cv::imwrite(output_image_ref_name, ref_image);
    }
  }
  image_count += 1;
}

void display_for_select()
{
  cv::Mat image;
  cv::exp(log_intensity_state, image);
  double minVal = 1.4;
  double maxVal = 0.4;
  image = (image - minVal) / (maxVal - minVal);
  cv::Mat img(image.rows, image.cols, CV_64FC1, (char *)image.data);
  img.convertTo(img, CV_8U, 255.0 / 1.0);
  cv::Mat cimg;
  cv::cvtColor(img, cimg, cv::COLOR_GRAY2RGB);

  // Plot image
  cv::imshow("Video", cimg);
  int keyPressed = cv::waitKey(30);
  for (int i = 0; i < n_target; i++)
  {
    std::cout << "Current event time: " << ts_global << std::endl;
    std::cout << "Press ESC to start selecting points, and press Enter when "
                 "finished.\n";
    std::cout << "Waiting to click to select a point ... \n";
    selectedPoint = cv::Point(-1, -1); // Reset selected point

    while ((selectedPoint.x == -1 && selectedPoint.y == -1) &&
           keyPressed != 13)
    {
      keyPressed = cv::waitKey(1);
    }
    if (keyPressed == 13)
    {
      // 13 is the ASCII code for "Enter" key
      std::cout << "Event Id: " << EventId << "\n";
      std::cout << "Selection completed.\n";
      n_target = i;
      break;
    }
    selectedPoints.push_back(selectedPoint); // Add selected point to the vector
    std::cout << "Selected point: (" << selectedPoint.x << ", "
              << selectedPoint.y << ")\n";
    if (i == (n_target - 1))
    {
      std::cout << "Reach to upper limit: " << n_target << " points"
                << std::endl;
      std::cout << "Selection completed.\n";
      std::cout << "Event Id: " << EventId << "\n";
    }
  }
  cv::waitKey(1);
}

// TODO: with new track management, n_target should = 1 here.
void init(Parameters &params)
{
  int n_target = 1; // TODO: **** Hard code for filter dimensions

  F = Eigen::MatrixXd::Zero(n_target * n_state, n_state);
  C = Eigen::MatrixXd::Zero(3, n_state);
  R = Eigen::MatrixXd::Identity(3, 3); // Measurement noise covariance
  P = Eigen::MatrixXd::Zero(n_target * n_state,
                            n_state);          // Estimate error covariance
  Q = Eigen::MatrixXd::Zero(n_state, n_state); // Process noise covariance
  A = Eigen::MatrixXd::Zero(2, 2);             // Sigma^(1/2)
  x0 = Eigen::MatrixXd::Zero(n_target, n_state);
  for (int j = 0; j < n_target; j++)
  {
    for (int i = 0; i < n_state; i++)
    {
      F(j * n_state + i, i) = 1;
    }
    F(j * n_state, 2) = params.dt;
    F(j * n_state + 1, 3) = params.dt;
    F(j * n_state + 6, 7) = params.dt;
  }

  Eigen::MatrixXd P_block = Eigen::MatrixXd::Zero(n_state, n_state);
  x0.block(0, 4, n_target, 1).setConstant(params.lambda_init);
  x0.block(0, 8, n_target, 2).setConstant(0);
  x0.block(0, 5, n_target, 1).setConstant(params.lambda_init);
  P_block.diagonal() << params.var_x, params.var_y, params.var_vx,
      params.var_vy, params.var_lambda_1, params.var_lambda_2, params.var_theta,
      params.var_q, 50, 50;
  Q.diagonal() << params.q_x, params.q_y, params.q_vx, params.q_vy,
      params.q_lambda_1, params.q_lambda_2, params.q_theta, params.q_q, 5, 5;
  for (int row = 0; row < n_target; row++)
  {
    P.block(row * n_state, 0, n_state, n_state) << P_block;
  }
}

void read_events(std::stringstream &ss, const int data_format, double &ts,
                 int &c, int &r, int &p)
{
  // 0: ts,c,r,p; 1: c,r,p,ts
  if (data_format == 0)
  {
    ss >> ts;
    ss.get();
    ss >> c;
    ss.get();
    ss >> r;
    ss.get();
    ss >> p;
  }
  else
  {
    ss >> c;
    ss.get();
    ss >> r;
    ss.get();
    ss >> p;
    ss.get();
    ss >> ts;
  }

  if (first_ts_flag == 0)
  {
    // t1 = ts;
    t1 = 0; // don't minus first ts
    first_ts_flag = 1;
    std::cout << "ts: " << ts << std::endl;
  }
  ts = ts - t1;
  ts = ts * 1e-6; // second
  c = c + 1;
  r = r + 1;

  ts_global = ts; // HACK
}

void load_gyro(std::ifstream &my_gyro, TrackManager &track_manager)
{
  std::string gyro_line;
  std::getline(my_gyro, gyro_line);
  std::stringstream ss_gyro(gyro_line);
  double gyroX, gyroY, gyroZ;

  ss_gyro >> gyro_ts;
  ss_gyro.get();
  ss_gyro >> gyroX;
  ss_gyro.get();
  ss_gyro >> gyroY;
  ss_gyro.get();
  ss_gyro >> gyroZ;
  gyro_ts = (gyro_ts - t1) * 1e-6;

  if (first_gyro_flag)
  {
    gyro_ts_last = gyro_ts;
    first_gyro_flag = false;
  }

  // Note: update your IMU parameters here
  // bias gyro - our davis 240c data
  // remove bias, radian/s to degree/s
  gyroX = (gyroX + 1.7) * 0.017453;
  gyroY = (gyroY + 0.9) * 0.017453;
  gyroZ = (gyroZ - 0.2) * 0.017453;
  // Update v dot in tracker
  double f_x = 186.9;
  double f_y = 185.9;

  // track_manager.getTrack(id)->update_gyro(vx_update, vy_update);
  std::vector<int> ids = track_manager.getValidTrackIds();
  double gyro_dt = gyro_ts - gyro_ts_last;
  for (int id : ids)
  {
    x_hat_track = track_manager.getTrack(id)->state(); // shape (1, 10)

    // Extract terms from x_hat
    double x0 = x_hat_track(0);
    double x1 = x_hat_track(1);

    double term1 = x0 * x1 / f_x;
    double term2 = f_x + (x0 * x0 / f_x);
    double vx_gyro = term1 * gyroX - term2 * gyroY + x1 * gyroZ;
    double term3 = f_y + (x1 * x1 / f_y);
    double term4 = x0 * x1 / f_y;
    double vy_gyro = term3 * gyroX - term4 * gyroY - x0 * gyroZ;

    // Apply gyro update
    track_manager.getTrack(id)->update_gyro_single_id(vx_gyro * gyro_dt, vy_gyro * gyro_dt);
  }
  gyro_ts_last = gyro_ts;
}
