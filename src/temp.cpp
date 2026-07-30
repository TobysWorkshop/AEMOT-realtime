// This file is licensed under the Apache License, Version 2.0,
// with additional restrictions under the Commons Clause.
// See the LICENSE file for more details.

// ============================================================================
// OVERVIEW
// ----------------------------------------------------------------------------
// This program is an event-camera based multi-target tracker.
// "Event cameras" (like DAVIS/DVS sensors) don't output normal frames -
// instead, each pixel independently reports a "+1" or "-1" event whenever
// its brightness changes by a threshold amount. This code:
//   1. Reads a stream of these (x, y, timestamp, polarity) events from a CSV
//   2. Reconstructs a rough "image" of the scene using a high-pass filter
//      (this is a classic trick for visualizing event data)
//   3. Tracks one or more moving blobs/targets using Kalman filters (one
//      filter per target, managed by an external TrackManager class)
//   4. Optionally detects NEW targets automatically using a "SAEdetector"
//      (Surface of Active Events detector)
//   5. Optionally decodes an LED communication signal from a tracked blob
//      (data_name starting with "comms")
//   6. Optionally fuses gyroscope data to compensate for camera rotation
//   7. Displays/saves a visualization video with tracked ellipses drawn
// ============================================================================

#include "yaml-cpp/yaml.h"      // Used indirectly by parameters.h to parse the .yaml config file
#include <eigen3/Eigen/Dense>   // Linear algebra library - used for all Kalman filter matrices/vectors
#include <chrono>
#include <deque>
#include <filesystem>           // Used for creating output directories, checking file existence
#include <fstream>               // File I/O (reading CSVs, writing output text files)
#include <iostream>
#include <math.h>
// #include <opencv4/opencv2/core/core.hpp>      // Alternate OpenCV include paths (commented out,
// #include <opencv4/opencv2/highgui/highgui.hpp> // kept in case the build environment changes)
// #include <opencv4/opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>     // OpenCV core types: cv::Mat, cv::Point, cv::Scalar, etc.
#include <opencv2/highgui/highgui.hpp> // OpenCV windowing/video I/O: imshow, VideoWriter, waitKey
#include <opencv2/imgproc.hpp>       // OpenCV drawing/image processing: ellipse, cvtColor, exp
#include <string>
#include <vector>
#include <queue>
#include <chrono>

// ---- Project-local headers (these define the "other modules" this file uses) ----
#include "kalman.hpp"        // Defines KalmanFilter class - one instance tracks one target's state
#include "parameters.h"      // Defines Parameters/DisplayParams structs + YAML loader functions
#include "decoding.hpp"      // Defines a decoder used to read LED-blink communication signals
#include "TrackManager.hpp"  // Defines TrackManager - owns/creates/deletes all KalmanFilter tracks
#include "SAEdetector.hpp"   // Defines SAEdetector - automatic new-target detector from event data

// ============================================================================
// GLOBAL STATE
// ----------------------------------------------------------------------------
// NOTE: Using globals here is not great practice, but this is a research/
// experimental codebase. Below is what each global is used for.
// ============================================================================

double TTC, contrast_threshold, first_ts_flag;
// TTC              = Time-To-Contact (collision estimate) between two chosen targets
// contrast_threshold = the per-event brightness-change magnitude, used by high_pass()
// first_ts_flag    = flag: have we seen the first event timestamp yet? (used to zero the clock)

int n_target, n_state, image_count, image_id, start_event_ts;
// n_target   = number of targets to track (from config, but also gets locally overridden in init())
// n_state    = size of the Kalman filter state vector per target
// image_count = counter used for naming saved output images/frames
// image_id   = index into the reference image timestamp list (for RGB overlay sync)

double t1;                 // First event's raw timestamp, used to zero-base all subsequent timestamps
double t_next_publish = 0; // Next wall-clock/event-time at which we should refresh the display
int EventId = 0;           // Running counter of how many event lines have been read
double ts_kf_last = 0;     // Timestamp of the last Kalman filter update
double gyro_ts = 0;        // Current gyro sample timestamp
double gyro_ts_last = 0;   // Previous gyro sample timestamp (for computing gyro_dt)
double next_save_traj_ts = 0;
double dt_save_traj = 0.0001; // Sub-sampling interval for saving trajectory (currently unused/legacy)
double alpha_dist = 1.4;      // Decay rate used to smooth/update the per-track association distance threshold
bool first_gyro_flag = true;  // Have we processed the first gyro sample yet?
bool use_gyro_flag;           // Whether gyro fusion is enabled (from config)

Eigen::MatrixXd x_hat_track; // Scratch variable: holds the CURRENT track's state vector (reused each loop)
Eigen::MatrixXd P_track;     // Scratch variable: holds the CURRENT track's state covariance matrix

// Choose two target IDs to compute time-to-contact; default id=0 and id=1
int id_ttc_left = 0;
int id_ttc_right = 1;

std::queue<std::pair<int, std::pair<int, int>>> same_ts_e_buffer; // Buffer of events sharing a timestamp (cleared on track deletion)
cv::Mat log_intensity_state, ts_array;
// log_intensity_state = per-pixel running "brightness" estimate built from events (the reconstructed image)
// ts_array            = per-pixel timestamp of the last event seen there (needed for the exponential decay)

// ---- Kalman filter matrices shared across the whole program (set once in init()) ----
Eigen::MatrixXd F, C, R, P, Q, A, P_x, x0, x_hat;
// F  = state transition matrix (predicts next state from current state)
// C  = measurement/observation matrix
// R  = measurement noise covariance
// P  = initial state covariance (per target)
// Q  = process noise covariance
// A  = auxiliary matrix (Sigma^(1/2), used elsewhere for shape estimation)
// x0 = initial state vector template (per target)
// x_hat = working state estimate used mainly during manual target selection (key 'Esc' handler)

std::ofstream output_txt_;
std::string input_event_path, output_image_path;
cv::Point selectedPoint(-1, -1);            // Most recently clicked point (set by the mouse callback)
std::vector<cv::Point> selectedPoints;      // All points selected so far (manual init) 
std::vector<cv::Point> selectedPointsSpeed; // (unused here) placeholder for velocity selection
cv::VideoWriter writer_ref, writer;         // OpenCV video writers: raw event-video and reference-image video

double ts_global = 0; // Mirror of the most recent event timestamp, visible to display_for_select()

//--------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------//

//--------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------//

// Mouse callback registered on the "Video" OpenCV window.
// Whenever the user left-clicks, we record the pixel coordinate into `selectedPoint`.
// This is how manual target initialization works (select_target_flag == 1) and also
// how the user can re-select points mid-run (see the ESC key handler in display()).
void onMouse(int event, int x, int y, int flags, void *userdata)
{
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    selectedPoint = cv::Point(x, y);
  }
}

// ---- Forward declarations of all functions defined later in this file ----
void high_pass(double ts, int x, int y, int p, int &alpha);        // per-pixel event-driven filter update
void high_pass_global(double ts, int &alpha);                      // vectorized/whole-image filter update
void init(Parameters &params);                                     // builds Kalman filter matrices
void read_events(std::stringstream &ss, const int data_format, double &ts,
                 int &c, int &r, int &p);                          // parses one CSV line into an event
void display(const DisplayParams &dispParams, Parameters &params,
             const double &ts, std::vector<cv::Mat> &output_video,
             std::vector<cv::Mat> &output_video_ref,
             std::vector<double> &image_ref_ts, int p, std::vector<KalmanFilter *> current_tracks); // renders + saves frame
void display_for_select();                                         // UI used only during initial manual point-picking
void load_gyro(std::ifstream &my_gyro, TrackManager &track_manager); // reads one gyro sample & applies rotational correction

// ============================================================================
// ENTRY POINT
// ============================================================================
int main(int argc, char *argv[])
{
  // argv[1] is unused in this snippet (likely reserved elsewhere, e.g. a mode flag).
  // argv[2] is the "data_name" - this is both:
  //   (a) used to find "../configs/<data_name>.yaml" (the run configuration), and
  //   (b) checked later to see if it starts with "comms" (enables LED decoding mode).
  std::string data_name = argv[2];
  Parameters params;       // Will hold all tunable tracking/filter parameters (populated from YAML)
  DisplayParams dispParams; // Will hold display/video-saving related parameters (populated from YAML)

  // ---- CONFIG INGEST POINT #1: YAML config file ----
  // loadParametersFromYAML / loadDispParametersFromYAML live in parameters.h/.cpp (external module).
  // This is where the "other module" (parameters.h) is invoked to hydrate the params structs.
  try
  {
    std::string file_config_path = "../configs/" + data_name + ".yaml";
    params = loadParametersFromYAML(file_config_path);
    dispParams = loadDispParametersFromYAML(file_config_path);
  }
  catch (const std::exception &ex)
  {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1; // Fail fast if config is missing/invalid
  }

  // Allocate the two per-pixel accumulator images used for the event->intensity reconstruction.
  log_intensity_state = cv::Mat::zeros(params.height, params.width, CV_64FC1);
  ts_array = cv::Mat::zeros(params.height, params.width, CV_64FC1);
  use_gyro_flag = params.use_gyro_flag;

  contrast_threshold = params.contrast_threshold;
  n_target = params.n_target;
  n_state = params.n_state;
  input_event_path = params.input_folder_path + params.input_data_name; // base path (no extension) for this dataset
  std::ofstream track_output_txt_, TTC_txt_, blob_measurements_txt_;    // output file streams (opened conditionally below)
  double distance = 0;
  int id = 0;






  std::string input_file_name = input_event_path + ".csv"; // ---- INGEST POINT #2: the event stream CSV ----

  // ---- CONFIG INGEST POINT #3 (optional): reference-image timestamps ----
  // If the dataset also has real camera frames (for visual comparison / overlay),
  // this loads their timestamps from image_ts.txt into a vector for later lookup.
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

  // ---- Set up display window + mouse callback for manual target selection ----
  cv::namedWindow("Video");
  cv::setMouseCallback("Video", onMouse); // registers onMouse() defined above
  std::vector<cv::Mat> output_video;
  std::vector<cv::Mat> output_video_ref;
  std::string output_ref_video_path = input_event_path + "_video_ref.avi";
  std::string output_video_path = input_event_path + "_video.avi";
  if (params.save_video_flag)
  {
    // Opens an .avi writer for the reconstructed event-video (MJPG codec, fixed 40 fps)
    writer.open(output_video_path,
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 40.0,
                cv::Size(params.width, params.height));

    if (params.ref_image_ts_flag)
    {
      // Second writer for the reference RGB video (note: size is 1px smaller in each dim)
      writer_ref.open(output_ref_video_path,
                      cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 40.0,
                      cv::Size(params.width - 1, params.height - 1));
    }
  }

  if (params.ref_image_ts_flag)
  {
    cv::namedWindow("Video_ref", cv::WINDOW_NORMAL);
  }

  // If saving individual frame images was requested, create the output folder.
  if (params.save_image_flag)
  {
    std::filesystem::path path(input_event_path + "/output_img");
    std::filesystem::create_directories(path);
    output_image_path = path.string();
  }


  std::map<int, std::ofstream> track_output_files; // (currently unused, reserved for per-track output files)

  if (params.compute_TTC_flag)
  {
    std::string output_TTC_path = input_event_path + "_ttc.txt";
    TTC_txt_.open(output_TTC_path);
  }

  // ---- CONFIG INGEST POINT #4 (optional): gyroscope CSV ----
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
    std::getline(my_gyro, gyro_line); // skip CSV header line
  }











  // ---- Open the main event stream file (the primary data ingest point) ----
  std::ifstream myFileFilter(input_file_name);
  if (!myFileFilter.is_open())
  {
    std::cout << input_file_name << std::endl;
    std::filesystem::path p(input_file_name);
    std::cout << "File exists: " << std::filesystem::exists(p) << std::endl;
    throw std::runtime_error("Could not open file");
  }

  std::string line;
  int c, r, p;         // column (x), row (y), polarity of a single event
  double ts;            // timestamp of that event
  Eigen::Vector3d e(3); // Reusable "event measurement" vector [c, r, ts] fed into Kalman updates

  // Skip the CSV header row.
  std::getline(myFileFilter, line);

  // Fast-forward the file pointer to a starting point, either by event COUNT or by TIME,
  // depending on which one the config specifies (event_num_start > 0 takes priority).
  if (params.event_num_start > 0)
  {
    while (std::getline(myFileFilter, line))
    {
      EventId++;
      if (EventId >= params.event_num_start)
      {
        std::stringstream ss(line);
        read_events(ss, params.data_format, ts, c, r, p); // parse the event we're stopping on
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
  // Three mutually exclusive ways to decide WHERE tracking starts:
  //   0 = read fixed pixel coordinates straight from the config file
  //   1 = build up a quick preview image and let the user click on targets
  //   2 = don't pre-select anything; let the SAEdetector find targets automatically as events arrive
  //--------------------------------------------------------------------------------------------------------//

  SAEdetector* detector = nullptr; // Only instantiated in mode 2 (auto-detection)
  double detection_event_count = 0; // Counts events since the detector last ran (rate limiting)

  // Mode 2: Set up the automatic event-based detector (SAEdetector) - "OTHER MODULE" CREATED HERE.
  if (params.select_target_flag == 2)
  {
    detector = new SAEdetector(params.height,
                               params.width,
                               params.SAE_ksize,
                               params.SAE_alpha,
                               params.SAE_min_contributions,
                               params.SAE_min_active_pixels,
                               params.SAE_detection_threshold,
                               params.detector_dt_threshold);
    // All the SAEdetector's tuning knobs come straight out of the YAML-loaded `params` struct.
  }
  else
  {
    throw std::runtime_error("Undefined params.select_target_flag");
  }

  //--------------------------------------------------------------------------------------------------------//
  //---------------                           SETUP TRACK MANAGER                            ---------------//
  //--------------------------------------------------------------------------------------------------------//

  // Detect "comms" mode purely from the dataset name prefix (e.g. "comms_led1").
  bool flag_decoding = 0;
  if (data_name.substr(0, 5) == "comms")
  {
    flag_decoding = 1;
  }
  std::cout << "flag_decoding: " << flag_decoding << std::endl;

  //--------------------------------------------------------------------------------------------------------//
  //--------------------------------------------------------------------------------------------------------//

  // Build the shared Kalman filter matrices (F, C, Q, R, P, A, x0) - see init() below for details.
  init(params);

  // ---- "OTHER MODULE" CREATED HERE: TrackManager ----
  // TrackManager owns the collection of KalmanFilter track objects and handles their
  // lifecycle (creation, deletion, evaluation). It's constructed with the shared filter
  // matrices computed above, plus a few more config values.
  TrackManager track_manager(params.dt, F, C, Q, R, P, A, x0, params.ring_buffer_len, n_state);
  track_manager.update_default_dist_threshold(params.dist_threshold);
  track_manager.update_frame_dimensions(params.height, params.width);
  track_manager.update_event_rate_threshold(params.event_rate_threshold);

  // Configure the criteria TrackManager uses internally to decide when a track should be deleted.
  track_manager.store_parameters(
    params.evaluate_ts_age,
    params.evaluate_dt_terminate,
    params.evaluate_low_activity_factor
  );

  int double_track_evaluation_counter = 0; // Rate-limits the "double track" (duplicate track) check


  // If in "comms" mode, wire up the decoder module inside the TrackManager (per-track decoders
  // are presumably created internally when addSelectedPoints/createNewTrack runs, and this call
  // configures/initialises them with shared decode settings).
  if (flag_decoding == 1)
    {
      track_manager.initialiseDecoder(params.dt, data_name, input_event_path, contrast_threshold);
      track_manager.setDecodeFlag(flag_decoding);
      track_manager.storeDecodeVariables(params.dt, data_name, input_event_path, contrast_threshold);
    }

  //--------------------------------------------------------------------------------------------------------//
  //--------------------------------------------------------------------------------------------------------//
  //--------------------------------------------------------------------------------------------------------//

  // (Legacy/commented-out) alternate per-blob measurement CSV output paths, kept for reference.
  // std::string blob_file_name = input_event_path + "_measurements.csv";
  // blob_measurements_txt_.open(blob_file_name);

  //--------------------------------------------------------------------------------------------------------//
  //--------------------------------------------------------------------------------------------------------//

  // Render one initial frame showing wherever tracking currently stands (right after setup).
  display(dispParams, params, ts, output_video, output_video_ref,
          image_ref_ts, p, track_manager.getTracks());

  double t_last = -1;
  bool f_associated_this_ts = 0; // Did ANY event at the current timestamp get matched to a track?
  bool f_write_position = 0;     // Should we log this timestamp's track position to file?










  
  // ============================================================================
  // MAIN EVENT PROCESSING LOOP
  // Reads the event CSV one line at a time and, for each event:
  //   1) updates the high-pass reconstruction image
  //   2) tries to associate the event with the nearest existing track ("data association")
  //   3) if unassociated, feeds it to the auto-detector (mode 2 only) to possibly spawn a new track
  //   4) periodically evaluates tracks for deletion (duplicates / stale / out-of-frame)
  //   5) periodically refreshes the display/video
  // ============================================================================
  while (std::getline(myFileFilter, line))
  {
    EventId++;
    std::stringstream ss(line);
    read_events(ss, params.data_format, ts, c, r, p); // ---- per-event ingest ----

    // Stop condition: either by max event count or by max timestamp, whichever the config sets.
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

    e << c, r, ts; // pack the event into an Eigen vector for downstream Kalman update calls
    if (t_last == -1){
      t_last = ts; // initialise on first iteration
    }

    high_pass(ts, c, r, p, params.alpha); // update the reconstructed-image pixel this event touched

    // Whenever the timestamp advances (new event time-slot begins), decide whether the
    // PREVIOUS time-slot should be marked as "had an associated event" for logging purposes.
    if (ts - t_last > 0){
      if (f_associated_this_ts){
        f_write_position = 1;
      }
      else {
        f_write_position = 0;
      }
      f_associated_this_ts = 0; // reset for the new timestamp
    }

    //--------------------------------------------------------------------------------------------------------//
    // (Several large commented-out blocks below are legacy/debug code for writing raw blob
    //  measurements or per-event track positions to file. Left in place but inactive.)
    //--------------------------------------------------------------------------------------------------------//

    bool f_event_associated = 0; // Was THIS specific event matched to an existing track?

    if (track_manager.len() > 0)
    {
      //////////////////////////////////////////////////////////////////////
      // --- LED communication decoding (comms mode only) ---
      // For every existing track, check whether this event is close enough to be
      // considered part of that track's LED blink pattern, and if so feed it to
      // that track's decoder object.
      if (flag_decoding)
      {
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
      // --- Nearest-neighbour data association ---
      // Find which existing track's predicted position is CLOSEST to this event's pixel.
      double dist_min = 1e6; // sentinel "very large" starting distance

      for (int i = 0; i < track_manager.len(); i++)
      {
        x_hat_track = track_manager.getTrack(i)->state();
        distance = sqrt(pow((c - x_hat_track(0)), 2) + pow((r - x_hat_track(1)), 2));

        if (distance < dist_min)
        {
          dist_min = distance;
          id = i; // remember the index of the closest track
        }
      }

      //--------------------------------------------------------------------------------------------------------//
      // Pull out the closest track's full state/covariance for the threshold logic below.
      x_hat_track = track_manager.getTrack(id)->state();
      P_track = track_manager.getTrack(id)->P_x();
      double ts_last_for_gamma = track_manager.getTrack(id)->get_ts_last_for_gamma();
      double dist_threshold_track = track_manager.getTrack(id)->get_dist_threshold();

      distance = sqrt(pow((c - x_hat_track(0)), 2) + pow((r - x_hat_track(1)), 2));

      // --- Adaptive association-distance threshold ---
      // Only recompute the "gate" size when the track's position is confident (low covariance).
      // The gate size adapts toward ~2.5x the target's estimated width/height (whichever is
      // bigger), blended smoothly over time using an exponential-decay factor `gamma` based on
      // how long it's been since the last update.
      if ((P_track(0, 0) < 3) && (P_track(1, 1) < 3))
      {
        double gamma = std::exp(-alpha_dist * (ts - ts_last_for_gamma));

        if (x_hat_track(4) > x_hat_track(5))
        {
          dist_threshold_track = gamma * dist_threshold_track + (1 - gamma) * 2.5 * x_hat_track(4);
        }
        else
        {
          dist_threshold_track = gamma * dist_threshold_track + (1 - gamma) * 2.5 * x_hat_track(5);
        }

        // Never let the gate shrink below the configured minimum.
        if (dist_threshold_track < params.dist_threshold)
        {
          dist_threshold_track = params.dist_threshold;
        }

        track_manager.getTrack(id)->update_distance_threshold(dist_threshold_track);
      }

      //--------------------------------------------------------------------------------------------------------//
      // If the event falls within the gate, treat it as belonging to this track and
      // feed it into that track's Kalman filter update step.
      if (distance < dist_threshold_track)
      {
        f_event_associated = 1;
        track_manager.getTrack(id)->update(e, 0, p);
      }
    }

    f_associated_this_ts = (f_associated_this_ts | f_event_associated);

    //------------------------------------------------------------------------------------------------//
    // --- Auto-detection of NEW targets (mode 2 only) ---
    // Any event that did NOT get associated to an existing track is treated as
    // "background/candidate" activity and passed to the SAEdetector.
    if ((params.select_target_flag == 2) &
        (f_event_associated == 0))
    {
      detector->addEvent({(double) c, (double) r, ts, (double) p});
      detection_event_count++;
    }

    // Actually attempt detection only when ALL of these hold:
    //  - we're in auto-detect mode
    //  - this event wasn't already claimed by a track
    //  - we haven't hit the max track count / max target count yet
    //  - we're far enough from existing tracks (or there are no tracks yet)
    //  - enough unassociated events have accumulated since the last detection attempt (rate limit)
    if ((params.select_target_flag == 2) &
        (f_event_associated == 0) &
        (track_manager.len() < params.MAX_NUMBER_OF_TRACKS) &
        (track_manager.len() < params.n_target) &
        (distance > params.detector_dist_threshold || track_manager.len() == 0) &
        (detection_event_count > params.SAE_operation_rate))
    {
        detection_event_count = 0;
        // (Alternate/legacy detection function kept for reference:)
        // int detector_output = detector->performDetection({(double) c, (double) r, ts, (double) p});
        int detector_output = detector->performDetection_dt({(double) c, (double) r, ts, (double) p});

        if (detector_output == 1)
        {
          // ---- NEW KalmanFilter TRACK CREATED HERE (inside TrackManager) ----
          track_manager.createNewTrack({(double) c, (double) r, ts});
        }
    }

    //------------------------------------------------------------------------------------------------//
    // --- Track evaluation / pruning ---
    // Periodically ask TrackManager to check whether any tracks should be deleted, using one of
    // three strategies depending on config and how many events have passed:
    std::vector<int> deleted_IDs, deleted_IDs_temp;

    // (a) Duplicate-track check: runs only every 100 events, and only if there's more than 1 track.
    double_track_evaluation_counter++;
    if ((track_manager.len() > 0) & (params.f_evaluate == 1) & ((double_track_evaluation_counter > 100) & (track_manager.len() > 1)))
    {
      double_track_evaluation_counter = 0;
      deleted_IDs_temp = track_manager.evaluateDoubleTracks();
      if (deleted_IDs_temp.size() > 0){
        deleted_IDs.insert(deleted_IDs.end(), deleted_IDs_temp.begin(), deleted_IDs_temp.end());
      }
    }
    // (b) Full evaluation (age/activity/etc.) - runs every event otherwise, if enabled.
    else if ((track_manager.len() > 0) & (params.f_evaluate == 1)){
      deleted_IDs_temp = track_manager.evaluateTracks(ts);
      if (deleted_IDs_temp.size() > 0){
        deleted_IDs.insert(deleted_IDs.end(), deleted_IDs_temp.begin(), deleted_IDs_temp.end());
      }
    }
    // (c) Fallback: only delete tracks that have left the frame (position-only check).
    else if ((track_manager.len() > 0) & (params.f_evaluate == 0)){
      deleted_IDs_temp = track_manager.evaluateTracksPosition();
      if (deleted_IDs_temp.size() > 0){
        deleted_IDs.insert(deleted_IDs.end(), deleted_IDs_temp.begin(), deleted_IDs_temp.end());
      }
    }

    // If anything was deleted, the same-timestamp event buffer is now stale, so clear it.
    if (deleted_IDs.size() > 0) {
      while(!same_ts_e_buffer.empty()){
        same_ts_e_buffer.pop();
      }
    }

    // Log every event that WAS associated to a track, in a simple CSV-like format.
    if (f_event_associated){
      blob_measurements_txt_ << std::setprecision(0) << std::fixed;
      blob_measurements_txt_ << c << "," << r << "," << p << "," << int(ts*1e6) << std::endl;
    }

    //--------------------------------------------------------------------------------------------------------//
    // --- Periodic display refresh ---
    // Rather than redrawing on every single event (which would be far too slow), the display
    // is only refreshed at `publish_framerate` Hz (measured in event-time, not wall-clock time).
    if (params.publish_framerate > 0 && ts > t_next_publish)
    {
      std::cout << ts << std::endl;
      high_pass_global(ts, params.alpha); // decay the WHOLE image up to `ts` (not just one pixel)
      display(dispParams, params, ts, output_video, output_video_ref,
              image_ref_ts, p, track_manager.getTracks());
      t_next_publish = ts + (1.0 / params.publish_framerate);
    }

    // --- Bookkeeping after evaluation: keep `id` valid if a track was deleted this iteration ---
    if (track_manager.len() > 0)
    {
      if (std::find(deleted_IDs.begin(), deleted_IDs.end(), id) == deleted_IDs.end()){
        // `id` itself wasn't deleted, but if a LOWER-indexed track was deleted, all higher
        // indices shift down by one, so we may need to decrement `id` to keep pointing at
        // the same logical track.
        bool is_larger = std::all_of(deleted_IDs.begin(), deleted_IDs.end(), [id](int elem) { return id >= elem;});
        if ((deleted_IDs.size()>0) & (is_larger)){
          id--;
        }

        if (id < track_manager.len()){
          track_manager.getTrack(id)->update_ts_last_for_gamma(ts); // record last-update time for the gate-smoothing logic above
        }
      }

      ts_kf_last = ts;
    }

    //--------------------------------------------------------------------------------------------------------//
    t_last = ts; // advance the "previous timestamp" marker for next iteration
  }

  writer.release(); // flush/close the output video file
  return 0;
}

// ============================================================================
// HIGH-PASS FILTER (per-pixel)
// ----------------------------------------------------------------------------
// This is the classic event-camera "surface of active events" reconstruction:
// each pixel's log-intensity estimate exponentially decays toward 0 over time,
// and gets bumped up/down by contrast_threshold whenever a new event arrives
// at that pixel. The decay uses the ELAPSED time since that pixel's last event
// (ts - ts_array), not wall-clock time, and `alpha` controls the decay speed.
// ============================================================================
void high_pass(double ts, int x, int y, int p, int &alpha)
{
  log_intensity_state.at<double>(y, x) =
      exp(-alpha * (ts - ts_array.at<double>(y, x))) *
      log_intensity_state.at<double>(y, x);
  log_intensity_state.at<double>(y, x) +=
      (p > 0) ? -contrast_threshold : contrast_threshold; // polarity determines the sign of the bump
  ts_array.at<double>(y, x) = ts; // remember when this pixel was last touched
};

// ============================================================================
// HIGH-PASS FILTER (whole image, vectorized)
// Applies the same exponential decay to EVERY pixel at once (used right before
// rendering a frame, so that pixels which haven't received an event recently
// still fade out correctly rather than staying "stuck" at their last value).
// ============================================================================
void high_pass_global(double ts, int &alpha)
{
  cv::Mat beta;
  cv::exp(-alpha * (ts - ts_array), beta); // decay factor per-pixel
  log_intensity_state = log_intensity_state.mul(beta); // element-wise multiply
  ts_array.setTo(ts); // mark every pixel as "up to date" as of `ts`
};

// ============================================================================
// RENDER + SAVE ONE DISPLAY FRAME
// ----------------------------------------------------------------------------
// Converts the current reconstructed log-intensity image into a viewable
// 8-bit RGB image, draws an ellipse for each track (position/size/orientation
// taken straight from that track's Kalman state), optionally overlays a
// reference RGB camera frame, handles a couple of keyboard shortcuts, and
// optionally writes the frame to a video file / PNG.
// ============================================================================
void display(const DisplayParams &dispParams, Parameters &params,
             const double &ts, std::vector<cv::Mat> &output_video,
             std::vector<cv::Mat> &output_video_ref,
             std::vector<double> &image_ref_ts, int p, std::vector<KalmanFilter *> current_tracks)
{
  cv::Mat image, ref_image;
  cv::exp(log_intensity_state, image); // undo the log: back to linear "intensity"
  double minVal = 1.4;
  double maxVal = 0.4;
  // NOTE: minVal > maxVal here, so this normalization is intentionally inverted
  // (produces a particular contrast look for this sensor's typical value range).
  image = (image - minVal) / (maxVal - minVal);
  cv::Mat img(image.rows, image.cols, CV_64FC1, (char *)image.data);
  img.convertTo(img, CV_8U, 255.0 / 1.0); // scale to 8-bit grayscale
  cv::Mat cimg;
  cv::cvtColor(img, cimg, cv::COLOR_GRAY2RGB); // convert to RGB so we can draw colored ellipses

  // Palette used for track visualization (not all currently applied per-track; see below).
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

    // State index 6 holds an ORIENTATION ANGLE in radians; convert to degrees (57.295 ~= 180/pi)
    // and wrap into [0, 360).
    double rotationAngle = x_hat_track(6) * 57.295;
    rotationAngle = std::fmod(rotationAngle, 360.0);
    if (rotationAngle < 0.0)
    {
      rotationAngle += 360.0;
    }

    // Draw the track's ellipse: center = (x, y) from state, size = (width, height) from state
    // indices 4/5 (target's estimated size/"lambda" parameters). abs() guards against the
    // filter occasionally producing a negative size estimate.
    // Confirmed ("validated") tracks are drawn in solid red; unvalidated "candidate" tracks
    // are optionally drawn in blue if the config asks to show candidates.
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

    // (Older, non-abs() version of the same drawing code, left commented for reference.)

    // Optionally overlay a second ellipse showing positional UNCERTAINTY (covariance),
    // scaled up by 50x so it's visible, in blue.
    if (dispParams.disp_covariance_flag)
    {
      cv::ellipse(
          cimg, cv::Point(x_hat_track(0), x_hat_track(1)),
          cv::Size(50 * P_track(0, 0), 50 * P_track(1, 1)),
          x_hat_track(6) * (180.0 / M_PI), 0, 360, cv::Scalar(255, 0, 0), 1,
          cv::LINE_AA);
    }
  }

  cv::imshow("Video", cimg); // show the reconstructed+annotated frame

  // --- Reference RGB image sync (optional) ---
  // Find the reference-camera frame whose timestamp is closest (>=) to the current event time,
  // and display/save it alongside the event-based reconstruction.
  if (params.ref_image_ts_flag)
  {
    double event_ts = ts * 1e6 + t1; // convert back to raw (un-zeroed) microsecond timestamp
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

    image_id = image_id + 1; // reference images are presumably 1-indexed on disk
    std::string ref_image_path = params.input_folder_path + "/reproject_rgb/" +
                                 std::to_string(image_id) + ".png";

    ref_image = cv::imread(ref_image_path); // ---- INGEST POINT: reads a PNG from disk per frame ----
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

  // --- Keyboard shortcuts ---
  int keyPressed = cv::waitKey(30);
  if (keyPressed == 27) // ESC: re-enter manual target selection mode mid-run
  {
    for (int i = 0; i < n_target; i++)
    {
      std::cout << "Press ESC to start selecting points, and press Enter when "
                   "finished.\n";
      std::cout << "Waiting to click to select a point ... \n";
      selectedPoint = cv::Point(-1, -1);
      while ((selectedPoint.x == -1 && selectedPoint.y == -1) &&
             keyPressed != 13)
      {
        keyPressed = cv::waitKey(1); // block until a click or Enter (13)
      }
      if (keyPressed == 13)
      {
        std::cout << "Selection completed.\n";
        std::cout << "Event Id: " << EventId << "\n";
        std::cout << x_hat << std::endl;
        // Re-initialise the filter matrices with fresh (larger) initial position variance,
        // since we're about to start a brand-new track from a click.
        params.var_x = 2;
        params.var_y = 2;
        init(params);
        break;
      }
      selectedPoints.push_back(selectedPoint);
      std::cout << "Selected point: (" << selectedPoint.x << ", "
                << selectedPoint.y << ")\n";
      std::cout << "Event Id: " << EventId << "\n";
      std::cout << "Time: " << ts << "\n";

      // Manually populate a fresh state vector for this clicked point.
      x_hat(i, 0) = selectedPoint.x;
      x_hat(i, 1) = selectedPoint.y;
      x_hat(i, 2) = 0; // vx
      x_hat(i, 3) = 0; // vy
      x_hat(i, 4) = 2; // size/lambda_1
      x_hat(i, 5) = 2; // size/lambda_2
    }
  }
  else if (keyPressed == 116 || keyPressed == 84) // 't' or 'T': print current status to console
  {
    std::cout << "ts: " << ts << " (second)\n";
    std::cout << "Event Id: " << EventId << "\n";
    std::cout << "Current location x: (" << x_hat(0, 0) << ", " << x_hat(0, 1)
              << ")\n";
    std::cout << "Current lambda: (" << x_hat(0, 4) << ")\n";
    std::cout << "Current P_x: (" << P_x(0, 0) << ", " << P_x(1, 1) << ")\n";
  }
  cv::waitKey(1); // tiny extra wait, common OpenCV idiom to let the GUI event loop process

  // --- Save outputs ---
  if (dispParams.save_video_flag)
  {
    writer.write(cimg);
    if (params.ref_image_ts_flag)
    {
      writer_ref.write(ref_image);
    }
  }
  if (dispParams.save_image_flag)
  {
    std::string output_image_name =
        output_image_path + "/image" + std::to_string(image_count) + ".png";
    cv::imwrite(output_image_name, cimg); // ---- OUTPUT: writes a PNG to disk ----

    if (params.ref_image_ts_flag)
    {
      std::string output_image_ref_name =
          output_image_path + "/image" + std::to_string(image_count) + "_ref.png";
      cv::imwrite(output_image_ref_name, ref_image);
    }
  }
  image_count += 1;
}

// ============================================================================
// MANUAL TARGET SELECTION UI (used once, before tracking starts, in mode 1)
// ----------------------------------------------------------------------------
// Shows the current reconstructed image and blocks waiting for the user to
// click `n_target` points (or press Enter early to select fewer). Populates
// the global `selectedPoints` vector, which main() later hands to the
// TrackManager to create the initial set of tracks.
// ============================================================================
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

  cv::imshow("Video", cimg);
  int keyPressed = cv::waitKey(30);
  for (int i = 0; i < n_target; i++)
  {
    std::cout << "Current event time: " << ts_global << std::endl;
    std::cout << "Press ESC to start selecting points, and press Enter when "
                 "finished.\n";
    std::cout << "Waiting to click to select a point ... \n";
    selectedPoint = cv::Point(-1, -1);

    while ((selectedPoint.x == -1 && selectedPoint.y == -1) &&
           keyPressed != 13)
    {
      keyPressed = cv::waitKey(1); // block until click or Enter
    }
    if (keyPressed == 13)
    {
      std::cout << "Event Id: " << EventId << "\n";
      std::cout << "Selection completed.\n";
      n_target = i; // user finished early; shrink n_target to however many were actually picked
      break;
    }
    selectedPoints.push_back(selectedPoint);
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

// ============================================================================
// BUILD THE KALMAN FILTER MATRICES
// ----------------------------------------------------------------------------
// Sets up the constant-velocity-style motion model matrices shared by every
// track's Kalman filter. The state vector (n_state entries, expected to be
// 10 based on usage elsewhere) appears to hold, per target:
//   [0] x position        [1] y position
//   [2] x velocity        [3] y velocity
//   [4] size/lambda_1     [5] size/lambda_2
//   [6] orientation theta [7] angular velocity q
//   [8], [9] extra states (initialised to 0, noise variance 50)
// ============================================================================
void init(Parameters &params)
{
  int n_target = 1; // NOTE: shadows the global n_target - hardcoded to 1 because these matrices
                     // describe a SINGLE track's filter; TrackManager makes one copy per target.

  F = Eigen::MatrixXd::Zero(n_target * n_state, n_state);
  C = Eigen::MatrixXd::Zero(3, n_state);        // maps state -> the 3 measured quantities (x, y, ts-related)
  R = Eigen::MatrixXd::Identity(3, 3);          // measurement noise covariance
  P = Eigen::MatrixXd::Zero(n_target * n_state, n_state); // initial estimate error covariance
  Q = Eigen::MatrixXd::Zero(n_state, n_state);  // process noise covariance
  A = Eigen::MatrixXd::Zero(2, 2);              // Sigma^(1/2) - used for shape/orientation estimation elsewhere
  x0 = Eigen::MatrixXd::Zero(n_target, n_state);

  for (int j = 0; j < n_target; j++)
  {
    for (int i = 0; i < n_state; i++)
    {
      F(j * n_state + i, i) = 1; // start as identity (each state persists by default)
    }
    // Constant-velocity coupling: position += velocity * dt
    F(j * n_state, 2) = params.dt;     // x += vx * dt
    F(j * n_state + 1, 3) = params.dt; // y += vy * dt
    F(j * n_state + 6, 7) = params.dt; // theta += angular_velocity * dt
  }

  Eigen::MatrixXd P_block = Eigen::MatrixXd::Zero(n_state, n_state);
  x0.block(0, 4, n_target, 1).setConstant(params.lambda_init);  // initial size param 1
  x0.block(0, 8, n_target, 2).setConstant(0);                   // initial extra states
  x0.block(0, 5, n_target, 1).setConstant(params.lambda_init);  // initial size param 2

  // Diagonal (uncorrelated) initial covariance, one variance value per state dimension.
  P_block.diagonal() << params.var_x, params.var_y, params.var_vx,
      params.var_vy, params.var_lambda_1, params.var_lambda_2, params.var_theta,
      params.var_q, 50, 50;

  // Diagonal process noise - how much we expect each state to drift/be uncertain per step.
  Q.diagonal() << params.q_x, params.q_y, params.q_vx, params.q_vy,
      params.q_lambda_1, params.q_lambda_2, params.q_theta, params.q_q, 5, 5;

  for (int row = 0; row < n_target; row++)
  {
    P.block(row * n_state, 0, n_state, n_state) << P_block;
  }
}

// ============================================================================
// PARSE ONE CSV LINE INTO AN EVENT (c, r, p, ts)
// ----------------------------------------------------------------------------
// Supports two possible CSV column orderings, chosen via params.data_format:
//   0: timestamp, column, row, polarity
//   1: column, row, polarity, timestamp
// Also handles: zero-basing the timestamp against the very first event seen
// (t1), converting microseconds -> seconds, and converting from 0-indexed to
// 1-indexed pixel coordinates (c+1, r+1) - possibly to reserve index 0 as a
// sentinel/border value elsewhere in the pipeline.
// ============================================================================
void read_events(std::stringstream &ss, const int data_format, double &ts,
                 int &c, int &r, int &p)
{
  if (data_format == 0)
  {
    ss >> ts;
    ss.get(); // consume the comma separator
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
    // t1 = ts;      // (originally would zero-base time to the first event; disabled below)
    t1 = 0;          // intentionally NOT subtracting the first timestamp - keeps raw values
    first_ts_flag = 1;
    std::cout << "ts: " << ts << std::endl;
  }
  ts = ts - t1;
  ts = ts * 1e-6; // convert microseconds -> seconds
  c = c + 1;       // shift to 1-indexed column
  r = r + 1;       // shift to 1-indexed row

  ts_global = ts;  // HACK: mirror into the global so display_for_select() can read it
}

// ============================================================================
// LOAD + APPLY ONE GYROSCOPE SAMPLE
// ----------------------------------------------------------------------------
// Reads a single line of gyro.csv (timestamp, gyroX, gyroY, gyroZ), applies a
// sensor-specific bias correction + unit conversion, then uses a pixel-domain
// optical-flow model (derived from the pinhole camera + rigid rotation
// equations) to estimate how much each track's apparent (vx, vy) should be
// adjusted to compensate for the camera's own rotation. This lets the tracker
// distinguish "the target moved" from "the camera rotated."
// ============================================================================
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
  gyro_ts = (gyro_ts - t1) * 1e-6; // zero-base + convert to seconds, same convention as event timestamps

  if (first_gyro_flag)
  {
    gyro_ts_last = gyro_ts;
    first_gyro_flag = false;
  }

  // Sensor-specific bias removal + unit conversion (radians/s -> degrees/s via 0.017453 = pi/180).
  // NOTE: these bias constants (1.7, 0.9, -0.2) and the DAVIS 240c comment indicate this is
  // calibrated for one specific camera/IMU pairing - would need re-tuning for other hardware.
  gyroX = (gyroX + 1.7) * 0.017453;
  gyroY = (gyroY + 0.9) * 0.017453;
  gyroZ = (gyroZ - 0.2) * 0.017453;

  // Camera focal lengths in pixels (also hardware-specific calibration constants).
  double f_x = 186.9;
  double f_y = 185.9;

  std::vector<int> ids = track_manager.getValidTrackIds();
  double gyro_dt = gyro_ts - gyro_ts_last;
  for (int id : ids)
  {
    x_hat_track = track_manager.getTrack(id)->state(); // shape (1, 10)

    double x0 = x_hat_track(0); // pixel x position (shadows the global x0 matrix intentionally, local scope)
    double x1 = x_hat_track(1); // pixel y position

    // Standard rotational optical-flow equations for a pinhole camera:
    //   vx_rot = (x*y/f_x)*wx - (f_x + x^2/f_x)*wy + y*wz
    //   vy_rot = (f_y + y^2/f_y)*wx - (x*y/f_y)*wy - x*wz
    double term1 = x0 * x1 / f_x;
    double term2 = f_x + (x0 * x0 / f_x);
    double vx_gyro = term1 * gyroX - term2 * gyroY + x1 * gyroZ;
    double term3 = f_y + (x1 * x1 / f_y);
    double term4 = x0 * x1 / f_y;
    double vy_gyro = term3 * gyroX - term4 * gyroY - x0 * gyroZ;

    // Apply the rotation-induced velocity as a correction to this track's Kalman state,
    // scaled by the elapsed time since the last gyro sample.
    track_manager.getTrack(id)->update_gyro_single_id(vx_gyro * gyro_dt, vy_gyro * gyro_dt);
  }
  gyro_ts_last = gyro_ts;
}