///// ROS2 NODE AND ENTRANCE POINT TO THE SYSTEM /////
///// HANDLES DATA ASSOCIATION AND DISTRIBUTION //////
///// PASSES QUERIES TO THE SAEdetector AND THEN TRACK REQUESTS TO THE TrackManager //////

////// HAS 2 CLASSES : ONE FOR PROCESSING EVENTS, AND ONE FOR THE ROS2 EVENT-PACKET INJEST POINT


// ROS2
#include "rclcpp/rclcpp.hpp"

// Message types for publishing and subscribing to topics
#include "std_msgs/msg/string.hpp"
#include "event_camera_msg/msg/event_packet.hpp" //check this formatting

// Event processor for decoding raw event messages
#include <iostream>
#include <event_camera_codecs/event_processor.h>

// Local files/other modules
#include "kalman.hpp"        // Defines KalmanFilter class - one instance tracks one target's state
#include "parameters.h"      // Defines Parameters/DisplayParams structs + YAML loader functions
#include "TrackManager.hpp"  // Defines TrackManager - owns/creates/deletes all KalmanFilter tracks
#include "SAEdetector.hpp"   // Defines SAEdetector - automatic new-target detector from event data

// Other includes
#include "yaml-cpp/yaml.h"      
#include <eigen3/Eigen/Dense>   
#include <chrono>
#include <deque>
#include <filesystem>           
#include <fstream>               
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <memory>
#include <cstdint>
#include <math.h>
#include <opencv2/core/core.hpp>     
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc.hpp>  
#include <string>
#include <vector>
#include <queue>
#include <chrono>

//// PARAMS
std::string EVENTS_IN_TOPIC = "/event_camera/events";
double DISPLAY_HZ = 30.0;

////////


///////////////////// GLOBAL STATE //////////////////
double TTC, contrast_threshold, first_ts_flag;
// TTC              = Time-To-Contact (collision estimate) between two chosen targets
// contrast_threshold = the per-event brightness-change magnitude, used by high_pass()
// first_ts_flag    = flag: have we seen the first event timestamp yet? (used to zero the clock)

int image_count = 0, image_id = 0, start_event_ts = 0;
// image_count = counter used for naming saved output images/frames
// image_id   = index into the reference image timestamp list (for RGB overlay sync)

double alpha_dist = 1.4; // Decay rate used to smooth/update the per-track association distance threshold

bool use_gyro_flag = false;

Eigen::MatrixXd x_hat_track, P_track;
// Choose two target IDs to compute time-to-contact; default id=0 and id=1
int id_ttc_left = 0;
int id_ttc_right = 1;

std::queue<std::pair<int, std::pair<int, int>>> same_ts_e_buffer; // Buffer of events sharing a timestamp (cleared on track deletion)
cv::Mat log_intensity_state, ts_array;
// log_intensity_state = per-pixel running "brightness" estimate built from events (the reconstructed image)
// ts_array            = per-pixel timestamp of the last event seen there (needed for the exponential decay)

// ---- Kalman filter matrix templates ----
// built once in EventProcessor::initialise(), then handed to TrackManager (which makes its own per-track copies)
Eigen::MatrixXd F, C, R, P, Q, A, P_x, x0, x_hat;
// F  = state transition matrix (predicts next state from current state)
// C  = measurement/observation matrix
// R  = measurement noise covariance
// P  = initial state covariance (per target)
// Q  = process noise covariance
// A  = auxiliary matrix (Sigma^(1/2), used elsewhere for shape estimation)
// x0 = initial state vector template (per target)
// x_hat = working state estimate used mainly during manual target selection (key 'Esc' handler)

std::string input_event_path, output_image_path;
cv::Point selectedPoint(-1, -1);           
std::vector<cv::Point> selectedPoints;      
cv::VideoWriter writer_ref, writer;         

double ts_global = 0;
int EventId = 0;

// ---- Forward declarations of all free functions defined later in this file ----
void high_pass(double ts, int x, int y, int p, int &alpha);       
void high_pass_global(double ts, int &alpha);                                                       
void display(const DisplayParams &dispParams, Parameters &params,
             const double &ts, std::vector<cv::Mat> &output_video,
             std::vector<cv::Mat> &output_video_ref,
             std::vector<double> &image_ref_ts, int p, std::vector<KalmanFilter *> current_tracks
);        



////////// EVENT PROCESSOR MODULE //////////////
class EventProcessor : public event_camera_codecs::EventProcessor {
    public:
        bool initialise(const std::string &config_name)
        {
            // PARSE THE CONFIG //
            try
            {
                std::string file_config_path = "../configs/" + config_name + ".yaml";
                params = loadParametersFromYAML(file_config_path);
                dispParams = loadDispParametersFromYAML(file_config_path);
            }
            catch (const std::exception &ex)
            {
                std::cerr << "Error: " << ex.what() << std::endl;
                return false; // Fail fast if config is missing/invalid
            }

            // Allocate the two per-pixel accumulator images used for the event->intensity reconstruction.
            log_intensity_state = cv::Mat::zeros(params.height, params.width, CV_64FC1);
            ts_array = cv::Mat::zeros(params.height, params.width, CV_64FC1);
            use_gyro_flag = params.use_gyro_flag;
            contrast_threshold = params.contrast_threshold;
            int n_state = params.n_state;
            input_event_path = params.input_folder_path + params.input_data_name; // base path (no extension) for this dataset

            // SET UP DISPLAY WINDOW //
            cv::namedWindow("Video");
            std::string output_ref_video_path = input_event_path + "_video_ref.avi";
            std::string output_video_path = input_event_path + "_video.avi";
            if (params.save_video_flag)
            {
                // Opens an .avi writer for the reconstructed event-video (MJPG codec, fixed 40 fps)
                writer.open(output_video_path,
                            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 40.0,
                            cv::Size(params.width, params.height)
                );

            }

            // If saving individual frame images was requested, create the output folder.
            if (params.save_image_flag)
            {
                std::filesystem::path path(input_event_path + "/output_img");
                std::filesystem::create_directories(path);
                output_image_path = path.string();
            }

            //blob_measurements_txt_.open(input_event_path + "_measurements.csv");

            // INITALISE SAE-DETECTOR //
            detector = std::make_unique<SAEdetector>(params.height,
                                    params.width,
                                    params.SAE_ksize,
                                    params.SAE_alpha,
                                    params.SAE_min_contributions,
                                    params.SAE_min_active_pixels,
                                    params.SAE_detection_threshold,
                                    params.detector_dt_threshold
            );
            

            // BUILD THE KALMAN FILTER MATRIX TEMPLATES //
            int n_target_local = 1;
            F = Eigen::MatrixXd::Zero(n_target_local * n_state, n_state);
            C = Eigen::MatrixXd::Zero(3, n_state);        // maps state -> the 3 measured quantities (x, y, ts-related)
            R = Eigen::MatrixXd::Identity(3, 3);          // measurement noise covariance
            P = Eigen::MatrixXd::Zero(n_target_local * n_state, n_state); // initial estimate error covariance
            Q = Eigen::MatrixXd::Zero(n_state, n_state);  // process noise covariance
            A = Eigen::MatrixXd::Zero(2, 2);              // Sigma^(1/2) - used for shape/orientation estimation elsewhere
            x0 = Eigen::MatrixXd::Zero(n_target_local, n_state);

            for (int j = 0; j < n_target_local; j++)
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
            x0.block(0, 4, n_target_local, 1).setConstant(params.lambda_init);  // initial size param 1
            x0.block(0, 8, n_target_local, 2).setConstant(0);                   // initial extra states
            x0.block(0, 5, n_target_local, 1).setConstant(params.lambda_init);  // initial size param 2

            // Diagonal (uncorrelated) initial covariance, one variance value per state dimension.
            P_block.diagonal() << params.var_x, params.var_y, params.var_vx,
                params.var_vy, params.var_lambda_1, params.var_lambda_2, params.var_theta,
                params.var_q, 50, 50;

            // Diagonal process noise - how much we expect each state to drift/be uncertain per step.
            Q.diagonal() << params.q_x, params.q_y, params.q_vx, params.q_vy,
                params.q_lambda_1, params.q_lambda_2, params.q_theta, params.q_q, 5, 5;

            for (int row = 0; row < n_target_local; row++)
            {
                P.block(row * n_state, 0, n_state, n_state) << P_block;
            }

            // SETUP THE TRACK MANAGER //
            track_manager = std::make_unique<TrackManager>(params.dt, F, C, Q, R, P, A, x0, params.ring_buffer_len, n_state);
            track_manager->update_default_dist_threshold(params.dist_threshold);
            track_manager->update_frame_dimensions(params.height, params.width);
            track_manager->update_event_rate_threshold(params.event_rate_threshold);
            track_manager->store_parameters(
                params.evaluate_ts_age, params.evaluate_dt_terminate, params.evaluate_low_activity_factor
            );

            // RENDER INITIAL EMPTY FRAME //
            display(dispParams, params, 0.0, output_video, output_video_ref, image_ref_ts, 0, track_manager->getTracks());

            return true;

        } // end initialise

        // This runs every time a decoded event is pulled from the current event packet
        void eventCD(uint64_t ts_ns, uint16_t c, uint16_t r, uint8_t p) override {
            // ts_ns = sensor timestamp in nanoseconds
            // c, r = column, row (pixel coordinates). Annalogous to x and y respectively
            // p = polarity

            std::lock_guard<std::mutex> lock(state_mutex);
            ///// HERE IS WHERE THE PIPELINE FROM THE ORIGINAL MAIN.CPP CODE IS IMPLEMENTED!! /////
            EventId++;
            
            if (!have_first_ts) // assign t1 on first event
            {
                t1 = static_cast<double>(ts_ns);
                have_first_ts = true;
            }
            // convert ns to seconds
            double ts = (static_cast<double>(ts_ns) - t1) * 1e-9;

            // pack the event into an Eigen vector for downstream Kalman update calls
            e << c, r, ts;
            if (t_last == -1.0){
                t_last = ts; // initialise on first iteration
            }

            // update the reconstructed-image pixel this event touched
            high_pass(ts, c, r, p, params.alpha);

            // Whenever the timestamp advances, decide whether the
            // previous time-slot should be marked as "had an associated event" for logging purposes
            if (ts - t_last > 0)
            {
                f_write_position = f_associated_this_ts;
                f_associated_this_ts = false;
            }

            bool f_event_associated = false; // was this specific event matched to an existing track?

            //check if the track manager has any existing tracks its managing
            if (track_manager->len() > 0)
            {
                // NEAREST NEIGHBOUR DATA ASSOCIATION //
                //Find which existing track's predicted position is closest to this event's location
                double dist_min = 1e6; // a very large starting distance

                // itterate through the track manager's current tracks to find the closest
                for (int i = 0; i < track_manager->len(); i++)
                {
                    // * if the track isn't active (just a waiting container), skip it
                    if (!track_manager->getTrack(i)->active) continue;

                    x_hat_track = track_manager->getTrack(i)->state();
                    distance = sqrt(pow((c - x_hat_track(0)), 2) + pow((r - x_hat_track(1)), 2));

                    if (distance < dist_min)
                    {
                        dist_min = distance;
                        id = i; // remember the index of the closest track
                    }
                }

                // Pull out the closest track's full state/covarience to check if its feasible to now associate it
                x_hat_track = track_manager->getTrack(id)->state();
                P_track = track_manager->getTrack(id)->P_x();
                double ts_last_for_gamma = track_manager->getTrack(id)->get_ts_last_for_gamma();
                double dist_threshold_track = track_manager->getTrack(id)->get_dist_threshold();

                distance = sqrt(pow((c - x_hat_track(0)), 2) + pow((r - x_hat_track(1)), 2));

                // Check the feasibility by computing an adaptive association-distance threshold
                // We'll only recompute the threshold if the track's position is confident (has low covariance)
                // The threshold size adapts towards 2.5x the target's estimated width/height (whichever is bigger)
                // This is blended smoothly over timeusing exponential decay factor gamma for how long it's been since the last track update
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

                    track_manager->getTrack(id)->update_distance_threshold(dist_threshold_track);
                }

                // If the event falls within the threshold, absorb it into this track and feed it into that track's Kalman filter update step
                if (distance < dist_threshold_track)
                {
                    f_event_associated = 1; // flag this event as associated! This will mean we skip allocating it as a new target
                    track_manager->getTrack(id)->update(e, 0, p);
                }

            } // end nearest-neighbour data association step

            f_associated_this_ts = (f_associated_this_ts | f_event_associated);

            // AUTO_DETECTION OF NEW TARGETS //
            // If the event did NOT get associated to an existing track, then we treat it as a potential background event or candidate event.
            // And so we must pass it to the SAEdetector to classify it
            if (!f_event_associated) // if the event hasn't been associated yet
            {
                // we add this event to the detector waiting room, ready for the next detection attempt
                detector->addEvent({(double) c, (double) r, ts, (double) p}); //*
                detection_event_count++; //*
            }

            // Only initiate a detection attempt when ALL of the following hold:
            // - this event hasn't been claimed by a track
            // - the track manager has available tracking slots
            // - we're far enough from existing tracks (or there are no tracks yet)
            // - enough unassociated events have accumulated in the waiting room since the last detection attempt
            if ((!f_event_associated) &
                (track_manager->hasAvailableSlots()) & // * 
                (distance > params.detector_dist_threshold || track_manager->len() == 0) &
                (detection_event_count > params.SAE_operation_rate))
            {
                detection_event_count = 0;
                // perform the detection
                int detector_output = detector->performDetection_dt({(double) c, (double) r, ts, (double) p});

                if (detector_output == 1) // confirmed candidate! Move to start a new track
                {
                    // ---- NEW KalmanFilter TRACK CREATED HERE (inside TrackManager) ----
                    track_manager->createNewTrack({(double) c, (double) r, ts});
                }
            }

            // TRACK EVALUATION AND PRUNING //
            // Periodically ask the TrackManager to check whether any tracks should be deleted.
            // How we do this (one of three ways) is based on the config and how many events have passed:
            std::vector<int> deleted_IDs, deleted_IDs_temp;

            // (a) Duplicate-track check: runs only every 100 events, and only if there's more than 1 track.
            double_track_evaluation_counter++;
            if ((track_manager->len() > 0) & (params.f_evaluate == 1) & ((double_track_evaluation_counter > 100) & (track_manager->len() > 1)))
            {
                double_track_evaluation_counter = 0;
                deleted_IDs_temp = track_manager->evaluateDoubleTracks();
                if (deleted_IDs_temp.size() > 0){
                    deleted_IDs.insert(deleted_IDs.end(), deleted_IDs_temp.begin(), deleted_IDs_temp.end());
                }
            }
            // (b) Full evaluation (age/activity/etc.) - runs only every 100 events, if enabled.
            else if ((track_manager->len() > 0) & (params.f_evaluate == 1) & ((double_track_evaluation_counter > 100)){
                deleted_IDs_temp = track_manager->evaluateTracks(ts);
                if (deleted_IDs_temp.size() > 0){
                    deleted_IDs.insert(deleted_IDs.end(), deleted_IDs_temp.begin(), deleted_IDs_temp.end());
                }
            }
            // (c) Fallback: only delete tracks that have left the frame (position-only check).
            else if ((track_manager->len() > 0) & (params.f_evaluate == 0)){
                deleted_IDs_temp = track_manager->evaluateTracksPosition();
                if (deleted_IDs_temp.size() > 0){
                    deleted_IDs.insert(deleted_IDs.end(), deleted_IDs_temp.begin(), deleted_IDs_temp.end());
                }
            }

            // If anything was deleted, the same-timestamp event buffer is now stale, so clear it.
            if (!deleted_IDs.empty()) {
                while(!same_ts_e_buffer.empty()){
                    same_ts_e_buffer.pop();
                }
            }

            // Log the event if it was associated to a track, in a simple CSV-like format.
            //if (f_event_associated){
            //    blob_measurements_txt_ << std::setprecision(0) << std::fixed;
            //    blob_measurements_txt_ << c << "," << r << "," << p << "," << int(ts*1e6) << std::endl;
            //}

            // BOOKEEPING AFTER EVALUATION //
            // Keep the track id valid if a track was deleted this iteration
            if (track_manager->len() > 0)
            {
                if (std::find(deleted_IDs.begin(), deleted_IDs.end(), id) == deleted_IDs.end()){
                    if (id < track_manager->len()){
                        track_manager->getTrack(id)->update_ts_last_for_gamma(ts); // record last-update time for the gate-smoothing logic above
                    }
                }

                ts_kf_last = ts;
            }

            t_last = ts; // advance the previous timestep marker, ready for the next event to come in!

        } // end eventCD() function

        // Seperate threaded function for rendering the display frames
        void renderFrame()
        {
            std::lock_guard<std::mutex> lock(state_mutex);

            if (!track_manager) return;

            high_pass_global(ts_kf_last_, params_.alpha);
            display(dispParams, params, ts_kf_last, output_video, output_video_ref,
                    image_ref_ts, 0, track_manager->getTracks());
        }

    private:
        Parameters params;
        DisplayParams dispParams;
        std::unique_ptr<TrackManager> track_manager;
        std::unique_ptr<SAEdetector> detector;

        // per-event state
        Eigen::Vector3d e = Eigen::Vector3d::Zero();
        double t1 = 0.0;
        bool have_first_ts = false;
        double t_last = -1.0;
        bool f_associated_this_ts = false;
        bool f_write_position = false;
        int double_track_evaluation_counter = 0;
        int id = 0;
        double distance = 0.0;
        double detection_event_count = 0.0;
        double t_next_publish = 0.0;
        double ts_kf_last = 0.0;

        std::vector<cv::Mat> output_video;
        std::vector<cv::Mat> output_video_ref;
        std::vector<double> image_ref_ts;

        std::mutex state_mutex;

        //std::ofstream blob_measurements_txt_;

}; // end EventProcessor class



///////////////// AEB MAIN NODE //////////////////////
class AEBNode : public rclcpp::Node
{
    public:
        AEBNode() : Node("AEB_node")
        {
            //// INIT ////

            // Declare and get parameters
            this->declare_parameter<std::string>("config_name", "default");

            std::string config_name = this->get_parameter("config_name").as_string();

            // Processor initialise() must be running before we start subscribing
            if (!processor.initialise(config_name))
            {
                RCLCPP_FATAL(this->get_logger(), "EventProcessor failed to initialise (bad config?)... shutting down.");
                rclcpp::shutdown();
                return;
            }

            // Create a wall-timer for the display refresh
            display_timer = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / DISPLAY_HZ),
                [this]() { processor.renderFrame(); });

            // SUBSCRIBERS
            event_sub = this->create_subscription<event_camera_msgs::msg::EventPacket>(
                EVENTS_IN_TOPIC, 10,
                std::bind(&AEBNode::eventpacket_cb, this, std::placeholders::_1)  
            );

            // PUBLISHERS
            status_publisher = this->create_publisher<std_msgs::msg::String>(
                "/AEB_status", 10
            );

            // Log startup message
            RCLCPP_INFO(this->get_logger(), "AEB Node has started.");
        }

    private:

        // runs every time we get an event packet
        void eventpacket_cb(const event_camera_msgs::msg::EventPacket::SharedPtr msg)
        {
            // Retrieve or instantiate the matching decoder instance for this packet type
            auto decoder = decoder_factory.getInstance(*msg);

            if (!decoder) {
                RCLCPP_ERROR(this->get_logger(), "Unknown package encoding: %s", msg->encoding.c_str());
                return;
            }
            
            // Unpack the compressed event packet array. This sends each unpacked event one by one to the EventProcessor::eventCD callback in the class below
            decoder->decode(*msg, &processor);
        }

        // Member Variables //

        // ROS2 things
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher;
        rclcpp::Subscription<event_camera_msgs::msg::EventPacket>::SharedPtr event_sub;

        // Event decoder structures
        event_camera_codecs::DecoderFactory<event_camera_msgs::msg::EventPacket, EventProcessor> decoder_factory;
        EventProcessor processor;

        // Timer for display refresh
        rclcpp::TimerBase::SharedPtr display_timer;
};




//////////////// ENTRY POINT /////////////////////////
int main(int argc, char * argv[])
{
    // Establish conenction to ROS2
    rclcpp::init(argc, argv);

    // Start up the node
    rclcpp::spin(std::make_shared<AEBNode>());

    // Shut down the process and disconnect from ROS2 on interrupt (Ctrl+C)
    rclcpp::shutdown();
    return 0;
}


////////////// OTHER FUNCTIONS /////////////////////

// HIGH PASS FILTER (per-pixel updates)
// A classic Surface of Active Events (SAE) visual reconstruction.
void high_pass(double ts, int x, int y, int p, int &alpha)
{
    log_intensity_state.at<double>(y, x) = exp(-alpha * (ts - ts_array.at<double>(y, x))) * log_intensity_state.at<double>(y, x);
        
    log_intensity_state.at<double>(y, x) += (p > 0) ? -contrast_threshold : contrast_threshold;
    ts_array.at<double>(y, x) = ts; // remember when this pixel was last activated
};

// HIGH PASS FILTER (whole-image updates)
// Applies an update to the entire SAE at once.
void high_pass_global(double ts, int &alpha)
{
    cv::Mat beta;
    cv::exp(-alpha * (ts - ts_array), beta); // decay factor per-pixel
    log_intensity_state = log_intensity_state.mul(beta);
    ts_array.setTo(ts); // mark every pixel as up to date as of ts
};

// RENDER AND SAVE ONE DISPLAY FRAME
// Converts the current reconstructed image into a viewable 8-bit RGB image.
// Draws an ellipse for each track (position/size/orientation taken straight from that track's Kalman state).
// Optionally writes the frame to a video file/png.
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
        cv::Scalar(128, 128, 0), cv::Scalar(128, 0, 128), cv::Scalar(0, 128, 128)
    };
    std::vector<cv::Scalar> colors_dark = {
        cv::Scalar(0, 0, 128), cv::Scalar(0, 128, 0), cv::Scalar(128, 0, 0),
        cv::Scalar(128, 128, 0), cv::Scalar(128, 0, 128), cv::Scalar(0, 128, 128)
    };

    for (int i = 0; i < current_tracks.size(); i++)
    {
        x_hat_track = current_tracks[i]->state();
        P_track = current_tracks[i]->P_x();

        // State index 6 holds an orientation angle in radians; convert to degrees (57.295 ~= 180/pi) and wrap into [0, 360)
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


    // --- KEYBOARD SHORTCUTS ---
    int keyPressed = cv::waitKey(30);
    if (keyPressed == 116 || keyPressed == 84) // 't' or 'T': print current status to console
    {
        std::cout << "ts: " << ts << " (second)\n";
        std::cout << "Event Id: " << EventId << "\n";
        std::cout << "Current location x: (" << x_hat(0, 0) << ", " << x_hat(0, 1) << ")\n";
        std::cout << "Current lambda: (" << x_hat(0, 4) << ")\n";
        std::cout << "Current P_x: (" << P_x(0, 0) << ", " << P_x(1, 1) << ")\n";
    }
    cv::waitKey(1);

    // --- Save outputs ---
    if (dispParams.save_video_flag)
    {
        writer.write(cimg);
    }
    if (dispParams.save_image_flag)
    {
        std::string output_image_name = output_image_path + "/image" + std::to_string(image_count) + ".png";
        cv::imwrite(output_image_name, cimg); // ---- writes a PNG to disk ----
    }
    image_count += 1;
}
