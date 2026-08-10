// Companion file
#include "processing.hpp"

// Local files
#include "kalman.hpp"       
#include "parameters.h"     
#include "TrackManager.hpp" 
#include "SAEdetector.hpp"  

// Other includes
#include <cstdint>
#include <iostream>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"      
#include <eigen3/Eigen/Dense>   
#include <algorithm>
#include <chrono>
#include <deque>
#include <filesystem>           
#include <fstream>               
#include <iomanip>
#include <memory>
#include <math.h>
#include <opencv2/core/core.hpp>     
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc.hpp>  
#include <string>
#include <vector>
#include <queue>

namespace processing {

    namespace { // global variables for this namespace

        // ---- run configuration / state ----
        Parameters params;
        DisplayParams dispParams;
        std::unique_ptr<TrackManager> track_manager;
        std::unique_ptr<SAEdetector> detector;

        // ---- display / video output ----
        std::vector<cv::Mat> output_video;
        std::vector<cv::Mat> output_video_ref;
        std::vector<double> image_ref_ts;
        cv::VideoWriter writer_ref, writer;
        std::string output_image_path;
        int image_count = 0;
        double t_next_publish = 0.0;

        // ---- reconstructed-image accumulators ----
        cv::Mat log_intensity_state, ts_array;
        double contrast_threshold = 0.0;
        bool use_gyro_flag = false;
        // Decay rate used to smooth/update the per-track association distance threshold.
        double alpha_dist = 1.4;
        // Buffer of events sharing a timestamp (cleared on track deletion).
        std::queue<std::pair<int, std::pair<int, int>>> same_ts_e_buffer;

        // ---- per-event bookkeeping ----
        uint64_t total_events = 0;
        bool have_first_ts = false;
        double t1 = 0.0;               // first event's raw sensor timestamp (µs), used as the zero point
        double t_last = -1.0;          // sentinel: -1.0 means "not yet set"
        double ts_kf_last = 0.0;
        double distance = 0.0;
        int id = 0;
        double detection_event_count = 0.0;
        int double_track_evaluation_counter = 0;
        int full_evaluation_counter = 0;
        bool f_associated_this_ts = false;
        bool f_write_position = false;
        Eigen::Vector3d e;              // event measurement, packed as (x, y, ts) each iteration
        Eigen::MatrixXd x_hat_track, P_track;

        // ---- Kalman filter matrix templates ----
        // built once in setup() then handed to TrackManager (which makes its own per-track copies)
        Eigen::MatrixXd F, C, R, P, Q, A, x0;

        std::string input_event_path;

    } // end namespace for global variables

    namespace {
        // Resolves the directory containing the currently-running executable, by
        // reading the Linux-specific /proc/self/exe self-symlink. This is
        // independent of the process's current working directory (unlike a plain
        // relative path), and independent of argv[0] (which isn't always a
        // resolvable path - e.g. when found via $PATH).
        //
        // Assumes configs/ sits next to the directory the built binary lives in
        // (src/../configs, per the project's current layout). If you later move
        // the build output somewhere else (e.g. a build/ directory), adjust the
        // ".." below - or better, make the configs directory itself overridable
        // (see the comment in setup() below).
        std::filesystem::path executable_directory() {
            char buffer[4096];
            const ssize_t length = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
            if (length <= 0) {
                throw std::runtime_error("could not resolve executable path via /proc/self/exe");
            }
            buffer[length] = '\0';
            return std::filesystem::path(buffer).parent_path();
        }
    }

    // ---- HIGH PASS FILTERS ---- //
    // (per-pixel updates) A classic Surface of Active Events (SAE) visual reconstruction.
    void high_pass(double ts, int x, int y, int p, int &alpha)
    {
        log_intensity_state.at<double>(y, x) = exp(-alpha * (ts - ts_array.at<double>(y, x))) * log_intensity_state.at<double>(y, x);
            
        log_intensity_state.at<double>(y, x) += (p > 0) ? -contrast_threshold : contrast_threshold;
        ts_array.at<double>(y, x) = ts; // remember when this pixel was last activated
    };
    // (whole-image updates) Applies an update to the entire SAE at once.
    void high_pass_global(double ts, int &alpha)
    {
        cv::Mat beta;
        cv::exp(-alpha * (ts - ts_array), beta); // decay factor per-pixel
        log_intensity_state = log_intensity_state.mul(beta);
        ts_array.setTo(ts); // mark every pixel as up to date as of ts
    };

    // ---- DISPLAY RENDERER --- //
    // Converts the current reconstructed image into a viewable 8-bit RGB image.
    // Draws an ellipse for each track (position/size/orientation taken straight from that track's Kalman state).
    // Optionally writes the frame to a video file/png.
    void display(
                const DisplayParams &dispParams, 
                Parameters &params,
                const double &ts,
                std::vector<cv::Mat> &output_video,
                std::vector<cv::Mat> &output_video_ref,
                std::vector<double> &image_ref_ts, 
                int p,
                std::vector<KalmanFilter *> current_tracks)
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
        cv::waitKey(1);

        // --- Save outputs ---
        if (params.save_video_flag && writer.isOpened())
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


    // ---- SETUP() ---- //
    // Runs once, before the first packet arrives.
    bool setup(const std::string &config_name) {
        // PARSE THE CONFIG //
        try {
            // configs/ is a sister directory of src/ (where this binary is built).
            // See executable_directory()'s comment above if you move the build output elsewhere.
            const std::filesystem::path file_config_path =
                executable_directory() / ".." / "configs" / (config_name + ".yaml");
            if (!std::filesystem::exists(file_config_path)) {
                throw std::runtime_error("config file not found: " + file_config_path.string());
            }
            params = loadParametersFromYAML(file_config_path.string());
            dispParams = loadDispParametersFromYAML(file_config_path.string());
        } catch (const std::exception &ex) {
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
        cv::resizeWindow("Video", params.width, params.height);
        std::string output_video_path = input_event_path + "_video.avi";
        if (params.save_video_flag)
        {
            // Prefer a codec/backend that works on Ubuntu without GStreamer drama
            int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
            writer.open(output_video_path, fourcc, 30.0, cv::Size(params.width, params.height), true);

            if (!writer.isOpened()) {
                // fallback
                fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
                writer.open(output_video_path, fourcc, 30.0, cv::Size(params.width, params.height), true);
            }
            if (!writer.isOpened()) {
                std::cerr << "ERROR: could not open VideoWriter for " << output_video_path << std::endl;
                params.save_video_flag = false;
            } else {
                std::cerr << "VideoWriter opened: " << output_video_path << std::endl;
            }
        }

        // If saving individual frame images was requested, create the output folder.
        if (params.save_image_flag)
        {
            std::filesystem::path path(input_event_path + "/output_img");
            std::filesystem::create_directories(path);
            output_image_path = path.string();
        }

        // INITALISE SAE-DETECTOR //
        detector = std::make_unique<SAEdetector>(
            params.height,
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
        track_manager = std::make_unique<TrackManager>(params.dt, F, C, Q, R, P, A, x0, params.ring_buffer_len, n_state, params.pool_size);
        track_manager->update_default_dist_threshold(params.dist_threshold);
        track_manager->update_frame_dimensions(params.height, params.width);
        track_manager->update_event_rate_threshold(params.event_rate_threshold);
        track_manager->store_parameters(
            params.evaluate_ts_age, params.evaluate_dt_terminate, params.evaluate_low_activity_factor
        );

        // RENDER INITIAL EMPTY FRAME //
        display(dispParams, params, 0.0, output_video, output_video_ref, image_ref_ts, 0, track_manager->getTracks());

        return true;
    } // end setup()


    // ---- EVENT PROCESSOR LOGIC ---- //
    // Runs once for every paclet pulled from the queue.
    void process_batch(const std::vector<sepia::dvs_event>& events) {
        // `events` is one batch (one USB packet's worth), events are in timestamp order within the batch and across batches.
        total_events += events.size();
        
        // debug (remove later):
        //std::cout << "received packet of " << events.size() << " events\n";
        
        // iterate over every event in the packet:
        for (const auto& event : events) {
            // event.t  -> microseconds
            // event.x  -> width pixel coord
            // event.y  -> height pixel coord
            // event.on -> true = ON, false = OFF

            if (!have_first_ts) // assign t1 on first event
            {
                t1 = static_cast<double>(event.t);
                have_first_ts = true;
            }
            // convert microseconds to seconds, relative to the first event
            double ts = (static_cast<double>(event.t) - t1) * 1e-6;

            //convert variables for AEMOT code compatibility
            uint16_t c = static_cast<uint16_t>(event.x);
            uint16_t r = static_cast<uint16_t>(event.y);
            uint8_t p = static_cast<uint8_t>(event.on);

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

            // NEAREST NEIGHBOUR DATA ASSOCIATION //
            //Find which existing track's predicted position is closest to this event's location
            bool any_active = false;
            double dist_min = 1e6; // a very large starting distance

            // itterate through the track manager's current tracks to find the closest
            for (int i = 0; i < track_manager->len(); i++)
            {
                // if the track isn't active (just a waiting container), skip it
                if (!track_manager->getTrack(i)->active) continue;
                any_active = true;
                x_hat_track = track_manager->getTrack(i)->state();
                distance = sqrt(pow((c - x_hat_track(0)), 2) + pow((r - x_hat_track(1)), 2));

                if (distance < dist_min)
                {
                    dist_min = distance;
                    id = i; // remember the index of the closest track
                }
            }

            if (any_active)
            {
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

                    if (x_hat_track(4) > x_hat_track(5)) {
                        dist_threshold_track = gamma * dist_threshold_track + (1 - gamma) * 2.5 * x_hat_track(4);
                    } else {
                        dist_threshold_track = gamma * dist_threshold_track + (1 - gamma) * 2.5 * x_hat_track(5);
                    }

                    // Never let the gate shrink below the configured minimum.
                    if (dist_threshold_track < params.dist_threshold) {
                        dist_threshold_track = params.dist_threshold;
                    }

                    track_manager->getTrack(id)->update_distance_threshold(dist_threshold_track);
                }

                // If the event falls within the threshold, absorb it into this track and feed it into that track's Kalman filter update step
                if (distance < dist_threshold_track)
                {
                    f_event_associated = true; // flag this event as associated! This will mean we skip allocating it as a new target
                    track_manager->getTrack(id)->update(e, 0, p);
                }
            }
            // end nearest-neighbour data association step

            f_associated_this_ts = (f_associated_this_ts | f_event_associated);

            // AUTO_DETECTION OF NEW TARGETS //
            // If the event did NOT get associated to an existing track, then we treat it as either a noise or object-candidate event.
            // And so we must pass it to the SAEdetector to classify it
            if (!f_event_associated) // if the event hasn't been associated yet
            {
                // we add this event to the detector waiting room, ready for the next detection attempt
                detector->addEvent({(double) c, (double) r, ts, (double) p});
                detection_event_count++;
            }

            // Only initiate a detection attempt when ALL of the following hold:
            // - this event hasn't been claimed by a track
            // - the track manager has available tracking slots
            // - we're far enough from existing tracks (or there are no tracks yet)
            // - enough unassociated events have accumulated in the waiting room since the last detection attempt
            if ((!f_event_associated) &
                (track_manager->hasAvailableSlot()) &
                (distance > params.detector_dist_threshold || track_manager->activeCount() == 0) &
                (detection_event_count > params.SAE_operation_rate))
            {
                detection_event_count = 0;
                // perform the detection
                int detector_output = detector->performDetection_dt({(double) c, (double) r, ts, (double) p});

                if (detector_output == 1) // confirmed candidate! Move to start a new track
                {
                    track_manager->createNewTrack({(double) c, (double) r, ts});

                    //debug
                    std::cerr << "[track] NEW track at (" << c << "," << r
                              << ") ts=" << ts
                              << " active=" << track_manager->activeCount() << std::endl;
                }
            }

            // TRACK EVALUATION AND PRUNING //
            // Periodically ask the TrackManager to check whether any tracks should be deleted.
            // How we do this (one of three ways) is based on the config and how many events have passed:
            std::vector<int> deleted_IDs, deleted_IDs_temp;

            // (a) Duplicate-track check: runs only every 100 events, and only if there's more than 1 track.
            double_track_evaluation_counter++;
            full_evaluation_counter++;

            if ((track_manager->activeCount() > 0) & (params.f_evaluate == 1) & ((double_track_evaluation_counter > 100) & (track_manager->activeCount() > 1)))
            {
                double_track_evaluation_counter = 0;
                deleted_IDs_temp = track_manager->evaluateDoubleTracks();
                if (deleted_IDs_temp.size() > 0){
                    deleted_IDs.insert(deleted_IDs.end(), deleted_IDs_temp.begin(), deleted_IDs_temp.end());
                }
            }
            // (b) Full evaluation (age/activity/etc.) - runs only every 100 events, if enabled.
            else if ((track_manager->activeCount() > 0) & (params.f_evaluate == 1) & (full_evaluation_counter > 1001)){
                full_evaluation_counter = 0;
                deleted_IDs_temp = track_manager->evaluateTracks(ts);
                if (deleted_IDs_temp.size() > 0){
                    deleted_IDs.insert(deleted_IDs.end(), deleted_IDs_temp.begin(), deleted_IDs_temp.end());
                }
            }
            // (c) Fallback: only delete tracks that have left the frame (position-only check).
            else if ((track_manager->activeCount() > 0) & (params.f_evaluate == 0)){
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

            // BOOKEEPING AFTER EVALUATION //
            // Keep the track id valid if a track was deleted this iteration
            if (track_manager->activeCount() > 0)
            {
                if (std::find(deleted_IDs.begin(), deleted_IDs.end(), id) == deleted_IDs.end()){
                    if (id >= 0 && id < track_manager->len()) {
                        track_manager->getTrack(id)->update_ts_last_for_gamma(ts); // record last-update time for the gate-smoothing logic above
                    }
                }
            }

            ts_kf_last = ts;
            t_last = ts; // advance the previous timestep marker, ready for the next event to come in!

            // PERIODIC DISPLAY REFRESH //
            // refresh at most once every 1/publish_framerate seconds of event time, on this same thread
            if (params.publish_framerate > 0 && ts > t_next_publish) {
                high_pass_global(ts, params.alpha);
                display(
                    dispParams, params, ts, output_video, output_video_ref, image_ref_ts, p,
                    track_manager->getTracks());
                t_next_publish = ts + (1.0 / params.publish_framerate);
            }
        }
    }

    void teardown() {
        if (writer.isOpened()) {
            writer.release();
        }

        std::cout << "processed " << total_events << " events total\n";
    }

} // namespace processing