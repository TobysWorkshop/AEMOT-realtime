
#include <vector>
#include <fstream>
#include <filesystem>
#include <iostream>

#include "TrackManager.hpp"
#include "kalman.hpp"

//------------------------------------------//
//-----        PUBLIC FUNCTIONS        -----//
//------------------------------------------//

//************************************************************************************//
//**********                          CONSTRUCTOR                           **********//
//************************************************************************************//
TrackManager::TrackManager(double dt, Eigen::MatrixXd F, Eigen::MatrixXd C,
                           Eigen::MatrixXd Q, Eigen::MatrixXd R,
                           Eigen::MatrixXd P, Eigen::MatrixXd A,
                           Eigen::MatrixXd x0, int ring_buffer_len, int n_state)
    : dt(dt), F(F), C(C), Q(Q), R(R), P(P), A(A),
      x0_template(x0), ring_buffer_len(ring_buffer_len), n_state(n_state) {
      };

void TrackManager::update_default_dist_threshold(double dist_threshold)
{
    default_dist_threshold = dist_threshold;
}

void TrackManager::update_frame_dimensions(int height, int width)
{
    this->height = height;
    this->width = width;
}

void TrackManager::update_event_rate_threshold(double threshold)
{
    this->event_rate_threshold = threshold;
}

void TrackManager::store_parameters(double ts_age, double dt_terminate, double low_activity_factor){
    this->evaluate_ts_age = ts_age;
    this->evaluate_dt_terminate = dt_terminate;
    this->evaluate_low_activity_factor = low_activity_factor;
}

//************************************************************************************//
//**********                         ACCESS TRACKS                          **********//
//************************************************************************************//
// Get the vector of all output/candidate tracks
std::vector<KalmanFilter *> TrackManager::getTracks()
{
    return output_tracks;
}

// Access individual tracks
KalmanFilter *TrackManager::getTrack(int id)
{
    if (id >= output_tracks.size() || id < 0)
    {
        throw std::runtime_error("Requested ID out of range of Output Tracks");
    }
    return output_tracks[id];
}

// Get valid track IDs
std::vector<int> TrackManager::getValidTrackIds()
{
    std::vector<int> valid_ids;
    for (int i = 0; i < output_tracks.size(); ++i)
    {
        if (output_tracks[i] != nullptr) // or add more checks if needed
        {
            valid_ids.push_back(i);
        }
    }
    return valid_ids;
}

//************************************************************************************//
//**********                       ADD MANUAL POINTS                        **********//
//************************************************************************************//
// Create trackers from the selected points
void TrackManager::addSelectedPoints(std::vector<cv::Point> selected_points, double ts)
{
    // Delete the existing trackers
    std::cout << "Initialising new tracks" << std::endl;
    for (int id = 0; id < output_tracks.size(); id++)
    {
        std::cout << "Tracks are deleted" << std::endl;
        deleteTrack(output_tracks, id);
    }
    output_tracks.clear();

    // Initialise the blobs
    for (int i = 0; i < selected_points.size(); i++)
    {
        Eigen::MatrixXd x0_new_track = x0_template;
        x0_new_track(0) = selected_points[i].x;
        x0_new_track(1) = selected_points[i].y;

        // Create new kalman filter pointer and store in vector
        KalmanFilter *new_kf = new KalmanFilter(dt, F, C, Q, R, P, A, x0_new_track, n_state, ring_buffer_len);
        new_kf->init(ts, " ");
        new_kf->update_distance_threshold(this->default_dist_threshold);
        new_kf->setID(next_unique_id);
        new_kf->openOutputFile(); // HACK
        new_kf->validated = 1;
        output_tracks.push_back(new_kf);

        next_unique_id++;
    }
}

//************************************************************************************//
//**********                        SPAWN CANDIDATES                        **********//
//************************************************************************************//
// Create new candidate tracker
void TrackManager::createNewTrack(Eigen::Vector<double,3> new_point){
    Eigen::MatrixXd x0_new_track = x0_template;
    x0_new_track(0) = new_point(0);
    x0_new_track(1) = new_point(1);

    // Create new kalman filter pointer and store in vector
    KalmanFilter *new_kf = new KalmanFilter(dt, F, C, Q, R, P, A, x0_new_track, n_state, ring_buffer_len);
    new_kf->init(new_point(2), " ");
    new_kf->update_distance_threshold(this->default_dist_threshold);
    new_kf->update_ts_last_for_gamma(new_point(2));
    new_kf->setID(next_unique_id);
    // new_kf->openOutputFile(); // HACK

    if (f_decode){
        new_kf->initialiseDecoder(decoder_dt, decoder_data_name, decoder_input_event_path, decoder_contrast_threshold, next_unique_id);
    }

    output_tracks.push_back(new_kf);

    next_unique_id++;
}

//************************************************************************************//
//**********                        EVALUATE TRACKS                         **********//
//************************************************************************************//
// Evaluate to 'validate' or 'delete' tracks
std::vector<int> TrackManager::evaluateTracks(double ts_now){
    std::vector<int> deleted_IDs; 

    // Iterate through all tracks and evaluate
    for (int i = output_tracks.size()-1; i >= 0; i--){

        if (ts_now - output_tracks[i]->get_t0() >= evaluate_ts_age){ 
            Eigen::Vector<double, 10> x_hat_i = output_tracks[i]->state().transpose();
            Eigen::Vector<double, 10> cov_i = output_tracks[i]->P_x().diagonal();
                   
            // Conditions
            bool out_of_frame = ((x_hat_i(0) < 10) || (x_hat_i(0) > width-11)) || ((x_hat_i(1) < 10) || (x_hat_i(1) > height-11));
            bool inactive = (abs(ts_now - output_tracks[i]->get_ts_last()) > evaluate_dt_terminate);
            bool low_activity = ((1/output_tracks[i]->dt_moving_avg)) < evaluate_low_activity_factor * event_rate_threshold;
            bool invalid_shape = (x_hat_i(4) <= 0 || x_hat_i(5) <= 0);

            if(out_of_frame || inactive || low_activity || invalid_shape){
                if (output_tracks[i]->validated == 1){
                    std::cout << out_of_frame << "\t" << inactive << "\t" << low_activity << "\t" << invalid_shape << std::endl;
                    // std::cout << inactive << "\t" << abs(ts_now - output_tracks[i]->get_ts_last()) << "\t" << evaluate_dt_terminate << "\t\t" << ts_now << "\t" << output_tracks[i]->get_ts_last() << "\n\n";
                }
                // std::cout << out_of_frame << "\t" << inactive << "\t" << low_activity << "\t" << invalid_shape << std::endl;

                deleted_IDs.push_back(i);
                deleteTrack(output_tracks, i);        
                continue;
            }

            // std::cout << i << "\t" << output_tracks[i]->dt_moving_avg << "\t\t" << (1/output_tracks[i]->dt_moving_avg) << "\t\t" << event_rate_threshold << std::endl; 
            bool rate = (1/output_tracks[i]->dt_moving_avg) > event_rate_threshold;

            if(rate & (output_tracks[i]->validated==0)){
                output_tracks[i]->validated = 1;
                output_tracks[i]->openOutputFile();
                if (f_decode){
                   output_tracks[i]->decoder->open_G_file();
                }
            }
        }
    }

    return deleted_IDs;
}


// Evaluate tracks just by position
std::vector<int> TrackManager::evaluateTracksPosition(){
    std::vector<int> deleted_IDs; 

    // Iterate through all tracks and evaluate
    for (int i = output_tracks.size()-1; i >= 0; i--){
        Eigen::Vector<double, 10> x_hat_i = output_tracks[i]->state().transpose();
                   
        // Conditions
        bool out_of_frame = ((x_hat_i(0) < 10) || (x_hat_i(0) > width-11)) || ((x_hat_i(1) < 10) || (x_hat_i(1) > height-11));

        if(out_of_frame){
            deleted_IDs.push_back(i);
            deleteTrack(output_tracks, i);        
            continue;
        }
    }
    return deleted_IDs;
}



// Evaluate if tracks are tracking the same object
std::vector<int> TrackManager::evaluateDoubleTracks(){
    std::vector<int> deleted_IDs, deleted_IDs_temp; 

    // Iterate through all pairs of tracks
    for (int i = 0; i < output_tracks.size(); i++){
        for (int j = 0; j < output_tracks.size(); j++){
            // Skip if it is the same track in both loops
            if (i == j){
                continue;
            }
            else {
                // Read the state
                Eigen::Vector<double, 10> x_hat_1 = output_tracks[i]->state().transpose();
                Eigen::Vector<double, 10> x_hat_2 = output_tracks[j]->state().transpose();

                // Computer x and y distance
                double x_dist = abs(x_hat_1(0) - x_hat_2(0));
                double y_dist = abs(x_hat_1(1) - x_hat_2(1));

                if ((x_dist < 15) & (y_dist < 15)){
                    Eigen::Vector2d v1_norm = x_hat_1.segment(2,3).normalized();
                    Eigen::Vector2d v2_norm = x_hat_2.segment(2,3).normalized();

                    if ((v1_norm.transpose() * v2_norm) < 0.95){
                        deleted_IDs_temp.push_back(i);
                    }                    
                }
            }
        }  
    }

    // Deleted tracks
    for (int i = deleted_IDs_temp.size() - 1; i >= 0; i--){
        deleted_IDs.push_back(i);
        deleteTrack(output_tracks, deleted_IDs_temp[i]); 
        // std::cout << "Uh oh\n";       
    }

    return deleted_IDs;
}

//************************************************************************************//
//**********                         COMMUNICATION                          **********//
//************************************************************************************//
void TrackManager::initialiseDecoder(double dt, std::string data_name, std::string input_event_path, double contrast_threshold)
{
    for (int id = 0; id < output_tracks.size(); id++)
    {
        output_tracks[id]->initialiseDecoder(dt, data_name, input_event_path, contrast_threshold, id);
    }
}

void TrackManager::storeDecodeVariables(double dt, std::string data_name, std::string input_event_path, double contrast_threshold)
{
    decoder_dt = dt;
    decoder_data_name = data_name;
    decoder_input_event_path = input_event_path;
    decoder_contrast_threshold = contrast_threshold;
}

//------------------------------------------//
//-----        PRIVATE FUNCTIONS       -----//
//------------------------------------------//
void TrackManager::deleteTrack(std::vector<KalmanFilter *> &track_array, int track_id)
{
    KalmanFilter *ptr_to_delete = track_array[track_id];
    delete ptr_to_delete;
    track_array.erase(track_array.begin() + track_id);
}