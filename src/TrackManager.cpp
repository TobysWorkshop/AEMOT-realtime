
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
                           Eigen::MatrixXd x0, int ring_buffer_len, int n_state,
                           int pool_size)
    : dt(dt), F(F), C(C), Q(Q), R(R), P(P), A(A),
      x0_template(x0), ring_buffer_len(ring_buffer_len), n_state(n_state), pool_size(pool_size)
{
    // Build the entire kalman filter line up on init
    output_tracks.reserve(pool_size);
    for (int i = 0; i < pool_size; ++i) 
    {
        KalmanFilter *kf = new KalmanFilter(dt, F, C, Q, R, P, A, x0_template, n_state, ring_buffer_len);
        kf->active = false;
        output_tracks.push_back(kf);
    }
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

TrackManager::~TrackManager()
{
    for (auto *kf : output_tracks)
    {
        delete kf;
    }
}

//************************************************************************************//
//**********                         ACCESS TRACKS                          **********//
//************************************************************************************//
// Get the vector of all output/candidate tracks
std::vector<KalmanFilter *> TrackManager::getTracks()
{
    std::vector<KalmanFilter *> active_tracks;
    for (auto *kf : output_tracks)
    {
        if (kf->active) active_tracks.push_back(kf);
    }
    return active_tracks;
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
        if (output_tracks[i]->active) // or add more checks if needed
        {
            valid_ids.push_back(i);
        }
    }
    return valid_ids;
}

//************************************************************************************//
//**********                        SPAWN CANDIDATES                        **********//
//************************************************************************************//
// Create new candidate tracker
void TrackManager::createNewTrack(Eigen::Vector<double,3> new_point)
{
    for (int i = 0; i < output_tracks.size(); ++i)
    {
        if (!output_tracks[i]->active)
        {
            // Reuse an inactive Kalman filter
            KalmanFilter *kf = output_tracks[i];

            Eigen::MatrixXd x0_new_track = x0_template;
            x0_new_track(0) = new_point(0);
            x0_new_track(1) = new_point(1);

            // reset the filter
            kf->reset(x0_new_track, new_point(2));

            kf->update_distance_threshold(default_dist_threshold);
            kf->update_ts_last_for_gamma(new_point(2));
            kf->setID(next_unique_id);
            kf->active = true;

            //debug
            std::cerr << "[new] slot=" << i
                        << " xy=(" << new_point(0) << "," << new_point(1) << ")"
                        << " t0=" << new_point(2)
                        << " activeCount=" << activeCount() << std::endl;

            if (f_decode){
                kf->initialiseDecoder(decoder_dt, decoder_data_name, decoder_input_event_path, decoder_contrast_threshold, next_unique_id);
            }

            next_unique_id++;
            return; // stop after filling one inactive slot
        }
    }
    
    std::cout << "Warning! Kalman lineup is exhausted - no free slot for an new track! Consider increasing pool_size.\n";
}

bool TrackManager::hasAvailableSlot()
{
    for (auto *kf : output_tracks)
    {
        if (!kf->active) return true;
    }
    return false;
}
//************************************************************************************//
//**********                        EVALUATE TRACKS                         **********//
//************************************************************************************//
// Evaluate to 'validate' or 'delete' tracks
std::vector<int> TrackManager::evaluateTracks(double ts_now){
    std::vector<int> deleted_IDs; 

    // Iterate through all tracks and evaluate
    for (int i = output_tracks.size()-1; i >= 0; i--){

        if(!output_tracks[i]->active) continue; // Skip inactive tracks

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


                //debug
                std::cerr << "[del] id=" << i
                            << " age=" << (ts_now - output_tracks[i]->get_t0())
                            << " out=" << out_of_frame
                            << " inact=" << inactive
                            << " low=" << low_activity
                            << " shape=" << invalid_shape
                            << " rate=" << (1.0 / output_tracks[i]->dt_moving_avg)
                            << " thr=" << event_rate_threshold
                            << " val=" << output_tracks[i]->validated
                            << " xy=(" << x_hat_i(0) << "," << x_hat_i(1) << ")"
                            << " lam=(" << x_hat_i(4) << "," << x_hat_i(5) << ")"
                            << std::endl;

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

        if(!output_tracks[i]->active) continue; // Skip inactive tracks

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

        if(!output_tracks[i]->active) continue; // Skip inactive tracks

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

int TrackManager::activeCount()
{
    int count = 0;
    for (auto *kf : output_tracks) if (kf->active) count++;
    return count;
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
    KalmanFilter *kf = track_array[track_id];
    kf->active = false; // Mark the Kalman filter as inactive
}