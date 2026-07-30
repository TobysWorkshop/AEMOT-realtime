#ifndef TRACKMANAGER_HPP
#define TRACKMANAGER_HPP

#include <vector>
// #include <opencv4/opencv2/core/core.hpp>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Dense>
#include <fstream>

#include "kalman.hpp"

class TrackManager
{

public:
    //------------------------------------------//
    //-----        PUBLIC FUNCTIONS        -----//
    //------------------------------------------//
    // Constructor Functions
    TrackManager(double dt, Eigen::MatrixXd F, Eigen::MatrixXd C,
                 Eigen::MatrixXd Q, Eigen::MatrixXd R,
                 Eigen::MatrixXd P, Eigen::MatrixXd A,
                 Eigen::MatrixXd x0, int ring_buffer_len, int n_state, 
                 int pool_size);

    // Add track to output
    void createNewTrack(Eigen::Vector<double,3> new_point);

    // Evaluation
    std::vector<int> evaluateTracks(double ts);
    std::vector<int> evaluateTracksPosition();
    std::vector<int> evaluateDoubleTracks();

    // Initialise Decoding
    void initialiseDecoder(double dt, std::string data_name, std::string input_event_path, double contrast_threshold);

    // Access the track objects
    std::vector<KalmanFilter *> getTracks();
    KalmanFilter *getTrack(int id);

    // Destructor
    ~TrackManager();

    bool hasAvailableSlot();

    int len(){ return output_tracks.size(); };

    void update_default_dist_threshold(double dist_threshold);
    void update_frame_dimensions(int height, int width);
    void update_event_rate_threshold(double threshold);

    void store_parameters(double ts_age, double dt_terminate, double low_activity_factor);

    void setDecodeFlag(bool flag){f_decode = flag;};
    void storeDecodeVariables(double dt, std::string data_name, std::string input_event_path, double contrast_threshold);

    std::vector<int> getValidTrackIds();

    int activeCount();

private:
    //------------------------------------------//
    //-----        PRIVATE FUNCTIONS       -----//
    //------------------------------------------//
    void deleteTrack(std::vector<KalmanFilter *> &track_array, int track_id);

    //------------------------------------------//
    //-----        PRIVATE VARIABLES       -----//
    //------------------------------------------//
    // Vector to store the filters: dynamic list of Kalman filter pointers
    std::vector<KalmanFilter *> output_tracks;

    double dt;
    Eigen::MatrixXd F, C, R, P, Q, A, x0_template;
    int ring_buffer_len = 1;
    int n_state = 10;

    double default_dist_threshold;
    int height = 720;
    int width = 1280;

    int pool_size = 10; // set the number of available kalman objects here!!

    double event_rate_threshold;

    int next_unique_id = 0; 

    bool f_decode = 0;
    double decoder_dt;
    std::string decoder_data_name;
    std::string decoder_input_event_path;
    double decoder_contrast_threshold;

    double evaluate_ts_age;
    double evaluate_dt_terminate;
    double evaluate_low_activity_factor;
  

    // params.dt, F, C, Q, R, P, A, x0, n_state, params.ring_buffer_len
};

#endif
