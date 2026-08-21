#include <eigen3/Eigen/Dense>
#include <iostream>
#include <numeric>
#include <utility>

#include "SAEdetector.hpp"
#include "parameters.h"

// SAEdetector::SAEdetector(const Parameters &params)
SAEdetector::SAEdetector(int height, int width, int ksize, double alpha, double min_contributions, double min_active_pixels, double detection_threshold, double dt_detection_threshold)
{
    // Update properties from configuration file
    m_width = width;
    m_height = height;
    m_ksize = ksize;
    m_alpha = alpha;
    m_min_contributions = min_contributions;
    m_detection_threshold = detection_threshold;
    m_min_active_pixels = min_active_pixels;

    m_dt_detection_threshold = dt_detection_threshold;


    // Initialise the SAE 
    SAE = new SurfaceOfActiveEvents(m_height, m_width);

    // Setup up storage images
    initialised_events.resize(m_height, m_width);
    initialised_events.setZero();
    init_events_patch.resize(m_ksize, m_ksize);
    init_events_patch.setZero();

    lx.resize(m_height, m_width);
    lx.setZero();
    lx_patch.resize(m_ksize, m_ksize);
    lx_patch.setZero();

    ly.resize(m_height, m_width);
    ly.setZero();
    ly_patch.resize(m_ksize, m_ksize);
    ly_patch.setZero();

    initialised_directions.resize(m_height, m_width);
    initialised_directions.setZero();
    init_directions_patch.resize(m_ksize, m_ksize);
    init_directions_patch.setZero();

    t_patch.resize(m_ksize, m_ksize);
    t_patch.setZero();

    // scratch buffers for optimisation
    const int max_patch_points = m_ksize * m_ksize;
    A_scratch.resize(max_patch_points, 2);
    A_scratch.setZero();
    W_scratch.resize(max_patch_points);
    W_scratch.setZero();

    // Set up patch points/pairs
    patch_offset = Eigen::VectorXd::LinSpaced(m_ksize, -m_ksize/2, m_ksize/2);

    for (int i = 0; i < m_ksize; i++){
        for (int j = 0; j < m_ksize; j++){
            patch_points.push_back(std::pair(i, j));
       }
    }

}

SAEdetector::~SAEdetector(){
    delete SAE;
}


//-----------------------------------//
//-----     Public Functions    -----//
//-----------------------------------//
// Main function to interface with
//-----    Add event to SAE    -----//
void SAEdetector::addEvent(Eigen::Vector4d e){
    SAE->addEvent(e);
    initialised_events(int(e(1)), int(e(0))) = 1;
    m_t_current = e(2);
}


int SAEdetector::performDetection(Eigen::Vector4d e){
    addEvent(e);
    getPatches(e);
    computeDirectionRegression(e);
    int detection_result = compareDirectionRegression(e);

    return detection_result;
}

int SAEdetector::performDetection_dt(Eigen::Vector4d e){
    // Get the SAE patch indexes
    Eigen::VectorXd x_indexes = Eigen::VectorXd::Constant(m_ksize, e(0)) + patch_offset;
    Eigen::VectorXd y_indexes = Eigen::VectorXd::Constant(m_ksize, e(1)) + patch_offset;

    // Set the patch flag to false if the requested indexes are out of the frame
    if (x_indexes[0] < 0 | x_indexes[x_indexes.size()-1] >= m_width |
        y_indexes[0] < 0 | y_indexes[y_indexes.size()-1] >= m_height){
        return -1;
    } 

    // First, make sure enough events in the patch have had events
    init_events_patch = initialised_events.block(y_indexes[0], x_indexes[0], m_ksize, m_ksize);
    if (init_events_patch.sum() < m_min_contributions){
        return -1;
    }   

    // Get the SAE patch
    t_patch = SAE->getImage().block(y_indexes[0], x_indexes[0], m_ksize, m_ksize);

    if ((e(2) - t_patch.array()).maxCoeff() < m_dt_detection_threshold){
        return 1;
    }
    else{
        return -1;
    }
}

Eigen::MatrixXd SAEdetector::getImage(){
    return SAE->getImage();
}

//-----------------------------------//
//-----    Private Functions    -----//
//-----------------------------------//

// Get the patches used for least squares regression
void SAEdetector::getPatches(Eigen::Vector4d e){
    
    // Create index values
    Eigen::VectorXd x_indexes = Eigen::VectorXd::Constant(m_ksize, e(0)) + patch_offset;
    Eigen::VectorXd y_indexes = Eigen::VectorXd::Constant(m_ksize, e(1)) + patch_offset;

    // Set the patch flag to false if the requested indexes are out of the frame
    if (x_indexes[0] < 0 | x_indexes[x_indexes.size()-1] >= m_width |
        y_indexes[0] < 0 | y_indexes[y_indexes.size()-1] >= m_height){
        m_f_patch_status = false;
        return;
    }

    // First get the initialised pixel patch to see if we should get the  others
    init_events_patch = initialised_events.block(y_indexes[0], x_indexes[0], m_ksize, m_ksize);
    
    if (init_events_patch.sum() < m_min_contributions){
        m_f_patch_status = false;
        return;
    }    

    t_patch = SAE->getImage().block(y_indexes[0], x_indexes[0], m_ksize, m_ksize);
    init_directions_patch = initialised_directions.block(y_indexes[0], x_indexes[0], m_ksize, m_ksize);
    lx_patch = lx.block(y_indexes[0], x_indexes[0], m_ksize, m_ksize);
    ly_patch = ly.block(y_indexes[0], x_indexes[0], m_ksize, m_ksize);

    m_f_patch_status = true;
}


//-----    Compute LSQ Regression    -----//
void SAEdetector::computeDirectionRegression(Eigen::Vector4d e){
    // If we weren't able to get the patches, we do not perform least squares
    if (m_f_patch_status == false){
        m_f_regression_status = false;
        return;
    }

    int num_init_points = init_events_patch.sum() - 1;
    auto A = A_scratch.topRows(num_init_points);
    auto W_diag = W_scratch.head(num_init_points);

    int idx = 0;
    for (int i = 0; i < patch_points.size(); i++){   
        // Get the indexes in the patch
        int x = patch_points[i].first;
        int y = patch_points[i].second;

        // If the points is not initialised or it is the center element (pixel we are looking at), skip it 
        if ((init_events_patch(y,x) == 0) || (patch_offset(x) == 0 && patch_offset(y) == 0)){
            continue;
        }

        // Add distance values to A
        A(idx,0) = patch_offset(x);
        A(idx,1) = patch_offset(y);

        // Compute dt term
        double exp_term = std::exp(2 * m_alpha * (m_t_current - t_patch(y, x)));
        W_diag(idx) = 1.0 /exp_term;

        idx++;
    }

    Eigen::Matrix2d res = A.transpose() * W_diag.asDiagonal() * A;

    // Compute eigenvector of smallest eigenvalue for l_perp, rotate 90deg 
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(res); 
    Eigen::Vector2d l_perp =  eigen_solver.eigenvectors().col(0);
    Eigen::Vector2d l = {-l_perp(1), l_perp(0)};

    //----- Store values -----//
    // Store direction estimate in whole array and patch
    lx(int(e(1)), int(e(0))) = l(0);
    ly(int(e(1)), int(e(0))) = l(1);

    lx_patch(int(m_ksize/2), int(m_ksize/2)) = l(0);
    ly_patch(int(m_ksize/2), int(m_ksize/2)) = l(1);

    // Recorded the pixel as having an initialised direction estimate
    initialised_directions(int(e(1)), int(e(0))) = 1;
    init_directions_patch(int(m_ksize/2), int(m_ksize/2)) = 1;
   
    m_f_regression_status = true;

}


int SAEdetector::compareDirectionRegression(Eigen::Vector4d e){
    // If flags for previous operations are invalid, we will not return a detection
    if ((m_f_regression_status == false) || (m_f_patch_status == false) || (init_directions_patch.sum()-1 < m_min_active_pixels)){
        return -1;
    }
    
    int num_init_points = init_directions_patch.sum() - 1;
    auto A = A_scratch.topRows(num_init_points);
    auto W_diag = W_scratch.head(num_init_points);

    int idx = 0;
    for (int i = 0; i < patch_points.size(); i++){   
        // Get the indexes in the patch
        int x = patch_points[i].first;
        int y = patch_points[i].second;

        // If the points is not initialised or it is the center element (pixel we are looking at), skip it 
        if ((init_directions_patch(y,x) == 0) || (patch_offset(x) == 0 && patch_offset(y) == 0)){
            continue;
        }

        // Add distance values to A
        A(idx,0) = lx_patch(y,x);
        A(idx,1) = ly_patch(y,x);

        // Compute dt term
        double exp_term = std::exp(2 * m_alpha * (m_t_current - t_patch(y, x)));
        W_diag(idx) = 1.0 /exp_term;

        idx++;
    }

    Eigen::Matrix2d res = A.transpose() * W_diag.asDiagonal() * A;

    // Compute eigenvector of smallest eigenvalue for l_perp, rotate 90deg 
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(res); 
    Eigen::Vector2d l_perp =  eigen_solver.eigenvectors().col(0);
    Eigen::Vector2d l_est = {-l_perp(1), l_perp(0)};

    Eigen::Vector2d l_event = {lx_patch(int(m_ksize/2), int(m_ksize/2)), ly_patch(int(m_ksize/2), int(m_ksize/2))};

    double l_inner = std::abs(l_event.transpose() * l_est);

    // Check if the magnitude of the difference is below our threshold
    if (l_inner >= m_detection_threshold){
        return 1;
    } 
    else {
        return -1;
    }
}