// This file is licensed under the Apache License, Version 2.0,
// with additional restrictions under the Commons Clause.
// See the LICENSE file for more details.

#include <eigen3/Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <iomanip>
#include <iostream>
#pragma once

#include "decoding.hpp"

class KalmanFilter
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
  KalmanFilter(double dt, const Eigen::MatrixXd &F, const Eigen::MatrixXd &C,
               const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R, const Eigen::MatrixXd &P,
               const Eigen::MatrixXd &A, const Eigen::MatrixXd &x0, int n_state,
               int ring_buffer_len);

  /**
   * Create a blank estimator.
   */
  KalmanFilter();

  /**
   * Initialize the filter with initial states as zero.
   */
  void init();

  /**
   * Initialize the filter with a guess for initial states.
   */
  void init(double t0, std::string input_folder_nam);

  /**
   * Update the estimated state based on measured values. The
   * time step is assumed to remain constant.
   */
  void update(const Eigen::Vector3d &y, int id, int p);

  void update_start_point(const Eigen::MatrixXd &x0) { x_hat = x0; }

  void update_distance_threshold(double distance_threshold);

  void update_ts_last_for_gamma(double ts);

  void update_gyro(const Eigen::MatrixXd vx_gyro_update, const Eigen::MatrixXd vy_gyro_update)
  {
    x_hat.block(0, 0, n_target, 1) = x_hat.block(0, 0, n_target, 1) + vx_gyro_update;
    x_hat.block(0, 1, n_target, 1) = x_hat.block(0, 1, n_target, 1) + vy_gyro_update;
  }

  void update_gyro_single_id(double vx_gyro_update, double vy_gyro_update)
  {
    x_hat(0) = x_hat(0) + vx_gyro_update;
    x_hat(1) = x_hat(1) + vy_gyro_update;
    // std::cout << vx_gyro_update << " " << vy_gyro_update << std::endl;
  }

  void update_matrix(const Eigen::MatrixXd &Fx, const Eigen::MatrixXd &Cx,
                     const Eigen::MatrixXd &Qx, const Eigen::MatrixXd &Rx, const Eigen::MatrixXd &Px,
                     const Eigen::MatrixXd &Ax, const Eigen::MatrixXd &x0x)
  {
    F = Fx;
    C = Cx;
    Q = Qx;
    R = Rx;
    P = Px;
    A = Ax;
    x_hat = x0x;
  }

  void reset(const Eigen::MatrixXd &x0_new, double ts);

  void initialiseDecoder(double dt, std::string data_name, std::string input_event_path, double contrast_threshold, int id);

  // Comms
  Decoding *decoder;

  /**
   * Return the current state and time.
   */
  Eigen::MatrixXd state() { return x_hat; };
  double time() { return t; };
  double get_ts_last() { return ts_last[0]; };
  double get_ts_last_for_gamma() { return ts_last_for_gamma; };
  double get_dist_threshold() { return dist_threshold; };

  double get_t0() {return t0; };

  int getID() {return track_id; };
  void setID(int track_id) { this->track_id = track_id; };

  void openOutputFile();

  double P_lambda() { return P(4, 4); };
  Eigen::MatrixXd P_x() { return P; };

  bool validated = 0;
  bool active = false;
  double dt_moving_avg = 1;

  //destructor
  ~KalmanFilter();

private:
  std::ofstream blob_measurements_txt_, states_txt_;

  int track_id = -1;;

  int n_state;
  int n_target = 1;
  // Matrices for computation
  Eigen::MatrixXd F, C, Q, R, P, K, P0, A;

  // System dimensions
  int m, n;

  // Distance threshold
  double dist_threshold;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::MatrixXd x_hat;

  std::vector<double> ts_last;
  double ts_last_for_gamma;

  Eigen::Vector2d e_tilda, y_hat, y_hat_sum, y_hat_buffer, e_tilda_buffer;
  Eigen::MatrixXd S, sqrtMinv;
  double y_square_sum, theta_buf;
  Eigen::Matrix<double, 3, 1> y_hat_fuse, y_true;
  Eigen::MatrixXd I2 = Eigen::MatrixXd::Identity(2, 2);
  std::ofstream track_output_txt_;

  Eigen::Matrix2d rotation_m, rotation_m_buf, diag_m, Omiga, A_rotation, C_lambda;
  Eigen::Matrix<double, 2, 1> C_lambda_diag;

  Eigen::Matrix<double, 1, 2> y_lambda, y_lambda_sum;
  Eigen::Matrix<double, 2, 1> temp_diag;
  Eigen::Matrix2d temp;
  int ring_buffer_len;
  double vx_gyro = 0;
  double vy_gyro = 0;

  typedef boost::circular_buffer<Eigen::MatrixXd> CircularBuffer;
  std::vector<boost::circular_buffer<std::tuple<double, double, double, double>>> ring_buffer_e;
  std::vector<boost::circular_buffer<std::pair<double, double>>> ring_buffer_x;
  std::vector<boost::circular_buffer<double>> ring_buffer_theta;
  std::vector<boost::circular_buffer<std::pair<double, double>>> ring_buffer_delta;


  bool f_equivalent_measurement_init = 0;
  int equiv_measurement_step = 1;
  int equiv_measurement_count = 0;
  Eigen::MatrixXd x_hat_pred, P_pred;


};
