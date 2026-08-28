#include "parameters.h"

Parameters loadParametersFromYAML(const std::string &yaml_file_path)
{
    const YAML::Node config = YAML::LoadFile(yaml_file_path);

    Parameters params;
    params.width = config["width"].as<int>();
    params.height = config["height"].as<int>();
    params.data_format = config["data_format"].as<int>();
    params.dist_threshold = config["dist_threshold"].as<double>();
    params.n_target = config["n_target"].as<int>();
    params.n_state = config["n_state"].as<int>();
    params.m = config["m"].as<int>();
    params.publish_framerate = config["publish_framerate"].as<int>();
    params.dt = config["dt"].as<double>();
    params.contrast_threshold = config["contrast_threshold"].as<double>();
    params.alpha = config["alpha"].as<int>();
    params.use_gyro_flag = config["use_gyro_flag"].as<int>();
    params.save_video_flag = config["save_video_flag"].as<int>();
    params.save_image_flag = config["save_image_flag"].as<int>();
    params.ref_image_ts_flag = config["ref_image_ts_flag"].as<int>();
    params.disp_covariance_flag = config["disp_covariance_flag"].as<int>();
    params.process_ts_start = config["process_ts_start"].as<double>();
    params.process_ts_end = config["process_ts_end"].as<double>();
    params.event_num_start = config["event_num_start"].as<int>();
    params.event_num_end = config["event_num_end"].as<double>();
    params.ring_buffer_len = config["ring_buffer_len"].as<int>();

    params.input_data_name = config["input_data_name"].as<std::string>();
    params.print_TTC_flag = config["print_TTC_flag"].as<int>();
    params.compute_TTC_flag = config["compute_TTC_flag"].as<int>();
    params.save_track_flag = config["save_track_flag"].as<int>();
    params.select_target_flag = config["select_target_flag"].as<int>();
    params.input_folder_path = config["input_folder_path"].as<std::string>();

    params.var_x = config["var_x"].as<double>();
    params.var_y = config["var_y"].as<double>();
    params.var_vx = config["var_vx"].as<double>();
    params.var_vy = config["var_vy"].as<double>();
    params.var_lambda_1 = config["var_lambda_1"].as<double>();
    params.var_lambda_2 = config["var_lambda_2"].as<double>();
    params.var_theta = config["var_theta"].as<double>();
    params.var_q = config["var_q"].as<double>();
    params.q_x = config["q_x"].as<double>();
    params.position_x_init = config["position_x_init"].as<std::vector<double>>();
    params.position_y_init = config["position_y_init"].as<std::vector<double>>();
    params.lambda_init = config["lambda_init"].as<double>();

    params.q_y = config["q_y"].as<double>();
    params.q_vx = config["q_vx"].as<double>();
    params.q_vy = config["q_vy"].as<double>();
    params.q_lambda_1 = config["q_lambda_1"].as<double>();
    params.q_lambda_2 = config["q_lambda_2"].as<double>();
    params.q_theta = config["q_theta"].as<double>();
    params.q_q = config["q_q"].as<double>();

    // Detector
    params.SAE_operation_rate = config["SAE_operation_rate"] ? config["SAE_operation_rate"].as<int>() : 500;
    params.SAE_ksize = config["SAE_ksize"].as<int>();
    params.SAE_alpha = config["SAE_alpha"].as<double>();
    params.SAE_min_contributions = config["SAE_min_contributions"].as<double>();
    params.SAE_detection_threshold = config["SAE_detection_threshold"].as<double>();
    params.SAE_min_active_pixels = config["SAE_min_active_pixels"].as<double>();
    params.MAX_NUMBER_OF_TRACKS = config["MAX_NUMBER_OF_TRACKS"].as<int>();

    params.SAE_recency_window = config["SAE_recency_window"].as<double>();

    params.event_rate_threshold = config["event_rate_threshold"].as<double>();
    params.f_show_candidates = config["f_show_candidates"].as<int>();
    params.f_evaluate = config["f_evaluate"].as<int>();

    params.detector_dist_threshold = config["detector_dist_threshold"].as<double>();
    params.detector_dt_threshold = config["detector_dt_threshold"].as<double>();
    params.evaluate_ts_age = config["evaluate_ts_age"].as<double>();
    params.evaluate_dt_terminate = config["evaluate_dt_terminate"].as<double>();
    params.evaluate_low_activity_factor = config["evaluate_low_activity_factor"].as<double>();

    //new
    params.pool_size = config["pool_size"] ? config["pool_size"].as<int>() : 10;
    params.use_dt_detector = config["use_dt_detector"] ? config["use_dt_detector"].as<int>() : 0;
    params.rate_per_area = config["rate_per_area"] ? config["rate_per_area"].as<double>() : 1.0;

    return params;
}

DisplayParams loadDispParametersFromYAML(const std::string &yaml_file_path)
{
    const YAML::Node config = YAML::LoadFile(yaml_file_path);
    DisplayParams dispParams;
    dispParams.save_video_flag = config["save_video_flag"].as<int>();
    dispParams.save_image_flag = config["save_image_flag"].as<int>();
    dispParams.disp_covariance_flag = config["disp_covariance_flag"].as<int>();

    return dispParams;
}