
#include "parameters_decoding.h"

DecodeParams loadDecodeParametersFromYAML(const std::string &yaml_file_path)
{
    const YAML::Node config = YAML::LoadFile(yaml_file_path);
    DecodeParams decodeParams;
    decodeParams.comms_range = config["comms_range"] ? config["comms_range"].as<double>() : 10;
    decodeParams.decoder_thresh_on = config["decoder_thresh_on"] ? config["decoder_thresh_on"].as<double>() : 6;
    decodeParams.decoder_thresh_off = config["decoder_thresh_off"] ? config["decoder_thresh_off"].as<double>() : -6;
    decodeParams.peak_thresh = config["peak_thresh"] ? config["peak_thresh"].as<double>() : 3.0;
    decodeParams.peak_win_size = config["peak_win_size"] ? config["peak_win_size"].as<int>() : 40;
    decodeParams.alpha_decoding = config["alpha_decoding"] ? config["alpha_decoding"].as<double>() : 8379;
    decodeParams.base_frequency = config["base_frequency"] ? config["base_frequency"].as<double>() : 3334;
    decodeParams.flag_save_DI = config["flag_save_DI"] ? config["flag_save_DI"].as<int>() : 0;
    decodeParams.flag_save_G = config["flag_save_G"] ? config["flag_save_G"].as<int>() : 1;

        return decodeParams;
}