#ifndef PARAMETERS_decoding
#define PARAMETERS_decoding
#include "yaml-cpp/yaml.h"
#include <string>

struct DecodeParams
{
  double comms_range;
  double decoder_thresh_on;
  double decoder_thresh_off;
  double peak_thresh;
  int peak_win_size;
  double alpha_decoding;
  double base_frequency;
  int flag_save_DI;
  int flag_save_G;
};

DecodeParams loadDecodeParametersFromYAML(const std::string &yaml_file_path);
#endif // PARAMETERS_decoding
