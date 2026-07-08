#include "decoding.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

Decoding::Decoding(double dt, const std::string &data_name, const std::string &input_event_path, const double &contrast_threshold, int id)
    : contrast_threshold(contrast_threshold)
{
  this->id = id;

  // I.setIdentity();
  DecodeParams decodeParams;
  std::string file_config_path = "../configs/" + data_name + ".yaml";

  decodeParams = loadDecodeParametersFromYAML(file_config_path);
  windowSize = decodeParams.peak_win_size;
  threshold = decodeParams.peak_thresh;
  unitT = 1.0 / decodeParams.base_frequency;
  unitT_07 = 0.5 * unitT; // Minimum timestamp between peaks
  alpha_decoding = decodeParams.alpha_decoding;
  comms_range = decodeParams.comms_range;
  flag_save_DI = decodeParams.flag_save_DI;
  flag_save_G = decodeParams.flag_save_G;


  if (flag_save_DI) {
    DI_txt_.open(input_event_path + "_DI_" + std::to_string(id) + ".txt");
  }

  if (flag_save_G) {
    G_filepath = input_event_path + "_G_" + std::to_string(id) + ".txt";

    // G_txt_.open(input_event_path + "_G_" + std::to_string(id) + ".txt");
    // G_txt_peak_.open(input_event_path + "_peak_" + std::to_string(id) + ".txt");
  }
}
Decoding::Decoding() {}

void Decoding::init(double ts, std::string output_decode_path)
{
  t_start = ts;
  t_stop = t_start + unit_T * 9;
  t_stop_right_limit = t_stop + unit_T * 1;

  // output_msg_.open(output_decode_path);
}

double Decoding::get_comms_range()
{
  return comms_range;
}

void Decoding::get_msg(const double &ts, const int &p, const double &track_x, const double &track_y, const double &track_vx, const double &track_vy)
{
  // high pass filter for decoding
  high_pass_decoding(ts, p, track_x, track_y, track_vx, track_vy);

}


void Decoding::open_G_file(){
  G_txt_.open(G_filepath);
}


// TODO: comms; High-pass Filter
void Decoding::high_pass_decoding(const double &ts, const int &p, const double &track_x, const double &track_y, const double &track_vx, const double &track_vy)
{
  hp_dt_ = ts - hp_t_prev_;
  hp_t_prev_ = ts;
  // hp_ = exp(-alpha_decoding * hp_dt_) * hp_; # HACK: comment this out, we aren't using it

  // low pass filter ////////////////////////////////////
  // hp_dt_ == 0, no drop, exp(-10 * alpha_decoding * hp_dt_) = 1
  if (p > 0)
  {
    hp_ += contrast_threshold;
    // if hp_dt_ = 0, do not change G_
    // G_ = exp(-5 * alpha_decoding * hp_dt_) * G_; // # TODO: moving this into hp_dt_ != 0 so it isn't evaluating exp(0)

    if (hp_dt_ == 0)
    {
      G_ += 1;
    }
    else
    {
      G_ = exp(-5 * alpha_decoding * hp_dt_) * G_;
      G_ += 1;
      // G_ += 1 / hp_dt_ / 1e6;
    }
  }
  else
  {
    hp_ -= contrast_threshold;
    // very large decay is similar to directly set G_ to almost zero
    // if hp_dt_ = 0, do not change G_
    // G_ = exp(-150 * alpha_decoding * hp_dt_) * G_; // # TODO: moving this into hp_dt_ != 0 so it isn't evaluating exp(0)

    // std::cout << "decay: " << exp(-150 * alpha_decoding * hp_dt_) << std::endl;
    // G_ = exp(-130 * alpha_decoding * hp_dt_) * G_;
    // not necessary when negative decay is large

    if (hp_dt_ == 0)
    {
      // G_ = exp(-1000 * alpha_decoding * 1e-5) * G_;
      // std::cout << "dt = 0: " << exp(-1000 * alpha_decoding * 1e-5) << std::endl;
      G_ += -1;
    }
    else
    {
      // TODO: G_ += -1 / hp_dt_ / 1e6 more correct but G_ += -1; perform better (higher G_)
      // G_ = exp(-1000 * alpha_decoding * hp_dt_) * G_;
      // std::cout << "decay: " << exp(-1000 * alpha_decoding * hp_dt_) << std::endl;
      
      G_ = exp(-150 * alpha_decoding * hp_dt_) * G_;
      G_ += -1;
      // G_ += -1 / hp_dt_ / 1e6;
    }
  }

  /////////// peak detection v2 - sliding window + non-maximum suppressiona word
  // process_peak(peak_, ts, G_);

  // x_hat_track = track_manager.getTrack(i)->state();

  if (flag_save_DI)
  {
    DI_txt_ << ts << "," << hp_ << "\n";
  }

  if ((hp_dt_>0) & flag_save_G & G_txt_.is_open()) // only save for unique timestamps (accumulate all events for a single timestamp)
  {
    G_txt_ << std::fixed << std::setprecision(7) << ts << "," << G_ << "," << track_x << "," << track_y << "," << track_vx << "," << track_vy << "\n";
  }


}

void Decoding::process_peak(std::pair<double, double> &peak_, const double &timestamp, const double &value)
{
  smoothedSignal_.push_back(value);
  smoothedSignal_t_.push_back(timestamp);
  if (smoothedSignal_.size() > (windowSize))
  {
    smoothedSignal_.pop_front();
    smoothedSignal_t_.pop_front();

    // The data distribution in the local window is not even, left side is denser
    int mid_id = (windowSize - 1) / 5;
    double smoothedValue = smoothedSignal_[mid_id];
    double smoothedValue_t = smoothedSignal_t_[mid_id];

    if (lastPeakTimestamp_ == 0.0 || (smoothedValue_t - lastPeakTimestamp_) >= unitT_07)
    {
      if (smoothedValue > threshold)
      {
        localWindow.assign(smoothedSignal_.begin(), smoothedSignal_.end());

        // TODO: make this efficient
        double localMaxValue = *std::max_element(localWindow.begin(), localWindow.end());

        int localMaxIndex = std::distance(localWindow.begin(), std::max_element(localWindow.begin(), localWindow.end()));

        if (localMaxIndex == mid_id)
        { // Make sure the peak is the local maximum
          peak_ = std::make_pair(smoothedValue_t, smoothedValue);
          lastPeakTimestamp_ = smoothedValue_t;
        }
      }
    }
  }
}
