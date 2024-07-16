
#ifndef __LOW_PASS_FILTER_HPP__
#define __LOW_PASS_FILTER_HPP__

#include <cmath>
#include <memory>
#include <string>

namespace encoder_filter
{

class LowPassFilter
{
public:
  // Default constructor
  LowPassFilter() : a1_(1.0), b1_(0.0)
  {
  }

  /*!
   * \brief Destructor of LowPassFilter class.
   */
  ~LowPassFilter()
  {
  }

  /*!
   * \brief Configure the LowPassFilter (access and process params).
   */
  bool configure(double a)
  {
    if (a < 0.0 || a > 1.0)
    {
      return false;
    }
    
    a1_ = a;
    b1_ = 1.0 - a1_;

    return true;
  }
  bool configure(double sampling_frequency, double damping_frequency, double damping_intensity)
  {
    a1_ = exp(
      -1.0 / sampling_frequency * (2.0 * M_PI * damping_frequency) /
      (pow(10.0, damping_intensity / -10.0)));
    b1_ = 1.0 - a1_;

    return true;
  }

  /*!
   * \brief Applies one iteration of the IIR filter.
   *
   * \param data_in input to the filter
   * \param data_out filtered output
   *
   * \returns false if filter is not configured, true otherwise
   */
  bool update(const double & data_in, double & data_out)
  {
    // // Filter
    // data_out = b1_ * old_value + a1_ * filtered_old_value;
    // filtered_old_value = data_out;
    // old_value = data_in;

    filtered_value = a1_ * filtered_old_value + b1_ * data_in;
    filtered_old_value = filtered_value;
    data_out = filtered_value;

    return true;
  }

private:
  // Filter parameters
  /** internal data storage (double). */
  double filtered_value{0.0}, filtered_old_value{0.0}, old_value{0.0};
  
  double a1_{1.0}; /**< feedbackward coefficient. */
  double b1_{0.0}; /**< feedforward coefficient. */
};

}

#endif  // __LOW_PASS_FILTER_HPP__
