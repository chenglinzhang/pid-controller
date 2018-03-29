#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error = 0.0;
  double i_error = 0.0;
  double d_error = 0.0;

  /*
  * Coefficients
  */ 
  double Kp = 0.0;
  double Ki = 0.0;
  double Kd = 0.0;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Twiddle for parameter tuning.
  */
  double Twiddle(double tolerance=0.2);

  /*
  * Twiddle parameter values.
  */
  const std::vector<double>& TwiddleParams();

private: 
  int n = 1000; // cycle
  int step = 0; // steps per cycle
  double err = 0.0; // err per cycle

  // twiddle
  double best_err = 0.0;
  std::vector<double> p{0.1, 0.1, 0.1};
  std::vector<double> dp{1, 1, 1};
};

#endif /* PID_H */
