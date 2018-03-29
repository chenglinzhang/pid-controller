#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;    
}

void PID::UpdateError(double cte) { 
  if (step % (2*n) == 0) {
    i_error = 0;
    err = 0;
    step = 0;
  } else {  
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    if (step >= n) {
      err += cte * cte;
    }
  }
  step++;
}

double PID::TotalError() {
    double total_error = -Kp * p_error - Kd * d_error - Ki * i_error;
    // within [-1, 1]
    if (total_error > 1.0) total_error = 1.0;
    if (total_error < -1.0) total_error = -1.0;
    return total_error;
}

double PID::Twiddle(double tolerance) {
  // twiddle in 2*n cycle
  if (step % (2*n) == 0) {
    // sum of dp values
    double sum_dp = 0.0;
    for (auto& value : dp) {
      sum_dp += value;
    }
    // twiddle if exceeding tolerance
    if (sum_dp > tolerance) {
      // index for parameter to tune
      int i = step % 3;
      // n accumulated err
      best_err = err/n;
      if (err < best_err) {
        // up tune
        best_err = err;
        dp[i] *= 1.1;
      } else {
        // down tune
        p[i] += dp[i];
        dp[i] *= 0.9;
      }
    }
  }
}

const std::vector<double>& PID::TwiddleParams() {
  return p;
}
