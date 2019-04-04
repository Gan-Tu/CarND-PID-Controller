#include "PID.h"
#include <time.h>

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

    this->p_error = 0.0;
    this->i_error = 0.0;
    this->d_error = 0.0;
    this->i_error_100_before = 0.0;

    this->started = 0;
    this->timestamp = clock();
    this->counter = 0;
}

void PID::UpdateError(double cte, double timestamp) {

    double prev_timestamp = this->timestamp;
    this->timestamp = timestamp;
    double dt = (timestamp - prev_timestamp) / CLOCKS_PER_SEC;

    double prev_error = this->p_error;
    this->p_error = cte;
    
    if (this->counter >= 100) {
        this->i_error_100_before += cte * dt;
    } else {
        this->counter++;
    }

    this->i_error += cte * dt;

    if (!this->started) {
        this->started = 1;
        this->d_error = 0;
        this->i_error = cte;
    } else {
        this->d_error = (cte - prev_error) / dt;
    }
}

double PID::TotalError() {
    return  this->p_error * this->Kp + (this->i_error - this->i_error_100_before) * this->Ki + this->d_error * this->Kd;
}

