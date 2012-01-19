//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_GAZEBO_PLUGINS_SENSOR_MODEL_H
#define HECTOR_GAZEBO_PLUGINS_SENSOR_MODEL_H

#include <gazebo/Param.hh>

namespace gazebo {

class SensorModel {
public:
  SensorModel(std::vector<Param*> &parameters, const std::string& prefix = "");
  virtual ~SensorModel();

  virtual void LoadChild(XMLConfigNode *node);

  virtual double operator()(double value) const { return value + current_error_; }
  virtual double operator()(double value, double dt) { return value + update(dt); }

  virtual double update(double dt);
  virtual void reset(double value = 0.0);

  virtual double getCurrentError() const { return current_error_; }
  virtual double getCurrentDrift() const { return current_drift_; }

public:
  double offset;
  double drift;
  double drift_frequency;
  double gaussian_noise;

private:
  ParamT<double> *offset_param_;
  ParamT<double> *drift_param_;
  ParamT<double> *drift_frequency_param_;
  ParamT<double> *gaussian_noise_param_;

  double current_drift_;
  double current_error_;

  static double gaussianKernel(double mu, double sigma);
};

SensorModel::SensorModel(std::vector<Param*> &parameters, const std::string &prefix)
{
  Param::Begin(&parameters);
  offset_param_          = new ParamT<double>(prefix + "offset", 0.0, 0);
  drift_param_           = new ParamT<double>(prefix + "drift", 0.0, 0);
  drift_frequency_param_ = new ParamT<double>(prefix + "driftFrequency", 0.0003, 0);
  gaussian_noise_param_  = new ParamT<double>(prefix + "gaussianNoise", 0.0, 0);
  Param::End();

  reset();
}

SensorModel::~SensorModel()
{
  delete offset_param_;
  delete drift_param_;
  delete drift_frequency_param_;
  delete gaussian_noise_param_;
}

void SensorModel::LoadChild(XMLConfigNode *node)
{
  offset_param_->Load(node);
  offset = offset_param_->GetValue();
  drift_param_->Load(node);
  drift = drift_param_->GetValue();
  drift_frequency_param_->Load(node);
  drift_frequency = drift_frequency_param_->GetValue();
  gaussian_noise_param_->Load(node);
  gaussian_noise = gaussian_noise_param_->GetValue();
}

double SensorModel::update(double dt)
{
  current_drift_ += dt * (-current_drift_ * drift_frequency + this->gaussianKernel(0, sqrt(2*drift_frequency)*drift));
  current_error_ = offset + current_drift_ + gaussianKernel(0, gaussian_noise);
  return current_error_;
}

double SensorModel::gaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal variables
  // see wikipedia
  double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
  //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
  // we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

void SensorModel::reset(double value)
{
  current_drift_ = 0.0;
  current_error_ = value;
}

}

#endif // HECTOR_GAZEBO_PLUGINS_SENSOR_MODEL_H
