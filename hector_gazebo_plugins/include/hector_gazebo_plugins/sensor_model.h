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

template <typename T>
class SensorModel_ {
public:
  SensorModel_(std::vector<Param*> &parameters, const std::string& prefix = "");
  virtual ~SensorModel_();

  virtual void Load(XMLConfigNode *node);

  virtual T operator()(const T& value) const { return value + current_error_; }
  virtual T operator()(const T& value, double dt) { return value + update(dt); }

  virtual T update(double dt);
  virtual void reset(const T& value = T());

  virtual const T& getCurrentError() const { return current_error_; }
  virtual const T& getCurrentDrift() const { return current_drift_; }

public:
  T offset;
  T drift;
  T drift_frequency;
  T gaussian_noise;

private:
  ParamT<T> *offset_param_;
  ParamT<T> *drift_param_;
  ParamT<T> *drift_frequency_param_;
  ParamT<T> *gaussian_noise_param_;

  T current_drift_;
  T current_error_;
};

template <typename T>
SensorModel_<T>::SensorModel_(std::vector<Param*> &parameters, const std::string &prefix)
{
  Param::Begin(&parameters);
  if (prefix.empty()) {
    offset_param_          = new ParamT<T>("offset", T(), 0);
    drift_param_           = new ParamT<T>("drift", T(), 0);
    drift_frequency_param_ = new ParamT<T>("driftFrequency", T(), 0);
    gaussian_noise_param_  = new ParamT<T>("gaussianNoise", T(), 0);
  } else {
    offset_param_          = new ParamT<T>(prefix + "Offset", T(), 0);
    drift_param_           = new ParamT<T>(prefix + "Drift", T(), 0);
    drift_frequency_param_ = new ParamT<T>(prefix + "DriftFrequency", T(), 0);
    gaussian_noise_param_  = new ParamT<T>(prefix + "GaussianNoise", T(), 0);
  }
  Param::End();

  reset();
}

template <typename T>
SensorModel_<T>::~SensorModel_()
{
  delete offset_param_;
  delete drift_param_;
  delete drift_frequency_param_;
  delete gaussian_noise_param_;
}

template <typename T>
void SensorModel_<T>::Load(XMLConfigNode *node)
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

namespace {
  template <typename T>
  static inline T SensorModelGaussianKernel(T mu, T sigma)
  {
    // using Box-Muller transform to generate two independent standard normally disbributed normal variables
    // see wikipedia
    T U = (T)rand()/(T)RAND_MAX; // normalized uniform random variable
    T V = (T)rand()/(T)RAND_MAX; // normalized uniform random variable
    T X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
    X = sigma * X + mu;
    return X;
  }

  template <typename T>
  static inline T SensorModelInternalUpdate(T& current_drift, T drift, T drift_frequency, T offset, T gaussian_noise, double dt)
  {
    current_drift = current_drift - dt * (current_drift * drift_frequency + SensorModelGaussianKernel(T(), sqrt(2*drift_frequency)*drift));
    return offset + current_drift + SensorModelGaussianKernel(T(), gaussian_noise);
  }
}

template <typename T>
T SensorModel_<T>::update(double dt)
{
  for(std::size_t i = 0; i < current_error_.size(); ++i) current_error_[i] = current_error_ = SensorModelInternalUpdate(current_drift_[i], drift[i], drift_frequency[i], offset[i], gaussian_noise[i], dt);
  return current_error_;
}

template <>
double SensorModel_<double>::update(double dt)
{
  current_error_ = SensorModelInternalUpdate(current_drift_, drift, drift_frequency, offset, gaussian_noise, dt);
  return current_error_;
}

template <>
Vector3 SensorModel_<Vector3>::update(double dt)
{
  current_error_.x = SensorModelInternalUpdate(current_drift_.x, drift.x, drift_frequency.x, offset.x, gaussian_noise.x, dt);
  current_error_.y = SensorModelInternalUpdate(current_drift_.y, drift.y, drift_frequency.y, offset.y, gaussian_noise.y, dt);
  current_error_.z = SensorModelInternalUpdate(current_drift_.z, drift.z, drift_frequency.z, offset.z, gaussian_noise.z, dt);
  return current_error_;
}

template <typename T>
void SensorModel_<T>::reset(const T& value)
{
  current_drift_ = T();
  current_error_ = value;
}

typedef SensorModel_<double> SensorModel;
typedef SensorModel_<Vector3> SensorModel3;

}

#endif // HECTOR_GAZEBO_PLUGINS_SENSOR_MODEL_H
