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

#include <sdf/sdf.hh>
#include <gazebo/math/Vector3.hh>

namespace gazebo {

template <typename T>
class SensorModel_ {
public:
  SensorModel_();
  virtual ~SensorModel_();

  virtual void Load(sdf::ElementPtr _sdf, const std::string& prefix = std::string());

  virtual T operator()(const T& value) const { return value + current_error_; }
  virtual T operator()(const T& value, double dt) { return value + update(dt); }

  virtual T update(double dt);
  virtual void reset(const T& value = T());

  virtual const T& getCurrentError() const { return current_error_; }
  virtual const T& getCurrentDrift() const { return current_drift_; }

  virtual void setCurrentDrift(const T& new_drift) { current_drift_ = new_drift; }

private:
  virtual bool LoadImpl(sdf::ElementPtr _element, T& _value);

public:
  T offset;
  T drift;
  T drift_frequency;
  T gaussian_noise;

private:
  T current_drift_;
  T current_error_;
};

template <typename T>
SensorModel_<T>::SensorModel_()
  : offset()
  , drift()
  , drift_frequency()
  , gaussian_noise()
{
  drift_frequency = 1.0/3600.0;
  reset();
}

template <typename T>
SensorModel_<T>::~SensorModel_()
{
}

template <typename T>
void SensorModel_<T>::Load(sdf::ElementPtr _sdf, const std::string& prefix)
{
  std::string _offset, _drift, _drift_frequency, _gaussian_noise;

  if (prefix.empty()) {
    _offset          = "offset";
    _drift           = "drift";
    _drift_frequency = "driftFrequency";
    _gaussian_noise  = "gaussianNoise";
  } else {
    _offset          = prefix + "Offset";
    _drift           = prefix + "Drift";
    _drift_frequency = prefix + "DriftFrequency";
    _gaussian_noise  = prefix + "GaussianNoise";
  }

  if (_sdf->HasElement(_offset))          LoadImpl(_sdf->GetElement(_offset), offset);
  if (_sdf->HasElement(_drift))           LoadImpl(_sdf->GetElement(_drift), drift);
  if (_sdf->HasElement(_drift_frequency)) LoadImpl(_sdf->GetElement(_drift_frequency), drift_frequency);
  if (_sdf->HasElement(_gaussian_noise))  LoadImpl(_sdf->GetElement(_gaussian_noise), gaussian_noise);
}

template <typename T>
bool SensorModel_<T>::LoadImpl(sdf::ElementPtr _element, T& _value) {
  if (!_element->GetValue()) return false;
  return _element->GetValue()->Get(_value);
}

namespace {
  template <typename T>
  static inline T SensorModelGaussianKernel(T mu, T sigma)
  {
    // using Box-Muller transform to generate two independent standard normally distributed normal variables
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
  for(std::size_t i = 0; i < current_error_.size(); ++i) current_error_[i] = SensorModelInternalUpdate(current_drift_[i], drift[i], drift_frequency[i], offset[i], gaussian_noise[i], dt);
  return current_error_;
}

template <>
double SensorModel_<double>::update(double dt)
{
  current_error_ = SensorModelInternalUpdate(current_drift_, drift, drift_frequency, offset, gaussian_noise, dt);
  return current_error_;
}

template <>
math::Vector3 SensorModel_<math::Vector3>::update(double dt)
{
  current_error_.x = SensorModelInternalUpdate(current_drift_.x, drift.x, drift_frequency.x, offset.x, gaussian_noise.x, dt);
  current_error_.y = SensorModelInternalUpdate(current_drift_.y, drift.y, drift_frequency.y, offset.y, gaussian_noise.y, dt);
  current_error_.z = SensorModelInternalUpdate(current_drift_.z, drift.z, drift_frequency.z, offset.z, gaussian_noise.z, dt);
  return current_error_;
}

template <typename T>
void SensorModel_<T>::reset(const T& value)
{
  current_drift_ = value;
  current_error_ = T();
}

typedef SensorModel_<double> SensorModel;
typedef SensorModel_<math::Vector3> SensorModel3;

}

#endif // HECTOR_GAZEBO_PLUGINS_SENSOR_MODEL_H
