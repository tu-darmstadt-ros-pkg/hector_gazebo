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
#if (GAZEBO_MAJOR_VERSION < 8)
#include <gazebo/math/Vector3.hh>
#endif

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <gazebo_ros/node.hpp>

#include <numeric>
#include <map>

namespace gazebo {


template <typename T>
class SensorModel_ {
public:
  SensorModel_();
  virtual ~SensorModel_();

  virtual void Load(gazebo_ros::Node::SharedPtr _node, sdf::ElementPtr _sdf, const std::string& prefix = std::string());

  virtual T operator()(const T& value) const { return value * scale_error + current_error_; }
  virtual T operator()(const T& value, double dt) { return value * scale_error + update(dt); }

  virtual T update(double dt);
  virtual void reset();
  virtual void reset(const T& value);

  virtual const T& getCurrentError() const { return current_error_; }
  virtual T getCurrentBias() const { return current_drift_ + offset; }
  virtual const T& getCurrentDrift() const { return current_drift_; }
  virtual const T& getScaleError() const { return scale_error; }

  virtual void setCurrentDrift(const T& new_drift) { current_drift_ = new_drift; }

  virtual rcl_interfaces::msg::SetParametersResult parametersChangedCallback(const std::vector<rclcpp::Parameter> & parameters);

private:
  virtual bool LoadImpl(sdf::ElementPtr _element, T& _value);

  virtual void initializeParameters(gazebo_ros::Node::SharedPtr _node, const std::string prefix);

  rcl_interfaces::msg::SetParametersResult setValue(const std::vector<rclcpp::Parameter> & parameters);

public:
  T offset;
  T drift;
  T drift_frequency;
  T gaussian_noise;
  T scale_error;

private:
  T current_drift_;
  T current_error_;

  std::string _offset, _drift, _drift_frequency, _gaussian_noise, _scale_error;

  struct ConfigSettings {
    ConfigSettings() {}
    ConfigSettings(T* v, const double initial, const double min, const double max) :
      v(v), initial(initial), min(min), max(max) 
    {}

    T* v;
    double initial;  // The initial value
    double min;
    double max;
  };

  // List of references between node parameters and local variables
  std::map<std::string, ConfigSettings> parameters_;
};

template <typename T>
SensorModel_<T>::SensorModel_()
  : offset()
  , drift()
  , drift_frequency()
  , gaussian_noise()
{
  drift_frequency = 1.0/3600.0; // time constant 1h
  scale_error = 1.0;

  _offset = "offset";
  _drift = "drift";
  _drift_frequency = "driftFrequency";
  _gaussian_noise = "gaussianNoise";
  _scale_error = "scaleError";

  reset();
}

template <typename T>
SensorModel_<T>::~SensorModel_()
{
}

template <typename T>
void SensorModel_<T>::Load(gazebo_ros::Node::SharedPtr _node, sdf::ElementPtr _sdf, const std::string& prefix)
{
  if (prefix.empty()) {
    _offset              = "offset";
    _drift               = "drift";
    _drift_frequency     = "driftFrequency";
    _gaussian_noise      = "gaussianNoise";
    _scale_error         = "scaleError";
  } else {
    _offset              = prefix + "Offset";
    _drift               = prefix + "Drift";
    _drift_frequency     = prefix + "DriftFrequency";
    _gaussian_noise      = prefix + "GaussianNoise";
    _scale_error         = prefix + "ScaleError";
  }

  if (_sdf->HasElement(_offset))              LoadImpl(_sdf->GetElement(_offset), offset);
  if (_sdf->HasElement(_drift))               LoadImpl(_sdf->GetElement(_drift), drift);
  if (_sdf->HasElement(_drift_frequency))     LoadImpl(_sdf->GetElement(_drift_frequency), drift_frequency);
  if (_sdf->HasElement(_gaussian_noise))      LoadImpl(_sdf->GetElement(_gaussian_noise), gaussian_noise);
  if (_sdf->HasElement(_scale_error))         LoadImpl(_sdf->GetElement(_scale_error), scale_error);

  // Initialize the links between the parameters and the local variables
  parameters_[_offset] = ConfigSettings(&offset, 0.0, -10.0, 10.0);
  parameters_[_drift] = ConfigSettings(&drift, 0.0, 0.0, 10.0);
  parameters_[_drift_frequency] = ConfigSettings(&drift_frequency, 0.0, 0.0, 1.0);
  parameters_[_gaussian_noise] = ConfigSettings(&gaussian_noise, 0.0, 0.0, 10.0);
  parameters_[_scale_error] = ConfigSettings(&scale_error, 1.0, 0.0, 2.0);

  initializeParameters(_node, prefix);

  reset();
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
    // current_drift = current_drift - dt * (current_drift * drift_frequency + SensorModelGaussianKernel(T(), sqrt(2*drift_frequency)*drift));
    current_drift = exp(-dt * drift_frequency) * current_drift + dt * SensorModelGaussianKernel(T(), sqrt(2*drift_frequency)*drift);
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

#if (GAZEBO_MAJOR_VERSION >= 8)
template <>
ignition::math::Vector3d SensorModel_<ignition::math::Vector3d>::update(double dt)
{
  current_error_.X() = SensorModelInternalUpdate(current_drift_.X(), drift.X(), drift_frequency.X(), offset.X(), gaussian_noise.X(), dt);
  current_error_.Y() = SensorModelInternalUpdate(current_drift_.Y(), drift.Y(), drift_frequency.Y(), offset.Y(), gaussian_noise.Y(), dt);
  current_error_.Z() = SensorModelInternalUpdate(current_drift_.Z(), drift.Z(), drift_frequency.Z(), offset.Z(), gaussian_noise.Z(), dt);
  return current_error_;
}
#else
template <>
math::Vector3 SensorModel_<math::Vector3>::update(double dt)
{
  current_error_.x = SensorModelInternalUpdate(current_drift_.x, drift.x, drift_frequency.x, offset.x, gaussian_noise.x, dt);
  current_error_.y = SensorModelInternalUpdate(current_drift_.y, drift.y, drift_frequency.y, offset.y, gaussian_noise.y, dt);
  current_error_.z = SensorModelInternalUpdate(current_drift_.z, drift.z, drift_frequency.z, offset.z, gaussian_noise.z, dt);
  return current_error_;
}
#endif

template <typename T>
void SensorModel_<T>::reset()
{
  for(std::size_t i = 0; i < current_drift_.size(); ++i) current_drift_[i] = SensorModelGaussianKernel(T::value_type(), drift[i]);
  current_error_ = T();
}

template <>
void SensorModel_<double>::reset()
{
  current_drift_ = SensorModelGaussianKernel(0.0, drift);
  current_error_ = 0.0;
}

#if (GAZEBO_MAJOR_VERSION >= 8)
template <>
void SensorModel_<ignition::math::Vector3d>::reset()
{
  current_drift_.X() = SensorModelGaussianKernel(0.0, drift.X());
  current_drift_.Y() = SensorModelGaussianKernel(0.0, drift.Y());
  current_drift_.Z() = SensorModelGaussianKernel(0.0, drift.Z());
  current_error_ = ignition::math::Vector3d();
}
#else
template <>
void SensorModel_<math::Vector3>::reset()
{
  current_drift_.x = SensorModelGaussianKernel(0.0, drift.x);
  current_drift_.y = SensorModelGaussianKernel(0.0, drift.y);
  current_drift_.z = SensorModelGaussianKernel(0.0, drift.z);
  current_error_ = math::Vector3();
}
#endif

template <typename T>
void SensorModel_<T>::reset(const T& value)
{
  current_drift_ = value;
  current_error_ = T();
}


template <>
void SensorModel_<double>::initializeParameters(gazebo_ros::Node::SharedPtr node, const std::string prefix)
{
  if (!node) {
    return;
  }
  
  // Declare the parameters
  node->declare_parameter<double>(_offset, offset);
  node->declare_parameter<double>(_drift, drift);
  node->declare_parameter<double>(_drift_frequency, drift_frequency);
  node->declare_parameter<double>(_gaussian_noise, gaussian_noise);
  node->declare_parameter<double>(_scale_error, gaussian_noise);
}

#if (GAZEBO_MAJOR_VERSION >= 8)
template <>
void SensorModel_<ignition::math::Vector3d>::initializeParameters(gazebo_ros::Node::SharedPtr node, const std::string prefix)
{
  if (!node) {
    return;
  }
  
  // Declare the parameters
  node->declare_parameter<double>(_offset + "/x", offset[0]);
  node->declare_parameter<double>(_offset + "/y", offset[1]);
  node->declare_parameter<double>(_offset + "/z", offset[2]);
  node->declare_parameter<double>(_drift + "/x", drift[0]);
  node->declare_parameter<double>(_drift + "/y", drift[1]);
  node->declare_parameter<double>(_drift + "/z", drift[2]);
  node->declare_parameter<double>(_drift_frequency + "/x", drift_frequency[0]);
  node->declare_parameter<double>(_drift_frequency + "/y", drift_frequency[1]);
  node->declare_parameter<double>(_drift_frequency + "/z", drift_frequency[2]);
  node->declare_parameter<double>(_gaussian_noise + "/x", gaussian_noise[0]);
  node->declare_parameter<double>(_gaussian_noise + "/y", gaussian_noise[1]);
  node->declare_parameter<double>(_gaussian_noise + "/z", gaussian_noise[2]);
  node->declare_parameter<double>(_scale_error + "/x", scale_error[0]);
  node->declare_parameter<double>(_scale_error + "/y", scale_error[1]);
  node->declare_parameter<double>(_scale_error + "/z", scale_error[2]);
}
#else
template <>
void SensorModel_<math::Vector3>::initializeParameters(gazebo_ros::Node::SharedPtr node, const std::string prefix)
{
  if (!node) {
    return;
  }

  // Declare the parameters
  node->declare_parameter<double>(_offset + "/x", offset[0]);
  node->declare_parameter<double>(_offset + "/y", offset[1]);
  node->declare_parameter<double>(_offset + "/z", offset[2]);
  node->declare_parameter<double>(_drift + "/x", drift[0]);
  node->declare_parameter<double>(_drift + "/y", drift[1]);
  node->declare_parameter<double>(_drift + "/z", drift[2]);
  node->declare_parameter<double>(_drift_frequency + "/x", drift_frequency[0]);
  node->declare_parameter<double>(_drift_frequency + "/y", drift_frequency[1]);
  node->declare_parameter<double>(_drift_frequency + "/z", drift_frequency[2]);
  node->declare_parameter<double>(_gaussian_noise + "/x", gaussian_noise[0]);
  node->declare_parameter<double>(_gaussian_noise + "/y", gaussian_noise[1]);
  node->declare_parameter<double>(_gaussian_noise + "/z", gaussian_noise[2]);
  node->declare_parameter<double>(_scale_error + "/x", scale_error[0]);
  node->declare_parameter<double>(_scale_error + "/y", scale_error[1]);
  node->declare_parameter<double>(_scale_error + "/z", scale_error[2]);
}
#endif


namespace helpers {
  template <typename T> struct scalar_value { static double toDouble(const T &orig) { return orig; } };
  template <typename T> struct scalar_value<std::vector<T> > { static double toDouble(const std::vector<T> &orig) { return (double) std::accumulate(orig.begin(), orig.end()) / orig.size(); } };
#if (GAZEBO_MAJOR_VERSION >= 8)
  template <> struct scalar_value<ignition::math::Vector3d> { static double toDouble(const ignition::math::Vector3d &orig) { return (orig.X() + orig.Y() + orig.Z()) / 3; } };
#else
  template <> struct scalar_value<math::Vector3> { static double toDouble(const math::Vector3 &orig) { return (orig.x + orig.y + orig.z) / 3; } };
#endif
}

template <>
rcl_interfaces::msg::SetParametersResult 
SensorModel_<double>::setValue(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto & parameter : parameters) {
    std::string name = parameter.get_name();

    auto search = parameters_.find(name);
    if (search != parameters_.end()) {
      // Set the value in the local variable
      if (parameter.as_double() >= search->second.min && 
          parameter.as_double() <= search->second.max) 
      {
        *(search->second.v) = parameter.as_double();
      } else {
        result.successful = false;
        result.reason = "Parameter outside range";
        break;
      }

      break;
    }
  }

  return result;
}

#if (GAZEBO_MAJOR_VERSION >= 8)
template <>
rcl_interfaces::msg::SetParametersResult 
SensorModel_<ignition::math::Vector3d>::setValue(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto & parameter : parameters) {
    std::string name = parameter.get_name();
    std::string p = name.substr(0, name.size() - 2);
    std::string v = name.substr(name.size() - 1, 1);

    auto search = parameters_.find(p);
    if (search != parameters_.end()) {
      if (parameter.as_double() >= search->second.min && 
          parameter.as_double() <= search->second.max) {

        // Set the value in the local variable
        ignition::math::Vector3d *ptr = search->second.v;

        if (v == "x") {
          ptr->operator[](0) = parameter.as_double(); 
        } else if (v == "y") {
          ptr->operator[](1) = parameter.as_double(); 
        } else if (v == "z") {
          ptr->operator[](2) = parameter.as_double(); 
        } else {
          result.successful = false;
          result.reason = "Invalid parameter";
          break;
        }
      } else {
        result.successful = false;
        result.reason = "Parameter out of range";
      }
    }
  }

  return result;
}
#else
template <>
rcl_interfaces::msg::SetParametersResult 
SensorModel_<math::Vector3>::setValue(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto & parameter : parameters) {
    std::string name = parameter.get_name();
    std::string p = name.substr(0, name.size() - 2);
    std::string v = name.substr(name.size() - 1, 1);

    auto search = parameters_.find(p);
    if (search != parameters_.end()) {
      if (parameter.as_double() >= search->second.min && 
          parameter.as_double() <= search->second.max) {

        // Set the value in the local variable
        math::Vector3 *ptr = search->second.v;

        if (v == "x") {
          ptr->operator[](0) = parameter.as_double(); 
        } else if (v == "y") {
          ptr->operator[](1) = parameter.as_double(); 
        } else if (v == "z") {
          ptr->operator[](2) = parameter.as_double(); 
        } else {
          result.successful = false;
          result.reason = "Invalid parameter";
          break;
        }
      } else {
        result.successful = false;
        result.reason = "Parameter out of range";
      }
    }
  }

  return result;
}
#endif


template <typename T>
rcl_interfaces::msg::SetParametersResult 
SensorModel_<T>::parametersChangedCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  result = setValue(parameters);

  return result;
}

typedef SensorModel_<double> SensorModel;
#if (GAZEBO_MAJOR_VERSION >= 8)
typedef SensorModel_<ignition::math::Vector3d> SensorModel3;
#else
typedef SensorModel_<math::Vector3> SensorModel3;
#endif

}

#endif // HECTOR_GAZEBO_PLUGINS_SENSOR_MODEL_H
