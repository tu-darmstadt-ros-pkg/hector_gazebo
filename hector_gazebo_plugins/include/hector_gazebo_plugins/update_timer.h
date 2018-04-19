//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
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

#ifndef HECTOR_GAZEBO_PLUGINS_UPDATE_TIMER_H
#define HECTOR_GAZEBO_PLUGINS_UPDATE_TIMER_H

#include <sdf/sdf.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>

#include <gazebo/common/Event.hh>
#include <gazebo/common/Events.hh>

namespace gazebo {

class UpdateTimer {
public:
  UpdateTimer()
    : connection_count_(0)
  {
  }

  virtual ~UpdateTimer()
  {
  }

  virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf, const std::string& _prefix = "update")
  {
    this->world_ = _world;

    if (_sdf->HasElement(_prefix + "Rate")) {
      double update_rate = 0.0;
      _sdf->GetElement(_prefix + "Rate")->GetValue()->Get(update_rate);
      update_period_ = update_rate > 0.0 ? 1.0/update_rate : 0.0;
    }

    if (_sdf->HasElement(_prefix + "Period")) {
      _sdf->GetElement(_prefix + "Period")->GetValue()->Get(update_period_);
    }

    if (_sdf->HasElement(_prefix + "Offset")) {
      _sdf->GetElement(_prefix + "Offset")->GetValue()->Get(update_offset_);
    }
  }

  virtual event::ConnectionPtr Connect(const boost::function<void()> &_subscriber, bool connectToWorldUpdateBegin = true)
  {
    if (connectToWorldUpdateBegin && !update_connection_) {
      update_connection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&UpdateTimer::Update, this));
    }
    connection_count_++;
    return update_event_.Connect(_subscriber);
  }

  virtual void Disconnect(event::ConnectionPtr const& _c = event::ConnectionPtr())
  {
    //if (_c) update_event_.Disconnect(_c);
    if (_c) update_event_.Disconnect(_c->Id());
    if (update_connection_ && (!_c || --connection_count_ == 0)) {
      // event::Events::DisconnectWorldUpdateBegin(update_connection_);
      // not sure if that'll work?
      update_connection_.reset();
    }
  }

  common::Time const& getUpdatePeriod() const {
    return update_period_;
  }

  void setUpdatePeriod(common::Time const& period) {
    update_period_ = period;
  }

  double getUpdateRate() const {
    double period = update_period_.Double();
    return (period > 0.0)  ? (1.0 / period) : 0.0;
  }

  void setUpdateRate(double rate) {
    update_period_ = (rate > 0.0) ? (1.0 / rate) : 0.0;
  }

  common::Time const& getLastUpdate() const {
    return last_update_;
  }

  common::Time getTimeSinceLastUpdate() const {
    if (last_update_ == common::Time()) return common::Time();
    return world_->SimTime() - last_update_;
  }

  virtual bool checkUpdate() const
  {
    double period = update_period_.Double();
    double step = world_->Physics()->GetMaxStepSize();
    if (period == 0) return true;
    double fraction = fmod((world_->SimTime() - update_offset_).Double() + (step / 2.0), period);
    return (fraction >= 0.0) && (fraction < step);
  }

  virtual bool update()
  {
    if (!checkUpdate()) return false;
    last_update_ = world_->SimTime();
    return true;
  }

  virtual bool update(double& dt)
  {
    dt = getTimeSinceLastUpdate().Double();
    return update();
  }

  virtual void Reset()
  {
    last_update_ = common::Time();
  }

protected:
  virtual bool Update()
  {
    if (!checkUpdate()) {
      return false;
    }
    update_event_();
    last_update_ = world_->SimTime();
    return true;
  }

private:
  physics::WorldPtr world_;
  common::Time update_period_;
  common::Time update_offset_;
  common::Time last_update_;

  event::EventT<void()> update_event_;
  unsigned int connection_count_;
  event::ConnectionPtr update_connection_;
};

} // namespace gazebo

#endif // HECTOR_GAZEBO_PLUGINS_UPDATE_TIMER_H
