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

#include <gazebo_rtt_plugin/gazebo_activity.h>
#include <gazebo/physics/PhysicsEngine.hh>

#include <rtt/base/RunnableInterface.hpp>

#include <ros/console.h>

namespace gazebo {

using namespace RTT::base;
using namespace common;

GazeboActivity::GazeboActivity(const std::string& name, RunnableInterface* run /*= 0*/ )
  : ActivityInterface(run), mname(name), running(false), active(false)
{
}

GazeboActivity::GazeboActivity(const std::string& name, common::Time period, physics::WorldPtr world, RunnableInterface* run /*= 0*/ )
  : ActivityInterface(run), mname(name), mworld(world), running(false), active(false)
{
  updateTimer.setUpdatePeriod(period);
}

GazeboActivity::~GazeboActivity()
{
  stop();
}

void GazeboActivity::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf, const std::string& _prefix)
{
  mworld = _world;
  updateTimer.Load(_world, _sdf, _prefix);
}

RTT::Seconds GazeboActivity::getPeriod() const
{
  double period = updateTimer.getUpdatePeriod().Double();
  if (period > 0.0)
    return RTT::Seconds(period);
  else
    return RTT::Seconds(mworld->GetPhysicsEngine()->GetUpdatePeriod());
}

bool GazeboActivity::setPeriod(RTT::Seconds s) {
  updateTimer.setUpdatePeriod(s);
  return true;
}

unsigned GazeboActivity::getCpuAffinity() const
{
  return ~0;
}

bool GazeboActivity::setCpuAffinity(unsigned cpu)
{
  return false;
}

RTT::os::ThreadInterface* GazeboActivity::thread()
{
  return 0;
}

bool GazeboActivity::initialize()
{
  return true;
}

void GazeboActivity::step()
{
}

void GazeboActivity::loop()
{
  this->step();
}

bool GazeboActivity::breakLoop()
{
  return false;
}


void GazeboActivity::finalize()
{
}

bool GazeboActivity::start()
{
  if ( active == true )
  {
    gzerr << "Unable to start slave as it is already started" << std::endl;
    return false;
  }

  active = true;
  updateTimer.Reset();

  if ( runner ? runner->initialize() : this->initialize() ) {
    running = this->isPeriodic();
  } else {
    active = false;
  }
  return active;
}


bool GazeboActivity::stop()
{
  if ( !active )
    return false;

  // use breakLoop if not periodic and within loop
  if ( this->isPeriodic() == false) {
    if ( running && (runner ? (runner->breakLoop() == false): (this->breakLoop() == false) ) )
      return false;
  }

  running = false;
  if (runner)
    runner->finalize();
  else
    this->finalize();
  active = false;
  return true;
}

bool GazeboActivity::isRunning() const
{
  return running;
}

bool GazeboActivity::isPeriodic() const
{
  return true;
}
bool GazeboActivity::isActive() const
{
  return active;
}

bool GazeboActivity::trigger()
{
  return false;
}

bool GazeboActivity::execute()
{
  if (!running) return false;
  if (!updateTimer.update()) return true;

  ROS_DEBUG_NAMED("gazebo_rtt_plugin", "Updating RTT plugin %s at t = %f", mname.c_str(), mworld->GetSimTime().Double());
  if (runner) runner->step(); else this->step();

  return true;
}

} // namespace gazebo
