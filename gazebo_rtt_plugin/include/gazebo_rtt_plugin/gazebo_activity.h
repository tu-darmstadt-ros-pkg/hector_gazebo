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

#ifndef GAZEBO_RTT_PLUGIN_GAZEBO_ACTIVITY_H
#define GAZEBO_RTT_PLUGIN_GAZEBO_ACTIVITY_H

#include <rtt/base/ActivityInterface.hpp>
#include <gazebo/physics/World.hh>

#include <hector_gazebo_plugins/update_timer.h>

namespace gazebo {

class GazeboActivity : public RTT::base::ActivityInterface
{
public:
    GazeboActivity(const std::string& name, RTT::base::RunnableInterface* run = 0);
    GazeboActivity(const std::string& name, common::Time period, physics::WorldPtr world, RTT::base::RunnableInterface* run = 0);
    ~GazeboActivity();

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf, const std::string& _prefix = std::string());

    RTT::Seconds getPeriod() const;
    bool setPeriod(RTT::Seconds s);

    unsigned getCpuAffinity() const;
    bool setCpuAffinity(unsigned cpu);

    RTT::os::ThreadInterface* thread();

    bool initialize();
    void step();
    void loop();
    bool breakLoop();
    void finalize();

    bool start();
    bool stop();

    bool isRunning() const;
    bool isPeriodic() const;
    bool isActive() const;

    bool execute();
    bool trigger();

private:
    std::string mname;
    physics::WorldPtr mworld;
    bool running;
    bool active;

    UpdateTimer updateTimer;
};

} // namespace gazebo

#endif // GAZEBO_RTT_PLUGIN_GAZEBO_ACTIVITY_H
