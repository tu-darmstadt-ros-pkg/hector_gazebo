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

#ifndef GAZEBO_RTT_PLUGIN_GAZEBO_RTT_PLUGIN_H
#define GAZEBO_RTT_PLUGIN_GAZEBO_RTT_PLUGIN_H

#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>

#include <rtt/TaskContext.hpp>

namespace gazebo
{
  class RTTPlugin : public ModelPlugin
  {
  public:
    RTTPlugin();
    virtual ~RTTPlugin();

    virtual void Init();
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

    // Called by the world update start event
    virtual void OnUpdate();

    // Called on model reset (stops and restarts the TaskContext)
    virtual void Reset();

  protected:
    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the world
    physics::WorldPtr world;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Pointer to the plugin's ExecutionEngine and TaskContext
    boost::shared_ptr<RTT::TaskContext> taskContext;

  private:
    class RTTOSInitializer;
    boost::shared_ptr<RTTOSInitializer> rttOsInitializer;
  };

} // namespace gazebo

#endif // GAZEBO_RTT_PLUGIN_GAZEBO_RTT_PLUGIN_H
