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

#ifndef GAZEBO_RTT_PLUGIN_GAZEBO_RTT_PUBLISHER_H
#define GAZEBO_RTT_PLUGIN_GAZEBO_RTT_PUBLISHER_H

#include <ros/publisher.h>

#include <rtt/base/ChannelElement.hpp>
#include <rtt/OutputPort.hpp>

namespace gazebo {

class RTTPublisher : public ros::Publisher
{
public:
  RTTPublisher() : connection_id(this) {}
  RTTPublisher(const ros::Publisher& pub) : ros::Publisher(pub), connection_id(this) {}

  template <typename T>
  class PublisherChannelElement : public RTT::base::ChannelElement<T>
  {
  public:
    PublisherChannelElement(const RTTPublisher* publisher) : publisher(publisher) {}
    virtual ~PublisherChannelElement() {}

    virtual bool write(typename RTT::base::ChannelElement<T>::param_t sample) {
      if (!publisher || !*publisher) return false;
      publisher->publish(sample);
      return true;
    }

  private:
    const RTTPublisher* publisher;
  };

  class ConnID : public RTT::internal::ConnID
  {
  public:
    ConnID(const RTTPublisher* publisher) : publisher(publisher) {}
    virtual ~ConnID() {}

    virtual bool isSameID(RTT::internal::ConnID const& id) const {
      const ConnID *other = dynamic_cast<const ConnID *>(&id);
      if (!other) return false;
      return other == this;
    }

    virtual RTT::internal::ConnID* clone() const {
      return new ConnID(*this);
    }

  private:
    const RTTPublisher* publisher;
  };

  template <typename T>
  bool connect(RTT::base::PortInterface *port) {
    RTT::OutputPort<T> *output_port = dynamic_cast<RTT::OutputPort<T> *>(port);
    if (!output_port) return false;

    channel.reset(new PublisherChannelElement<T>(this));
    return output_port->addConnection(&connection_id, channel, RTT::ConnPolicy());
  }

private:
  ConnID connection_id;
  RTT::base::ChannelElementBase::shared_ptr channel;
};

} // namespace gazebo

#endif // GAZEBO_RTT_PLUGIN_GAZEBO_RTT_PUBLISHER_H
