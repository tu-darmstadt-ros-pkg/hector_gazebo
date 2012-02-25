/*
 *  gazebo thermal camera emulator plugin
 *  Copyright (C) 2012
 *     Stefan Kohlbrecher, TU Darmstadt
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

//based on:
/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 @mainpage
   Desc: GazeboRosThermalCamera plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
   SVN info: $Id$
 @htmlinclude manifest.html
 @b GazeboRosThermalCamera plugin broadcasts ROS Image messages
 */

#include <algorithm>
#include <assert.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <hector_gazebo_thermal_camera/gazebo_ros_thermal_camera.h>

#include <gazebo/Timer.hh>
#include <gazebo/Sensor.hh>
#include <gazebo/Model.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/MonoCameraSensor.hh>
#include <gazebo/Pose3d.hh>


#include "sensor_msgs/Image.h"
#include "sensor_msgs/fill_image.h"
#include "image_transport/image_transport.h"

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>

namespace gazebo
{

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_thermal_camera", GazeboRosThermalCamera);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosThermalCamera::GazeboRosThermalCamera(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<MonoCameraSensor*>(this->parent);

  if (!this->myParent)
    gzthrow("GazeboRosThermalCamera controller requires a Camera Sensor as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace","/",0);
  this->imageTopicNameP = new ParamT<std::string>("imageTopicName","image_raw", 0);
  this->cameraInfoTopicNameP = new ParamT<std::string>("cameraInfoTopicName","camera_info", 0);
  this->pointCloudTopicNameP = new ParamT<std::string>("pointCloudTopicName","points", 0);
  this->cameraNameP = new ParamT<std::string>("cameraName","", 0);
  this->frameNameP = new ParamT<std::string>("frameName","generic_camera_link", 0);
  // camera parameters 
  this->CxPrimeP = new ParamT<double>("CxPrime",0, 0); // default to 0 for compute on the fly
  this->CxP  = new ParamT<double>("Cx" ,0, 0); // default to 0 for compute on the fly
  this->CyP  = new ParamT<double>("Cy" ,0, 0); // default to 0 for compute on the fly
  this->focal_lengthP  = new ParamT<double>("focal_length" ,0, 0); // == image_width(px) / (2*tan( hfov(radian) /2)), default to 0 for compute on the fly
  this->hack_baselineP  = new ParamT<double>("hackBaseline" ,0, 0); // hack for right stereo camera
  this->pointCloudCutoffP  = new ParamT<double>("pointCloudCutoff" ,0.4, 0);
  this->distortion_k1P  = new ParamT<double>("distortion_k1" ,0, 0);
  this->distortion_k2P  = new ParamT<double>("distortion_k2" ,0, 0);
  this->distortion_k3P  = new ParamT<double>("distortion_k3" ,0, 0);
  this->distortion_t1P  = new ParamT<double>("distortion_t1" ,0, 0);
  this->distortion_t2P  = new ParamT<double>("distortion_t2" ,0, 0);
  Param::End();

  this->imageConnectCount = 0;
  this->infoConnectCount = 0;
  this->pointCloudConnectCount = 0;

  last_image_pub_time_       = Time(0);
  last_camera_info_pub_time_ = Time(0);
  last_point_cloud_pub_time_ = Time(0);

  // set sensor update rate to controller update rate?
  //(dynamic_cast<OgreCamera*>(this->myParent))->SetUpdateRate(this->updateRateP->GetValue());
}

void GazeboRosThermalCamera::configCallback(hector_gazebo_thermal_camera::GazeboRosThermalCameraConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request for the gazebo ros camera: %s. New rate: %.2f", this->cameraName.c_str(), config.imager_rate);

  (dynamic_cast<OgreCamera*>(this->myParent))->SetUpdateRate(config.imager_rate);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosThermalCamera::~GazeboRosThermalCamera()
{
  delete this->robotNamespaceP;
  delete this->rosnode_;
  delete this->managernode_;
  delete this->manager_;
  delete this->imageTopicNameP;
  delete this->cameraInfoTopicNameP;
  delete this->pointCloudTopicNameP;
  delete this->frameNameP;
  delete this->CxPrimeP;
  delete this->CxP;
  delete this->CyP;
  delete this->focal_lengthP;
  delete this->hack_baselineP;
  delete this->pointCloudCutoffP;
  delete this->distortion_k1P;
  delete this->distortion_k2P;
  delete this->distortion_k3P;
  delete this->distortion_t1P;
  delete this->distortion_t2P;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosThermalCamera::LoadChild(XMLConfigNode *node)
{
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  this->imageTopicNameP->Load(node);
  this->cameraInfoTopicNameP->Load(node);
  this->pointCloudTopicNameP->Load(node);
  this->cameraNameP->Load(node);
  this->frameNameP->Load(node);
  this->CxPrimeP->Load(node);
  this->CxP->Load(node);
  this->CyP->Load(node);
  this->focal_lengthP->Load(node);
  this->hack_baselineP->Load(node);
  this->pointCloudCutoffP->Load(node);
  this->distortion_k1P->Load(node);
  this->distortion_k2P->Load(node);
  this->distortion_k3P->Load(node);
  this->distortion_t1P->Load(node);
  this->distortion_t2P->Load(node);
  this->imageTopicName = this->imageTopicNameP->GetValue();
  this->cameraInfoTopicName = this->cameraInfoTopicNameP->GetValue();
  this->pointCloudTopicName = this->pointCloudTopicNameP->GetValue();
  this->frameName = this->frameNameP->GetValue();
  this->CxPrime = this->CxPrimeP->GetValue();
  this->Cx = this->CxP->GetValue();
  this->Cy = this->CyP->GetValue();
  this->focal_length = this->focal_lengthP->GetValue();
  this->hack_baseline = this->hack_baselineP->GetValue();
  this->pointCloudCutoff = this->pointCloudCutoffP->GetValue();
  this->distortion_k1 = this->distortion_k1P->GetValue();
  this->distortion_k2 = this->distortion_k2P->GetValue();
  this->distortion_k3 = this->distortion_k3P->GetValue();
  this->distortion_t1 = this->distortion_t1P->GetValue();
  this->distortion_t2 = this->distortion_t2P->GetValue();
  if ((this->distortion_k1 != 0.0) || (this->distortion_k2 != 0.0) ||
      (this->distortion_k3 != 0.0) || (this->distortion_t1 != 0.0) ||
      (this->distortion_t2 != 0.0))
    ROS_WARN("gazebo_ros_camera simulation does not support non-zero distortion parameters right now, your simulation maybe wrong.");

  this->cameraName = this->cameraNameP->GetValue();
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace+"/"+this->cameraName);
  if (!this->cameraName.empty())
    this->managernode_ = new ros::NodeHandle(this->robotNamespace+"/"+this->cameraName+"_manager");
  else
    this->managernode_ = new ros::NodeHandle(this->robotNamespace+"/"+this->GetName()+"_manager");
  this->manager_ = new nodelet::Loader(*this->managernode_);
  this->itnode_ = new image_transport::ImageTransport(*this->rosnode_);

  if (!this->cameraName.empty()) {
    dyn_srv_ = new dynamic_reconfigure::Server<hector_gazebo_thermal_camera::GazeboRosThermalCameraConfig>(*this->rosnode_);
    dynamic_reconfigure::Server<hector_gazebo_thermal_camera::GazeboRosThermalCameraConfig>::CallbackType f = boost::bind(&GazeboRosThermalCamera::configCallback, this, _1, _2);
    dyn_srv_->setCallback(f);
  }


#ifdef USE_CBQ
  this->image_pub_ = this->itnode_->advertise(
    this->imageTopicName,1,
    boost::bind( &GazeboRosThermalCamera::ImageConnect,this),
    boost::bind( &GazeboRosThermalCamera::ImageDisconnect,this), ros::VoidPtr(), &this->camera_queue_);

  ros::AdvertiseOptions camera_info_ao = ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
    this->cameraInfoTopicName,1,
    boost::bind( &GazeboRosThermalCamera::InfoConnect,this),
    boost::bind( &GazeboRosThermalCamera::InfoDisconnect,this), ros::VoidPtr(), &this->camera_queue_);
  this->camera_info_pub_ = this->rosnode_->advertise(camera_info_ao);

  ros::SubscribeOptions zoom_so = ros::SubscribeOptions::create<std_msgs::Float64>(
    "set_hfov",1,
    boost::bind( &GazeboRosThermalCamera::SetHFOV,this,_1),
    ros::VoidPtr(), &this->camera_queue_);
  this->cameraHFOVSubscriber_ = this->rosnode_->subscribe(zoom_so);

  ros::SubscribeOptions rate_so = ros::SubscribeOptions::create<std_msgs::Float64>(
    "set_update_rate",1,
    boost::bind( &GazeboRosThermalCamera::SetUpdateRate,this,_1),
    ros::VoidPtr(), &this->camera_queue_);
  this->cameraUpdateRateSubscriber_ = this->rosnode_->subscribe(rate_so);

  ros::AdvertiseOptions point_cloud_ao = ros::AdvertiseOptions::create<pcl::PointCloud<pcl::PointXYZ> >(
    this->pointCloudTopicName,1,
    boost::bind( &GazeboRosThermalCamera::PointCloudConnect,this),
    boost::bind( &GazeboRosThermalCamera::PointCloudDisconnect,this), ros::VoidPtr(), &this->camera_queue_);
  this->point_cloud_pub_ = this->rosnode_->advertise(point_cloud_ao);

#else
  this->image_pub_ = this->rosnode_->advertise<sensor_msgs::Image>(this->imageTopicName,1,
    boost::bind( &GazeboRosThermalCamera::ImageConnect, this),
    boost::bind( &GazeboRosThermalCamera::ImageDisconnect, this));
  this->camera_info_pub_ = this->rosnode_->advertise<sensor_msgs::CameraInfo>(this->cameraInfoTopicName,1,
    boost::bind( &GazeboRosThermalCamera::InfoConnect, this),
    boost::bind( &GazeboRosThermalCamera::InfoDisconnect, this));
  this->point_cloud_pub_ = this->rosnode_->advertise<pcl::PointCloud<pcl::PointXYZ> >(this->pointCloudTopicName,1,
    boost::bind( &GazeboRosThermalCamera::PointCloudConnect, this),
    boost::bind( &GazeboRosThermalCamera::PointCloudDisconnect, this));
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosThermalCamera::InfoConnect()
{
  this->infoConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosThermalCamera::InfoDisconnect()
{
  this->infoConnectCount--;
}

////////////////////////////////////////////////////////////////////////////////
// Set Horizontal Field of View
void GazeboRosThermalCamera::SetHFOV(const std_msgs::Float64::ConstPtr& hfov)
{
  (dynamic_cast<OgreCamera*>(this->myParent))->SetFOV(hfov->data);
}

////////////////////////////////////////////////////////////////////////////////
// Set Update Rate
void GazeboRosThermalCamera::SetUpdateRate(const std_msgs::Float64::ConstPtr& update_rate)
{
  (dynamic_cast<OgreCamera*>(this->myParent))->SetUpdateRate(update_rate->data);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosThermalCamera::ImageConnect()
{
  this->imageConnectCount++;
  this->myParent->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosThermalCamera::ImageDisconnect()
{
  this->imageConnectCount--;

  if (this->pointCloudConnectCount == 0)
  {
    this->myParent->SimulateDepthData(false);
    if (this->imageConnectCount == 0)
      this->myParent->SetActive(false);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosThermalCamera::PointCloudConnect()
{
  this->pointCloudConnectCount++;
  this->myParent->SetActive(true);
  this->myParent->SimulateDepthData(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosThermalCamera::PointCloudDisconnect()
{
  this->pointCloudConnectCount--;

  if (this->pointCloudConnectCount == 0)
  {
    this->myParent->SimulateDepthData(false);
    if (this->imageConnectCount == 0)
      this->myParent->SetActive(false);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosThermalCamera::InitChild()
{
  // sensor generation off by default
  this->myParent->SetActive(false);
  this->myParent->SimulateDepthData(false);

  // set buffer size
  this->width            = this->myParent->GetImageWidth();
  this->height           = this->myParent->GetImageHeight();
  this->depth            = this->myParent->GetImageDepth();
  if (this->myParent->GetImageFormat() == "L8")
  {
    this->type = sensor_msgs::image_encodings::MONO8;
    this->skip = 1;
  }
  else if (this->myParent->GetImageFormat() == "R8G8B8")
  {
    this->type = sensor_msgs::image_encodings::RGB8;
    this->skip = 3;
  }
  else if (this->myParent->GetImageFormat() == "B8G8R8")
  {
    this->type = sensor_msgs::image_encodings::BGR8;
    this->skip = 3;
  }
  else if (this->myParent->GetImageFormat() == "BAYER_RGGB8")
  {
    ROS_WARN("bayer simulation maybe computationally expensive.");
    this->type = sensor_msgs::image_encodings::BAYER_RGGB8;
    this->skip = 1;
  }
  else if (this->myParent->GetImageFormat() == "BAYER_BGGR8")
  {
    ROS_WARN("bayer simulation maybe computationally expensive.");
    this->type = sensor_msgs::image_encodings::BAYER_BGGR8;
    this->skip = 1;
  }
  else if (this->myParent->GetImageFormat() == "BAYER_GBRG8")
  {
    ROS_WARN("bayer simulation maybe computationally expensive.");
    this->type = sensor_msgs::image_encodings::BAYER_GBRG8;
    this->skip = 1;
  }
  else if (this->myParent->GetImageFormat() == "BAYER_GRBG8")
  {
    ROS_WARN("bayer simulation maybe computationally expensive.");
    this->type = sensor_msgs::image_encodings::BAYER_GRBG8;
    this->skip = 1;
  }
  else
  {
    ROS_ERROR("Unsupported Gazebo ImageFormat\n");
    this->type = sensor_msgs::image_encodings::BGR8;
    this->skip = 3;
  }

  /// Compute camera parameters if set to 0
  if (this->CxPrime == 0)
    this->CxPrime = ((double)this->width+1.0) /2.0;
  if (this->Cx == 0)
    this->Cx = ((double)this->width+1.0) /2.0;
  if (this->Cy == 0)
    this->Cy = ((double)this->height+1.0) /2.0;


  double computed_focal_length = ((double)this->width) / (2.0 *tan(this->myParent->GetHFOV().GetAsRadian()/2.0));
  if (this->focal_length == 0)
    this->focal_length = computed_focal_length;
  else
    if (fabs(this->focal_length - computed_focal_length) > 1e-8) // check against float precision
      ROS_WARN("The <focal_length>[%f] you have provided for camera [%s] is inconsistent with specified image_width [%d] and HFOV [%f].   Please double check to see that focal_length = width / (2.0 * tan( HFOV/2.0 )), the explected focal_lengtth value is [%f], please update your camera model description accordingly.",
                this->focal_length,this->myParent->GetName().c_str(),this->width,this->myParent->GetHFOV().GetAsRadian(),
                computed_focal_length);


#ifdef USE_CBQ
  // start custom queue for camera
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosThermalCamera::CameraQueueThread,this ) );
#endif


}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosThermalCamera::UpdateChild()
{

  // Only publish to ROS when the camera updates
  Time sensor_update_time = (dynamic_cast<OgreCamera*>(this->myParent))->GetLastRenderTime();

  // as long as ros is connected, parent is active
  if (this->myParent->IsActive())
  {
    if (sensor_update_time > last_image_pub_time_)
      this->PutCameraData();
  }

  /// publish CameraInfo
  if (this->infoConnectCount > 0) //publish regardless camera image subscription
    this->PublishCameraInfo();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosThermalCamera::FiniChild()
{
  this->myParent->SetActive(false);
  this->manager_->clear();
  this->managernode_->shutdown();
  this->rosnode_->shutdown();
#ifdef USE_CBQ
  this->camera_queue_.clear();
  this->camera_queue_.disable();
  this->callback_queue_thread_.join();
#endif

}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosThermalCamera::PutCameraData()
{
  //ROS_ERROR("debug %d %s",this->imageConnectCount,this->GetName().c_str());
  const unsigned char *src;
  const float* depths;

  // Get a pointer to image data
  {
    // prevent parent from updating the image and the depth data while we are retrieving it
    boost::recursive_mutex::scoped_lock mr_lock(*Simulator::Instance()->GetMRMutex());

    //DIAGNOSTICTIMER(timer("gazebo_ros_camera: GetImageData",6));
    src = this->myParent->GetImageData(0);
    if (src)
    {
      //double tmpT0 = Simulator::Instance()->GetWallTime();

      unsigned char dst[this->width*this->height];

      this->lock.lock();
      // copy data into image
      this->imageMsg.header.frame_id = this->frameName;
      Time lastRenderTime = (dynamic_cast<OgreCamera*>(this->myParent))->GetLastRenderTime();
      //Time lastRenderTime = Simulator::Instance()->GetSimTime();
      //printf("name[%s] render[%f] vs. sim time[%f], diff[%f]\n",this->GetName().c_str(),lastRenderTime.Double(),Simulator::Instance()->GetSimTime().Double(),lastRenderTime.Double()-Simulator::Instance()->GetSimTime().Double());
      //ROS_DEBUG("camera time %f %d %d",lastRenderTime.Double(),lastRenderTime.sec,lastRenderTime.nsec);
      this->imageMsg.header.stamp.sec = lastRenderTime.sec;
      this->imageMsg.header.stamp.nsec = lastRenderTime.nsec;

      //double tmpT1 = Simulator::Instance()->GetWallTime();
      //double tmpT2;

      /// @todo: don't bother if there are no subscribers
      if (this->image_pub_.getNumSubscribers() > 0)
      {

        //convertImageFormat(dst,src);

        // copy from src to imageMsg
        fillImage(this->imageMsg,
                  this->type,
                  this->height,
                  this->width,
                  this->skip*this->width,
                  (void*)src );

        //tmpT2 = Simulator::Instance()->GetWallTime();

        // publish to ros
        last_image_pub_time_ = Simulator::Instance()->GetSimTime();
        this->image_pub_.publish(this->imageMsg);
      }

      //double tmpT3 = Simulator::Instance()->GetWallTime();

      this->lock.unlock();
    }
  }

  {
    // prevent parent from updating the image and the depth data while we are retrieving it
    boost::recursive_mutex::scoped_lock mr_lock(*Simulator::Instance()->GetMRMutex());

    depths = this->myParent->GetDepthData(0);
    if (depths)
    {
      //double tmpT0 = Simulator::Instance()->GetWallTime();
      this->lock.lock();
      this->pointCloudMsg.header.frame_id = this->frameName;
      Time lastRenderTime = (dynamic_cast<OgreCamera*>(this->myParent))->GetLastRenderTime();
      this->pointCloudMsg.header.stamp.sec = lastRenderTime.sec;
      this->pointCloudMsg.header.stamp.nsec = lastRenderTime.nsec;
      this->pointCloudMsg.width = this->width;
      this->pointCloudMsg.height = this->height;

      ///copy from depth to pointCloudMsg
      fillDepthImage(this->pointCloudMsg,
                this->height,
                this->width,
                this->skip,
                (void*)depths );
      /*
      for (int i = 0 ; i < this->height; i++)
      for (int j = 0 ; j < this->width; i++) {
        ROS_ERROR("test %d %d %f",i,j,depths[j + i*this->width]);
      }
      */

      //tmpT2 = Simulator::Instance()->GetWallTime();

      // publish to ros
      this->point_cloud_pub_.publish(this->pointCloudMsg);

      //double tmpT3 = Simulator::Instance()->GetWallTime();

      this->lock.unlock();
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
// Convert image format from gazebo sensor provided foramt to desired ros format
void GazeboRosThermalCamera::convertImageFormat(unsigned char *dst, const unsigned char *src)
{
      // do last minute conversion if Bayer pattern is requested but not provided, go from R8G8B8
      // deprecated in gazebo2 branch, keep for backwards compatibility
      if (this->myParent->GetImageFormat() == "BAYER_RGGB8" && this->depth == 3)
      {
        for (int i=0;i<this->width;i++)
        {
          for (int j=0;j<this->height;j++)
          {
            //
            // RG
            // GB
            //
            // determine position
            if (j%2) // even column
              if (i%2) // even row, red
                dst[i+j*this->width] = src[i*3+j*this->width*3+0];
              else // odd row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
            else // odd column
              if (i%2) // even row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
              else // odd row, blue
                dst[i+j*this->width] = src[i*3+j*this->width*3+2];
          }
        }
        src=dst;
      }
      else if (this->myParent->GetImageFormat() == "BAYER_BGGR8" && this->depth == 3)
      {
        for (int i=0;i<this->width;i++)
        {
          for (int j=0;j<this->height;j++)
          {
            //
            // BG
            // GR
            //
            // determine position
            if (j%2) // even column
              if (i%2) // even row, blue
                dst[i+j*this->width] = src[i*3+j*this->width*3+2];
              else // odd row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
            else // odd column
              if (i%2) // even row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
              else // odd row, red
                dst[i+j*this->width] = src[i*3+j*this->width*3+0];
          }
        }
        src=dst;
      }
      else if (this->myParent->GetImageFormat() == "BAYER_GBRG8" && this->depth == 3)
      {
        for (int i=0;i<this->width;i++)
        {
          for (int j=0;j<this->height;j++)
          {
            //
            // GB
            // RG
            //
            // determine position
            if (j%2) // even column
              if (i%2) // even row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
              else // odd row, blue
                dst[i+j*this->width] = src[i*3+j*this->width*3+2];
            else // odd column
              if (i%2) // even row, red
                dst[i+j*this->width] = src[i*3+j*this->width*3+0];
              else // odd row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
          }
        }
        src=dst;
      }
      else if (this->myParent->GetImageFormat() == "BAYER_GRBG8" && this->depth == 3)
      {
        for (int i=0;i<this->width;i++)
        {
          for (int j=0;j<this->height;j++)
          {
            //
            // GR
            // BG
            //
            // determine position
            if (j%2) // even column
              if (i%2) // even row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
              else // odd row, red
                dst[i+j*this->width] = src[i*3+j*this->width*3+0];
            else // odd column
              if (i%2) // even row, blue
                dst[i+j*this->width] = src[i*3+j*this->width*3+2];
              else // odd row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
          }
        }
        src=dst;
      }
}

////////////////////////////////////////////////////////////////////////////////
// Fill depth information
bool GazeboRosThermalCamera::fillDepthImage(pcl::PointCloud<pcl::PointXYZ>& point_cloud,
                             uint32_t rows_arg,
                             uint32_t cols_arg,
                             uint32_t step_arg,
                             void* data_arg)
{
  point_cloud.points.resize(0);
  point_cloud.is_dense = true;
  //point_cloud.channels.resize(1);
  //point_cloud.channels[0].values.resize(0);

  float* toCopyFrom = (float*)data_arg;
  int index = 0;

  double hfov = this->myParent->GetHFOV().GetAsRadian();
  double fl = ((double)this->width) / (2.0 *tan(hfov/2.0));
  //ROS_ERROR("debug hfov: %f fl: %f w: %f h: %f",hfov, fl, (double)cols_arg, (double)rows_arg);
  //ROS_ERROR("debug %f %f", this->myParent->GetFarClip() , this->myParent->GetNearClip());

  // convert depth to point cloud
  for (uint32_t j=0; j<rows_arg; j++)
  {
    double pAngle;
    if (rows_arg>1) pAngle = atan2( (double)j - 0.5*(double)(rows_arg-1), fl);
    else            pAngle = 0.0;

    for (uint32_t i=0; i<cols_arg; i++)
    {
      double yAngle;
      if (cols_arg>1) yAngle = atan2( (double)i - 0.5*(double)(cols_arg-1), fl);
      else            yAngle = 0.0;

      //double depth = (this->myParent->GetFarClip() - this->myParent->GetNearClip()) * ( toCopyFrom[index++] ) + 1.0 * this->myParent->GetNearClip() ;
      //double depth = ( this->myParent->GetFarClip() + this->myParent->GetNearClip() + toCopyFrom[index++] )
      //               * 0.5 ; //this->myParent->GetNearClip() ;
      double depth = toCopyFrom[index++]; // + 0.0*this->myParent->GetNearClip();

      // in optical frame
      // hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
      // to urdf, where the *_optical_frame should have above relative
      // rotation from the physical camera *_frame
      pcl::PointXYZ point;
      point.x      = depth * tan(yAngle);
      point.y      = depth * tan(pAngle);
      if(depth > this->pointCloudCutoff)
      {
        point.z    = depth;
      }
      else //point in the unseeable range
      {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
        point_cloud.is_dense = false;
      }
      point_cloud.points.push_back(point);
    }
  }
  //this->pointCloudMsg.channels[0].values.push_back(1.0);
  return true;
}
////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosThermalCamera::PublishCameraInfo()
{
  // fill CameraInfo
  this->cameraInfoMsg.header.frame_id = this->frameName;
  Time lastRenderTime = (dynamic_cast<OgreCamera*>(this->myParent))->GetLastRenderTime();
  //Time lastRenderTime = Simulator::Instance()->GetSimTime();
  this->cameraInfoMsg.header.stamp.sec = lastRenderTime.sec;
  this->cameraInfoMsg.header.stamp.nsec = lastRenderTime.nsec;
  this->cameraInfoMsg.height = this->height;
  this->cameraInfoMsg.width  = this->width;
  // distortion
#if ROS_VERSION_MINIMUM(1, 3, 0)
  this->cameraInfoMsg.distortion_model = "plumb_bob";
  this->cameraInfoMsg.D.resize(5);
#endif
  this->cameraInfoMsg.D[0] = this->distortion_k1;
  this->cameraInfoMsg.D[1] = this->distortion_k2;
  this->cameraInfoMsg.D[2] = this->distortion_k3;
  this->cameraInfoMsg.D[3] = this->distortion_t1;
  this->cameraInfoMsg.D[4] = this->distortion_t2;
  // original camera matrix
  this->cameraInfoMsg.K[0] = this->focal_length;
  this->cameraInfoMsg.K[1] = 0.0;
  this->cameraInfoMsg.K[2] = this->Cx;
  this->cameraInfoMsg.K[3] = 0.0;
  this->cameraInfoMsg.K[4] = this->focal_length;
  this->cameraInfoMsg.K[5] = this->Cy;
  this->cameraInfoMsg.K[6] = 0.0;
  this->cameraInfoMsg.K[7] = 0.0;
  this->cameraInfoMsg.K[8] = 1.0;
  // rectification
  this->cameraInfoMsg.R[0] = 1.0;
  this->cameraInfoMsg.R[1] = 0.0;
  this->cameraInfoMsg.R[2] = 0.0;
  this->cameraInfoMsg.R[3] = 0.0;
  this->cameraInfoMsg.R[4] = 1.0;
  this->cameraInfoMsg.R[5] = 0.0;
  this->cameraInfoMsg.R[6] = 0.0;
  this->cameraInfoMsg.R[7] = 0.0;
  this->cameraInfoMsg.R[8] = 1.0;
  // camera projection matrix (same as camera matrix due to lack of distortion/rectification) (is this generated?)
  this->cameraInfoMsg.P[0] = this->focal_length;
  this->cameraInfoMsg.P[1] = 0.0;
  this->cameraInfoMsg.P[2] = this->Cx;
  this->cameraInfoMsg.P[3] = -this->focal_length * this->hack_baseline;
  this->cameraInfoMsg.P[4] = 0.0;
  this->cameraInfoMsg.P[5] = this->focal_length;
  this->cameraInfoMsg.P[6] = this->Cy;
  this->cameraInfoMsg.P[7] = 0.0;
  this->cameraInfoMsg.P[8] = 0.0;
  this->cameraInfoMsg.P[9] = 0.0;
  this->cameraInfoMsg.P[10] = 1.0;
  this->cameraInfoMsg.P[11] = 0.0;

  last_camera_info_pub_time_ = Simulator::Instance()->GetSimTime();
  this->camera_info_pub_.publish(this->cameraInfoMsg);
}


#ifdef USE_CBQ
////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosThermalCamera::CameraQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->camera_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#endif

}
