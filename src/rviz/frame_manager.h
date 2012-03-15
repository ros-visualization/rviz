/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_FRAME_MANAGER_H
#define RVIZ_FRAME_MANAGER_H

#include <map>

#include <ros/time.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include <boost/thread/mutex.hpp>

#include <geometry_msgs/Pose.h>

#include <tf/message_filter.h>

/*  Macro to issue warning when using deprecated functions with gcc4 or MSVC7: */
#if defined(__GNUC__) && ( __GNUC__ >= 4 )
    #define DEPRECATED(x) __attribute__((deprecated)) x
#elif defined(__VISUALC__) && (__VISUALC__ >= 1300)
    #define DEPRECATED(x) __declspec(deprecated) x
#else
    #define DEPRECATED(x) x
#endif

namespace tf
{
class TransformListener;
}

namespace rviz
{
class Display;

class FrameManager;
typedef boost::shared_ptr<FrameManager> FrameManagerPtr;
typedef boost::weak_ptr<FrameManager> FrameManagerWPtr;

/** @brief Helper class for transforming data into Ogre's world frame (the fixed frame).
 *
 * During one frame update (nominally 33ms), the tf tree stays consistent and queries are cached for speedup.
 */
class FrameManager
{
public:
  /** @brief Return a shared pointer to a single instance of FrameManager.
   *
   * FrameManager is a singleton which only exists as long as shared
   * pointers from instance() are held, because only a weak pointer is
   * kept internally. */
  static FrameManagerPtr instance();

  ~FrameManager();

  /** @brief Set the frame to consider "fixed", into which incoming data is transformed.
   *
   * The fixed frame serves as the reference for all getTransform()
   * and transform() functions in FrameManager. */
   void setFixedFrame(const std::string& frame);

  // @deprecated "relative" flag is no longer needed.
  template<typename Header>
  __attribute__((deprecated))
  bool getTransform(const Header& header, Ogre::Vector3& position, Ogre::Quaternion& orientation, bool relative)
  {
    return getTransform(header, position, orientation);
  }

  /** @brief Return the pose for a header, relative to the fixed frame, in Ogre classes.
   * @param[in] header The source of the frame name and time.
   * @param[out] position The position of the header frame relative to the fixed frame.
   * @param[out] orientation The orientation of the header frame relative to the fixed frame.
   * @return true on success, false on failure. */
  template<typename Header>
  bool getTransform(const Header& header, Ogre::Vector3& position, Ogre::Quaternion& orientation)
  {
    return getTransform(header.frame_id, header.stamp, position, orientation);
  }

  /** @brief Return the pose for a frame relative to the fixed frame, in Ogre classes, at a given time.
   * @param[in] frame The frame to find the pose of.
   * @param[in] time The time at which to get the pose.
   * @param[out] position The position of the frame relative to the fixed frame.
   * @param[out] orientation The orientation of the frame relative to the fixed frame.
   * @return true on success, false on failure. */
  bool getTransform(const std::string& frame, ros::Time time, Ogre::Vector3& position, Ogre::Quaternion& orientation);

  /** @brief Transform a pose from a frame into the fixed frame.
   * @param[in] header The source of the input frame and time.
   * @param[in] pose The input pose, relative to the header frame.
   * @param[out] position Position part of pose relative to the fixed frame.
   * @param[out] orientation: Orientation part of pose relative to the fixed frame.
   * @return true on success, false on failure. */
  template<typename Header>
  bool transform(const Header& header, const geometry_msgs::Pose& pose, Ogre::Vector3& position, Ogre::Quaternion& orientation)
  {
    return transform(header.frame_id, header.stamp, pose, position, orientation);
  }

  // @deprecated "relative" flag is no longer needed.
  __attribute__((deprecated))
  bool transform(const std::string& frame, ros::Time time, const geometry_msgs::Pose& pose, Ogre::Vector3& position, Ogre::Quaternion& orientation, bool relative)
  {
    return transform(frame, time, pose, position, orientation);
  }

  /** @brief Transform a pose from a frame into the fixed frame.
   * @param[in] frame The input frame.
   * @param[in] time The time at which to get the pose.
   * @param[in] pose The input pose, relative to the input frame.
   * @param[out] position Position part of pose relative to the fixed frame.
   * @param[out] orientation: Orientation part of pose relative to the fixed frame.
   * @return true on success, false on failure. */
  bool transform(const std::string& frame, ros::Time time, const geometry_msgs::Pose& pose, Ogre::Vector3& position, Ogre::Quaternion& orientation);

  /** @brief Clear the internal cache. */
  void update();

  // find out what went wrong during a transformation
  bool frameHasProblems(const std::string& frame, ros::Time time, std::string& error);
  bool transformHasProblems(const std::string& frame, ros::Time time, std::string& error);

  template<class M>
  void registerFilterForTransformStatusCheck(tf::MessageFilter<M>* filter, Display* display)
  {
    filter->registerCallback(boost::bind(&FrameManager::messageCallback<M>, this, _1, display));
    filter->registerFailureCallback(boost::bind(&FrameManager::failureCallback<M>, this, _1, _2, display));
  }

  const std::string& getFixedFrame() { return fixed_frame_; }
  tf::TransformListener* getTFClient() { return tf_; }

  std::string discoverFailureReason(const std::string& frame_id, const ros::Time& stamp, const std::string& caller_id, tf::FilterFailureReason reason);

private:
  FrameManager();

  template<class M>
  void messageCallback(const boost::shared_ptr<M const>& msg, Display* display)
  {
    messageArrived(msg->header.frame_id, msg->header.stamp, msg->__connection_header ? (*msg->__connection_header)["callerid"] : "unknown", display);
  }

  template<class M>
  void failureCallback(const boost::shared_ptr<M const>& msg, tf::FilterFailureReason reason, Display* display)
  {
    messageFailed(msg->header.frame_id, msg->header.stamp, msg->__connection_header ? (*msg->__connection_header)["callerid"] : "unknown", reason, display);
  }

  void messageArrived(const std::string& frame_id, const ros::Time& stamp, const std::string& caller_id, Display* display);
  void messageFailed(const std::string& frame_id, const ros::Time& stamp, const std::string& caller_id, tf::FilterFailureReason reason, Display* display);

  struct CacheKey
  {
    CacheKey(const std::string& f, ros::Time t)
    : frame(f)
    , time(t)
    {}

    bool operator<(const CacheKey& rhs) const
    {
      if (frame != rhs.frame)
      {
        return frame < rhs.frame;
      }

      return time < rhs.time;
    }

    std::string frame;
    ros::Time time;
  };

  struct CacheEntry
  {
    CacheEntry(const Ogre::Vector3& p, const Ogre::Quaternion& o)
    : position(p)
    , orientation(o)
    {}

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
  };
  typedef std::map<CacheKey, CacheEntry > M_Cache;

  boost::mutex cache_mutex_;
  M_Cache cache_;

  tf::TransformListener* tf_;
  std::string fixed_frame_;
};

}

#endif // RVIZ_FRAME_MANAGER_H
