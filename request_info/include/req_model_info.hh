#ifndef GAZEBO_MODEL_MOVE_HH_
#define GAZEBO_MODEL_MOVE_HH_

#include <gazebo/msgs/pose_animation.pb.h>
#include <vector>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <request_info/GetModelInfo.h>

namespace gazebo
{
  /// \brief A plugin to transport a model from point to point using
  /// pose animation.
  class ShowInfo : public ModelPlugin
  {
    ///  Constructor
    public: 
      ShowInfo();

    ///  Plugin Load function
    ///  _parent Model pointer to the model defining this plugin
    ///  _sdf pointer to the SDF of the model
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// ROS service function
    bool GetModelInfo(request_info::GetModelInfo::Request &req,
                      request_info::GetModelInfo::Response &res);

    ///  Pointer to the model that defines this plugin
    private: physics::ModelPtr model;
    private: physics::EntityPtr entity;
    ros::ServiceServer service_;
    ros::NodeHandle nh_;
  };
};

#endif