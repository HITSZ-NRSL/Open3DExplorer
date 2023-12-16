#ifndef GAZEBO_CONTINUOUS_TRACK_SIMPLE
#define GAZEBO_CONTINUOUS_TRACK_SIMPLE

#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_continuous_track/gazebo_wrap.hpp>
#include <ros/package.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>

namespace gazebo {

class ContinuousTrackSimple : public ModelPlugin {
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    // assert the given sdf can be parsed as plugin property config
    const sdf::ElementPtr formatted_sdf(ToPluginSDF(_sdf));

    const std::string plugin_name(formatted_sdf->GetAttribute("name")->GetAsString());
    std::cout << "[" << plugin_name << "]:"
              << " Start loading plugin" << std::endl;

    GZ_ASSERT(wrap::Physics(_model->GetWorld())->GetType() == "ode",
              "ContinuousTrackSimple only supports ODE.");

    // load from [sprocket] element
    double sprocket_diameter;
    {
      const sdf::ElementPtr sprocket_elem(formatted_sdf->GetElement("sprocket"));
      // [joint]
      sprocket_joint_ = _model->GetJoint(sprocket_elem->GetElement("joint")->Get< std::string >());
      GZ_ASSERT(sprocket_joint_,
                "Cannot find a joint with the value of [sprocket]::[joint] element in sdf");
      GZ_ASSERT(sprocket_joint_->HasType(physics::Joint::HINGE_JOINT),
                "[sprocket]::[joint] must be a rotatinal joint");
      std::cout << "[" << plugin_name << "]:"
                << " Found the sprocket joint \"" << sprocket_joint_->GetScopedName() << "\""
                << std::endl;
      // [pitch_diameter]
      sprocket_diameter = sprocket_elem->GetElement("pitch_diameter")->Get< double >();
      GZ_ASSERT(sprocket_diameter > 0.,
                "[sprocket]::[pitch_diameter] must be a positive real number");
      std::cout << "[" << plugin_name << "]:"
                << " Set the pitch diameter of the sprocket to " << sprocket_diameter << std::endl;
    }

    // load from [track] element
    {
      const sdf::ElementPtr track_elem(formatted_sdf->GetElement("track"));
      // [segment] (multiple, +)
      for (sdf::ElementPtr segment_elem = track_elem->GetElement("segment"); segment_elem;
           segment_elem = segment_elem->GetNextElement("segment")) {
        // [joint]
        const physics::JointPtr segment_joint(
            _model->GetJoint(segment_elem->GetElement("joint")->Get< std::string >()));
        GZ_ASSERT(
            segment_joint,
            "Cannot find a joint with the value of [track]::[segment]::[joint] element in sdf");
        std::cout << "[" << plugin_name << "]:"
                  << " Found the joint \"" << segment_joint->GetScopedName()
                  << "\" for a track segment" << std::endl;
        // [pitch_diameter] (for rotational joint only)
        if (segment_joint->HasType(physics::Joint::HINGE_JOINT)) {
          GZ_ASSERT(
              segment_elem->HasElement("pitch_diameter"),
              "No [track]::[segment]::[pitch_diameter] element for a rotational segment in sdf");
          const double segment_diameter(
              segment_elem->GetElement("pitch_diameter")->Get< double >());
          GZ_ASSERT(segment_diameter > 0.,
                    "[track]::[segment]::[pitch_diameter] must be a positive real number");
          segment_updaters_.push_back(boost::bind(&ContinuousTrackSimple::UpdateRotationalSegment,
                                                  segment_joint, _1,
                                                  sprocket_diameter / segment_diameter));
          std::cout << "[" << plugin_name << "]:"
                    << " Set the pitch diameter for the track segment \""
                    << segment_joint->GetScopedName() << "\" to " << segment_diameter << std::endl;
        } else if (segment_joint->HasType(physics::Joint::SLIDER_JOINT)) {
          segment_updaters_.push_back(
              boost::bind(&ContinuousTrackSimple::UpdateTranslationalSegment, segment_joint, _1,
                          sprocket_diameter / 2));
        } else {
          GZ_ASSERT(false, "[track]::[segment]::[joint] must be a rotational or translational "
                           "joint having exactory 1 axis");
        }
      }
    }

    // enable callback on beggining of every world step
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ContinuousTrackSimple::Update, this, _1));

    // done!!
    std::cout << "[" << plugin_name << "]:"
              << " Loaded plugin" << std::endl;
  }

private:
  // **************
  // updating track
  // **************

  void Update(const common::UpdateInfo &_info) const {
    // get rotational velocity of sprocket
    const double sprocket_vel(sprocket_joint_->GetVelocity(0));

    // set velocities of track segments according to the sprocket velocity
    for (const boost::function< void(const double) > &segment_updater : segment_updaters_) {
      segment_updater(sprocket_vel);
    }
  }

  static void UpdateRotationalSegment(const physics::JointPtr &segment_joint,
                                      const double sprocket_vel, const double sprocket2segment) {
    // set the velocity of track segment according to the sprocket velocity
    SetJointMotorVelocity(segment_joint, 0, sprocket_vel * sprocket2segment);
  }

  static void UpdateTranslationalSegment(const physics::JointPtr &segment_joint,
                                         const double sprocket_vel, const double sprocket2segment) {
    // Note:
    //   When resetting the segment position, preserveWorldVelocity must be **true**.
    //   If false, the physics engine may zero the world velocity of the child link.
    //   This causes unrealistic behaviours (ex. no acceleration free fall).

    // reset segment position
    wrap::SetPosition(segment_joint, 0, 0.0, /* preserveWorldVelocity */ true);

    // set the velocity of track segment according to the sprocket velocity
    SetJointMotorVelocity(segment_joint, 0, sprocket_vel * sprocket2segment);
  }

  static void SetJointMotorVelocity(const physics::JointPtr &_joint, const unsigned int _index,
                                    const double _velocity) {
    // using ODE's joint motors function

    // set force/torque limit
    const double effort_limit(_joint->GetEffortLimit(_index));
    _joint->SetParam("fmax", _index,
                     // negative value means unlimited
                     effort_limit > 0. ? effort_limit : std::numeric_limits< double >::max());

    // set velocity clamped by velocity limit
    const double velocity_limit(_joint->GetVelocityLimit(_index));
    _joint->SetParam("vel", 0,
                     // negative value means unlimited
                     velocity_limit > 0.
                         ? ignition::math::clamp(_velocity, -velocity_limit, velocity_limit)
                         : _velocity);
  }

  // **************
  // formatting sdf
  // **************

  // get a sdf element which has been initialized by the plugin format file.
  // the initialied sdf may look empty but have a format information.
  static sdf::ElementPtr LoadPluginFormat() {
    const sdf::ElementPtr fmt(new sdf::Element());
    GZ_ASSERT(sdf::initFile(ros::package::getPath("gazebo_continuous_track") +
                                "/sdf/continuous_track_simple_plugin.sdf",
                            fmt),
              "Cannot initialize sdf by continuous_track_simple_plugin.sdf");
    return fmt;
  }

  // merge the plugin format sdf and the given sdf.
  // assert if the given sdf does not match the format
  // (ex. no required element, value type mismatch, ...).
  static sdf::ElementPtr ToPluginSDF(const sdf::ElementPtr &_src) {
    static const sdf::ElementPtr fmt(LoadPluginFormat());
    const sdf::ElementPtr dst(fmt->Clone());
    GZ_ASSERT(
        sdf::readString("<sdf version='" SDF_VERSION "'>" + _src->ToString("") + "</sdf>", dst),
        "The given sdf does not match ContinuousTrackSimple plugin format");
    return dst;
  }

private:
  // the sprocket joint
  physics::JointPtr sprocket_joint_;
  // functor updating each track segment
  std::vector< boost::function< void(const double) > > segment_updaters_;
  // callback connection handle
  event::ConnectionPtr update_connection_;
};

} // namespace gazebo

#endif