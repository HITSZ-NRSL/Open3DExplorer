#ifndef GAZEBO_WRAP
#define GAZEBO_WRAP

#include <map>
#include <string>

#include <gazebo/physics/physics.hh>

#include <gazebo_continuous_track/gazebo_patch.hpp>

//
// Functions wrapping API difference between Gazebo versions
//

namespace gazebo {
namespace wrap {

#if GAZEBO_MAJOR_VERSION < 7 || (GAZEBO_MAJOR_VERSION == 7 && GAZEBO_MINOR_VERSION < 10)
#error "gazebo_continuous_track supports Gazebo 7.10 or later"
#endif

#if GAZEBO_MAJOR_VERSION < 8
template < typename T, class Derived > struct StaticVar { static T value_; };
template < typename T, class Derived > T StaticVar< T, Derived >::value_;
#endif

// *****
// World
// *****

static inline std::string Name(const physics::WorldPtr &_world) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _world->Name();
#else
  return _world->GetName();
#endif
}

static inline physics::PhysicsEnginePtr Physics(const physics::WorldPtr &_world) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _world->Physics();
#else
  return _world->GetPhysicsEngine();
#endif
}

// ****
// Link
// ****

static inline ignition::math::Pose3d WorldPose(const physics::LinkPtr &_link) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _link->WorldPose();
#else
  return _link->GetWorldPose().Ign();
#endif
}

// *****
// Joint
// *****

static inline ignition::math::Vector3d GlobalAxis(const physics::JointPtr &_joint,
                                                  const unsigned int _index) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _joint->GlobalAxis(_index);
#else
  return _joint->GetGlobalAxis(_index).Ign();
#endif
}

static inline double Position(const physics::JointPtr &_joint, const unsigned int _index) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _joint->Position(_index);
#else
  return *(_joint->GetAngle(_index));
#endif
}

#if GAZEBO_MAJOR_VERSION < 8
// Pointer types of private members we want to access
typedef bool (physics::Joint ::*FindAllConnectedLinksPtrT)(const physics::LinkPtr &,
                                                           physics::Link_V &);

// Actual implementation of SetPosition().
// The member variables StaticVar<>::value_ is initialized
// to &Joint::ComputeChildLinkPose and &Joint::FindAllConnectedLinks by SetPositionImplInitializer.
class SetPositionImpl : private StaticVar< FindAllConnectedLinksPtrT, SetPositionImpl > {
  template < FindAllConnectedLinksPtrT FindAllConnectedLinksPtr >
  friend class SetPositionImplInitializer;
  typedef StaticVar< FindAllConnectedLinksPtrT, SetPositionImpl > FindAllConnectedLinksPtrVar;

public:
  static bool Call(const physics::JointPtr &_joint, const unsigned int _index, double _position,
                   const bool _preserveWorldVelocity) {
    {
      // limit desired position
      const double lower_limit(*(_joint->GetLowerLimit(_index)));
      const double upper_limit(*(_joint->GetUpperLimit(_index)));
      _position = lower_limit < upper_limit
                      ? ignition::math::clamp(_position, lower_limit, upper_limit)
                      : ignition::math::clamp(_position, upper_limit, lower_limit);
    }

    if (_preserveWorldVelocity) {
      // child link's current pose & new pose based on position change
      const math::Pose child_pose(_joint->GetChild()->GetWorldPose());
      const math::Pose new_child_pose(patch::ChildLinkPose(_joint, _index, _position));

      // populate child links recursively
      physics::Link_V links;
      ((*_joint).*FindAllConnectedLinksPtrVar::value_)(_joint->GetParent(), links);

      // update pose of each child link on the basis of joint position change
      for (const physics::LinkPtr &link : links) {
        // NEVER EVER call Link::MoveFrame(child_pose, new_child_pose)
        // because there is a critical bug which zeros link world velocity
        link->SetWorldPose((link->GetWorldPose() - child_pose) + new_child_pose);
      }
      return true;
    } else {
      return _joint->SetPosition(_index, _position);
    }
  }
};

// The constructor initializes SetPositionImpl::StaticVar<>::value_
// according to the template variables.
template < FindAllConnectedLinksPtrT FindAllConnectedLinksPtr > class SetPositionImplInitializer {
public:
  SetPositionImplInitializer() {
    SetPositionImpl::FindAllConnectedLinksPtrVar::value_ = FindAllConnectedLinksPtr;
  }

private:
  static SetPositionImplInitializer instance_;
};
template < FindAllConnectedLinksPtrT FindAllConnectedLinksPtr >
SetPositionImplInitializer< FindAllConnectedLinksPtr >
    SetPositionImplInitializer< FindAllConnectedLinksPtr >::instance_;

// Instantiate Initializer with &Joint::ComputeChildLinkPose and &Joint::FindAllConnectedLinks.
// This calls the constructor of Initializer,
// and it initializes SetPositionImpl::StaticVar<>::value_ to the private member pointers.
template class SetPositionImplInitializer< &physics::Joint::FindAllConnectedLinks >;
#endif

static inline bool SetPosition(const physics::JointPtr &_joint, const unsigned int _index,
                               const double _position, const bool _preserveWorldVelocity = false) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _joint->SetPosition(_index, _position, _preserveWorldVelocity);
#else
  return SetPositionImpl::Call(_joint, _index, _position, _preserveWorldVelocity);
#endif
}

static inline ignition::math::Pose3d WorldPose(const physics::JointPtr &_joint) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _joint->WorldPose();
#else
  return _joint->GetWorldPose().Ign();
#endif
}

} // namespace wrap
} // namespace gazebo

#endif