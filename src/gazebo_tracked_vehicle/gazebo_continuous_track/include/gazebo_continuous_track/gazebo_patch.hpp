#ifndef GAZEBO_PATCH
#define GAZEBO_PATCH

#include <string>

#include <gazebo/physics/ode/ODEJoint.hh>
#include <gazebo/physics/physics.hh>

//
// Functions compensating missing APIs or giving workaround for bugs
//

namespace gazebo {
namespace patch {

// Utility to append a static member variable for a non-templated class
// without defining the static member in .cpp file.
// The template param Derived is required
// to give different variable instance for each Derived class.
template < typename T, class Derived > struct StaticVar { static T value_; };
template < typename T, class Derived > T StaticVar< T, Derived >::value_;

// *****
// Model
// *****

// Magic to access private member functions. This technique is legal but of course not recommended.

// Pointer type of the private members we want to access
typedef physics::Model_V physics::Model::*ModelsPtrT;

// Actual implementation of CreateNestedModel().
// The member variables StaticVar<>::value_ is initialized to &Model::models
// by CreateNestedModelImplInitializer.
class CreateNestedModelImpl : private StaticVar< ModelsPtrT, CreateNestedModelImpl > {
  template < ModelsPtrT ModelsPtr > friend class CreateNestedModelImplInitializer;
  typedef StaticVar< ModelsPtrT, CreateNestedModelImpl > ModelsPtrVar;

public:
  static physics::ModelPtr Call(const physics::ModelPtr &_model, const std::string &_name) {
#if GAZEBO_MAJOR_VERSION >= 8
    const physics::ModelPtr nested_model(_model->GetWorld()->Physics()->CreateModel(_model));
#else
    const physics::ModelPtr nested_model(
        _model->GetWorld()->GetPhysicsEngine()->CreateModel(_model));
#endif
    nested_model->SetName(_name);
    nested_model->SetWorld(_model->GetWorld());
    ((*_model).*ModelsPtrVar::value_).push_back(nested_model);
    return nested_model;
  }
};

// The constructor initializes CreateNestedModelImpl::StaticVar<>::value_
// according to the template variables.
template < ModelsPtrT ModelsPtr > class CreateNestedModelImplInitializer {
public:
  CreateNestedModelImplInitializer() { CreateNestedModelImpl::ModelsPtrVar::value_ = ModelsPtr; }

private:
  static CreateNestedModelImplInitializer instance_;
};
template < ModelsPtrT ModelsPtr >
CreateNestedModelImplInitializer< ModelsPtr >
    CreateNestedModelImplInitializer< ModelsPtr >::instance_;

// Instantiate Initializer with &Model::models
// This calls the constructor of Initializer,
// and it initializes CreateNestedModelImpl::StaticVar<>::value_ to the private member pointers.
template class CreateNestedModelImplInitializer< &physics::Model::models >;

// physics::Model::CreateLink(), CreateJoint() exist, but no CreateNestedModel() ....
static inline physics::ModelPtr CreateNestedModel(const physics::ModelPtr &_model,
                                                  const std::string &_name) {
  return CreateNestedModelImpl::Call(_model, _name);
}

//
typedef void (physics::Model::*RemoveLinkPtrT)(const std::string &);

class RemoveLinkImpl : private StaticVar< RemoveLinkPtrT, RemoveLinkImpl > {
  template < RemoveLinkPtrT RemoveLinkPtr > friend class RemoveLinkImplInitializer;
  typedef StaticVar< RemoveLinkPtrT, RemoveLinkImpl > RemoveLinkPtrVar;

public:
  static void Call(const physics::ModelPtr &_model, const physics::LinkPtr &_link) {
    // remove joints connected the link from the model
    for (const physics::JointPtr &joint : _link->GetParentJoints()) {
      if (_model->GetJoint(joint->GetScopedName())) {
        _model->RemoveJoint(joint->GetScopedName());
      }
    }
    for (const physics::JointPtr &joint : _link->GetChildJoints()) {
      if (_model->GetJoint(joint->GetScopedName())) {
        _model->RemoveJoint(joint->GetScopedName());
      }
    }

    // remove the link from the link cache in the model
    ((*_model).*RemoveLinkPtrVar::value_)(_link->GetScopedName());

    // remove sdf update functions related to this link instance
    // or cause a runtime error on the next call of the parent model sdf's Update()
    UnsetUpdateFuncs(_link->GetSDF());

    // remove the link as a child entity from the model
    _model->RemoveChild(_link->GetId());
  }

private:
  static void UnsetUpdateFuncs(const sdf::ElementPtr &_sdf) {
    // unset update functions on this element
    for (std::size_t attr_id = 0; attr_id < _sdf->GetAttributeCount(); ++attr_id) {
      _sdf->GetAttribute(attr_id)->SetUpdateFunc(nullptr);
    }
    const sdf::ParamPtr value(_sdf->GetValue());
    if (value) {
      value->SetUpdateFunc(nullptr);
    }

    // recursively unset update functions on child elements
    for (sdf::ElementPtr elem = _sdf->GetFirstElement(); elem; elem = elem->GetNextElement()) {
      UnsetUpdateFuncs(elem);
    }
    for (std::size_t desc_id = 0; desc_id < _sdf->GetElementDescriptionCount(); ++desc_id) {
      UnsetUpdateFuncs(_sdf->GetElementDescription(desc_id));
    }
  }
};

template < RemoveLinkPtrT RemoveLinkPtr > class RemoveLinkImplInitializer {
public:
  RemoveLinkImplInitializer() { RemoveLinkImpl::RemoveLinkPtrVar::value_ = RemoveLinkPtr; }

private:
  static RemoveLinkImplInitializer instance_;
};
template < RemoveLinkPtrT RemoveLinkPtr >
RemoveLinkImplInitializer< RemoveLinkPtr > RemoveLinkImplInitializer< RemoveLinkPtr >::instance_;

template class RemoveLinkImplInitializer< &physics::Model::RemoveLink >;

// alternative of physics::Model::RemoveChild().
// this alternative is required because of a critical bug in the original RemoveChild();
// when a link to be removed is given,
// the original tries to remove the link and also joints connected to the link.
// however, it can wrongly remove joints which are not connected to the link
// because it does not care about namescope of links.
static inline void RemoveLink(const physics::ModelPtr &_model, const physics::LinkPtr &_link) {
  RemoveLinkImpl::Call(_model, _link);
}

// *****
// Joint
// *****

#if GAZEBO_MAJOR_VERSION >= 8
typedef ignition::math::Pose3d (physics::Joint::*ChildLinkPosePtrT)(unsigned int, double);
#else
typedef math::Pose (physics::Joint::*ChildLinkPosePtrT)(unsigned int, double);
#endif

class ChildLinkPoseImpl : private StaticVar< ChildLinkPosePtrT, ChildLinkPoseImpl > {
  template < ChildLinkPosePtrT ChildLinkPosePtr > friend class ChildLinkPoseImplInitializer;
  typedef StaticVar< ChildLinkPosePtrT, ChildLinkPoseImpl > ChildLinkPosePtrVar;

public:
  static ignition::math::Pose3d Call(const physics::JointPtr &_joint, const unsigned int _index,
                                     const double _position) {
#if GAZEBO_MAJOR_VERSION >= 8
    return ((*_joint).*ChildLinkPosePtrVar::value_)(_index, _position);
#else
    return ((*_joint).*ChildLinkPosePtrVar::value_)(_index, _position).Ign();
#endif
  }
};

template < ChildLinkPosePtrT ChildLinkPosePtr > class ChildLinkPoseImplInitializer {
public:
  ChildLinkPoseImplInitializer() {
    ChildLinkPoseImpl::ChildLinkPosePtrVar::value_ = ChildLinkPosePtr;
  }

private:
  static ChildLinkPoseImplInitializer instance_;
};
template < ChildLinkPosePtrT ChildLinkPosePtr >
ChildLinkPoseImplInitializer< ChildLinkPosePtr >
    ChildLinkPoseImplInitializer< ChildLinkPosePtr >::instance_;

#if GAZEBO_MAJOR_VERSION >= 8
template class ChildLinkPoseImplInitializer< &physics::Joint::ChildLinkPose >;
#else
template class ChildLinkPoseImplInitializer< &physics::Joint::ComputeChildLinkPose >;
#endif

// physics::Joint::ChildLinkPose() are safe to be public
// as it does not change member variables (so could be a const member function)
// but it is actually a non-const protected function ...
static inline ignition::math::Pose3d
ChildLinkPose(const physics::JointPtr &_joint, const unsigned int _index, const double _position) {
  return ChildLinkPoseImpl::Call(_joint, _index, _position);
}

} // namespace patch
} // namespace gazebo

#endif