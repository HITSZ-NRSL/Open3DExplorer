#ifndef GAZEBO_CONTINUOUS_TRACK
#define GAZEBO_CONTINUOUS_TRACK

#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/ode/ode.h>
#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo_continuous_track/gazebo_continuous_track_properties.hpp>
#include <gazebo_continuous_track/gazebo_patch.hpp>
#include <gazebo_continuous_track/gazebo_wrap.hpp>
#include <ros/package.h>

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/pointer_cast.hpp>

namespace gazebo {

class ContinuousTrack : public ModelPlugin {

private:
  typedef ContinuousTrackProperties Properties;

public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    GZ_ASSERT(wrap::Physics(_model->GetWorld())->GetType() == "ode",
              "ContinuousTrack only supports ODE.");

    // load properties from sdf
    const Properties prop(_model, _sdf);

    std::cout << "[" << prop.name << "]:"
              << " Start loading plugin" << std::endl;

    // compose the track according to the propaties
    track_.name = prop.name;
    ComposeSprocket(prop.sprocket);
    ComposeBelt(_model, prop.trajectory, prop.pattern);
    InitTrack(prop);

    // advertise the visual topic to toggle track visuals
    InitVisualPublisher(wrap::Name(_model->GetWorld()));

    // enable callback on beggining of every world step to keep updating the track
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ContinuousTrack::UpdateTrack, this, _1));

    std::cout << "[" << prop.name << "]:"
              << " Loaded plugin" << std::endl;
  }

private:
  // ******************
  // Building the track
  // ******************

  struct Track {
    std::string name;
    //
    struct Sprocket {
      physics::JointPtr joint;
      // scale from joint position to length along track
      // (equals to the pitch radius of sprocket)
      double joint_to_track;
    };
    Sprocket sprocket;
    //
    struct Belt {
      struct Segment {
        struct Variant {
          physics::JointPtr joint;
          physics::ODELinkPtr link;
        };
        std::vector< Variant > variants;
        // scale from joint position to length along track
        // (1.0 if joint is translational, rotational_radius if rotational)
        double joint_to_track;
        // length of segment
        double length;
      };
      std::vector< Segment > segments;
      // circumferential length of track
      double perimeter;
      std::size_t elements_per_round;
      // index of variant to be enabled
      std::size_t variant_id;
    };
    Belt belt;
  };

  void ComposeSprocket(const Properties::Sprocket &_prop) {
    track_.sprocket.joint = _prop.joint;
    track_.sprocket.joint_to_track = _prop.pitch_diameter / 2.;
  }

  void ComposeBelt(const physics::ModelPtr &_model, const Properties::Trajectory &_traj_prop,
                   const Properties::Pattern &_pattern_prop) {
    ComposeSegments(_model, _traj_prop, _pattern_prop);
    track_.belt.elements_per_round = _pattern_prop.elements_per_round;
    track_.belt.variant_id = 0; // initialized by InitTrack() later
  }

  void ComposeSegments(const physics::ModelPtr &_model, const Properties::Trajectory &_traj_prop,
                       const Properties::Pattern &_pattern_prop) {
    namespace im = ignition::math;

    // fill fields related to length (_segment.joint_to_track, _segment.length, and _perimeter).
    // this resizes segments.
    FillSegmentLength(_traj_prop);

    // populate base sdfs which segment links/joints will inherit
    const sdf::ElementPtr base_model_sdf(CreateBaseVariantModelSDF(_model->GetSDF()));
    const std::vector< sdf::ElementPtr > base_link_sdfs(PopulateBaseSegmentLinkSDFs(_traj_prop));
    const std::vector< sdf::ElementPtr > base_joint_sdfs(PopulateBaseSegmentJointSDFs(_traj_prop));

    // compose segments for each variant
    for (std::size_t variant_id = 0; variant_id < _pattern_prop.elements.size(); ++variant_id) {

      // copies of base sdfs which will be modified for this variant
      std::vector< sdf::ElementPtr > link_sdfs;
      for (const sdf::ElementPtr &base_link_sdf : base_link_sdfs) {
        link_sdfs.push_back(base_link_sdf->Clone());
      }

      // separation between adjacent elements
      const double len_step(track_.belt.perimeter / _pattern_prop.elements_per_round);
      // length left on the current segment to place elements
      double len_left(0.);
      // length traveled on the current segment
      double len_traveled(0.);
      // index of the element to be placed next
      // (variant[0] places element[n-1] first, ... variant[n-1] does element[0])
      std::size_t elem_id(_pattern_prop.elements.size() - 1 - variant_id);

      // distribute elements along the track
      std::size_t elem_count(0);
      for (std::size_t segm_id = 0; segm_id < _traj_prop.segments.size(); ++segm_id) {
        const Properties::Trajectory::Segment &segment_prop(_traj_prop.segments[segm_id]);
        const Track::Belt::Segment &segment(track_.belt.segments[segm_id]);
        const sdf::ElementPtr &link_sdf(link_sdfs[segm_id]);

        // update length left for the current segment
        len_left += segment.length;

        // base pose of pattern elements on the current segment
        im::Pose3d base_pose(
            ComputeChildPoseOffset(segment_prop.joint, 0, len_traveled / segment.joint_to_track));
        const im::Pose3d base_pose_step(
            ComputeChildPoseOffset(segment_prop.joint, 0, len_step / segment.joint_to_track));

        // place elements onto link sdf as long as length remains
        while (len_left >= 0. && elem_count < _pattern_prop.elements_per_round) {
          const Properties::Pattern::Element &element_prop(_pattern_prop.elements[elem_id]);

          // add <collision> of the element to link sdf
          for (std::size_t collision_id = 0; collision_id < element_prop.collision_sdfs.size();
               ++collision_id) {
            // add new <collision> on the link sdf and copy base values
            const sdf::ElementPtr collision_elem(link_sdf->AddElement("collision"));
            collision_elem->Copy(element_prop.collision_sdfs[collision_id]);
            // give <collision> a unique name
            collision_elem->GetAttribute("name")->Set(
                "element" + boost::lexical_cast< std::string >(elem_count) + "_collision" +
                boost::lexical_cast< std::string >(collision_id));
            // set <collision>/<pose>
            const sdf::ElementPtr pose_elem(collision_elem->GetElement("pose"));
            const im::Pose3d pose_offset(pose_elem->Get< im::Pose3d >());
            pose_elem->Set(pose_offset + base_pose);
          }

          // add <visual> of the element to link sdf
          for (std::size_t visual_id = 0; visual_id < element_prop.visual_sdfs.size();
               ++visual_id) {
            // add new <collision> on the link sdf and copy base values
            const sdf::ElementPtr visual_elem(link_sdf->AddElement("visual"));
            visual_elem->Copy(element_prop.visual_sdfs[visual_id]);
            // give <visual> a unique name
            visual_elem->GetAttribute("name")->Set(
                "element" + boost::lexical_cast< std::string >(elem_count) + "_visual" +
                boost::lexical_cast< std::string >(visual_id));
            // set <visual>/<pose>
            const sdf::ElementPtr pose_elem(visual_elem->GetElement("pose"));
            const im::Pose3d pose_offset(pose_elem->Get< im::Pose3d >());
            pose_elem->Set(pose_offset + base_pose);
          }

          // step everything
          len_left -= len_step;
          len_traveled += len_step;
          elem_id = (elem_id + 1) % _pattern_prop.elements.size();
          ++elem_count;
          base_pose = base_pose_step + base_pose;
        }

        // update length traveled for the next segment
        len_traveled -= segment.length;
      }

      // model for variant links/joints
      const sdf::ElementPtr model_sdf(base_model_sdf->Clone());
      model_sdf->GetAttribute("name")->Set(track_.name + "_variant" +
                                           boost::lexical_cast< std::string >(variant_id));
      const physics::ModelPtr model(
          patch::CreateNestedModel(_model, model_sdf->GetAttribute("name")->GetAsString()));
      model->Load(model_sdf);
      model->Init();

      // create link/joint for each segment on the basis of updated sdfs
      for (std::size_t segm_id = 0; segm_id < _traj_prop.segments.size(); ++segm_id) {
        Track::Belt::Segment &segment(track_.belt.segments[segm_id]);
        const Properties::Trajectory::Segment &segment_prop(_traj_prop.segments[segm_id]);

        Track::Belt::Segment::Variant variant;

        // link
        // Physics()->CreateLink() does not register a new link to the model
        // and does not show up the link correctly on gzclient (gazebo7&9)
        const sdf::ElementPtr &link_sdf(link_sdfs[segm_id]);
        variant.link = boost::dynamic_pointer_cast< physics::ODELink >(
            model->CreateLink(link_sdf->GetAttribute("name")->GetAsString()));
        variant.link->Load(link_sdf);
        variant.link->Init();
        // copy base link pose because it may be changed by another plugin loaded before this
        variant.link->SetWorldPose(wrap::WorldPose(segment_prop.joint->GetChild()));

        // joint
        const sdf::ElementPtr joint_sdf(base_joint_sdfs[segm_id]->Clone());
        joint_sdf->GetElement("child")->Set(variant.link->GetScopedName());
        // Physics()->CreateJoint() does not register a new joint to the model
        // and does not show up the joint correctly on gzclient (gazebo7&9)
        variant.joint = model->CreateJoint(joint_sdf->GetAttribute("name")->GetAsString(),
                                           joint_sdf->GetAttribute("type")->GetAsString(),
                                           segment_prop.joint->GetParent(), variant.link);
        variant.joint->Load(joint_sdf);
        variant.joint->Init();
        // set initial zero velocity
        SetJointMotorVelocity(variant.joint, 0, 0.);

        segment.variants.push_back(variant);
        std::cout << "[" << track_.name << "]:"
                  << " Created " << variant.link->GetScopedName() << " and "
                  << variant.joint->GetScopedName() << std::endl;
      }
    }
  }

  void FillSegmentLength(const Properties::Trajectory &_traj_prop) {
    namespace im = ignition::math;

    track_.belt.perimeter = 0.;

    for (const Properties::Trajectory::Segment &segment_prop : _traj_prop.segments) {
      Track::Belt::Segment segment;

      if (segment_prop.joint->HasType(physics::Joint::HINGE_JOINT)) {
        // calc radius of rotation (= distance between link position and joint axis)
        const im::Vector3d joint_pos(wrap::WorldPose(segment_prop.joint).Pos());
        const im::Vector3d joint_axis(wrap::GlobalAxis(segment_prop.joint, 0));
        im::Vector3d link_pos(wrap::WorldPose(segment_prop.joint->GetChild()).Pos());
        // Vector3d::DistToLine() is not a const function for some reason ...
        const double radius(link_pos.DistToLine(joint_pos, joint_pos + joint_axis));
        segment.joint_to_track = radius;
        // length of segment is radius * rotation-angle
        segment.length = radius * segment_prop.end_position;
      } else if (segment_prop.joint->HasType(physics::Joint::SLIDER_JOINT)) {
        // length of segment equals to end-position of joint
        segment.joint_to_track = 1.0;
        segment.length = segment_prop.end_position;
      } else {
        // never fall here as joint type has been checked in the property loader
        GZ_ASSERT(false, "Bug. Unexpected joint type.");
      }

      track_.belt.segments.push_back(segment);

      track_.belt.perimeter += segment.length;
    }
  }

  static sdf::ElementPtr CreateBaseVariantModelSDF(const sdf::ElementPtr &_src) {
    sdf::ElementPtr dst(_src->Clone());
    if (dst->HasElement("pose")) {
      dst->RemoveChild(dst->GetElement("pose"));
    }
    while (dst->HasElement("include")) {
      dst->RemoveChild(dst->GetElement("include"));
    }
    while (dst->HasElement("model")) {
      dst->RemoveChild(dst->GetElement("model"));
    }
    while (dst->HasElement("link")) {
      dst->RemoveChild(dst->GetElement("link"));
    }
    while (dst->HasElement("joint")) {
      dst->RemoveChild(dst->GetElement("joint"));
    }
    while (dst->HasElement("plugin")) {
      dst->RemoveChild(dst->GetElement("plugin"));
    }
    while (dst->HasElement("gripper")) {
      dst->RemoveChild(dst->GetElement("gripper"));
    }
    return dst;
  }

  static std::vector< sdf::ElementPtr >
  PopulateBaseSegmentLinkSDFs(const Properties::Trajectory &_traj_prop) {
    std::vector< sdf::ElementPtr > sdfs;
    for (const Properties::Trajectory::Segment &segment_prop : _traj_prop.segments) {
      sdfs.push_back(segment_prop.joint->GetChild()->GetSDF()->Clone());
    }
    return sdfs;
  }

  static std::vector< sdf::ElementPtr >
  PopulateBaseSegmentJointSDFs(const Properties::Trajectory &_traj_prop) {
    std::vector< sdf::ElementPtr > sdfs;
    for (const Properties::Trajectory::Segment &segment_prop : _traj_prop.segments) {
      sdfs.push_back(segment_prop.joint->GetSDF()->Clone());
    }
    return sdfs;
  }

  static ignition::math::Pose3d ComputeChildPoseOffset(const physics::JointPtr &_joint,
                                                       const double _from, const double _to) {
    return
        // child position when the joint position is <to>
        patch::ChildLinkPose(_joint, 0, _to)
        // when <from>
        - patch::ChildLinkPose(_joint, 0, _from);
  }

  // **********************
  // Initializing the track
  // **********************

  void InitTrack(const Properties &_prop) {
    // clean up seed links/joints of variants no longer required
    for (const Properties::Trajectory::Segment &segment_prop : _prop.trajectory.segments) {
      // get link before removing joint
      // because joint->GetChild() does not work once joint has been removed (= detached)
      const physics::LinkPtr link(segment_prop.joint->GetChild());
      // remove segment link (this also remove joints connected to the link)
      patch::RemoveLink(link->GetModel(), link);

      std::cout << "[" << track_.name << "]:"
                << " Removed " << segment_prop.joint->GetScopedName() << " and "
                << link->GetScopedName() << std::endl;
    }

    // init collide mode of body links wrapped by the track
    {
      // populate ODE space IDs in links wrapped by the track
      std::set< dSpaceID > space_ids;
      space_ids.insert(
          boost::dynamic_pointer_cast< physics::ODELink >(track_.sprocket.joint->GetParent())
              ->GetSpaceId());
      space_ids.insert(
          boost::dynamic_pointer_cast< physics::ODELink >(track_.sprocket.joint->GetChild())
              ->GetSpaceId());
      for (const Track::Belt::Segment &segment : track_.belt.segments) {
        const physics::ODELinkPtr parent(boost::dynamic_pointer_cast< physics::ODELink >(
            segment.variants[0].joint->GetParent()));
        space_ids.insert(parent->GetSpaceId());
      }
      // set collide mode such that collide external environments and does not collide the track
      for (const dSpaceID &space_id : space_ids) {
        dGeomSetCategoryBits((dGeomID)space_id, GZ_GHOST_COLLIDE);
        dGeomSetCollideBits((dGeomID)space_id, GZ_ALL_COLLIDE);
      }
    }

    // initialize variant id
    track_.belt.variant_id =
        CalcVariantId(wrap::Position(track_.sprocket.joint, 0) * track_.sprocket.joint_to_track);

    // init collide mode of the track
    for (std::size_t variant_id = 0; variant_id < track_.belt.segments[0].variants.size();
         ++variant_id) {
      const physics::ODELinkPtr &link(track_.belt.segments[0].variants[variant_id].link);
      if (variant_id == track_.belt.variant_id) {
        // visible on gzclient
        QueueVisibleMsgs(link->GetModel(), true);
        // do collide environment, does not collide the wrapped body and the track itself
        dGeomSetCategoryBits((dGeomID)link->GetSpaceId(), GZ_GHOST_COLLIDE);
        dGeomSetCollideBits((dGeomID)link->GetSpaceId(), GZ_ALL_COLLIDE);
      } else {
        // invisible on gzclient
        QueueVisibleMsgs(link->GetModel(), false);
        // collide nothing
        dGeomSetCategoryBits((dGeomID)link->GetSpaceId(), GZ_NONE_COLLIDE);
        dGeomSetCollideBits((dGeomID)link->GetSpaceId(), GZ_NONE_COLLIDE);
      }
    }
  }

  // ******************
  // Updating the track
  // ******************

  void UpdateTrack(const common::UpdateInfo &_info) {
    // enable visuals of the current variant and disable others
    PublishVisibleMsgs();

    // state of the track
    const double track_pos(wrap::Position(track_.sprocket.joint, 0) *
                           track_.sprocket.joint_to_track);
    const double track_vel(track_.sprocket.joint->GetVelocity(0) * track_.sprocket.joint_to_track);

    // new variant id to be enabled
    const std::size_t new_variant_id(CalcVariantId(track_pos));

    if (track_.belt.variant_id != new_variant_id) {
      // schedule enabling visuals of new variant & disabling visuals of last variant
      // (actual publishment is performed at the begging of the next step
      //  because the position of the new variant has not been updated)
      const physics::ODELinkPtr new_link(track_.belt.segments[0].variants[new_variant_id].link);
      QueueVisibleMsgs(new_link->GetModel(), true);
      dGeomSetCategoryBits((dGeomID)new_link->GetSpaceId(), GZ_GHOST_COLLIDE);
      dGeomSetCollideBits((dGeomID)new_link->GetSpaceId(), GZ_ALL_COLLIDE);
      const physics::ODELinkPtr old_link(
          track_.belt.segments[0].variants[track_.belt.variant_id].link);
      QueueVisibleMsgs(old_link->GetModel(), false);
      dGeomSetCategoryBits((dGeomID)old_link->GetSpaceId(), GZ_NONE_COLLIDE);
      dGeomSetCollideBits((dGeomID)old_link->GetSpaceId(), GZ_NONE_COLLIDE);

      track_.belt.variant_id = new_variant_id;
    }

    // length which a element is distributed along the track
    const double len_per_element(track_.belt.perimeter / track_.belt.elements_per_round);
    // track pos normalized in [-len_per_element / 2, len_per_element / 2)
    const double track_pos_per_element(track_pos -
                                       len_per_element * std::floor(track_pos / len_per_element) -
                                       len_per_element / 2.);

    // set enabled or disabled for all links.
    // this is because all links have been enabled at the beginning of every world update
    // as the links are connected to another link which is enabled.
    for (const Track::Belt::Segment &segment : track_.belt.segments) {
      for (std::size_t variant_id = 0; variant_id < segment.variants.size(); ++variant_id) {
        if (variant_id == track_.belt.variant_id) {
          // enable the link (should happen nothing but just in case)
          segment.variants[variant_id].link->SetEnabled(true);

          // set position & velocity
          wrap::SetPosition(segment.variants[variant_id].joint, 0,
                            track_pos_per_element / segment.joint_to_track, true);
          SetJointMotorVelocity(segment.variants[variant_id].joint, 0,
                                track_vel / segment.joint_to_track);
        } else {
          segment.variants[variant_id].link->SetEnabled(false);
        }
      }
    }
  }

  std::size_t CalcVariantId(const double _track_pos) const {
    // length which a element, or a set of unique elements is distributed along the track
    const double len_per_element(track_.belt.perimeter / track_.belt.elements_per_round);
    const double len_per_elements(len_per_element * track_.belt.segments[0].variants.size());

    // track pos normalized in [0, len_per_elements)
    const double track_pos_per_elements(_track_pos - len_per_elements *
                                                         std::floor(_track_pos / len_per_elements));

    // new variant id to be enabled
    return static_cast< std::size_t >(std::floor(track_pos_per_elements / len_per_element));
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

  // ****************************
  // Transporting visual messages
  // ****************************

  void InitVisualPublisher(const std::string &_world_name) {
    node_.reset(new transport::Node());
    node_->Init(_world_name);
    visual_publisher_ = node_->Advertise< msgs::Visual >("~/visual");
  }

  void QueueVisibleMsgs(const physics::ModelPtr &_model, const bool _visible) {
    msgs::VisualPtr msg(new msgs::Visual());
    msg->set_name(_model->GetScopedName());
    msg->set_parent_name(_model->GetParent()->GetScopedName());
    msg->set_visible(_visible);
    visible_msgs_.push(msg);
  }

  void PublishVisibleMsgs() {
    while (!visible_msgs_.empty()) {
      visual_publisher_->Publish(*visible_msgs_.front());
      visible_msgs_.pop();
    }
  }

private:
  // transport to toggling track visuals
  transport::NodePtr node_;
  transport::PublisherPtr visual_publisher_;
  std::queue< msgs::VisualPtr > visible_msgs_;
  // the track model
  Track track_;
  // callback connection handle
  event::ConnectionPtr update_connection_;
};

} // namespace gazebo

#endif