#ifndef GAZEBO_CONTINUOUS_TRACK_PROPERTIES
#define GAZEBO_CONTINUOUS_TRACK_PROPERTIES

#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <ros/package.h>

namespace gazebo {

class ContinuousTrackProperties {

public:
  ContinuousTrackProperties(const physics::ModelPtr &_model, const sdf::ElementPtr &_sdf) {
    // convert to a plugin sdf having element descriptions.
    // this also asserts the given sdf matches the plugin sdf format.
    const sdf::ElementPtr formatted_sdf(ToPluginSDF(_sdf));

    name = formatted_sdf->GetAttribute("name")->GetAsString();
    sprocket = LoadSprocket(_model, formatted_sdf->GetElement("sprocket"));
    trajectory = LoadTrajectory(_model, formatted_sdf->GetElement("trajectory"));
    pattern = LoadPattern(_model, formatted_sdf->GetElement("pattern"));
  }

  virtual ~ContinuousTrackProperties() {}

  // *****************
  // public properties
  // *****************

  std::string name;
  struct Sprocket {
    physics::JointPtr joint;
    double pitch_diameter;
  };
  Sprocket sprocket;

  struct Trajectory {
    struct Segment {
      physics::JointPtr joint;
      double end_position;
    };
    std::vector< Segment > segments;
  };
  Trajectory trajectory;

  struct Pattern {
    std::size_t elements_per_round;
    struct Element {
      std::vector< sdf::ElementPtr > collision_sdfs;
      std::vector< sdf::ElementPtr > visual_sdfs;
    };
    std::vector< Element > elements;
  };
  Pattern pattern;

private:
  // ******************
  // loading properties
  // ******************

  static Sprocket LoadSprocket(const physics::ModelPtr &_model, const sdf::ElementPtr &_sdf) {
    // format has been checked by ToPluginSDF(). no need to check if required elements exist.

    Sprocket sprocket;

    // [joint]
    sprocket.joint = _model->GetJoint(_sdf->GetElement("joint")->Get< std::string >());
    GZ_ASSERT(sprocket.joint,
              "Cannot find a joint with the value of [sprocket]::[joint] element in sdf");
    GZ_ASSERT(sprocket.joint->HasType(physics::Joint::HINGE_JOINT),
              "[sprocket]::[joint] must be a rotatinal joint");

    // [pitch_diameter]
    sprocket.pitch_diameter = _sdf->GetElement("pitch_diameter")->Get< double >();
    GZ_ASSERT(sprocket.pitch_diameter > 0.,
              "[sprocket]::[pitch_diameter] must be a positive real number");

    return sprocket;
  }

  static Trajectory LoadTrajectory(const physics::ModelPtr &_model, const sdf::ElementPtr &_sdf) {
    // format has been checked by ToPluginSDF(). no need to check if required elements exist.

    Trajectory trajectory;

    // [segment] (multiple, +)
    for (sdf::ElementPtr segment_elem = _sdf->GetElement("segment"); segment_elem;
         segment_elem = segment_elem->GetNextElement("segment")) {
      Trajectory::Segment segment;

      // []::[joint]
      segment.joint = _model->GetJoint(segment_elem->GetElement("joint")->Get< std::string >());
      GZ_ASSERT(segment.joint, "Cannot find a joint with the value of "
                               "[trajectory]::[segment]::[joint] element in sdf");
      GZ_ASSERT(segment.joint->HasType(physics::Joint::HINGE_JOINT) ||
                    segment.joint->HasType(physics::Joint::SLIDER_JOINT),
                "[trajectory]::[segment]::[joint] must be a rotatinal or translational joint");

      // []::[end_position]
      segment.end_position = segment_elem->GetElement("end_position")->Get< double >();
      GZ_ASSERT(segment.end_position > 0.,
                "[trajectory]::[segment]::[end_position] must be a positive real number");

      trajectory.segments.push_back(segment);
    }

    return trajectory;
  }

  static Pattern LoadPattern(const physics::ModelPtr &_model, const sdf::ElementPtr &_sdf) {
    // format has been checked by ToPluginSDF(). no need to check if required elements exist.

    Pattern pattern;

    // [elements_per_round]
    pattern.elements_per_round = _sdf->GetElement("elements_per_round")->Get< std::size_t >();
    GZ_ASSERT(pattern.elements_per_round > 0,
              "[pattern]::[elements_per_round] must be positive intger");

    // [element] (multiple, +)
    for (sdf::ElementPtr element_elem = _sdf->GetElement("element"); element_elem;
         element_elem = element_elem->GetNextElement("element")) {
      Pattern::Element element;

      // []::[collision] (multiple, *)
      if (element_elem->HasElement("collision")) {
        for (sdf::ElementPtr collision_elem = element_elem->GetElement("collision"); collision_elem;
             collision_elem = collision_elem->GetNextElement("collision")) {
          element.collision_sdfs.push_back(collision_elem->Clone());
        }
      }

      // []::[visual] (multiple, *)
      if (element_elem->HasElement("visual")) {
        for (sdf::ElementPtr visual_elem = element_elem->GetElement("visual"); visual_elem;
             visual_elem = visual_elem->GetNextElement("visual")) {
          element.visual_sdfs.push_back(visual_elem->Clone());
        }
      }

      pattern.elements.push_back(element);
    }

    return pattern;
  }

  // **************
  // formatting sdf
  // **************

  // get a sdf element which has been initialized by the plugin format file.
  // the initialied sdf may look empty but have a format information.
  static sdf::ElementPtr LoadPluginFormat() {
    const sdf::ElementPtr fmt(new sdf::Element());
    GZ_ASSERT(sdf::initFile(ros::package::getPath("gazebo_continuous_track") +
                                "/sdf/continuous_track_plugin.sdf",
                            fmt),
              "Cannot initialize sdf by continuous_track_plugin.sdf");
    return fmt;
  }

  // merge the plugin format sdf and the given sdf.
  // assert if the given sdf does not match the format
  // (ex. no required element, value type mismatch, ...).
  static sdf::ElementPtr ToPluginSDF(const sdf::ElementPtr &_sdf) {
    static const sdf::ElementPtr fmt(LoadPluginFormat());
    const sdf::ElementPtr dst(fmt->Clone());
    GZ_ASSERT(
        sdf::readString("<sdf version='" SDF_VERSION "'>" + _sdf->ToString("") + "</sdf>", dst),
        "The given sdf does not match ContinuousTrack plugin format");
    return dst;
  }
};
} // namespace gazebo

#endif