cmake_minimum_required(VERSION 2.8.3)
project(gazebo_continuous_track_example)

find_package(catkin REQUIRED COMPONENTS
  gazebo_continuous_track
  gazebo_ros
  gazebo_ros_control
  xacro
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_continuous_track_example
  CATKIN_DEPENDS gazebo_continuous_track gazebo_ros gazebo_ros_control xacro
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)




# Function to generate a gazebo model in 2 steps
#   1. generate .urdf file from .urdf.xacro file
#   2. generate .sdf file from .urdf file
function(generate_gazebo_model name)
  #########
  # Step 1

  # Add a command to gemerate the .urdf file from the .urdf.xacro file
  # (this command also creates parent directories)
  set(
    urdf_file
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/${name}_gazebo.urdf
  )
  xacro_add_xacro_file(
    urdf_xacro/${name}_gazebo.urdf.xacro
    ${urdf_file}
  )

  #########
  # Step 2

  set(
    model_dir
    ${CMAKE_CURRENT_SOURCE_DIR}/model/${name}
  )

  # Add a command to generate model.config
  set(
    config_file
    ${model_dir}/model.config
  )
  string(
    CONCAT config_content
    "<?xml version=\"1.0\" ?>"
    "<model>"
    "    <name>" ${name} "</name>"
    "    <version>1.0</version>"
    "    <sdf version=\"1.6\">model.sdf</sdf>"
    "    <author>"
    "        <name></name>"
    "        <email></email>"
    "    </author>"
    "    <description></description>"
    "</model>"
  )
  add_custom_command(
    OUTPUT ${config_file}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${model_dir}
    COMMAND ${CMAKE_COMMAND} -E echo ${config_content} > ${config_file}
    COMMENT "Generating ${config_file}"
    VERBATIM
  )

  # Add a command to generate model.sdf from the .urdf file
  set(
    sdf_file
    ${model_dir}/model.sdf
  )
  add_custom_command(
    OUTPUT ${sdf_file}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${model_dir}
    COMMAND gz sdf -p ${urdf_file} > ${sdf_file}
    DEPENDS ${urdf_file}
    COMMENT "Generating ${sdf_file} from ${urdf_file}"
    VERBATIM
  )

  # Add a target to actually run the commands
  add_custom_target(
    ${PROJECT_NAME}_${name}_model ALL
    SOURCES ${config_file} ${sdf_file}
  )
endfunction()

generate_gazebo_model(example_lugged_wheel)
generate_gazebo_model(example_track)
generate_gazebo_model(example_track_simple)
generate_gazebo_model(example_track_simple_wheels)
