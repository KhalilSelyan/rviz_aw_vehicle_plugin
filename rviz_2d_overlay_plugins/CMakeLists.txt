cmake_minimum_required(VERSION 3.8)
project(rviz_2d_overlay_plugins)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(tier4_perception_msgs REQUIRED)
find_package(tier4_planning_msgs REQUIRED)
ament_auto_find_build_dependencies() 
# if your cmake has this line it should probably find autoware_auto_vehicle_msgs package if not you should add to ament_target_dependencies()

find_package(rviz_2d_overlay_msgs REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(std_msgs REQUIRED)

set(
        headers_to_moc
        include/overlay_text_display.hpp
        include/signal_display.h
        include/SteeringWheelDisplay.h
        include/GearDisplay.h
        include/SpeedDisplay.h
        include/TurnSignalsDisplay.h
        include/TrafficDisplay.h
        include/SpeedLimitDisplay.h
)

qt5_add_resources(RESOURCES_RCC resources.qrc)


foreach (header "${headers_to_moc}")
    qt5_wrap_cpp(display_moc_files "${header}")
endforeach ()

set(
        display_source_files
        src/overlay_text_display.cpp
        src/overlay_utils.cpp
        src/signal_display.cpp
        src/SteeringWheelDisplay.cpp 
        src/GearDisplay.cpp          
        src/SpeedDisplay.cpp         
        src/TurnSignalsDisplay.cpp   
        src/TrafficDisplay.cpp
        src/SpeedLimitDisplay.cpp
        ${RESOURCES_RCC} # Add this line

)

add_library(
        ${PROJECT_NAME} SHARED
        ${display_moc_files}
        ${display_source_files}
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

target_include_directories(
        ${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${Qt5Widgets_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
        ${PROJECT_NAME} PUBLIC
        rviz_ogre_vendor::OgreMain
        rviz_ogre_vendor::OgreOverlay
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "${PROJECT_NAME}_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(
        ${PROJECT_NAME}
        PUBLIC
        rviz_common
        rviz_rendering
        rviz_2d_overlay_msgs
        autoware_auto_vehicle_msgs
        autoware_perception_msgs
        tier4_perception_msgs
        tier4_planning_msgs
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rviz_common
        rviz_ogre_vendor
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS
        DESTINATION lib/${PROJECT_NAME}
)

ament_package(
        CONFIG_EXTRAS "rviz_2d_overlay_plugins-extras.cmake"
)