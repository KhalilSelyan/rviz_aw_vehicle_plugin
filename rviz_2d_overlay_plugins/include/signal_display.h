// -*- mode: c++; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Team Spatzenhirn
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef SIGNAL_DISPLAY_H
#define SIGNAL_DISPLAY_H
#ifndef Q_MOC_RUN
#include <rviz_common/display.hpp>
#include "overlay_utils.hpp"
#include <OgreColourValue.h>
#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <QImage>
#include <QString>
#include "SteeringWheelDisplay.h"
#include "GearDisplay.h"
#include "SpeedDisplay.h"
#include "TurnSignalsDisplay.h"
#endif

namespace rviz_2d_overlay_plugins
{
    class SignalDisplay
        : public rviz_common::Display
    {
        Q_OBJECT
    public:
        SignalDisplay();
        virtual ~SignalDisplay();

    protected:
        void onInitialize() override;
        void update(float wall_dt, float ros_dt) override;
        void reset() override;
        void onEnable() override;
        void onDisable() override;
        void processMessage(const std::shared_ptr<const autoware_auto_vehicle_msgs::msg::GearReport> &msg);
        void processMessage(const std::shared_ptr<const autoware_auto_vehicle_msgs::msg::SteeringReport> &msg);
        void processMessage(const std::shared_ptr<const autoware_auto_vehicle_msgs::msg::VelocityReport> &msg);
        void processMessage(const std::shared_ptr<const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport> &msg);
        void processMessage(const std::shared_ptr<const autoware_auto_vehicle_msgs::msg::HazardLightsReport> &msg);

    private Q_SLOTS:
        void updateOverlaySize();
        void updateOverlayPosition();
        void updateSignalData();

    private:
        std::mutex mutex_;
        rviz_2d_overlay_plugins::OverlayObject::SharedPtr overlay_;
        rviz_common::properties::IntProperty *property_width_;
        rviz_common::properties::IntProperty *property_height_;
        rviz_common::properties::IntProperty *property_left_;
        rviz_common::properties::IntProperty *property_top_;
        rviz_common::properties::ColorProperty *property_signal_color_;
        rviz_common::properties::RosTopicProperty *steering_topic_property_;
        rviz_common::properties::RosTopicProperty *gear_topic_property_;
        rviz_common::properties::RosTopicProperty *speed_topic_property_;
        rviz_common::properties::RosTopicProperty *turn_signals_topic_property_;
        rviz_common::properties::RosTopicProperty *hazard_lights_topic_property_;

        void drawBackground(QPainter &painter, const QRectF &backgroundRect);
        std::unique_ptr<SteeringWheelDisplay> steering_wheel_display_;
        std::unique_ptr<GearDisplay> gear_display_;
        std::unique_ptr<SpeedDisplay> speed_display_;
        std::unique_ptr<TurnSignalsDisplay> turn_signals_display_;

        rclcpp::Node::SharedPtr rviz_node_;
        rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_sub_;
        rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_sub_;
        rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr speed_sub_;
        rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_signals_sub_;
        rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_lights_sub_;

        std::thread executor_thread_;
        rclcpp::executors::SingleThreadedExecutor executor_;
        bool executor_running_;

        void updateGearData(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr &msg);
        void updateSteeringData(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr &msg);
        void updateSpeedData(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr &msg);
        void updateTurnSignalsData(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr &msg);
        void updateHazardLightsData(const autoware_auto_vehicle_msgs::msg::HazardLightsReport::ConstSharedPtr &msg);
        void drawWidget(QImage &hud);
    };
}

#endif
