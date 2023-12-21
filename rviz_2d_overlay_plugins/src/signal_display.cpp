#include "signal_display.h"

#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTexture.h>
#include <OgreTechnique.h>
#include <OgreHardwarePixelBuffer.h>
#include <rviz_rendering/render_system.hpp>
#include <QPainter>
#include <QFontDatabase>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace rviz_2d_overlay_plugins
{

    SignalDisplay::SignalDisplay()
    {
        property_width_ = new rviz_common::properties::IntProperty("Width", 322, "Width of the overlay", this, SLOT(updateOverlaySize()));
        property_height_ = new rviz_common::properties::IntProperty("Height", 175, "Height of the overlay", this, SLOT(updateOverlaySize()));
        property_left_ = new rviz_common::properties::IntProperty("Left", 10, "Left position of the overlay", this, SLOT(updateOverlayPosition()));
        property_top_ = new rviz_common::properties::IntProperty("Top", 10, "Top position of the overlay", this, SLOT(updateOverlayPosition()));
        property_signal_color_ = new rviz_common::properties::ColorProperty("Signal Color", QColor(94, 130, 255), "Color of the signal arrows", this, SLOT(updateSignalData()));

        // Initialize the RViz node
        rviz_node_ = std::make_shared<rclcpp::Node>("rviz_2d_overlay_plugins");

        // TODO: These are still buggy, on button click they crash rviz
        gear_topic_property_ = new rviz_common::properties::RosTopicProperty("Gear Topic", "/vehicle/status/gear_status", rosidl_generator_traits::data_type<autoware_auto_vehicle_msgs::msg::GearReport>(), "Topic for Gear Data", this);
        turn_signals_topic_property_ = new rviz_common::properties::RosTopicProperty("Turn Signals Topic", "/vehicle/status/turn_indicators_status", rosidl_generator_traits::data_type<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(), "Topic for Turn Signals Data", this);
        speed_topic_property_ = new rviz_common::properties::RosTopicProperty("Speed Topic", "/vehicle/status/velocity_status", rosidl_generator_traits::data_type<autoware_auto_vehicle_msgs::msg::VelocityReport>(), "Topic for Speed Data", this);
        steering_topic_property_ = new rviz_common::properties::RosTopicProperty("Steering Topic", "/vehicle/status/steering_status", rosidl_generator_traits::data_type<autoware_auto_vehicle_msgs::msg::SteeringReport>(), "Topic for Steering Data", this);
        hazard_lights_topic_property_ = new rviz_common::properties::RosTopicProperty("Hazard Lights Topic", "/vehicle/status/hazard_lights_status", rosidl_generator_traits::data_type<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(), "Topic for Hazard Lights Data", this);

        // Initialize the component displays
        steering_wheel_display_ = std::make_unique<SteeringWheelDisplay>();
        gear_display_ = std::make_unique<GearDisplay>();
        speed_display_ = std::make_unique<SpeedDisplay>();
        turn_signals_display_ = std::make_unique<TurnSignalsDisplay>();
    }

    void SignalDisplay::onInitialize()
    {
        rviz_common::Display::onInitialize();
        rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
        static int count = 0;
        std::stringstream ss;
        ss << "SignalDisplayObject" << count++;
        overlay_.reset(new rviz_2d_overlay_plugins::OverlayObject(ss.str()));
        overlay_->show();
        updateOverlaySize();
        updateOverlayPosition();
    }

    SignalDisplay::~SignalDisplay()
    {
        overlay_.reset();
    }

    void SignalDisplay::reset()
    {
        rviz_common::Display::reset();
        overlay_->hide();
    }

    void SignalDisplay::updateOverlaySize()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
        overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
        queueRender();
    }

    void SignalDisplay::updateOverlayPosition()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
        queueRender();
    }

    void SignalDisplay::update(float /* wall_dt */, float /* ros_dt */)
    {

        if (!overlay_)
        {
            return;
        }
        rviz_2d_overlay_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
        QImage hud = buffer.getQImage(*overlay_);
        hud.fill(Qt::transparent);
        drawWidget(hud);
    }

    void SignalDisplay::onEnable()
    {
        if (overlay_)
        {
            overlay_->show();

            if (!executor_running_)
            {
                executor_running_ = true;
                executor_thread_ = std::thread([this]
                                               {
            executor_.add_node(rviz_node_);
            while (executor_running_) {
                executor_.spin_some();
            } });

                gear_sub_ = rviz_node_->create_subscription<autoware_auto_vehicle_msgs::msg::GearReport>(
                    gear_topic_property_->getTopic().toStdString(), rclcpp::QoS(rclcpp::KeepLast(5)).durability_volatile().reliable(),
                    [this](const autoware_auto_vehicle_msgs::msg::GearReport::SharedPtr msg)
                    {
                        updateGearData(msg);
                    });

                steering_sub_ = rviz_node_->create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
                    steering_topic_property_->getTopic().toStdString(), rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(),
                    [this](const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg)
                    {
                        updateSteeringData(msg);
                    });

                speed_sub_ = rviz_node_->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
                    speed_topic_property_->getTopic().toStdString(), rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(),
                    [this](const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
                    {
                        updateSpeedData(msg);
                    });

                turn_signals_sub_ = rviz_node_->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
                    turn_signals_topic_property_->getTopic().toStdString(), rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(),
                    [this](const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::SharedPtr msg)
                    {
                        updateTurnSignalsData(msg);
                    });

                hazard_lights_sub_ = rviz_node_->create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
                    hazard_lights_topic_property_->getTopic().toStdString(), rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(),
                    [this](const autoware_auto_vehicle_msgs::msg::HazardLightsReport::SharedPtr msg)
                    {
                        updateHazardLightsData(msg);
                    });
            }
        }
    }

    void SignalDisplay::onDisable()
    {
        executor_running_ = false;
        if (executor_thread_.joinable())
        {
            executor_thread_.join();
        }

        if (overlay_)
        {
            overlay_->hide();
            gear_sub_.reset();
            steering_sub_.reset();
            speed_sub_.reset();
            turn_signals_sub_.reset();
            hazard_lights_sub_.reset();
        }
    }

    void SignalDisplay::updateHazardLightsData(const autoware_auto_vehicle_msgs::msg::HazardLightsReport::ConstSharedPtr &msg)
    {
        if (turn_signals_display_)
        {
            turn_signals_display_->updateHazardLightsData(msg);
        }
    }

    void SignalDisplay::updateGearData(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr &msg)
    {
        if (gear_display_)
        {
            gear_display_->updateGearData(msg);
        }
    }

    void SignalDisplay::updateSteeringData(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr &msg)
    {
        if (steering_wheel_display_)
        {
            steering_wheel_display_->updateSteeringData(msg);
        }
    }

    void SignalDisplay::updateSpeedData(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr &msg)
    {
        if (speed_display_)
        {
            speed_display_->updateSpeedData(msg);
        }
    }

    void SignalDisplay::updateTurnSignalsData(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr &msg)
    {
        if (turn_signals_display_)
        {
            turn_signals_display_->updateTurnSignalsData(msg);
        }
    }

    void SignalDisplay::drawWidget(QImage &hud)
    {

        if (!overlay_->isVisible())
        {
            return;
        }

        QPainter painter(&hud);
        painter.setRenderHint(QPainter::Antialiasing, true);

        QRectF backgroundRect(0, 0, hud.width(), hud.height());
        drawBackground(painter, backgroundRect);
        // Draw components
        if (steering_wheel_display_)
        {
            steering_wheel_display_->drawSteeringWheel(painter, backgroundRect);
        }
        if (gear_display_)
        {
            gear_display_->drawGearIndicator(painter, backgroundRect);
        }
        if (speed_display_)
        {
            speed_display_->drawSpeedDisplay(painter, backgroundRect);
        }
        if (turn_signals_display_)
        {
            turn_signals_display_->drawArrows(painter, backgroundRect, property_signal_color_->getColor());
        }

        painter.end();
    }

    void SignalDisplay::drawBackground(QPainter &painter, const QRectF &backgroundRect)
    {
        painter.setBrush(QColor(0, 0, 0, 255 * 0.2)); // Black background with opacity
        painter.setPen(Qt::NoPen);
        painter.drawRoundedRect(backgroundRect, backgroundRect.height() / 2, backgroundRect.height() / 2); // Circular ends
    }

} // namespace aw_vehicle_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_2d_overlay_plugins::SignalDisplay, rviz_common::Display)