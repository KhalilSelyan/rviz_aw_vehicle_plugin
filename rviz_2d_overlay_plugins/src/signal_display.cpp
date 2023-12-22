#include "signal_display.h"

#include <qtextstream.h>
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTexture.h>
#include <OgreTechnique.h>
#include <OgreHardwarePixelBuffer.h>
#include <rviz_rendering/render_system.hpp>
#include <QPainter>
#include <QFontDatabase>
#include <QPointer>
#include <QThread>
#include <QDebug>
#include <qdebug.h>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

namespace rviz_2d_overlay_plugins
{

    SignalDisplay::SignalDisplay()
    {
        property_width_ = new rviz_common::properties::IntProperty("Width", 517, "Width of the overlay", this, SLOT(updateOverlaySize()));
        property_height_ = new rviz_common::properties::IntProperty("Height", 175, "Height of the overlay", this, SLOT(updateOverlaySize()));
        property_left_ = new rviz_common::properties::IntProperty("Left", 10, "Left position of the overlay", this, SLOT(updateOverlayPosition()));
        property_top_ = new rviz_common::properties::IntProperty("Top", 10, "Top position of the overlay", this, SLOT(updateOverlayPosition()));
        property_signal_color_ = new rviz_common::properties::ColorProperty("Signal Color", QColor(94, 130, 255), "Color of the signal arrows", this, SLOT(updateOverlayColor()));
    }

    void SignalDisplay::triggerRender(int gear)
    {
        qDebug() << "triggerRender called" << gear;

        queueRender();
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

    void SignalDisplay::update(float /* wall_dt */, float /* ros_dt */)
    {
        qDebug() << "update called";

        if (!overlay_)
        {
            return;
        }
        rviz_2d_overlay_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
        QImage hud = buffer.getQImage(*overlay_);
        hud.fill(Qt::transparent);
        drawWidget(hud);
        qDebug() << "update end";
    }

    void SignalDisplay::onEnable()
    {
        if (overlay_)
        {
            overlay_->show();
        }

        // Initialize the component displays
        steering_wheel_display_ = std::make_unique<SteeringWheelDisplay>();
        gear_display_ = std::make_unique<GearDisplay>();
        speed_display_ = std::make_unique<SpeedDisplay>();
        turn_signals_display_ = std::make_unique<TurnSignalsDisplay>();

        // Connect the signals
        auto woo = connect(
            gear_display_.get(), &GearDisplay::newDataReceived, this, &SignalDisplay::triggerRender);

        if (!woo)
        {
            RCLCPP_INFO(rclcpp::get_logger("rcl"), "Failed to connect");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("rcl"), "Connected");
        }
    }

    void SignalDisplay::onDisable()
    {

        if (overlay_)
        {
            overlay_->hide();
        }
    }

    void SignalDisplay::drawWidget(QImage &hud)
    {

        qDebug() << "drawWidget called";

        std::lock_guard<std::mutex> lock(mutex_);
        if (!overlay_->isVisible())
        {
            return;
        }

        QPainter painter(&hud);
        painter.setRenderHint(QPainter::Antialiasing, true);

        QRectF backgroundRect(0, 0, 322, hud.height());
        drawBackground(painter, backgroundRect, 0.5);

        // Draw components
        if (steering_wheel_display_)
        {
            steering_wheel_display_->drawSteeringWheel(painter, backgroundRect);
            queueRender();
        }
        if (gear_display_)
        {
            gear_display_->drawGearIndicator(painter, backgroundRect);
            queueRender();
        }
        if (speed_display_)
        {
            speed_display_->drawSpeedDisplay(painter, backgroundRect);
            queueRender();
        }
        if (turn_signals_display_)
        {
            turn_signals_display_->drawArrows(painter, backgroundRect, property_signal_color_->getColor());
            queueRender();
        }

        // a 27px space between the two halves of the HUD

        QRectF smallerBackgroundRect(349, 0, 168, hud.height() / 2);

        drawBackground(painter, smallerBackgroundRect, 0.5);

        painter.end();
        qDebug() << "drawWidget end";
        queueRender();
        qDebug() << "drawWidget end2";
    }

    void SignalDisplay::drawBackground(QPainter &painter, const QRectF &backgroundRect, float opacity)
    {
        painter.setBrush(QColor(0, 0, 0, 255 * opacity)); // Black background with opacity
        painter.setPen(Qt::NoPen);
        painter.drawRoundedRect(backgroundRect, backgroundRect.height() / 2, backgroundRect.height() / 2); // Circular ends
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

    void SignalDisplay::updateOverlayColor()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queueRender();
    }

} // namespace aw_vehicle_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_2d_overlay_plugins::SignalDisplay, rviz_common::Display)