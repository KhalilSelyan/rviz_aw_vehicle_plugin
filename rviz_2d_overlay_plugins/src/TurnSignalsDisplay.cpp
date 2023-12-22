#include "TurnSignalsDisplay.h"

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

namespace rviz_2d_overlay_plugins
{

    TurnSignalsDisplay::TurnSignalsDisplay() : current_turn_signal_(0)
    {

        last_toggle_time_ = std::chrono::steady_clock::now();

        arrowImage.load(":/assets/images/arrow.png");
    }

    TurnSignalsDisplay::~TurnSignalsDisplay()
    {
        // Cleanup if necessary
    }

    void TurnSignalsDisplay::onInitialize()
    {
        RTDClass::onInitialize();
    }

    void TurnSignalsDisplay::processMessage(autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr msg)
    {
        try
        {
            // Assuming msg->report is the field you're interested in
            current_turn_signal_ = msg->report;
            queueRender();
        }
        catch (const std::exception &e)
        {
            // Log the error
            std::cerr << "Error in processMessage: " << e.what() << std::endl;
        }
    }

    void TurnSignalsDisplay::drawArrows(QPainter &painter, const QRectF &backgroundRect, const QColor &color)
    {
        QImage scaledLeftArrow = arrowImage.scaled(64, 43, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        QImage scaledRightArrow = scaledLeftArrow.mirrored(true, false);
        int arrowYPos = (backgroundRect.height() / 3 - scaledLeftArrow.height() / 2);
        int leftArrowXPos = backgroundRect.width() / 4 - scaledLeftArrow.width(); // Adjust as needed
        int rightArrowXPos = backgroundRect.width() * 3 / 4;                      // Adjust as needed

        bool leftActive = (current_turn_signal_ == autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT ||
                           current_hazard_lights_ == autoware_auto_vehicle_msgs::msg::HazardLightsReport::ENABLE);
        bool rightActive = (current_turn_signal_ == autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT ||
                            current_hazard_lights_ == autoware_auto_vehicle_msgs::msg::HazardLightsReport::ENABLE);

        // Color the arrows based on the state of the turn signals and hazard lights by having them blink on and off
        if (this->blink_on_)
        {
            if (leftActive)
            {
                scaledLeftArrow = coloredImage(scaledLeftArrow, color);
            }
            if (rightActive)
            {
                scaledRightArrow = coloredImage(scaledRightArrow, color);
            }
        }

        // Draw the arrows
        painter.drawImage(QPointF(leftArrowXPos, arrowYPos), scaledLeftArrow);
        painter.drawImage(QPointF(rightArrowXPos, arrowYPos), scaledRightArrow);

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_toggle_time_) >= blink_interval_)
        {
            blink_on_ = !blink_on_; // Toggle the blink state
            last_toggle_time_ = now;
        }
    }

    QImage TurnSignalsDisplay::coloredImage(const QImage &source, const QColor &color)
    {
        QImage result = source;
        QPainter p(&result);
        p.setCompositionMode(QPainter::CompositionMode_SourceAtop);
        p.fillRect(result.rect(), color);
        p.end();
        return result;
    }

} // namespace rviz_2d_overlay_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_2d_overlay_plugins::TurnSignalsDisplay, rviz_common::Display)
