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

        arrowImage.load(":/assets/images/arrow.png");
    }

    TurnSignalsDisplay::~TurnSignalsDisplay()
    {
        // Cleanup if necessary
    }

    void TurnSignalsDisplay::onEnable()
    {
        subscribe();
    }

    void TurnSignalsDisplay::onDisable()
    {
        unsubscribe();
    }

    void TurnSignalsDisplay::processMessage(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr msg)
    {
        try
        {
            current_turn_signal_ = msg->report; // Assuming this field contains the turn signal state
            queueRender();
        }
        catch (const std::exception &e)
        {
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
                           current_turn_signal_ == autoware_auto_vehicle_msgs::msg::HazardLightsReport::ENABLE);
        bool rightActive = (current_turn_signal_ == autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT ||
                            current_turn_signal_ == autoware_auto_vehicle_msgs::msg::HazardLightsReport::ENABLE);
        QColor overlayColor = color;

        QImage leftArrow = (leftActive ? coloredImage(scaledLeftArrow, overlayColor) : scaledLeftArrow);
        painter.drawImage(QPointF(leftArrowXPos, arrowYPos), leftArrow);

        QImage rightArrow = (rightActive ? coloredImage(scaledRightArrow, overlayColor) : scaledRightArrow);
        painter.drawImage(QPointF(rightArrowXPos, arrowYPos), rightArrow);
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
