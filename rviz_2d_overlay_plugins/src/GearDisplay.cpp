#include "GearDisplay.h"

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

    GearDisplay::GearDisplay() : current_gear_(0)
    {

        int fontId = QFontDatabase::addApplicationFont(":/assets/font/Quicksand/static/Quicksand-Regular.ttf");
        int fontId2 = QFontDatabase::addApplicationFont(":/assets/font/Quicksand/static/Quicksand-Bold.ttf");
        if (fontId == -1 || fontId2 == -1)
        {
            std::cout << "Failed to load the Quicksand font.";
        }
    }

    GearDisplay::~GearDisplay()
    {
        // Cleanup if necessary
    }

    void GearDisplay::updateGearData(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr &msg)
    {
        current_gear_ = msg->report; // Assuming msg->report contains the gear information
    }

    void GearDisplay::drawGearIndicator(QPainter &painter, const QRectF &backgroundRect)
    {
        QFont gearFont("Quicksand", 16, QFont::Bold);
        painter.setFont(gearFont);
        QPen borderPen(Qt::white);
        borderPen.setWidth(4);
        painter.setPen(borderPen);

        int gearBoxSize = 30;
        int gearX = backgroundRect.left() + 30 + gearBoxSize;
        int gearY = backgroundRect.height() - gearBoxSize - 20;
        QRect gearRect(gearX, gearY, gearBoxSize, gearBoxSize);
        painter.setBrush(QColor(0, 0, 0, 0));
        painter.drawRoundedRect(gearRect, 5, 5);
        painter.drawText(gearRect, Qt::AlignCenter, current_gear_ == 0 ? "N" : std::to_string(current_gear_).c_str());
    }

} // namespace rviz_2d_overlay_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_2d_overlay_plugins::GearDisplay, rviz_common::Display)
