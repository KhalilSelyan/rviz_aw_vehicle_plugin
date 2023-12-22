#include "GearDisplay.h"

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
        printf("Is this ever called?\n");
    }

    void GearDisplay::onInitialize()
    {
        RTDClass::onInitialize();
    }

    void GearDisplay::processMessage(autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        try
        {

            // Assuming msg->report is the field you're interested in

            qDebug() << "currentgear before is: " << current_gear_;
            current_gear_ = msg->report;

            qDebug() << "current_gear_ value is: " << current_gear_;

            newDataReceived(msg->report);
            emit newDataReceived(msg->report);
            qDebug() << "newDataReceived emitted with gear:" << msg->report;
        }
        catch (const std::exception &e)
        {
            // Log the error
            std::cerr << "Error in processMessage: " << e.what() << std::endl;
        }
    }

    void GearDisplay::drawGearIndicator(QPainter &painter, const QRectF &backgroundRect)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        qDebug() << "drawGearIndicator current_gear_ value is: " << current_gear_;

        // we deal with the different gears here
        std::string gearString;
        switch (current_gear_)
        {
        case autoware_auto_vehicle_msgs::msg::GearReport::NEUTRAL:
            gearString = "N";
            break;
        case autoware_auto_vehicle_msgs::msg::GearReport::LOW:
        case autoware_auto_vehicle_msgs::msg::GearReport::LOW_2:
            gearString = "L";
            break;
        case autoware_auto_vehicle_msgs::msg::GearReport::NONE:
            gearString = "P";
            break;
        // all the drive gears from DRIVE to DRIVE_18
        default:
            gearString = "D";
            break;
        }

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
        painter.drawText(gearRect, Qt::AlignCenter, QString::fromStdString(gearString));

        queueRender();
    }

} // namespace rviz_2d_overlay_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_2d_overlay_plugins::GearDisplay, rviz_common::Display)
