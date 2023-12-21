#ifndef TURN_SIGNALS_DISPLAY_H
#define TURN_SIGNALS_DISPLAY_H
#ifndef Q_MOC_RUN
#include <rviz_common/ros_topic_display.hpp>
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
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <chrono>
#endif
namespace rviz_2d_overlay_plugins
{

    class TurnSignalsDisplay : public rviz_common::Display
    {
        Q_OBJECT
    public:
        TurnSignalsDisplay();
        virtual ~TurnSignalsDisplay() override;
        void drawArrows(QPainter &painter, const QRectF &backgroundRect, const QColor &color);
        void updateTurnSignalsData(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr &msg);
        void updateHazardLightsData(const autoware_auto_vehicle_msgs::msg::HazardLightsReport::ConstSharedPtr &msg);

    private:
        QImage arrowImage;
        int current_turn_signal_;   // Internal variable to store turn signal state
        int current_hazard_lights_; // Internal variable to store hazard lights state
        QImage coloredImage(const QImage &source, const QColor &color);

        std::chrono::steady_clock::time_point last_toggle_time_;
        bool blink_on_ = false;
        const std::chrono::milliseconds blink_interval_{500}; // Blink interval in milliseconds
    };

} // namespace rviz_2d_overlay_plugins

#endif // TURN_SIGNALS_DISPLAY_H
