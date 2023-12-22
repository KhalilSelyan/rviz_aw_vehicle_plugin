#ifndef SPEED_DISPLAY_H
#define SPEED_DISPLAY_H
#ifndef Q_MOC_RUN
#include <rviz_common/ros_topic_display.hpp>
#include "overlay_utils.hpp"
#include <OgreColourValue.h>
#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <QImage>
#include <QString>
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#endif

namespace rviz_2d_overlay_plugins
{

    class SpeedDisplay : public rviz_common::RosTopicDisplay<autoware_auto_vehicle_msgs::msg::VelocityReport>
    {
        Q_OBJECT
    public:
        SpeedDisplay();
        virtual ~SpeedDisplay() override;
        void drawSpeedDisplay(QPainter &painter, const QRectF &backgroundRect);

    protected:
        virtual void onInitialize() override;
        virtual void processMessage(autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg) override;

    private:
        float current_speed_; // Internal variable to store current speed
    };

} // namespace rviz_2d_overlay_plugins

#endif // SPEED_DISPLAY_H
