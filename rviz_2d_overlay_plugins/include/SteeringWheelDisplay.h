#ifndef STEERING_WHEEL_DISPLAY_H
#define STEERING_WHEEL_DISPLAY_H
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
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include <QString>
#endif

namespace rviz_2d_overlay_plugins
{

    class SteeringWheelDisplay : public rviz_common::RosTopicDisplay<autoware_auto_vehicle_msgs::msg::SteeringReport>
    {
        Q_OBJECT
    public:
        SteeringWheelDisplay();
        virtual ~SteeringWheelDisplay() override;
        void drawSteeringWheel(QPainter &painter, const QRectF &backgroundRect);
        void updateTopic(const QString &new_topic)
        {
            this->setTopic(new_topic, rosidl_generator_traits::data_type<autoware_auto_vehicle_msgs::msg::SteeringReport>());
        }

    protected:
        void onEnable() override;
        void onDisable() override;
        void processMessage(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg) override;

    private:
        float steering_angle_ = 0.0f;
        QImage wheelImage;
        QImage coloredImage(const QImage &source, const QColor &color);
    };

} // namespace rviz_2d_overlay_plugins

#endif // STEERING_WHEEL_DISPLAY_H
