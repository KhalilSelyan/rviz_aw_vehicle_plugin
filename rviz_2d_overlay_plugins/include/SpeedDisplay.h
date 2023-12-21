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
#include <rviz_common/properties/color_property.hpp>
#include <QImage>
#include <QString>
#include "autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp"
#endif

namespace rviz_2d_overlay_plugins
{

    class SpeedDisplay : public rviz_common::RosTopicDisplay<autoware_auto_vehicle_msgs::msg::VehicleKinematicState>
    {
        Q_OBJECT
    public:
        SpeedDisplay();
        virtual ~SpeedDisplay() override;
        void drawSpeedDisplay(QPainter &painter, const QRectF &backgroundRect);
        void updateTopic(const QString &new_topic)
        {
            this->setTopic(new_topic, rosidl_generator_traits::data_type<autoware_auto_vehicle_msgs::msg::VehicleKinematicState>());
        }

    protected:
        void onEnable() override;
        void onDisable() override;
        void processMessage(const autoware_auto_vehicle_msgs::msg::VehicleKinematicState::ConstSharedPtr msg) override;

    private:
        float current_speed_; // Internal variable to store current speed
    };

} // namespace rviz_2d_overlay_plugins

#endif // SPEED_DISPLAY_H
