#ifndef TRAFFIC_DISPLAY_H
#define TRAFFIC_DISPLAY_H
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
// #include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
// #include <autoware_perception_msgs/msg/traffic_signal.hpp>
// #include <autoware_perception_msgs/msg/traffic_signal_element.hpp>
#endif

namespace rviz_2d_overlay_plugins
{

    class TrafficDisplay : public rviz_common::Display
    {
        Q_OBJECT
    public:
        TrafficDisplay();
        virtual ~TrafficDisplay() override;
        void drawTrafficLightIndicator(QPainter &painter, const QRectF &backgroundRect);
        // void updateTrafficLightData(const autoware_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr msg);

    private:
        int current_traffic_; // Internal variable to store current gear
        QImage traffic_light_image_;
        // yellow #CFC353
        QColor yellow = QColor(207, 195, 83);
        // red #CF5353
        QColor red = QColor(207, 83, 83);
        // green #53CF5F
        QColor green = QColor(83, 207, 95);
        // gray #C2C2C2
        QColor gray = QColor(194, 194, 194);

        QImage coloredImage(const QImage &source, const QColor &color);
    };

} // namespace rviz_2d_overlay_plugins

#endif // TRAFFIC_DISPLAY_H
