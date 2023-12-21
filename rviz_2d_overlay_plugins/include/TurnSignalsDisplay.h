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
#endif
namespace rviz_2d_overlay_plugins
{

    class TurnSignalsDisplay : public rviz_common::RosTopicDisplay<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>
    {
        Q_OBJECT
    public:
        TurnSignalsDisplay();
        virtual ~TurnSignalsDisplay() override;
        void drawArrows(QPainter &painter, const QRectF &backgroundRect, const QColor &color);
        void updateTopic(const QString &new_topic)
        {
            this->setTopic(new_topic, rosidl_generator_traits::data_type<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>());
        }

    protected:
        void onEnable() override;
        void onDisable() override;
        void processMessage(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr msg) override;

    private:
        QImage arrowImage;
        int current_turn_signal_; // Internal variable to store turn signal state
        QImage coloredImage(const QImage &source, const QColor &color);
    };

} // namespace rviz_2d_overlay_plugins

#endif // TURN_SIGNALS_DISPLAY_H
