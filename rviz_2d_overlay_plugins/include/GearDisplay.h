#ifndef GEAR_DISPLAY_H
#define GEAR_DISPLAY_H
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
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include <mutex>
#endif

namespace rviz_2d_overlay_plugins
{

    class GearDisplay : public rviz_common::RosTopicDisplay<autoware_auto_vehicle_msgs::msg::GearReport>
    {
        Q_OBJECT
    public:
        GearDisplay();
        virtual ~GearDisplay() override;
        void drawGearIndicator(QPainter &painter, const QRectF &backgroundRect);

    signals:
        void newDataReceived(int gear);

    public Q_SLOTS:
        void updateUI();

    protected:
        virtual void onInitialize() override;
        virtual void processMessage(autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr msg) override;

    private:
        std::mutex mutex_;
        int current_gear_; // Internal variable to store current gear
    };

} // namespace rviz_2d_overlay_plugins

#endif // GEAR_DISPLAY_H
