/**
 * @brief Tank level plugin
 * @file tank_level.cpp
 * @author Anne Steenbeek <annesteenbeek@gmail.com>
 */

#include <mavros/mavros_plugin.h>

#include <mavros_agrodrone/TankLevel.h>

namespace mavros {
namespace agro_plugins{
/**
 * @brief tank level plugin
 *
 * broadcast the level of the spray tank connected to the companion computer
 */
class TankLevelPlugin : public plugin::PluginBase {
public:
    TankLevelPlugin(): PluginBase(),
        tl_nh("~")
    { };

    void initialize(UAS &uas_) {
        PluginBase::initialize(uas_);

        tank_level_sub = tl_nh.subscribe("/tank_sensor/tank_level", 10, &TankLevelPlugin::tank_level_cb, this);
    }

    Subscriptions get_subscriptions() {
        return { /* Rx disabled */ };
    }

private:
    ros::NodeHandle tl_nh;

    ros::Subscriber tank_level_sub;


    void tank_level_cb(const mavros_agrodrone::TankLevel::ConstPtr &req) {
        set_tank_level(req->percentage, req->raw);
        }

    void set_tank_level(const uint8_t percentage, const uint32_t raw) {

        mavlink::agrodrone::msg::TANK_LEVEL tank;

        const uint8_t tgt_sys_id = 0;

        tank.target_system = tgt_sys_id;
        tank.perc = percentage;
        tank.raw = raw;

        UAS_FCU(m_uas)->send_message_ignore_drop(tank);
        }
}; // TankLevelPlugin    
}  // agro_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::agro_plugins::TankLevelPlugin, mavros::plugin::PluginBase)
