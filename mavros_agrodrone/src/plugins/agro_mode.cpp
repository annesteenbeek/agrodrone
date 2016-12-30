/**
 * @brief handle mode messages
 * @file agro_mode.cpp
 * @author Anne Steenbeek <annesteenbeek@gmail.com>
 */

#include <mavros/mavros_plugin.h>

#include <agrodrone/CompanionMode.h>
#include <agrodrone/SetCompanionMode.h>

namespace mavros {
namespace agro_plugins{
using utils::enum_value;
using mavlink::agrodrone::AGRO_SUB_MODE;
/**
 * @brief handle the state messages of the companion computer from and to MAVLink
 */
class AgroModePlugin : public plugin::PluginBase {
public:
    AgroModePlugin(): PluginBase(),
        am_nh("~")
    { };

    void initialize(UAS &uas_) {
        PluginBase::initialize(uas_);

        agro_mode_sub = am_nh.subscribe("agro_mode", 10, &AgroModePlugin::send_agro_mode_cb, this);
        client = am_nh.serviceClient<agrodrone::SetCompanionMode>("/set_companion_mode");
    }

    Subscriptions get_subscriptions() {
        return {
            make_handler(&AgroModePlugin::handle_set_agro_mode)
        };
    }

private:
    ros::NodeHandle am_nh;

    ros::Subscriber agro_mode_sub;
    ros::ServiceClient client;

    /* -*- rx handlers -*- */
    void handle_set_agro_mode(const mavlink::mavlink_message_t *msg, mavlink::agrodrone::msg::SET_AGRO_MODE &sam) {
        ROS_INFO("Recived mode change");

        agrodrone::SetCompanionMode srv;
        srv.request.mode_to_set = AgroModePlugin::mode_enum_to_string(sam.agro_mode);

        // Service returns false if it was unable to set agro mode.
        if (!client.call(srv)) {
            ROS_ERROR("Unable to set Agro mode: %u", sam.agro_mode); 
            // TODO add qgroundcontrol responses for failures and warnings
        }
    }


    void send_agro_mode_cb(const agrodrone::CompanionMode::ConstPtr &req) {
        mavlink::agrodrone::msg::GET_AGRO_MODE mode;

        const uint8_t tgt_sys_id = 0; // Broadcast this message
        uint8_t agro_mode = AgroModePlugin::string_to_mode_enum(req->mode);
        uint8_t agro_sub_mode = AgroModePlugin::string_to_sub_mode_enum(req->state);

        mode.target_system = tgt_sys_id;
        mode.agro_mode = agro_mode;
        mode.agro_sub_mode = agro_sub_mode;
        UAS_FCU(m_uas)->send_message_ignore_drop(mode);
    }

    static uint8_t string_to_mode_enum(std::string const& agro_mode) {
        //TODO convert case switches to boost bi-map? 
            using mavlink::agrodrone::AGRO_MODE;
            if(agro_mode == "Autospray")   return enum_value(AGRO_MODE::AUTOSPRAY);
            if(agro_mode == "RTD")         return enum_value(AGRO_MODE::RTD);
            if(agro_mode == "Inactive")    return enum_value(AGRO_MODE::INACTIVE);
            else {
                /* ROS_ERROR_ONCE("Tried to send unknown mode to MAVLink: %s", agro_mode); */
                return enum_value(AGRO_MODE::UNK);
            }
        } 

    static std::string mode_enum_to_string(uint8_t &_AGRO_MODE) {
        using mavlink::agrodrone::AGRO_MODE;
        switch(_AGRO_MODE) {
            case enum_value(AGRO_MODE::AUTOSPRAY):   return "Autospray";
            case enum_value(AGRO_MODE::RTD):         return "RTD";
            case enum_value(AGRO_MODE::INACTIVE):    return "Inactive";
            case enum_value(AGRO_MODE::UNK):         return "";
            default:
                /* ROS_ERROR_ONCE("Tried to convert unknown enum to string: %d", AGRO_MODE); */
                return "";
        }
    }

    static uint8_t string_to_sub_mode_enum(std::string const& agro_sub_mode) {
        using mavlink::agrodrone::AGRO_SUB_MODE;
            if(agro_sub_mode == "Pending")           return enum_value(AGRO_SUB_MODE::PENDING);
            if(agro_sub_mode == "TrackSpray")        return enum_value(AGRO_SUB_MODE::TRACK_SPRAY);
            if(agro_sub_mode == "ResumeSpray")       return enum_value(AGRO_SUB_MODE::RESUME_SPRAY);
            if(agro_sub_mode == "Docked")            return enum_value(AGRO_SUB_MODE::DOCKED);
            if(agro_sub_mode == "PositionAboveDock") return enum_value(AGRO_SUB_MODE::POSITION_ABOVE_DOCK);
            else{
                // TODO: make sure to warn on every unknown mode change, not just the first
                /* ROS_WARN_ONCE("Tried to send unknown sub_mode to MAVLink: %s", agro_sub_mode); */
                return enum_value(AGRO_SUB_MODE::UNK);
            }
    }
}; // AgroModePlugin    
} // agro_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::agro_plugins::AgroModePlugin, mavros::plugin::PluginBase)
