/**
 * @brief handle mode messages
 * @file agro_mode.cpp
 * @author Anne Steenbeek <annesteenbeek@gmail.com>
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <agrodrone/companion_mode.h>
#include <agrodrone/set_companion_mode.h>

namespace mavplugin {
/**
 * @brief handle the state messages of the companion computer from and to MAVLink
 */
class AgroModePlugin : public MavRosPlugin{
public:
    AgroModePlugin():
        tl_nh("~"),
        uas(nullptr)
    { };

    void initialize(UAS &uas_) {
        uas = &uas_;

        agro_mode_sub = tl_nh.subscribe("agro_mode", 10, &AgroModePlugin::send_agro_mode_cb, this);
        ros::ServiceClient client = tl_nh.ServiceClient<agrodrone::SetCompanionMode>("set_companion_mode");
    }

    const message_map get_rx_handlers() {
        return {
            MESSAGE_HANDLER(MAVLINK_MSG_ID_SET_AGR_MODE, &AgroModePlugin::handle_set_agro_mode); 
        };
    }

private:
    ros::NodeHandle tl_nh;
    UAS *uas;

    ros::Subscriber tank_level_sub;

    /* -*- rx handlers -*- */
    void handle_set_agro_mode(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
        mavlink_msg_set_agro_mode_t sam;
        mavlink_msg_set_agro_mode_decode(msg, &sam);
        
        agrodrone::SetCompanionMode srv;
        srv.request.mode_to_set = AgroModePlugin::mode_enum_to_string(sam.agro_mode);

        // Service returns false if it was unable to set agro mode.
        if (!client.call(srv)) {
            ROS_ERROR("Unable to set Agro mode: %d", sam.agro_mode); 
            // TODO add qgroundcontrol responses for failures and warnings
        }
    }


    void send_agro_mode_cb(const agrodrone::CompanionMode::ConstPtr &req) {
        mavlink_message_t msg;

        const uint8_t tgt_sys_id = 0; // Broadcast this message
        uint8_t agro_mode = AgroModePlugin::string_to_mode_enum(req->mode);
        uint8_t agro_sub_mode = AgroModePlugin::string_to_sub_mode_enum(req->state);

        mavlink_msg_get_agro_mode_pack_chan(
                UAS_PACK_CHAN(uas),
                &msg,
                tgt_sys_id,
                agro_mode,
                agro_sub_mode);
        UAS_FCU(uas)->send_message(&msg);
        }

    static uint8_t string_to_mode_enum(std::string agro_mode) {
        //TODO convert case switches to map? 
        switch(agro_mode) {
            case "Autospray":   return AGRO_MODE_AUTOSPRAY;
            case "RTD":         return AGRO_MODE_RTD;
            case "Inactive":    return AGRO_MODE_INACTIVE;
            default:
                ROS_ERROR_ONCE("Tried to send unknown mode to MAVLink: %s", msg.mode);
                return AGRO_MODE_UNK;
        } 
    } 

    static std:string mode_enum_to_string(uint8_t &AGRO_MODE) {
        switch(AGRO_MODE) {
            case AGRO_MODE_AUTOSPRAY:   return "Autospray";
            case AGRO_MODE_RTD:         return "RTD";
            case AGRO_MODE_INACTIVE:    return "Inactive";
            case AGRO_MODE_UNK:         return "";
            default:
                ROS_ERROR_ONCE("Tried to convert unknown enum to string: %d", AGRO_MODE);
                return "";
        }
    }

    static uint8_t string_to_sub_mode_enum(std::string agro_sub_mode) {
        switch(agro_sub_mode) {
            case "Pending":     return AGRO_SUB_MODE_PENDING;
            case "TrackSpray"   return AGRO_SUB_MODE_TRACK_SPRAY;
            case "ResumeSpray"  return AGRO_SUB_MODE_RESUME_SPRAY;
            case "Docked"       return AGRO_SUB_MODE_DOCKED;
            case "PositionAboveDock"    return AGRO_SUB_MODE_POSITION_ABOVE_DOCK;
            default:
                // TODO: make sure to warn on every unknown mode change, not just the first
                ROS_WARN_ONCE("Tried to send unknown sub_mode to MAVLink: %s", msg.state);
                return AGRO_SUB_MODE_UNK;
        }
    }
}; // AgroModePlugin    
}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::AgroModePlugin, mavplugin::MavRosPlugin)
