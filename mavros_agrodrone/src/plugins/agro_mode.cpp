/**
 * @brief handle mode messages
 * @file agro_mode.cpp
 * @author Anne Steenbeek <annesteenbeek@gmail.com>
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <agrodrone/CompanionMode.h>
#include <agrodrone/SetCompanionMode.h>

namespace mavplugin {
/**
 * @brief handle the state messages of the companion computer from and to MAVLink
 */
class AgroModePlugin : public MavRosPlugin{
public:
    AgroModePlugin():
        am_nh("~"),
        uas(nullptr)
    { };

    void initialize(UAS &uas_) {
        uas = &uas_;

        agro_mode_sub = am_nh.subscribe("/commander/companion_mode", 10, &AgroModePlugin::send_agro_mode_cb, this);
        client = am_nh.serviceClient<agrodrone::SetCompanionMode>("/commander/set_companion_mode");
    }

    const message_map get_rx_handlers() {
        return {
/* MAVLINK_MSG_ID_SET_AGRO_MODE */
            MESSAGE_HANDLER(239, &AgroModePlugin::handle_set_agro_mode)
        };
    }

private:
    ros::NodeHandle am_nh;
    UAS *uas;

    ros::Subscriber agro_mode_sub;
    ros::ServiceClient client;

    /* -*- rx handlers -*- */
    void handle_set_agro_mode(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
        ROS_INFO("Recived mode change");
        mavlink_set_agro_mode_t sam;
        mavlink_msg_set_agro_mode_decode(msg, &sam);
        

        agrodrone::SetCompanionMode srv;
        srv.request.mode_to_set = AgroModePlugin::mode_enum_to_string(sam.agro_mode);

        // Service returns false if it was unable to set agro mode.
        if (!client.call(srv)) {
            ROS_ERROR("Unable to set Agro mode: %u", sam.agro_mode); 
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

    static uint8_t string_to_mode_enum(std::string const& agro_mode) {
        //TODO convert case switches to boost bi-map? 
            if(agro_mode == "Autospray")   return AGRO_MODE_AUTOSPRAY;
            if(agro_mode == "RTD")         return AGRO_MODE_RTD;
            if(agro_mode == "Inactive")    return AGRO_MODE_INACTIVE;
            else {
                /* ROS_ERROR_ONCE("Tried to send unknown mode to MAVLink: %s", agro_mode); */
                return AGRO_MODE_UNK;
            }
        } 

    static std::string mode_enum_to_string(uint8_t &AGRO_MODE) {
        switch(AGRO_MODE) {
            case AGRO_MODE_AUTOSPRAY:   return "Autospray";
            case AGRO_MODE_RTD:         return "RTD";
            case AGRO_MODE_INACTIVE:    return "Inactive";
            case AGRO_MODE_UNK:         return "";
            default:
                /* ROS_ERROR_ONCE("Tried to convert unknown enum to string: %d", AGRO_MODE); */
                return "";
        }
    }

    static uint8_t string_to_sub_mode_enum(std::string const& agro_sub_mode) {
            if(agro_sub_mode == "Pending")           return AGRO_SUB_MODE_PENDING;
            if(agro_sub_mode == "TrackSpray")        return AGRO_SUB_MODE_TRACK_SPRAY;
            if(agro_sub_mode == "ResumeSpray")       return AGRO_SUB_MODE_RESUME_SPRAY;
            if(agro_sub_mode == "Docked")            return AGRO_SUB_MODE_DOCKED;
            if(agro_sub_mode == "PositionAboveDock") return AGRO_SUB_MODE_POSITION_ABOVE_DOCK;
            else{
                // TODO: make sure to warn on every unknown mode change, not just the first
                /* ROS_WARN_ONCE("Tried to send unknown sub_mode to MAVLink: %s", agro_sub_mode); */
                return AGRO_SUB_MODE_UNK;
            }
    }
}; // AgroModePlugin    
}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::AgroModePlugin, mavplugin::MavRosPlugin)
