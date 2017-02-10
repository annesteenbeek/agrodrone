#!/user/bin/env python

from src.lib.state import FlightState
from mavros_msgs.msg import CommandCode, Waypoint

class TrackSpray(FlightState):
    """
    This state is used to monitor the process of spraying
    """

    def enter(self, event_data):
        """
        This function is called when the mode enters this state
        """
        # TODO check if tank is connected and do pre spray checks...
        # TODO implement mission sanity check; correct takeoff, altitude etc..
        # TODO give takeoff control back to user
        self.vehicle.set_mode_mission()
        self.final_waypoint_flag = False
        #  self.vehicle.command_mission_start()
        super(TrackSpray, self).enter(event_data)


    def is_state_complete(self):
        """
        This method checks if the final waypoint has been passed and returns True if so
        """
        # TODO Could implement getting acc radius parameter for distance check
        # TODO check for absolute altitude
        # Walk backwards from array, get first item that has CMD_ID 16 (waypoint)
        # Set final waypoint flag if it is set as current waypoint
        # If it is no longer current waypoint, it means mission is finished
        # (Only works for PX4 apperently)
        result = False
        if self.vehicle.mission_list is not None:
            for waypoint in self.vehicle.mission_list[::-1]:
                if waypoint.command == CommandCode.NAV_WAYPOINT:
                    if waypoint.is_current: # prevent needles calculation
                        # final waypoint has been reached
                        lat = waypoint.x_lat
                        lon = waypoint.y_long
                        alt = waypoint.z_alt
                        if waypoint.frame == Waypoint.FRAME_GLOBAL_REL_ALT:
                            relative = True
                        elif waypoint.frame == Waypoint.FRAME_GLOBAL:
                            relative = False
                        else:
                            rospy.logerr("Unknown waypoint frame when calculating distance.")
                            # TODO handle different frames for missions
                            # Let it crash, this should be handled if it ever happens

                        dist = self.vehicle.get_distance(lat, lon, alt, relative)
                        result = True
                    break # final waypoint has been found, exit loop
        return result


    def run(self):
        """
        This function is called on every loop iteration of the main control loop when this function is active
        """
        pass
