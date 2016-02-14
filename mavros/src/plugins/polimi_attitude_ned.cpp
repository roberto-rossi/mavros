/**
 * @brief B_matrix plugin
 * @file B_matrix.cpp
 * @author Mirko <mirko@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <mavros_msgs/polimi_attitude_ned.h>

namespace mavplugin {
/**
 * @brief polimi_attitude_ned plugin
 */
class polimi_attitude_nedPlugin : public MavRosPlugin {
public:
    polimi_attitude_nedPlugin() :
        polimi_attitude_ned_nh("~polimi_attitude_ned")
    { }

	void initialize(UAS &uas_)	{
        polimi_attitude_ned_pub = polimi_attitude_ned_nh.advertise<mavros_msgs::polimi_attitude_ned>("pub", 10);
	};

	const message_map get_rx_handlers() {
		return {
                   MESSAGE_HANDLER(MAVLINK_MSG_ID_POLIMI_ATTITUDE_NED, &polimi_attitude_nedPlugin::handle_polimi_attitude_ned)
		};
	}

private:
    ros::NodeHandle polimi_attitude_ned_nh;
    ros::Publisher polimi_attitude_ned_pub;

	/* -*- rx handlers -*- */
    void handle_polimi_attitude_ned(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {

        mavlink_polimi_attitude_ned_t port;
        mavlink_msg_polimi_attitude_ned_decode(msg, &port);

        auto polimi_attitude_ned_msg = boost::make_shared<mavros_msgs::polimi_attitude_ned>();
        polimi_attitude_ned_msg->x = port.x;
        polimi_attitude_ned_msg->y = port.y;
        polimi_attitude_ned_msg->z = port.z;
        polimi_attitude_ned_msg->w = port.w;
        polimi_attitude_ned_pub.publish(polimi_attitude_ned_msg);
	}

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::polimi_attitude_nedPlugin, mavplugin::MavRosPlugin)
