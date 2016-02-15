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
#include <mavros_msgs/csi_dot.h>

namespace mavplugin {
/**
 * @brief csi_dot_matrix plugin
 */
class csi_dotPlugin : public MavRosPlugin {
public:
    csi_dotPlugin() :
        csi_dot_nh("~csi_dot")
	{ };

    void initialize(UAS &uas_)	{
        csi_dot_pub = csi_dot_nh.advertise<mavros_msgs::csi_dot>("pub", 10);
	};

	const message_map get_rx_handlers() {
		return {
                   MESSAGE_HANDLER(MAVLINK_MSG_ID_CSI_DOT_MATRIX, &csi_dotPlugin::handle_csi_dot)
		};
	}

private:
    ros::NodeHandle csi_dot_nh;
    ros::Publisher csi_dot_pub;

	/* -*- rx handlers -*- */
    void handle_csi_dot(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {

        mavlink_csi_dot_matrix_t port;
        mavlink_msg_csi_dot_matrix_decode(msg, &port);

        auto csi_dot_msg = boost::make_shared<mavros_msgs::csi_dot>();
        csi_dot_msg->csi_dot[0] = port.value[0];
        csi_dot_msg->csi_dot[1] = port.value[1];
        csi_dot_msg->csi_dot[2] = port.value[2];
        csi_dot_msg->csi_dot[3] = port.value[3];
        csi_dot_msg->csi_dot[4] = port.value[4];
        csi_dot_msg->csi_dot[5] = port.value[5];
        csi_dot_msg->csi_dot[6] = port.value[6];
        csi_dot_msg->csi_dot[7] = port.value[7];
        csi_dot_msg->csi_dot[8] = port.value[8];
        csi_dot_msg->csi_dot[9] = port.value[9];
        csi_dot_msg->csi_dot[10] = port.value[10];
        csi_dot_pub.publish(csi_dot_msg);
	}

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::csi_dotPlugin, mavplugin::MavRosPlugin)
