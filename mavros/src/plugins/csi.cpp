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
#include <mavros_msgs/csi.h>

namespace mavplugin {
/**
 * @brief csi_matrix plugin
 */
class csiPlugin : public MavRosPlugin {
public:
    csiPlugin() :
        csi_nh("~csi")
    { }

	void initialize(UAS &uas_)	{
        csi_pub = csi_nh.advertise<mavros_msgs::csi>("pub", 10);
	};

	const message_map get_rx_handlers() {
		return {
                   MESSAGE_HANDLER(MAVLINK_MSG_ID_CSI_MATRIX, &csiPlugin::handle_csi)
		};
	}

private:
    ros::NodeHandle csi_nh;
    ros::Publisher csi_pub;

	/* -*- rx handlers -*- */
    void handle_csi(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {

        mavlink_csi_matrix_t port;
        mavlink_msg_csi_matrix_decode(msg, &port);

        auto csi_msg = boost::make_shared<mavros_msgs::csi>();
        csi_msg->csi[0] = port.value[0];
        csi_msg->csi[1] = port.value[1];
        csi_msg->csi[2] = port.value[2];
        csi_msg->csi[3] = port.value[3];
        csi_msg->csi[4] = port.value[4];
        csi_msg->csi[5] = port.value[5];
        csi_msg->csi[6] = port.value[6];
        csi_msg->csi[7] = port.value[7];
        csi_msg->csi[8] = port.value[8];
        csi_msg->csi[9] = port.value[9];
        csi_msg->csi[10] = port.value[10];
        csi_pub.publish(csi_msg);
	}

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::csiPlugin, mavplugin::MavRosPlugin)
