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
#include <mavros_msgs/csi_dot.h>

namespace mavplugin {
/**
 * @brief csi_dot_matrix plugin
 */
class csi_dotPlugin : public MavRosPlugin {
public:
    csi_dotPlugin() :
        csi_dot_nh("~csi_dot"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)	{
		uas = &uas_;
        csi_dot_pub = csi_dot_nh.advertise<mavros::csi_dot>("pub", 10);
        uas->sig_connection_changed.connect(boost::bind(&csi_dotPlugin::connection_cb, this, _1));
	};

	const message_map get_rx_handlers() {
		return {
                   MESSAGE_HANDLER(MAVLINK_MSG_ID_CSI_DOT_MATRIX, &csi_dotPlugin::handle_csi_dot)
		};
	}

private:
	std::recursive_mutex mutex;
    ros::NodeHandle csi_dot_nh;
	UAS *uas;
    ros::Publisher csi_dot_pub;

	/* -*- rx handlers -*- */
    void handle_csi_dot(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
        mavlink_csi_dot_matrix_t port;
        mavlink_msg_csi_dot_matrix_decode(msg, &port);
		lock_guard lock(mutex);

        auto csi_dot_msg = boost::make_shared<mavros_msgs::csi_dot>();
        csi_dot_msg->csi_dot = port.value;
        csi_dot_pub.publish(csi_dot_msg);
	}

	/* -*- callbacks -*- */
	void connection_cb(bool connected) {
		lock_guard lock(mutex);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::csi_dotPlugin, mavplugin::MavRosPlugin)
