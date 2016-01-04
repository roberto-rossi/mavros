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
#include <mavros_msgs/csi.h>

namespace mavplugin {
/**
 * @brief csi_matrix plugin
 */
class csiPlugin : public MavRosPlugin {
public:
    csiPlugin() :
        csi_nh("~csi"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)	{
		uas = &uas_;
        csi_pub = csi_nh.advertise<mavros::csi>("pub", 10);
        uas->sig_connection_changed.connect(boost::bind(&csiPlugin::connection_cb, this, _1));
	};

	const message_map get_rx_handlers() {
		return {
                   MESSAGE_HANDLER(MAVLINK_MSG_ID_CSI_MATRIX, &csiPlugin::handle_csi)
		};
	}

private:
	std::recursive_mutex mutex;
    ros::NodeHandle csi_nh;
	UAS *uas;
    ros::Publisher csi_pub;

	/* -*- rx handlers -*- */
    void handle_csi(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
        mavlink_csi_matrix_t port;
        mavlink_msg_csi_matrix_decode(msg, &port);
		lock_guard lock(mutex);

        auto csi_msg = boost::make_shared<mavros_msgs::csi>();
        csi_msg->csi = port.value;
        csi_pub.publish(csi_msg);
	}

	/* -*- callbacks -*- */
	void connection_cb(bool connected) {
		lock_guard lock(mutex);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::csiPlugin, mavplugin::MavRosPlugin)
