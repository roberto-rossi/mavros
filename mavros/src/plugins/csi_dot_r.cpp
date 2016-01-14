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
#include <mavros_msgs/csi_dot_r.h>

namespace mavplugin {
/**
 * @brief csi_dot_r plugin
 */
class csi_dot_rPlugin : public MavRosPlugin {
public:
    csi_dot_rPlugin() :
        csi_dot_r_nh("~csi_dot_r"),
		uas(nullptr)
	{ };

    void initialize(UAS &uas_) {
		uas = &uas_;
        csi_dot_r_sub = csi_dot_r_nh.subscribe("sub", 100, &csi_dot_rPlugin::override_cb, this);
        uas->sig_connection_changed.connect(boost::bind(&csi_dot_rPlugin::connection_cb, this, _1));
	};
    const message_map get_rx_handlers() {
        return {
        };
    }
private:
	std::recursive_mutex mutex;
    ros::NodeHandle csi_dot_r_nh;
	UAS *uas;
    ros::Subscriber csi_dot_r_sub;

	/* -*- low-level send functions -*- */
    void csi_dot_r_override(const boost::array<float_t, 11> &value) {
        mavlink_message_t msg;
        float value1 [11];
        std::copy(value.begin(),value.end(),value1);
        mavlink_msg_csi_dot_r_matrix_pack_chan(UAS_PACK_CHAN(uas), &msg, 0, value1);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- callbacks -*- */
	void connection_cb(bool connected) {
		lock_guard lock(mutex);
	}

    void override_cb(const mavros_msgs::csi_dot_r::ConstPtr req) {
        csi_dot_r_override(req->csi_r_dot);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::csi_dot_rPlugin, mavplugin::MavRosPlugin)

