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
#include <mavros_msgs/g_matrix.h>

namespace mavplugin {
/**
 * @brief g_matrix plugin
 */
class g_matrixPlugin : public MavRosPlugin {
public:
    g_matrixPlugin() :
        g_nh("~g_matrix"),
		uas(nullptr)
	{ };

    void initialize(UAS &uas_) {
		uas = &uas_;
        g_sub = g_nh.subscribe("sub", 100, &g_matrixPlugin::override_cb, this);
        uas->sig_connection_changed.connect(boost::bind(&g_matrixPlugin::connection_cb, this, _1));
	};
    const message_map get_rx_handlers() {
        return {
        };
    }
private:
	std::recursive_mutex mutex;
    ros::NodeHandle g_nh;
	UAS *uas;
    ros::Subscriber g_sub;

	/* -*- low-level send functions -*- */
    void g_override(const boost::array<float_t, 10> &value) {
        mavlink_message_t msg;
        float value1 [10];
        std::copy(value.begin(),value.end(),value1);
        mavlink_msg_g_matrix_pack_chan(UAS_PACK_CHAN(uas), &msg, 0, value1);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- callbacks -*- */
	void connection_cb(bool connected) {
		lock_guard lock(mutex);
	}

    void override_cb(const mavros_msgs::g_matrix::ConstPtr req) {
        g_override(req->g);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::g_matrixPlugin, mavplugin::MavRosPlugin)

