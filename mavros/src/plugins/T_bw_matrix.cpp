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
#include <mavros_msgs/T_bw_matrix.h>

namespace mavplugin {
/**
 * @brief T_bw_matrix plugin
 */
class T_bw_matrixPlugin : public MavRosPlugin {
public:
    T_bw_matrixPlugin() :
        T_bw_nh("~T_bw_matrix"),
		uas(nullptr)
	{ };

    void initialize(UAS &uas_) {
		uas = &uas_;
        T_bw_sub = T_bw_nh.subscribe("sub", 100, &T_bw_matrixPlugin::override_cb, this);
        uas->sig_connection_changed.connect(boost::bind(&T_bw_matrixPlugin::connection_cb, this, _1));
	};
    const message_map get_rx_handlers() {
        return {
        };
    }
private:
	std::recursive_mutex mutex;
    ros::NodeHandle T_bw_nh;
	UAS *uas;
    ros::Subscriber T_bw_sub;

	/* -*- low-level send functions -*- */
    void T_bw_override(const boost::array<float_t, 12> &value) {
        mavlink_message_t msg;
        float value1 [12];
        std::copy(value.begin(),value.end(),value1);
        mavlink_msg_t_bw_matrix_pack_chan(UAS_PACK_CHAN(uas), &msg, 0, value1);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- callbacks -*- */
	void connection_cb(bool connected) {
		lock_guard lock(mutex);
	}

    void override_cb(const mavros_msgs::T_bw_matrix::ConstPtr req) {
        T_bw_override(req->T_bw);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::T_bw_matrixPlugin, mavplugin::MavRosPlugin)

