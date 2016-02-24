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
#include <mavros_msgs/Bt_tb_i_matrix.h>

namespace mavplugin {
/**
 * @brief Bt_tb_i_matrix plugin
 */
class Bt_tb_i_matrixPlugin : public MavRosPlugin {
public:
    Bt_tb_i_matrixPlugin() :
        Bt_tb_i_nh("~Bt_tb_i_matrix"),
		uas(nullptr)
	{ };

    void initialize(UAS &uas_) {
		uas = &uas_;
        Bt_tb_i_sub = Bt_tb_i_nh.subscribe("sub", 100, &Bt_tb_i_matrixPlugin::override_cb, this);
        uas->sig_connection_changed.connect(boost::bind(&Bt_tb_i_matrixPlugin::connection_cb, this, _1));
	};
    const message_map get_rx_handlers() {
        return {
        };
    }
private:
	std::recursive_mutex mutex;
    ros::NodeHandle Bt_tb_i_nh;
	UAS *uas;
    ros::Subscriber Bt_tb_i_sub;

	/* -*- low-level send functions -*- */
    void Bt_tb_i_override(const boost::array<float_t, 48> &value) {
        mavlink_message_t msg;
        float value1 [48];
        std::copy(value.begin(),value.end(),value1);
        mavlink_msg_bt_tb_i_matrix_pack_chan(UAS_PACK_CHAN(uas), &msg, 0, value1);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- callbacks -*- */
	void connection_cb(bool connected) {
		lock_guard lock(mutex);
	}

    void override_cb(const mavros_msgs::Bt_tb_i_matrix::ConstPtr req) {
        Bt_tb_i_override(req->Bt_tb_i);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::Bt_tb_i_matrixPlugin, mavplugin::MavRosPlugin)

