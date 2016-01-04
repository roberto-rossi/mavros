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
#include <mavros_msgs/Bb_tb_i_matrix.h>

namespace mavplugin {
/**
 * @brief Bb_tb_i_matrix plugin
 */
class Bb_tb_i_matrixPlugin : public MavRosPlugin {
public:
    Bb_tb_i_matrixPlugin() :
        Bb_tb_i_nh("~Bb_tb_i_matrix"),
		uas(nullptr)
	{ };

    void initialize(UAS &uas_) {
		uas = &uas_;
        Bb_tb_i_sub = Bb_tb_i_nh.subscribe("sub", 100, &Bb_tb_i_matrixPlugin::override_cb, this);
        uas->sig_connection_changed.connect(boost::bind(&Bb_tb_i_matrixPlugin::connection_cb, this, _1));
	};
    const message_map get_rx_handlers() {
        return {
        };
    }
private:
	std::recursive_mutex mutex;
    ros::NodeHandle Bb_tb_i_nh;
	UAS *uas;
    ros::Subscriber Bb_tb_i_sub;

	/* -*- low-level send functions -*- */
    void Bb_tb_i_override(const boost::array<float_t, 48> &value) {
        mavlink_message_t msg;
        float value1 [48];
        std::copy(&value[0],&value[48],value1);
        mavlink_msg_bb_tb_i_matrix_pack_chan(UAS_PACK_CHAN(uas), &msg, 0, value1);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- callbacks -*- */
	void connection_cb(bool connected) {
		lock_guard lock(mutex);
	}

    void override_cb(const mavros_msgs::Bb_tb_i_matrix::ConstPtr req) {
        Bb_tb_i_override(req->Bb_tb_i);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::Bb_tb_i_matrixPlugin, mavplugin::MavRosPlugin)

