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
#include <mavros_msgs/B_matrix.h>

namespace mavplugin {
/**
 * @brief B_matrix plugin
 */
class B_matrixPlugin : public MavRosPlugin {
public:
    B_matrixPlugin() :
        B_nh("~B_matrix"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

        B_sub = B_nh.subscribe("sub", 100, &B_matrixPlugin::override_cb, this);

        uas->sig_connection_changed.connect(boost::bind(&B_matrixPlugin::connection_cb, this, _1));
	};
    const message_map get_rx_handlers() {
        return {
        };
    }
private:
	std::recursive_mutex mutex;
	ros::NodeHandle B_nh;
	UAS *uas;

	ros::Subscriber B_sub;

	/* -*- low-level send functions -*- */

    void B_override(const boost::array<float_t, 100> &value) {
		mavlink_message_t msg;
        float value1 [50];
        for(int j = 0; j < 50; j++) {
                    value1[j]=value[j];
                }
        std::copy(&value[0],&value[49],value1);
        mavlink_msg_b1_matrix_pack_chan(UAS_PACK_CHAN(uas), &msg, 0,
				value1
				);
		UAS_FCU(uas)->send_message(&msg);
		
        std::copy(&value[49],&value[99],value1);
        for(int j = 50; j < 100; j++) {
                    value1[j-50]=value[j];
                }
        mavlink_msg_b2_matrix_pack_chan(UAS_PACK_CHAN(uas), &msg, 0,
				value1
				);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- callbacks -*- */

	void connection_cb(bool connected) {
		lock_guard lock(mutex);
	}

    void override_cb(const mavros_msgs::B_matrix::ConstPtr req) {
        B_override(req->B);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::B_matrixPlugin, mavplugin::MavRosPlugin)

