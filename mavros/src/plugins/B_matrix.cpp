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
#include <iostream.h>

#include <mavros_msgs/B_matrix.h>

namespace mavplugin {
/**
 * @brief B_matrix plugin
 */
class BPlugin : public MavRosPlugin {
public:
	BPlugin() :
		B_nh("~B"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		B_sub = B_nh.subscribe("sub", 10, &BPlugin::override_cb, this);

		uas->sig_connection_changed.connect(boost::bind(&BPlugin::connection_cb, this, _1));
	};

private:
	std::recursive_mutex mutex;
	ros::NodeHandle B_nh;
	UAS *uas;

	ros::Subscriber B_sub;

	/* -*- low-level send functions -*- */

	void B_override(float value[100]) {
		mavlink_message_t msg;
		float value1[50];
		std::copy(&value[0],&value[50],value1);
		mavlink_msg_b1_matrix_pack_chan(UAS_PACK_CHAN(uas), &msg,
				value1
				);
		UAS_FCU(uas)->send_message(&msg);
		
		std::copy(&value[50],&value[100],value1);;
		mavlink_msg_b2_matrix_pack_chan(UAS_PACK_CHAN(uas), &msg,
				value1
				);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- callbacks -*- */

	void connection_cb(bool connected) {
		lock_guard lock(mutex);
	}

	void override_cb(const mavros::B::ConstPtr req) {
		B_override(req->value);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::BPlugin, mavplugin::MavRosPlugin)

