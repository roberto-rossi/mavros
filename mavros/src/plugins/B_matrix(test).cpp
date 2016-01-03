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

		B_pub = B_nh.advertise<mavros::B>("pub", 10);
		B_sub = B_nh.subscribe("sub", 10, &BPlugin::override_cb, this);

		uas->sig_connection_changed.connect(boost::bind(&BPlugin::connection_cb, this, _1));
	};

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_B_MATRIX, &BPlugin::handle_B)
			      
		};
	}

private:
	std::recursive_mutex mutex;
	ros::NodeHandle B_nh;
	UAS *uas;

	ros::Publisher B_pub;
	ros::Subscriber B_sub;

	/* -*- rx handlers -*- */

	void handle_B(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_b_matrix_t port;
		mavlink_msg_b_matrix_decode(msg, &port);
		lock_guard lock(mutex);

		auto B_msg = boost::make_shared<mavros::B>();

		B_msg->B = port.value;
                
  		B_pub.publish(B_msg);
	}

	/* -*- low-level send functions -*- */

	void B_override(float[100] value) {
		mavlink_message_t msg;

		mavlink_msg_b_matrix_pack_chan(UAS_PACK_CHAN(uas), &msg,
				value
				);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- callbacks -*- */

	void connection_cb(bool connected) {
		lock_guard lock(mutex);
	}

	void override_cb(const mavros::B::ConstPtr req) {
//		if (!uas->is_ardupilotmega())
//			ROS_WARN_THROTTLE_NAMED(30, "man", "Manipulator controller override not supported by this FCU!");

		B_override(req->value);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::BPlugin, mavplugin::MavRosPlugin)

