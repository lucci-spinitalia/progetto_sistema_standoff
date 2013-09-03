#ifndef _SEGWAY_CAN_H
#define _SEGWAY_CAN_H

#include "segway_config.h"

/* MACRO */
#define segway_configure_none(socket, message_param) (segway_configure_load(socket, 0x00, message_param))
#define segway_configure_max_vel(socket, message_param) (segway_configure_load(socket, 0x01, convert_to_ieee754(message_param)))
#define segway_configure_max_acc(socket, message_param) (segway_configure_load(socket, 0x02, convert_to_ieee754(message_param)))
#define segway_configure_max_decel(socket, message_param) (segway_configure_load(socket, 0x03, convert_to_ieee754(message_param)))
#define segway_configure_max_dtz(socket, message_param) (segway_configure_load(socket, 0x04, convert_to_ieee754(message_param)))
#define segway_configure_coastdown(socket, message_param) (segway_configure_load(socket, 0x05, convert_to_ieee754(message_param)))
#define segway_configure_max_turn_rate(socket, message_param) (segway_configure_load(socket, 0x06, convert_to_ieee754(message_param)))
#define segway_configure_max_turn_accel(socket, message_param) (segway_configure_load(socket, 0x07, convert_to_ieee754(message_param)))
#define segway_configure_tire_diameter(socket, message_param) (segway_configure_load(socket, 0x08, convert_to_ieee754(message_param)))
#define segway_configure_wheel_track_width(socket, message_param) (segway_configure_load(socket, 0x09, convert_to_ieee754(message_param)))
#define segway_configure_transmission_ratio(socket, message_param) (segway_configure_load(socket, 0x0a, convert_to_ieee754(message_param)))
#define segway_configure_input_config_bitmap(socket, message_param) (segway_configure_load(socket, 0x0b, message_param))
#define segway_configure_eth_ip(socket, message_param) (segway_configure_load(socket, 0x0c, message_param))
#define segway_configure_eth_port(socket, message_param) (segway_configure_load(socket, 0x0d, message_param))
#define segway_configure_eth_subnet(socket, message_param) (segway_configure_load(socket, 0x0e, message_param))
#define segway_configure_eth_gatway(socket, message_param) (segway_configure_load(socket, 0x0f, message_param))
#define segway_configure_feedback_bitmap1(socket, message_param) (segway_configure_load(socket, 0x10, message_param))
#define segway_configure_feedback_bitmap2(socket, message_param) (segway_configure_load(socket, 0x11, message_param))
#define segway_configure_feedback_bitmap3(socket, message_param) (segway_configure_load(socket, 0x12, message_param))
#define segway_configure_force_config_feedback_bitmap(socket, message_param) (segway_configure_load(socket, 0x13, message_param))
#define segway_configure_audio_command(socket, message_param) (segway_configure_load(socket, 0x14, message_param))
#define segway_configure_operational_mode(socket, message_param) (segway_configure_load(socket, 0x15, message_param))
#define segway_configure_sp_faultlog(socket, message_param) (segway_configure_load(socket, 0x16, message_param))
#define segway_configure_reset_integrator(socket, message_param) (segway_configure_load(socket, 0x17, message_param))
#define segway_configure_reset_params_to_default(socket, message_param) (segway_configure_load(socket, 0x18, message_param))

extern int segway_init(int *, struct segway_struct *);
extern int segway_check(int *socket);
extern int segway_motion_set(int socket, float velocity, float yaw, int scale_value);
extern int segway_configure_load(int socket, __u32 message_id, __u32 message_param);
extern void segway_config_update(struct can_frame *frame);
extern int segway_configure_feedback(int socket, __u32 feedback1_param, __u32 feedback2_param, __u32 feedback3_param);
extern int segway_configure_feedback3(int socket, __u32 message_param);
extern int segway_configure_feedback2(int socket, __u32 message_param);
extern int segway_configure_feedback1(int socket, __u32 message_param);

#endif
