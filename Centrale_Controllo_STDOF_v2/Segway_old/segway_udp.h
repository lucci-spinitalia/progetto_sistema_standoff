#ifndef _SEGWAY_UDP_H
#define _SEGWAY_UDP_H

/* MACRO */
#define segway_configure_none(socket, dest_address, message_param)                (segway_configure_load(socket, dest_address, 0x00, message_param))
#define segway_configure_max_vel(socket, dest_address, message_param)             (segway_configure_load(socket, dest_address, 0x01, convert_to_ieee754(message_param)))
#define segway_configure_max_acc(socket, dest_address, message_param)             (segway_configure_load(socket, dest_address, 0x02, convert_to_ieee754(message_param)))
#define segway_configure_max_decel(socket, dest_address, message_param)           (segway_configure_load(socket, dest_address, 0x03, convert_to_ieee754(message_param)))
#define segway_configure_max_dtz(socket, dest_address, message_param)             (segway_configure_load(socket, dest_address, 0x04, convert_to_ieee754(message_param)))
#define segway_configure_coastdown(socket, dest_address, message_param)           (segway_configure_load(socket, dest_address, 0x05, convert_to_ieee754(message_param)))
#define segway_configure_max_turn_rate(socket, dest_address, message_param)       (segway_configure_load(socket, dest_address, 0x06, convert_to_ieee754(message_param)))
#define segway_configure_max_turn_accel(socket, dest_address, message_param)      (segway_configure_load(socket, dest_address, 0x07, convert_to_ieee754(message_param)))
#define segway_configure_tire_diameter(socket, dest_address, message_param)       (segway_configure_load(socket, dest_address, 0x08, convert_to_ieee754(message_param)))
#define segway_configure_wheel_track_width(socket, dest_address, message_param)   (segway_configure_load(socket, dest_address, 0x09, convert_to_ieee754(message_param)))
#define segway_configure_transmission_ratio(socket, dest_address, message_param)  (segway_configure_load(socket, dest_address, 0x0a, convert_to_ieee754(message_param)))
#define segway_configure_input_config_bitmap(socket, dest_address, message_param) (segway_configure_load(socket, dest_address, 0x0b, message_param))
#define segway_configure_eth_ip(socket, dest_address, message_param)              (segway_configure_load(socket, dest_address, 0x0c, message_param))
#define segway_configure_eth_port(socket, dest_address, message_param)            (segway_configure_load(socket, dest_address, 0x0d, message_param))
#define segway_configure_eth_subnet(socket, dest_address, message_param)          (segway_configure_load(socket, dest_address, 0x0e, message_param))
#define segway_configure_eth_gatway(socket, dest_address, message_param)          (segway_configure_load(socket, dest_address, 0x0f, message_param))
#define segway_configure_feedback_bitmap1(socket, dest_address, message_param)    (segway_configure_load(socket, dest_address, 0x10, message_param))
#define segway_configure_feedback_bitmap2(socket, dest_address, message_param)    (segway_configure_load(socket, dest_address, 0x11, message_param))
#define segway_configure_feedback_bitmap3(socket, dest_address, message_param)    (segway_configure_load(socket, dest_address, 0x12, message_param))
#define segway_configure_force_config_feedback_bitmap(socket, dest_address, message_param)      (segway_configure_load(socket, dest_address, 0x13, message_param))
#define segway_configure_audio_command(socket, dest_address, message_param)       (segway_configure_load(socket, dest_address, 0x14, message_param))
#define segway_configure_operational_mode(socket, dest_address, message_param)    (segway_configure_load(socket, dest_address, 0x15, message_param))
#define segway_configure_sp_faultlog(socket, dest_address, message_param)         (segway_configure_load(socket, dest_address, 0x16, message_param))
#define segway_configure_reset_integrator(socket, dest_address, message_param)    (segway_configure_load(socket, dest_address, 0x17, message_param))
#define segway_configure_reset_params_to_default(socket, dest_address, message_param)  (segway_configure_load(socket, dest_address, 0x18, message_param))

struct udp_frame
{
  __u16 udp_id;
  __u8 data[10];
};

extern int segway_read(int socket, struct udp_frame *frame, struct sockaddr_in *address);
extern int segway_check(int *socket);
extern int segway_motion_set(int socket, struct sockaddr_in *dest_address, float velocity, float yaw, int scale_value);
extern int segway_configure_load(int socket, struct sockaddr_in *dest_address, __u32 message_id, __u32 message_param);
extern void segway_config_update(struct udp_frame *frame);

#endif
