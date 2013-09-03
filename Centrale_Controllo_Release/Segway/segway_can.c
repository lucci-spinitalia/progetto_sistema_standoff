#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/select.h>
#include <linux/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include "segway_can.h"

#define SEGWAY_PARAM 85
#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

void *bitmap_ptr_all[SEGWAY_PARAM];
int *bitmap[SEGWAY_PARAM];
int bitmap_count;
__u32 bitmap1;
__u32 bitmap2;
__u32 bitmap3;

int segway_check(int *socket)
{
  int bytes_read;
  struct can_frame frame;
  int select_result = -1;
  int nfds = 0;
  fd_set rd;
  struct timeval select_timeout;

  FD_ZERO(&rd);
  select_timeout.tv_sec = 0;
  select_timeout.tv_usec = 100000;

  if(*socket > 0)
  {
    FD_SET(*socket, &rd);
    nfds = max(nfds, *socket);
  }

  segway_configure_none(*socket, 0x00);

  select_result = select(nfds + 1, &rd, NULL, NULL, &select_timeout);

  if(select_result == -1)
    return -1;

  if(*socket > 0)
  {
    if(FD_ISSET(*socket, &rd))
    {
      bytes_read = read(*socket, &frame, sizeof(struct can_frame));

      if(bytes_read > 0)
      {
        if(frame.can_id > 0x500)
          return 1;
      }
      else
        return -1;
    }
  }

  return -1;
}

void segway_config_update(struct can_frame *frame)
{
  int ptr_count;

  // the can index show me which variable I have to update.
  // it starts form 0x502, so the last nibble minus 2 is the
  // pointer index that I will use
  ptr_count = ((frame->can_id & 0xf) - 2) * 2;

  // The number of param to update is ptr_count + 1.
  if(bitmap_count < (ptr_count + 1))
    return;

  //printf("Bitmap count: %d\n", bitmap_count);
  //printf("%d <-> %08lx\n", ptr_count, bitmap[ptr_count]);
  
  *((__u32 *)bitmap[ptr_count]) = (((frame->data[0] << 24) & 0xFF000000) | ((frame->data[1] << 16) & 0x00FF0000) | ((frame->data[2] << 8) & 0x0000FF00) | (frame->data[3] & 0x000000FF));

  ptr_count++;
  
  if(bitmap_count < (ptr_count + 1))
    return;
  
  //printf("%d <-> %08lx\n", ptr_count, bitmap[ptr_count]);
  
  *((__u32 *)bitmap[ptr_count]) = (((frame->data[4] << 24) & 0xFF000000) | ((frame->data[5] << 16) & 0x00FF0000) | ((frame->data[6] << 8) & 0x0000FF00) | (frame->data[7] & 0x000000FF));

}


int segway_configure_load(int socket, __u32 message_id, __u32 message_param)
{
  struct can_frame frame;
  int byte_sent;

  memset(&frame, 0, sizeof(struct can_frame));

  frame.can_id = 0x501;
  frame.can_dlc = 8;

  // I need to store the previouse value as big-endian, so I use the gcc builtin
  // function __builtin_bswap32  // 
  message_id = __builtin_bswap32(message_id);
  message_param = __builtin_bswap32(message_param);

  memcpy(frame.data, &message_id, sizeof(__u32));
  memcpy(&frame.data[4], &message_param, sizeof(__u32));

  byte_sent = write(socket, &frame, sizeof(frame));

  usleep(50000);

  return byte_sent;
}

int segway_motion_set(int can_socket, float velocity, float yaw, int scale_value)
{
  struct can_frame frame;

  float velocity_scaled; 
  float yaw_scaled;

  __u32 ieee754_velocity_scaled;
  __u32 ieee754_yaw_scaled;

  int byte_sent;

  velocity_scaled = velocity / scale_value;

  // To reduce the sensitivity of the joystick
  velocity_scaled *= fabs(velocity_scaled);

  ieee754_velocity_scaled = (*(__u32*)&velocity_scaled);
  
  //printf("Velocity: %08lx <-> %.1f\n", ieee754_velocity_scaled, velocity_scaled);

  yaw_scaled = yaw / scale_value;

  // To reduce the sensitivity of the joystick
  yaw_scaled *= fabs(yaw_scaled);

  ieee754_yaw_scaled = (*(__u32*)&yaw_scaled);

  //printf("Yaw: %08lx <-> %.1f\n", ieee754_yaw_scaled, yaw_scaled);
  
  memset(&frame, 0, sizeof(struct can_frame));

  frame.can_id = 0x500;
  frame.can_dlc = 8;

  // I need to store the previouse value as big-endian, so I use the gcc builtin
  // function __builtin_bswap32
  ieee754_velocity_scaled = __builtin_bswap32(ieee754_velocity_scaled);
  ieee754_yaw_scaled = __builtin_bswap32(ieee754_yaw_scaled);

  memcpy(&frame.data[0], &ieee754_velocity_scaled, 4);
  memcpy(&frame.data[4], &ieee754_yaw_scaled, 4);

  byte_sent = write(can_socket, &frame, sizeof(frame));
  return byte_sent;
  
}

int segway_init(int *socket, struct segway_struct *segway_status)
{
  int byte_sent;
  int i;
  void *segway_status_ptr;

  bitmap1 = 0xFFFFFFFF;
  bitmap2 = 0xFFFFFFFF;
  bitmap3 = 0xFFFFFFFF;

  segway_status_ptr = segway_status;

  for(i = 0; i < SEGWAY_PARAM; i++)
  {
    bitmap_ptr_all[i] = segway_status_ptr;
    segway_status_ptr += sizeof(typeof(segway_status_ptr)) ;
  }

  // Check if segway is on
  if(segway_check(socket) < 1)
    return -1;

  //set configuration bitmap to none, so I don't receive all that messages during initialization
  byte_sent = segway_configure_feedback(*socket, NONE, NONE, NONE);

  if(byte_sent < 0)
    return -1;

  // Set max velocity
  byte_sent = segway_configure_max_vel(*socket, MAX_VELOCITY);

  if(byte_sent < 0)
    return -1;

  // Set max acceleration
  byte_sent = segway_configure_max_acc(*socket, MAX_ACCELERATION);

  if(byte_sent < 0)
    return -1;

  // Set max deceleration
  byte_sent = segway_configure_max_decel(*socket, MAX_DECELERATION);

  if(byte_sent < 0)
    return -1;

  // Set max acceleration
  byte_sent = segway_configure_max_dtz(*socket, MAX_DTZ_DECEL_RATE);

  if(byte_sent < 0)
    return -1;

  // Set costdown
  byte_sent = segway_configure_coastdown(*socket, COASTDOWN_ACCEL);

  if(byte_sent < 0)
    return -1;

  // Set max turn rate
  byte_sent = segway_configure_max_turn_rate(*socket, MAX_TURN_RATE);

  if(byte_sent < 0)
    return -1;

  // Set max turn accel
  byte_sent = segway_configure_max_turn_accel(*socket, MAX_TURN_ACCEL);

  if(byte_sent < 0)
    return -1;

  // Set tire diameter
  byte_sent = segway_configure_tire_diameter(*socket, TIRE_DIAMETER);

  if(byte_sent < 0)
    return -1;

  // Set wheel track width
  byte_sent = segway_configure_wheel_track_width(*socket, WHEEL_TRACK_WIDTH);

  if(byte_sent < 0)
    return -1;

  // Set max
  byte_sent = segway_configure_transmission_ratio(*socket, TRANSMISSION_RATIO);

  if(byte_sent < 0)
    return -1;

  // Enable feedback bitmap
  byte_sent = segway_configure_feedback(*socket, 
					FAULT_STATUS_WORD_1|FAULT_STATUS_WORD_2|FAULT_STATUS_WORD_3|FAULT_STATUS_WORD_4|MCU_0_FAULT_STATUS|
					MCU_1_FAULT_STATUS|MCU_2_FAULT_STATUS|MCU_3_FAULT_STATUS|OPERATIONAL_STATE|LINEAR_VEL_MPS,
					LINEAR_POS_M|FRONT_BASE_BATT_1_SOC|FRONT_BASE_BATT_2_SOC|REAR_BASE_BATT_1_SOC|REAR_BASE_BATT_2_SOC|
					FRONT_BASE_BATT_1_TEMP_DEGC|FRONT_BASE_BATT_2_TEMP_DEGC|REAR_BASE_BATT_1_TEMP_DEGC|REAR_BASE_BATT_2_TEMP_DEGC, 
					NONE);

  if(byte_sent < 0)
    return -1;

  return 0;
}

int segway_configure_feedback1(int socket, __u32 message_param)
{
  int i;

  bitmap1 = message_param;

  for(i = 0; i < 32; i++)
  {
    if((bitmap1 >> i) & 0x01)
    {
      bitmap[bitmap_count] = bitmap_ptr_all[i];
      bitmap_count++;
    }
  }

  return segway_configure_feedback_bitmap1(socket, message_param);
}


int segway_configure_feedback2(int socket,__u32 message_param)
{
  int i;

  bitmap2 = message_param;

  for(i = 32; i < 64; i++)
  {
    if((bitmap2 >> (i - 32)) & 0x01)
    {
      bitmap[bitmap_count] = bitmap_ptr_all[i];
      bitmap_count++;
    }
  }

  return segway_configure_feedback_bitmap2(socket, message_param);
}

int segway_configure_feedback3(int socket,__u32 message_param)
{
  int i;

  bitmap3 = message_param;
  
  for(i = 64; i < SEGWAY_PARAM; i++)
  {
    if((bitmap3 >> (i - 64)) & 0x01)
    {
      bitmap[bitmap_count] = bitmap_ptr_all[i];
      bitmap_count++;
    }
  }
  
  return segway_configure_feedback_bitmap3(socket, message_param);
}

int segway_configure_feedback(int socket, __u32 feedback1_param, __u32 feedback2_param, __u32 feedback3_param)
{
  if(segway_configure_feedback1(socket, feedback1_param) < 0)
    return -1;

  if(segway_configure_feedback2(socket, feedback2_param) < 0)
    return -1;

  if(segway_configure_feedback3(socket, feedback3_param) < 0)
    return -1;

  return 0;
}
