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
#include "segway_config.h"

#define SEGWAY_PARAM 85
#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

int *bitmap[SEGWAY_PARAM];
int bitmap_count;

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
