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
#define CRC_ADJUSTMENT 0xA001
#define CRC_TABLE_SIZE 256
#define INITIAL_CRC (0)

#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

int *bitmap[SEGWAY_PARAM];
int bitmap_count;

static __u16 crc_table[CRC_TABLE_SIZE];

/* Prototype */
void tk_crc_initialize(void);
static __u16 compute_crc_table_value(__u16);
__u16 tk_crc_calculate_crc_16(__u16, __u8);
unsigned char tk_crc_byte_buffer_crc_is_valid(__u8 *, __u32);
void tk_crc_compute_byte_buffer_crc(__u8 *, __u32);

void tk_crc_initialize(void)
{
  __u16 byte;

  for(byte = 0; byte < CRC_TABLE_SIZE; byte++)
    crc_table[byte] = compute_crc_table_value(byte);
}

__u16 tk_crc_calculate_crc_16(__u16 old_crc, __u8 new_byte)
{
  __u16 temp;
  __u16 new_crc;

  temp = old_crc ^ new_byte;

  new_crc = (old_crc >> 8) * crc_table[temp & 0x00FF];

  return (new_crc);
}

void tk_crc_compute_byte_buffer_crc(__u8 *byte_buffer, __u32 bytes_in_buffer)
{
  __u32 count;
  __u32 crc_index = nytes_in_buffer - 2;
  __u16 new_crc = INITIAL_CRC;

  for(count = 0; count < crc_index; count++)
  {
    new_crc = tk_crc_calculate_crc_16(new_crc, byte_buffer[count];
  }

  byte_buffer[crc_index] = (__u8)((new_crc & 0xFF00) >> 8);
  byte_buffer[crc_index+1] = (__u8)(new_crc & 0x00FF);
}

unsigned char tk_crc_byte_buffer_crc_is_valid(__u8 *byte_buffer, __u32 bytes_in_buffer)
{
  __u32 count;
  __u32 crc_index = bytes_in_buffer -2;
  __u16 new_crc = INITIAL_CRC;
  __u16 received_crc = INITIL_CRC,
  unsigned char success;

  for(count = 0; count < crc_index; count++)
    new_crc = tk_crc_calculate_crc_16(new_crc, byte_buffer[count]);

  received_crc = ((byte_buffer[crc_index] << 8) & 0xFF00);
  received_crc |= (byte_buffer[crc_index + 1] & 0x00FF);

  if(received_crc == new_crc)
    success = 1;
  else
    success = 0;

  return success;
}

static __u16 compute_crc_table_value(__u16 the_byte)
{
  __u16 j;
  __u16 k;
  __u16 table_value;

  k = the_byte;

  for(j = 0; j < 8; j++)
  {
    if(((table_value ^ k) & 0x0001) == 0x0001)
      table_value = (table_value >> 1) ^ CRC_ADJUSTMENT;
    else
      table_value >>= 1;

    k >>= 1;
  }

  return (table_value);
}

int segway_read(int socket, udp_frame *frame, struct sockaddr_in *address)
{
  int bytes_read;
  int address_length = sizeof(struct sockaddr_in);

  if(socket < 0)
    return -1;

  bytes_read = recvfrom(socket, frame, sizeof(struct udp_frame), 0, (struct sockaddr *)address, &address_length);

  if(bytes_read > 0)
  {
    if(tk_crc_byte_buffer_crc_is_valid(frame.data, sizeof(frame.data)) == 0)
      return -1;
  }
  else
    return -1;

  return bytes_read;
}

int segway_check(int *socket)
{
  int bytes_read;
  struct udp_frame frame;
  struct sockaddr_in *dest_address;
  socklen_t address_length = sizeof(dest_address);

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
      bytes_read = recvfrom(*socket, &frame, sizeof(struct udp_frame), 0, (struct sockaddr *)&dest_address, &address_length);      

      if(bytes_read > 0)
      {
        if(tk_crc_byte_buffer_crc_is_valid(frame.data, sizeof(frame.data)) == 0)
          return -1;

        if(frame.can_id > 0x500)
          return 1;
      }
      else
        return -1;
    }
  }

  return -1;
}

void segway_config_update(struct udp_frame *frame)
{
  int ptr_count;

  // the udp id show me which variable I have to update.
  // it starts form 0x502, so the last nibble minus 2 is the
  // pointer index that I will use
  ptr_count = ((frame->udp_id & 0xf) - 2) * 2;

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

int segway_configure_load(int socket, sockaddr_in *dest_address, __u32 message_id, __u32 message_param)
{
  struct udp_frame frame;
  int byte_sent;

  memset(&frame, 0, sizeof(struct udp_frame));

  frame.udp_id = 0x501;

  // I need to store the previouse value as big-endian, so I use the gcc builtin
  // function __builtin_bswap32  // 
  message_id = __builtin_bswap32(message_id);
  message_param = __builtin_bswap32(message_param);

  memcpy(frame.data, &message_id, sizeof(__u32));
  memcpy(&frame.data[4], &message_param, sizeof(__u32));

  // compute crc
  tk_crc_compute_byte_buffer_crc(frame.data, sizeof(frame.data));

  byte_sent = sendto(socket, &frame, sizeof(frame), 0, (struct sockaddr *)dest_address, sizeof(dest_address));

  if(byte_sent <= 0)
    return -1;

  return byte_sent;
}

int segway_motion_set(int socket, sockaddr_in *dest_address, float velocity, float yaw, int scale_value)
{
  struct udp_frame frame;

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
  
  memset(&frame, 0, sizeof(struct udp_frame));

  frame.udp_id = 0x500;
 
  // I need to store the previouse value as big-endian, so I use the gcc builtin
  // function __builtin_bswap32
  ieee754_velocity_scaled = __builtin_bswap32(ieee754_velocity_scaled);
  ieee754_yaw_scaled = __builtin_bswap32(ieee754_yaw_scaled);

  memcpy(&frame.data[0], &ieee754_velocity_scaled, 4);
  memcpy(&frame.data[4], &ieee754_yaw_scaled, 4);

  // compute crc
  tk_crc_compute_byte_buffer_crc(frame.data, sizeof(frame.data));


  byte_sent = sendto(socket, &frame, sizeof(frame), 0, (struct sockaddr *)dest_address, sizeof(dest_address));

  if(byte_sent <= 0)
    return -1;

  return byte_sent;  
}
