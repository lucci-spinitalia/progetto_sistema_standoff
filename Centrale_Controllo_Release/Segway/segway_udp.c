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
#include "segway_udp.h"

#define SEGWAY_PARAM 85
#define CRC_ADJUSTMENT 0xA001
#define CRC_TABLE_SIZE 256
#define INITIAL_CRC (0)

#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

__u32 bitmap1;
__u32 bitmap2;
__u32 bitmap3;

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

  new_crc = (old_crc >> 8) ^ crc_table[temp & 0x00FF];

  return (new_crc);
}

void tk_crc_compute_byte_buffer_crc(__u8 *byte_buffer, __u32 bytes_in_buffer)
{
  __u32 count;
  __u32 crc_index = bytes_in_buffer - 2;
  __u16 new_crc = INITIAL_CRC;

  for(count = 0; count < crc_index; count++)
  {
    new_crc = tk_crc_calculate_crc_16(new_crc, byte_buffer[count]);
  }

  byte_buffer[crc_index] = (__u8)((new_crc & 0xFF00) >> 8);
  byte_buffer[crc_index+1] = (__u8)(new_crc & 0x00FF);
}

unsigned char tk_crc_byte_buffer_crc_is_valid(__u8 *byte_buffer, __u32 bytes_in_buffer)
{
  __u32 count;
  __u32 crc_index = bytes_in_buffer -2;
  __u16 new_crc = INITIAL_CRC;
  __u16 received_crc = INITIAL_CRC;
  unsigned char success;

  for(count = 0; count < crc_index; count++)
  {
    new_crc = tk_crc_calculate_crc_16(new_crc, byte_buffer[count]);
  }

  received_crc = ((byte_buffer[crc_index] << 8) & 0xFF00);
  received_crc |= (byte_buffer[crc_index + 1] & 0x00FF);

  //printf("new_crc: %x\treceived_crc: %x\n", new_crc, received_crc);
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

  table_value = 0;

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

int segway_read(int socket, union segway_union *segway_status)
{
  int bytes_read;
  __u8 udfb_data[(SEGWAY_PARAM*4) + 3];

  if(socket < 0)
    return -1;

  bytes_read = recvfrom(socket, udfb_data, sizeof(udfb_data), 0, NULL, NULL);

  if(bytes_read > 0)
  {
    //int i = 0;
    segway_config_update(udfb_data, segway_status);

    //for(i = 0; i < bytes_read; i++)
    //printf("[%x]", udfb_data[i]);

    //printf("\n");
  }
  else
    return -1;

  return bytes_read;
}

void segway_config_update(__u8 *udfb_data, union segway_union *segway_status)
{
  int i;
  int count = 0;

  for(i = 0; i < 32; i++)
  {
    if((bitmap1 >> i) & 0x01)
    {
      segway_status->segway_feedback[i] = (((udfb_data[count*4] << 24) & 0xFF000000) | ((udfb_data[count*4 + 1] << 16) & 0x00FF0000) | ((udfb_data[count*4 + 2] << 8) & 0x0000FF00) | (udfb_data[count*4 + 3] & 0x000000FF));
      //printf("Sizeof(): %i Bitmap[%i]: %08lx\n", bitmap_count, i, segway_status->segway_feedback[i]);   
      count++;
    }
  }

  for(i = 32; i < 64; i++)
  {
    if((bitmap2 >> (i - 32)) & 0x01)
    {
      segway_status->segway_feedback[i] = (((udfb_data[count*4] << 24) & 0xFF000000) | ((udfb_data[count*4 + 1] << 16) & 0x00FF0000) | ((udfb_data[count*4 + 2] << 8) & 0x0000FF00) | (udfb_data[count*4 + 3] & 0x000000FF));
      //printf("Sizeof(): %i Bitmap[%i]: %08lx\n", bitmap_count, i, segway_status->segway_feedback[i]);   
      count++;
    }
  }

  for(i = 64; i < SEGWAY_PARAM; i++)
  {
    if((bitmap3 >> (i - 64)) & 0x01)
    {
      segway_status->segway_feedback[i] = (((udfb_data[count*4] << 24) & 0xFF000000) | ((udfb_data[count*4 + 1] << 16) & 0x00FF0000) | ((udfb_data[count*4 + 2] << 8) & 0x0000FF00) | (udfb_data[count*4 + 3] & 0x000000FF));
      //printf("Sizeof(): %i Bitmap[%i]: %08lx\n", bitmap_count, i, segway_status->segway_feedback[i]);   
      count++;
    }
  }
}

int segway_configure_load(int socket, struct sockaddr_in *dest_address, __u32 message_id, __u32 message_param)
{
  struct udp_frame frame;
  int byte_sent = 0;
  
  memset(&frame, 0, sizeof(struct udp_frame));

  //This address is 0x501 in big-endian
  frame.param.udp_id = 0x0105;

  // I need to store the previouse value as big-endian, so I use the gcc builtin
  // function __builtin_bswap32  // 
  message_id = __builtin_bswap32(message_id);
  message_param = __builtin_bswap32(message_param);

  memcpy(frame.param.data, &message_id, sizeof(__u32));
  memcpy(&frame.param.data[4], &message_param, sizeof(__u32));

  // compute crc
  tk_crc_compute_byte_buffer_crc(frame.frame, sizeof(frame.frame));

  byte_sent = sendto(socket, &frame, sizeof(frame), 0, (struct sockaddr *)dest_address, sizeof(*dest_address));

  return byte_sent; 
}

int segway_motion_set(int socket, struct sockaddr_in *dest_address, float velocity, float yaw, int scale_value)
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

  ieee754_velocity_scaled = convert_to_ieee754(velocity_scaled);
  
//  printf("Velocity: %08lx <-> %.1f\n", ieee754_velocity_scaled, velocity_scaled);

  yaw_scaled = yaw / scale_value;

  // To reduce the sensitivity of the joystick
  yaw_scaled *= fabs(yaw_scaled);

  ieee754_yaw_scaled = convert_to_ieee754(yaw_scaled);

//  printf("Yaw: %08lx <-> %.1f\n", ieee754_yaw_scaled, yaw_scaled);
  
  memset(&frame, 0, sizeof(struct udp_frame));

  // The address is 0x500 in big-endian
  frame.param.udp_id = 0x005; //little-endian
 
  // I need to store the previouse value as big-endian, so I use the gcc builtin
  // function __builtin_bswap32
  ieee754_velocity_scaled = __builtin_bswap32(ieee754_velocity_scaled);
  ieee754_yaw_scaled = __builtin_bswap32(ieee754_yaw_scaled);

  memcpy(&frame.param.data[0], &ieee754_velocity_scaled, 4);
  memcpy(&frame.param.data[4], &ieee754_yaw_scaled, 4);

  // compute crc
  tk_crc_compute_byte_buffer_crc(frame.frame, sizeof(frame.frame));

  byte_sent = sendto(socket, &frame, sizeof(frame), 0, (struct sockaddr *)dest_address, sizeof(*dest_address));

  if(byte_sent <= 0)
    return -1;

  return byte_sent;  
}

int segway_init(int socket, struct sockaddr_in *address, union segway_union *segway_status)
{
  int byte_sent = 0;

  bitmap1 = 0xFFFFFFFF;
  bitmap2 = 0xFFFFFFFF;
  bitmap3 = 0xFFFFFFFF;

  tk_crc_initialize();

  //set configuration bitmap to none, so I don't receive all that messages during initialization
  //printf("segway_configure_feedback null\n");
  byte_sent = segway_configure_feedback(socket, address, NONE, NONE, NONE);

  if(byte_sent <= 0)
    return -1;

  usleep(20000);

  // Set max velocity
  //printf("segway_configure_max_vel\n");
  byte_sent = segway_configure_max_vel(socket, address, 1/*MAX_VELOCITY*/);

  if(byte_sent <= 0)
    return -1;

  usleep(20000);
 
  // Set max acceleration
  //printf("segway_configure_max_acc\n");
  byte_sent = segway_configure_max_acc(socket, address, 1/*MAX_ACCELERATION*/);

  if(byte_sent <= 0)
    return -1;

  usleep(20000);

  // Set max deceleration
  //printf("segway_configure_max_decel\n");
  byte_sent = segway_configure_max_decel(socket, address, 1/*MAX_DECELERATION*/);

  if(byte_sent <= 0)
    return -1;

  usleep(20000);
    
  // Set max acceleration
  //printf("segway_configure_max_dtz\n");
  byte_sent = segway_configure_max_dtz(socket, address, 1/*MAX_DTZ_DECEL_RATE*/);

  if(byte_sent <= 0)
    return -1;

  usleep(20000);

  // Set costdown
  //printf("segway_configure_coastdown\n");
  byte_sent = segway_configure_coastdown(socket, address, 1/*COASTDOWN_ACCEL*/);

  if(byte_sent <= 0)
    return -1;

  usleep(20000);
    
  // Set max turn rate
  //printf("segway_configure_max_turn_rate\n");
  byte_sent = segway_configure_max_turn_rate(socket, address, 1/*MAX_TURN_RATE*/);

  if(byte_sent <= 0)
    return -1;

  usleep(20000);

  // Set max turn accel
  //printf("segway_configure_max_turn_accel\n");
  byte_sent = segway_configure_max_turn_accel(socket, address, 1/*MAX_TURN_ACCEL*/);

  if(byte_sent <= 0)
    return -1;

  usleep(20000);

  // Set tire diameter
  //printf("segway_configure_tire_diameter\n");
  byte_sent = segway_configure_tire_diameter(socket, address, TIRE_DIAMETER);

  if(byte_sent <= 0)
    return -1;

  usleep(20000);

  // Set wheel track width
  //printf("segway_configure_wheel_track\n");
  byte_sent = segway_configure_wheel_track_width(socket, address, WHEEL_TRACK_WIDTH);

  if(byte_sent <= 0)
    return -1;

  usleep(20000);

  // Set max
  //printf("segway_configure_ransmission_ratio\n");
  byte_sent = segway_configure_transmission_ratio(socket, address, TRANSMISSION_RATIO);

  if(byte_sent <= 0)
    return -1;

  usleep(20000);

  // Enable feedback bitmap
  //printf("segway_configure_feedback enable\n");
  byte_sent = segway_configure_feedback(socket, address,
					FAULT_STATUS_WORD_1|FAULT_STATUS_WORD_2|FAULT_STATUS_WORD_3|FAULT_STATUS_WORD_4|MCU_0_FAULT_STATUS|
					MCU_1_FAULT_STATUS|MCU_2_FAULT_STATUS|MCU_3_FAULT_STATUS|OPERATIONAL_STATE|LINEAR_VEL_MPS,
					LINEAR_POS_M|FRONT_BASE_BATT_1_SOC|FRONT_BASE_BATT_2_SOC|REAR_BASE_BATT_1_SOC|REAR_BASE_BATT_2_SOC|
					FRONT_BASE_BATT_1_TEMP_DEGC|FRONT_BASE_BATT_2_TEMP_DEGC|REAR_BASE_BATT_1_TEMP_DEGC|REAR_BASE_BATT_2_TEMP_DEGC, 
					NONE);

  if(byte_sent <= 0)
    return -1;

  return 1;
}

int segway_configure_feedback1(int socket, struct sockaddr_in *address, __u32 message_param)
{
  bitmap1 = message_param;

  //printf("Configure feedback1: bitmap1:  %i  bitmap_count: %i\n", bitmap1, bitmap_count);
  return segway_configure_feedback_bitmap1(socket, address, message_param);
}


int segway_configure_feedback2(int socket, struct sockaddr_in *address, __u32 message_param)
{
  bitmap2 = message_param;

  return segway_configure_feedback_bitmap2(socket, address, message_param);
}

int segway_configure_feedback3(int socket, struct sockaddr_in *address, __u32 message_param)
{
  bitmap3 = message_param;
  return segway_configure_feedback_bitmap3(socket, address, message_param);
}

int segway_configure_feedback(int socket, struct sockaddr_in *address, __u32 feedback1_param, __u32 feedback2_param, __u32 feedback3_param)
{ 
  if(segway_configure_feedback1(socket, address, feedback1_param) < 0)
    return -1;

  if(segway_configure_feedback2(socket, address, feedback2_param) < 0)
    return -1;

  if(segway_configure_feedback3(socket, address, feedback3_param) < 0)
    return -1;

  return 1;
}

