#include <time.h>
#include <sys/time.h>

#include <sys/types.h> 
#include <sys/socket.h> 
#include <sys/ioctl.h> 

#include <net/if.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h>

#include <errno.h>

#include <locale.h>

#include <string.h>
#include <math.h>

#include <netinet/in.h> 
#include <arpa/inet.h>

#include <signal.h>

#include <termios.h>

#include <sys/stat.h>
#include <fcntl.h>

#include "segway_config.h" 
#include "segway_udp_v2.h" 
#include "arm_udp.h"
#include "arm_rs485.h"
#include "nmea/nmea.h"
#include "rover_rtb.h"
#include "gps_generate.h"
#include "rs232.h"
#include "lms511_tcp.h"
//#include "kalman.h"

#define LMS511
//#define LMS511_DEBUG
#define GPS
//#define GPS_DEBUG
//#define GPS_SIMUL

/* Gpio */
#define AIR_PUMP_GPIO 89
#define CCU_ADDRESS "192.168.1.102"
#define CCU_PORT_ARM 8000
#define CCU_PORT_GPS 8001

/* Segway */
#define SEGWAY_ADDRESS "192.168.1.40"
#define SEGWAY_PORT 8080
#define SEGWAY_INACTIVITY_TIMEOUT_US 1000000

/* Arm */
#define ARM_ADDRESS "192.168.1.28"
#define ARM_PORT 8012
#define ARM_DEVICE_FILE "/dev/ttyUSB0"
#define ARM_MAX_POSITION 0.1 // value in degree under which the link will be stopped

#define SCU_STATUS_PORT 8016
#define SCU_SEGWAY_FROM_JOYSTICK_PORT 8017

#define ARM_ERROR_POSITION 100

/* LMS511 */
#ifdef LMS511
#define LMS511_ADDRESS "192.168.1.104"
#define LMS511_PORT 2111
#define LMS511_MIN_DIST 1000 // in mm
#define TIMEOUT_USEC_LMS511 250000
#endif

/* Return to Base */
#define RTB_CONTROL_ANGLE_MAX 45
#define RTB_CONTROL_ANGLE_MIN 20
#define RTB_CONTRO_MIN_SPEED 0.5  // value from 0 to 1
#define RTB_CONTROL_ANGLE_PROP_GAIN 0.4
#define RTB_CONTROL_SPEED_GAIN 1

#define JOY_MAX_VALUE 32767 

#define TIMEOUT_SEC 0 
#define ARM_TIMEOUT_USEC 10000
#define AUTOMOTION_TIMEOUT_USEC 50000
#define STOP_TIMEOUT_USEC 50000
#define RETURN_TO_BASE_TIMEOUT_SEC 300
#define RETURN_TO_BASE_CTRL_TIMEOUT_USEC 100000
#define REQUEST_TIMEOUT_USEC 80000  // è il valore più vicion a AUTOMOVE_TIMEOUT_USEC / 7 o INIT_POSITION_TIMEOUT_USEC / 7
#define AUTOMOVE_TIMEOUT_USEC 500000
#define HOMING_TIMEOUT_USEC 300000
#define ARM_POSITION_TIMEOUT_USEC 100000
#define ARM_BATTERY_TIMEOUT_SEC 1
#define ARM_BATTERY_FILTER_COUNT 20
#define AIRPUMP_TIMEOUT_SEC 20
#define INIT_POSITION_TIMEOUT_USEC 500000
#define SEGWAY_TIMEOUT_USEC 100000
#define GPS_FIX_TIMEOUT_SEC 20

#define GPS_INDEX 0

/* The joint_yz_step is the incremental value to add to y and z coordinates.
 If a joystick message arrive every 50ms and I want make 1 meter every 30 seconds
 then the value have to be:
 value = (1 meter / 30 seconds) * 50 ms = 0.033 * 50ms = 1.67 * 10^-3
 */

// Standoff Control Unit scu_state
#define SCU_ARM_IDLE 0
#define SCU_ARM_REST 1
#define SCU_ARM_MOVE 2
#define SCU_ARM_STOP 3
#define SCU_ARM_AUTO_MOVE 4
#define SCU_ARM_AUTO_MOVE_ABORT 5
#define SCU_ARM_END_EFFECTOR_1 6
#define SCU_ARM_END_EFFECTOR_2 7
#define SCU_ARM_END_EFFECTOR_3 8
#define SCU_ARM_END_EFFECTOR_4 9
#define SCU_ARM_END_EFFECTOR_5 10
#define SCU_ARM_END_EFFECTOR_6 11
#define SCU_ARM_END_EFFECTOR_7 12
#define SCU_ARM_OUT_OF_SERVICE 13
#define SCU_ARM_UNKNOWN 14

/* SCU Request */
#define SCU_RQST_STATE 0
#define SCU_RQST_RTB 1
#define SCU_RQST_RESET_BASE 2
#define SCU_RQST_LASER_ON 3
#define SCU_RQST_LASER_OFF 4

#define ARM_JOINT_YZ_STEP_M 0.0009

// limiti di giunto in radianti
float arm_upper_limit[] =
{
0.94, 2.46, 2.51, 3.14, 1.83, 5.18, 4
};
float arm_lower_limit[] =
{
-3.5, -0.84, -2.59, -3.14, -1.55, -5.18, -1
};

/* Macro */
#undef max 
#define max(x,y) ((x) > (y) ? (x) : (y))

#undef min
#define min(x,y) ((x) < (y) ? (x) : (y))

/* Prototype */
int eth_check_connection();
int gpio_export(int pin_number);
int gpio_set_value(int pin_number, int value);
int gpio_set_direction(int pin_number, int value);
int gpio_generic_set_value(char *path, int value);
void arm_status_update(unsigned char *arm_state, unsigned char *arm_next_state,
    unsigned char *arm_prev_state, struct arm_frame arm_message);
void segway_status_update(union segway_union *, int, struct sockaddr_in *, long int);
void gps_compare_log(double latitude, double longitude, double latitude_odometry,
    double longitude_odometry, double velocity, double PDOP, double HDOP, double vdop,
    int sat_inview, int sat_used, double direction, double direction_bussola, long time_us);

void gps_text_log(char *string);
int generate_path(int free_slots, char *motion_file);

/* Gps */
#ifdef GPS
// Gps rs232 device
int gps_device = -1;

nmeaINFO info;
#endif

/* Status client interface*/
int scu_state_socket = -1;
unsigned char scu_state_rqst_flag = 0;

/* Robotic Arm Client*/
int arm_udp_device = -1;
int arm_rs485_device = -1;
unsigned char automove_timer_flag = 0;
char automove_file[100];
char *automove_file_ptr;
unsigned char homing_timer_flag = 0;
unsigned char stop_timer_flag = 0;
int arm_query_link = -1;
int actuator_wait = 0;

char uart_token[] =
{
'\r', '\n'
};

char gps_token[] =
{
'\n'
};

char arm_table_query = 0;
char arm_table_running = 0;

/* Arm battery interface */
int arm_battery_fd = -1;
FILE *arm_battery_file = NULL;

/* Segway */
int segway_socket = -1;

/* Segway info from joystick */
int segway_from_joystick_socket = -1;

/* Air pump */
int airpump_enable_flag = -1;

/* CCU interface */
int ccu_socket = -1;

/* Return to Base */
extern RTB_status RTBstatus;

/* LMS511 */
int lms511_socket = -1;
struct sockaddr_in lms511_address;

// Gps socket to communicate with CCU
int gps_socket = -1;

/* Generic */
unsigned char show_arm_state_flag = 0;
unsigned char show_arm_position_flag = 0;
unsigned char show_arm_ayz_init_flag = 0;
unsigned char arm_out_of_service_enable_flag = 1;
unsigned char laser_enable_flag = 1;
unsigned char show_laser_data_flag = 0;
unsigned char show_rtb_state_flag = 0;
unsigned char scu_rqst_rtb_flag = 0;
unsigned char show_gps_state_flag = 0;
unsigned char gps_log_flag = 0;
unsigned char gps_enable_flag = 1;

float arm_x, arm_y, arm_z;
long motor_step[MOTOR_NUMBER];

char *translate_state(int state_id)
{
  switch(state_id)
  {
    case SCU_ARM_IDLE:
      return "SCU_ARM_IDLE";
      break;

    case SCU_ARM_REST:
      return "SCU_ARM_REST";
      break;

    case SCU_ARM_MOVE:
      return "SCU_ARM_MOVE";
      break;

    case SCU_ARM_STOP:
      return "SCU_ARM_STOP";
      break;

    case SCU_ARM_AUTO_MOVE:
      return "SCU_ARM_AUTO_MOVE";
      break;

    case SCU_ARM_AUTO_MOVE_ABORT:
      return "SCU_ARM_AUTO_MOVE_ABORT";
      break;

    case SCU_ARM_END_EFFECTOR_1:
      return "SCU_ARM_END_EFFECTOR_1";
      break;

    case SCU_ARM_END_EFFECTOR_2:
      return "SCU_ARM_END_EFFECTOR_2";
      break;

    case SCU_ARM_END_EFFECTOR_3:
      return "SCU_ARM_END_EFFECTOR_3";
      break;

    case SCU_ARM_END_EFFECTOR_4:
      return "SCU_ARM_END_EFFECTOR_4";
      break;

    case SCU_ARM_END_EFFECTOR_5:
      return "SCU_ARM_END_EFFECTOR_5";
      break;

    case SCU_ARM_END_EFFECTOR_6:
      return "SCU_ARM_END_EFFECTOR_6";
      break;

    case SCU_ARM_END_EFFECTOR_7:
      return "SCU_ARM_END_EFFECTOR_7";
      break;

    case SCU_ARM_OUT_OF_SERVICE:
      return "SCU_ARM_OUT_OF_SERVICE";
      break;

    case SCU_ARM_UNKNOWN:
      return "SCU_ARM_UNKNOWN";
      break;

    default:
      return "SCU_ARM_INVALID";
      break;
  }

  return "\0";
}

void signal_handler(int signum)
{
  // Garbage collection
  printf("Terminating program...\n");

  if(gps_enable_flag)
    close(gps_device);

  close(gps_socket);
  close(scu_state_socket);
  close(arm_rs485_device);
  close(arm_udp_device);

  if(arm_battery_file > 0)
    fclose(arm_battery_file);

  close(arm_battery_fd);
  close(segway_socket);
  close(segway_from_joystick_socket);
  close(ccu_socket);
  RTB_internal_clean_cache();
  lms511_dispose();
  close(lms511_socket);
  exit(signum);
}

int init_interpolation_mode(char *motion_file)
{
  int i;
  struct arm_rs485_frame buffer;
  int time = 8000;

  arm_stop(0);
  if(arm_set_command(0, "VT", 0) <= 0) //set velocity target
    return -1;

  if(arm_set_command_without_value(0, "ZS") <= 0)	//Clear faults
    return -1;

  if(arm_set_command_without_value(0, "MD") <= 0) // active settings
    return -1;

  if(arm_set_command_without_value(0, "MDCFG(1)") <= 0) // active settings
    return -1;

  if(arm_set_command_without_value(SMART_MOTOR_SYNC_INDEX, "MDCFG(3)") <= 0) // active settings
    return -1;

  //if(arm_set_command_without_value(0, "Q1") <= 0) // active settings
  //  return -1;

  for(i = 0; i < SMART_MOTOR_NUMBER; i++)
  {
    buffer.arm_command_param.index = i + 1;
    buffer.arm_command_param.request_position = 0;
    buffer.arm_command_param.request_trajectory_status = 0;
    buffer.arm_command_param.request_interpolation_status = 0;
    buffer.arm_command_param.request_error_status = 0;

    buffer.arm_command_param.command[0] = buffer.arm_command_param.index + 128;
    buffer.arm_command_param.command[1] = 0xfb;
    buffer.arm_command_param.command[2] = time & 0xff;
    buffer.arm_command_param.command[3] = (time >> 8) & 0xff;
    buffer.arm_command_param.command[4] = 0;
    arm_rs485_load_tx(buffer);

    buffer.arm_command_param.command[1] = 0xfa;
    buffer.arm_command_param.command[2] = arm_link[i].actual_position & 0xFF;
    buffer.arm_command_param.command[3] = (arm_link[i].actual_position >> 8) & 0xFF;
    buffer.arm_command_param.command[4] = (arm_link[i].actual_position >> 16) & 0xFF;
    buffer.arm_command_param.command[5] = (arm_link[i].actual_position >> 24) & 0xFF;
    buffer.arm_command_param.command[6] = 0;
    arm_rs485_load_tx(buffer);
  }

  /*if(arm_set_command_without_value(0, "RAC") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RAMPS") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RAT") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RBa") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RBe") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RBh") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RBk") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RBo") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RBs") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RBt") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RBv") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RDEA") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RDEL") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RDT") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RMODE") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "REL") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "REPTR") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RFW") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RKA") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RKC") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RKCS") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RKD") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RKG") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RKI") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RKL") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RKP") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RKS") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RKV") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RMODE") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RPA") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RPC") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RPT") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RT") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RVA") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RVC") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RVL") <= 0) // active settings
   return -1;
   if(arm_set_command_without_value(0, "RVT") <= 0) // active settings
   return -1;*/

  if(arm_set_command_without_value(0, "G") <= 0)
    return -1;

  arm_table_query = 0;
  arm_table_running = 0;

  // Inizializzo il file
  strcpy(automove_file, motion_file);
  automove_file_ptr = motion_file;

  return 0;
}

int interpolation_status_request(int link_to_query)
{
  int automatic_query = 0;

  if(link_to_query > 0)
  {
    arm_query_position(link_to_query);

    // If it's a smartmotor
    if((link_to_query <= MOTOR_NUMBER) && (link_to_query != 7))
      arm_set_command_without_value(link_to_query, "Q");
  }
  else
  {
    arm_query_position(0);

    for(automatic_query = 1; automatic_query <= SMART_MOTOR_NUMBER; automatic_query++)
      arm_set_command_without_value(automatic_query, "Q");
  }

  return 1;
}

float calc_max_distance(long position_target[], long position_last[], long arm_gear[],
    int *max_distance_index)
{
  int motor_num = 0;
  float max_distance;
  float distance;
  const float arm_encoder_factor = 0.09; // 360 / 4000

  max_distance = max(
      fabs((position_target[0] - position_last[0]) * arm_encoder_factor / arm_gear[0]),
      fabs((position_target[1] - position_last[1]) * arm_encoder_factor / arm_gear[1]));

  if(fabs((position_target[0] - position_last[0]) * arm_encoder_factor / arm_gear[0])
      > fabs((position_target[1] - position_last[1]) * arm_encoder_factor / arm_gear[1]))
    *max_distance_index = 0;
  else
    *max_distance_index = 1;

  distance = max_distance;

  // trovo chi deve percorrere più step, ovvero chi ci metterà di più a finire il  movimento considerando la stessa
  // velocità per tutti
  for(motor_num = 2; motor_num < SMART_MOTOR_NUMBER; motor_num++)
  {
    max_distance = max(
        fabs(
            (position_target[motor_num] - position_last[motor_num]) * arm_encoder_factor
                / arm_gear[motor_num]), max_distance);

    if(max_distance > distance)
    {
      distance = max_distance;
      *max_distance_index = motor_num;
    }
  }

  return max_distance;
}

// genera il percorso da inviare agli interpolatori smartmotor leggendo
// una sola riga alla volta.
int generate_path(int free_slots, char *motion_file)
{
  int automatic_motion_flag = 0;
  long time = 8000; // diviso 8000 da il tempo in secondi
  int max_vel = 15; // grad/s riferita alla velocità di giunto. Per tradurlo in velocità motore, bisognerebbe applicare il fattore di riduzione
  int motor_num = 0;
  float max_distance;
  int max_distance_index;
  int slots;
  static long position_last[SMART_MOTOR_NUMBER];
  long position_target[SMART_MOTOR_NUMBER];
  long position_gear[SMART_MOTOR_NUMBER];
  float time_slice;
  const int max_slots = 43;
  int direction[SMART_MOTOR_NUMBER];

  struct arm_rs485_frame buffer;

  if(free_slots == 0)
    return 0;
  else if(free_slots > 0)
  {
    automatic_motion_flag = arm_automatic_motion_xyz_start(motion_file);

    if(automatic_motion_flag <= 0)
      return -2;
  }

  if(((arm_link[6].actual_position != arm_link[6].position_target)
      || (arm_link[6].trajectory_status != 0))
      && (arm_link[6].timeout_counter < LINK_TIMEOUT_LIMIT))
  {
    if(arm_link[6].trajectory_status == 0)
    {
      if(motion_file != NULL)
        actuator_wait = 2;
      else
      {
        if(actuator_wait == 0)
        {
          actuator_wait = 1;

          for(motor_num = 0; motor_num < SMART_MOTOR_NUMBER; motor_num++)
          {
            buffer.arm_command_param.index = motor_num + 1;
            buffer.arm_command_param.request_position = 0;
            buffer.arm_command_param.request_trajectory_status = 0;
            buffer.arm_command_param.request_interpolation_status = 0;
            buffer.arm_command_param.request_error_status = 0;

            buffer.arm_command_param.command[0] = buffer.arm_command_param.index + 128;
            buffer.arm_command_param.command[1] = 0xfb;
            buffer.arm_command_param.command[2] = 0;
            buffer.arm_command_param.command[3] = 0;
            buffer.arm_command_param.command[4] = 0;
            arm_rs485_load_tx(buffer);

            buffer.arm_command_param.command[1] = 0xfa;
            buffer.arm_command_param.command[2] = position_last[motor_num] & 0xFF;
            buffer.arm_command_param.command[3] = (position_last[motor_num] >> 8) & 0xFF;
            buffer.arm_command_param.command[4] = (position_last[motor_num] >> 16) & 0xFF;
            buffer.arm_command_param.command[5] = (position_last[motor_num] >> 24) & 0xFF;
            buffer.arm_command_param.command[6] = 0;
            arm_rs485_load_tx(buffer);
          }
        }
      }

      int check_run = 0;
      for(motor_num = 0; motor_num < SMART_MOTOR_NUMBER; motor_num++)
        check_run |= arm_link[motor_num].md_running;

      if(check_run == 0)
      {
        arm_set_command(7, "PT", arm_link[6].position_target);
        arm_set_command_without_value(7, "G");
      }
    }

    arm_query_position(7);
    arm_query_trajectory(7);

    return 0;
  }

  if(actuator_wait == 1)
  {
    // faccio ripartire i motori inizializzandoli alla posizione attuale
    for(motor_num = 0; motor_num < SMART_MOTOR_NUMBER; motor_num++)
    {
      buffer.arm_command_param.index = motor_num + 1;
      buffer.arm_command_param.request_position = 0;
      buffer.arm_command_param.request_trajectory_status = 0;
      buffer.arm_command_param.request_interpolation_status = 0;
      buffer.arm_command_param.request_error_status = 0;

      buffer.arm_command_param.command[0] = buffer.arm_command_param.index + 128;
      buffer.arm_command_param.command[1] = 0xfb;
      buffer.arm_command_param.command[2] = time & 0xff;
      buffer.arm_command_param.command[3] = (time >> 8) & 0xff;
      buffer.arm_command_param.command[4] = 0;
      arm_rs485_load_tx(buffer);

      buffer.arm_command_param.command[1] = 0xfa;
      buffer.arm_command_param.command[2] = arm_link[motor_num].actual_position & 0xFF;
      buffer.arm_command_param.command[3] = (arm_link[motor_num].actual_position >> 8) & 0xFF;
      buffer.arm_command_param.command[4] = (arm_link[motor_num].actual_position >> 16) & 0xFF;
      buffer.arm_command_param.command[5] = (arm_link[motor_num].actual_position >> 24) & 0xFF;
      buffer.arm_command_param.command[6] = 0;
      arm_rs485_load_tx(buffer);

      if(arm_set_command_without_value(0, "G") <= 0)
        return -1;

      free_slots--;
    }
  }

  //printf("---------------- GENERATE PATH %d-----------------\n", free_slots);

  if((motion_file != NULL) || (actuator_wait == 2))
  {
    for(motor_num = 0; motor_num < SMART_MOTOR_NUMBER; motor_num++)
      position_last[motor_num] = arm_link[motor_num].actual_position;
  }

  for(slots = 0; slots < free_slots; slots++)
  {
    for(motor_num = 0; motor_num < SMART_MOTOR_NUMBER; motor_num++)
    {
      position_target[motor_num] = arm_link[motor_num].position_target;
      position_gear[motor_num] = arm_link[motor_num].gear;

      if(position_target[motor_num] < position_last[motor_num])
        direction[motor_num] = -1;
      else
        direction[motor_num] = 1;

      /*printf("[%d] Actual position: %ld target position %ld last position %ld direction %d\n", motor_num,
       arm_link[motor_num].actual_position,
       position_target[motor_num],
       position_last[motor_num],
       direction[motor_num]);*/
    }

    max_distance = calc_max_distance(position_target, position_last, position_gear,
        &max_distance_index);

    // calcolo il tempo di percorrenza maggiore alla velocità massima
    time = max_distance * 8000 / max_vel;

    //printf("position target[%d]: %ld, position last: %ld\n", max_distance_index, position_target[max_distance_index], position_last[max_distance_index]);
    //printf("max_distance[%d]: %f, time: %ld\n", max_distance_index, max_distance, time);

    // Se la divisione precedente è minore di 1, allora devo compensare la mancanza di
    // precisione
    if(time == 0)
      time = 1;

    time_slice = (float) time / 8000;

    if(time_slice > max_slots)
    {
      //printf("Non ci sono abbastanza slot per generare la tragliettoria!\n");
      return 0;
    }

    if(time_slice > (free_slots - slots))
    {
      //printf("Troppi pochi slots: %d...riprova più in là!\n", (free_slots - slots));
      break;
    }

    // aggiorno il puntatore al file
    arm_automatic_motion_xyz_update_cursor();

    while(time_slice > 1)
    {
      // calcolo il nuovo pezzo di tragliettoria in modo che il motore
      // ci impieghi massimo 8 secondi (valore limite della tabella)
      for(motor_num = 0; motor_num < SMART_MOTOR_NUMBER; motor_num++)
      {
        if(motor_num == max_distance_index)
          position_target[motor_num] = (position_last[motor_num]
              + direction[motor_num] * (time / time_slice) * max_vel * arm_link[motor_num].gear
                  / 720);
        else
          position_target[motor_num] = position_last[motor_num]
              + (position_target[motor_num] - position_last[motor_num]) / time_slice;

        /*printf("[%d] Actual position: %ld target position %ld last position %ld\n", motor_num,
         arm_link[motor_num].actual_position,
         position_target[motor_num],
         position_last[motor_num]);*/
      }

      // ricalcolo i nuovi tempi dopo aver aggiustato le cose
      // in teoria questo passaggio potrebbe essere saltato in quanto il tempo sarà sicuramente
      // 65535, ovvero il limite massimo consentito. Questo perchè ho calcolato la distanza in
      // modo da rientrare nel suddetto intervallo di tempo
      max_distance = calc_max_distance(position_target, position_last, position_gear,
          &max_distance_index);
      time = max_distance * 8000 / max_vel;
      //printf("slice max_distance[%d]: %ld, time: %d\n", max_distance_index, max_distance, time);

      // scrivo le nuove posizioni in tabella
      for(motor_num = 0; motor_num < SMART_MOTOR_NUMBER; motor_num++)
      {
        buffer.arm_command_param.index = motor_num + 1;
        buffer.arm_command_param.request_position = 0;
        buffer.arm_command_param.request_trajectory_status = 0;
        buffer.arm_command_param.request_interpolation_status = 0;
        buffer.arm_command_param.request_error_status = 0;

        buffer.arm_command_param.command[0] = buffer.arm_command_param.index + 128;
        buffer.arm_command_param.command[1] = 0xfb;
        buffer.arm_command_param.command[2] = time & 0xff;
        buffer.arm_command_param.command[3] = (time >> 8) & 0xff;
        buffer.arm_command_param.command[4] = 0;
        arm_rs485_load_tx(buffer);

        buffer.arm_command_param.command[1] = 0xfa;
        buffer.arm_command_param.command[2] = position_target[motor_num] & 0xFF;
        buffer.arm_command_param.command[3] = (position_target[motor_num] >> 8) & 0xFF;
        buffer.arm_command_param.command[4] = (position_target[motor_num] >> 16) & 0xFF;
        buffer.arm_command_param.command[5] = (position_target[motor_num] >> 24) & 0xFF;
        buffer.arm_command_param.command[6] = 0;
        arm_rs485_load_tx(buffer);

        position_last[motor_num] = position_target[motor_num];
        position_target[motor_num] = arm_link[motor_num].position_target;

        //printf("position last[%d] %ld\n", motor_num, position_last[motor_num]);
      }

      // aggiorno gli slot scritti
      slots++;

      // dentro il precedente for ho spostato le posiizoni target ed aggiornato quelle precedenti,
      // quindi devo rifare le stesse considerazioni per le traglietorie lunghe
      max_distance = calc_max_distance(position_target, position_last, position_gear,
          &max_distance_index);
      time = max_distance * 8000 / max_vel;

      time_slice = (float) time / 8000;
    }

    for(motor_num = 0; motor_num < SMART_MOTOR_NUMBER; motor_num++)
    {
      buffer.arm_command_param.index = motor_num + 1;
      buffer.arm_command_param.request_position = 0;
      buffer.arm_command_param.request_trajectory_status = 0;
      buffer.arm_command_param.request_interpolation_status = 0;
      buffer.arm_command_param.request_error_status = 0;

      /*if(motor_num == 1)
       printf("time %x %x\n", time & 0xff, (time >> 8) & 0xff);*/

      buffer.arm_command_param.command[0] = buffer.arm_command_param.index + 128;
      buffer.arm_command_param.command[1] = 0xfb;
      buffer.arm_command_param.command[2] = time & 0xff;
      buffer.arm_command_param.command[3] = (time >> 8) & 0xff;
      buffer.arm_command_param.command[4] = 0;
      arm_rs485_load_tx(buffer);

      /*if(motor_num == 1)
       printf("pos %x %x %x %x\n", (arm_link[motor_num].position_target >> 24) & 0xFF, (arm_link[motor_num].position_target >> 16) & 0xFF,
       (arm_link[motor_num].position_target >> 8) & 0xFF, arm_link[motor_num].position_target & 0xFF);*/

      buffer.arm_command_param.command[1] = 0xfa;
      buffer.arm_command_param.command[2] = arm_link[motor_num].position_target & 0xFF;
      buffer.arm_command_param.command[3] = (arm_link[motor_num].position_target >> 8) & 0xFF;
      buffer.arm_command_param.command[4] = (arm_link[motor_num].position_target >> 16) & 0xFF;
      buffer.arm_command_param.command[5] = (arm_link[motor_num].position_target >> 24) & 0xFF;
      buffer.arm_command_param.command[6] = 0;
      arm_rs485_load_tx(buffer);

      position_last[motor_num] = arm_link[motor_num].position_target;
      //printf("position last[%d] %ld\n", motor_num, position_last[motor_num]);
    }

    //automatic_motion_flag = arm_automatic_motion_xyz_start(NULL);

    //if(automatic_motion_flag <= 0)
    break;

  }

  if(free_slots == -1)
  {
    //printf("Closing path. . .\n");

    for(motor_num = 0; motor_num < SMART_MOTOR_NUMBER; motor_num++)
    {
      buffer.arm_command_param.index = motor_num + 1;
      buffer.arm_command_param.request_position = 0;
      buffer.arm_command_param.request_trajectory_status = 0;
      buffer.arm_command_param.request_interpolation_status = 0;
      buffer.arm_command_param.request_error_status = 0;

      buffer.arm_command_param.command[0] = buffer.arm_command_param.index + 128;
      buffer.arm_command_param.command[1] = 0xfb;
      buffer.arm_command_param.command[2] = 0;
      buffer.arm_command_param.command[3] = 0;
      buffer.arm_command_param.command[4] = 0;
      arm_rs485_load_tx(buffer);

      /*if(motor_num == 1)
       printf("pos %x %x %x %x\n", (position_last[motor_num] >> 24) & 0xFF, (position_last[motor_num] >> 16) & 0xFF,
       (position_last[motor_num] >> 8) & 0xFF, position_last[motor_num] & 0xFF);*/

      buffer.arm_command_param.command[1] = 0xfa;
      buffer.arm_command_param.command[2] = position_last[motor_num] & 0xFF;
      buffer.arm_command_param.command[3] = (position_last[motor_num] >> 8) & 0xFF;
      buffer.arm_command_param.command[4] = (position_last[motor_num] >> 16) & 0xFF;
      buffer.arm_command_param.command[5] = (position_last[motor_num] >> 24) & 0xFF;
      buffer.arm_command_param.command[6] = 0;
      arm_rs485_load_tx(buffer);

      //position_last[motor_num] = arm_link[motor_num].position_target;
    }
  }

  if((motion_file != NULL) || (actuator_wait > 0))
  {
    //printf("GO!!\n");
    actuator_wait = 0;

    if(arm_set_command_without_value(0, "G") <= 0)
      return -1;
  }

  return 0;
}

int interpolation_write_sync(unsigned long sync)
{
  struct arm_rs485_frame buffer;

  buffer.arm_command_param.index = 0;
  buffer.arm_command_param.request_position = 0;
  buffer.arm_command_param.request_trajectory_status = 0;
  buffer.arm_command_param.request_interpolation_status = 0;
  buffer.arm_command_param.request_error_status = 0;

  buffer.arm_command_param.command[0] = 128;
  buffer.arm_command_param.command[1] = 0xfd;
  buffer.arm_command_param.command[2] = sync & 0xFF;
  buffer.arm_command_param.command[3] = (sync >> 8) & 0xFF;
  buffer.arm_command_param.command[4] = (sync >> 16) & 0xFF;
  buffer.arm_command_param.command[5] = (sync >> 24) & 0xFF;

  arm_rs485_load_tx(buffer);

  return 0;
}

int main(int argc, char **argv)
{
  int argc_count;
  if(argc > 1)
  {
    for(argc_count = 1; argc_count < argc; argc_count++)
    {
      if(strcmp(argv[argc_count], "--show-state") == 0)
        show_arm_state_flag = 1;
      else if(strcmp(argv[argc_count], "--show-ayz") == 0)
        show_arm_position_flag = 1;
      else if(strcmp(argv[argc_count], "--no-out-of-service") == 0)
        arm_out_of_service_enable_flag = 0;
      else if(strcmp(argv[argc_count], "--no-laser") == 0)
        laser_enable_flag = 0;
      else if(strcmp(argv[argc_count], "--show-laser") == 0)
        show_laser_data_flag = 1;
      else if(strcmp(argv[argc_count], "--show-rtb") == 0)
        show_rtb_state_flag = 1;
      else if(strcmp(argv[argc_count], "--show-gps") == 0)
        show_gps_state_flag = 1;
      else if(strcmp(argv[argc_count], "--gps-log") == 0)
        gps_log_flag = 1;
      else if(strcmp(argv[argc_count], "--no-gps") == 0)
        gps_enable_flag = 0;
      else
      {
        printf("Usage:\n\t%s [option]\nOption:\n", argv[0]);
        printf("\t--show-state\tprint the arm state to screen\n");
        printf("\t--show-ayz\tprint current links position\n");
        printf("\t--no-out-of-service\tdisable OUT OF SERVICE state\n");
        printf("\t--no-laser\tdisable laser range finder\n");
        printf("\t--show-laser\tprint minimum obstacle's distance from laser\n");
        printf("\t--show-rtb\tprint rtb state and current gps coord\n");
        printf("\t--show-gps\tprint pgs state and current gps coord\n");
        printf("\t--gps-log\tmake a log of nmea string from gps\n");
        exit(0);
      }
    }
  }

  /* Standoff Control Unit*/
  unsigned char scu_prev_state = SCU_ARM_IDLE;
  unsigned char scu_next_state = SCU_ARM_IDLE; // it's used to return to the previouse scu_state after an automatic move is completed
  unsigned char scu_state = SCU_ARM_REST;

  if(show_arm_state_flag)
    printf("scu state\t[SCU_ARM_REST] as default in main\n");

  /* LMS511 */
  int lms511_count;
  unsigned int lms511_dist_flag;
  unsigned char lms511_dist_index = 0;
  unsigned int lms511_min_dist = 8000;

  /* Robotic Arm */
  unsigned char robotic_arm_selected = 1;
  char arm_buffer[ARM_RS485_BUFFER_SIZE];
  long arm_number_convertion_result;
  char *arm_token_number;
  unsigned char request_timer_flag = 0;
  unsigned char arm_request_position = 0;
  unsigned char arm_request_trajectory = 0;
  unsigned char arm_request_interpolation = 0;
  unsigned char arm_request_error_position = 0;

//struct timespec arm485_timer_start, arm485_timer_stop;

  /* Arm battery interface */
  unsigned int battery_read_flag = 0;
  unsigned int battery_level = 0;
  unsigned int battery_level_med = 3000;
  long battery_level_accum = 0;
  unsigned int battery_count = 0;
  char arm_battery_buffer[32];

  /* Segway */
  struct sockaddr_in segway_address;
  union segway_union segway_status;
  unsigned char segway_check_counter = 0;
  unsigned char segway_down_flag = 0;
  long segway_socket_timeout = 0;

  struct timespec segway_timer_start, segway_timer_stop;
  long segway_elapsed_time_ns = 0;
  unsigned char segway_timer_enable_flag = 0;

  /* CCU interface */
  struct sockaddr_in ccu_socket_addr_dest;
  struct sockaddr_in ccu_socket_addr_src;
  char arm_position_for_ccu[128];

  /* Segway info from ccu */
  struct sockaddr_in segway_from_joystick_addr_src;
  __u8 segway_buffer[(SEGWAY_PARAM * 4) + 3];

  /* Robotic Arm Client*/
  struct sockaddr_in arm_address;
  struct arm_frame arm_command;
  int query_link_count;
  char arm_position_initialized = 0;
  struct timespec request_timer_start, request_timer_stop;
  long request_elapsed_time_ns = 0;

  /* Status client interface*/
  socklen_t scu_state_addr_dest_len = sizeof(struct sockaddr_in);
  struct sockaddr_in scu_state_addr_dest;
  struct sockaddr_in scu_state_addr_src;
  unsigned char scu_state_buffer;

  /* Gps */
#ifdef GPS
// Gps rs232 device
  char gps_device_buffer[RS232_BUFFER_SIZE];
  char *nmea_token;
  char nmea_message[256];
//char nmea_message_log[2048];
//int it = 0;

  nmeaPARSER parser;
  char gps_fix_flag = 0;
  char gps_simulate_flag = 0;
  long gps_fix_timer_hs = 0;

  struct timespec gps_timer_stop;

// Gps socket to communicate with CCU
  struct sockaddr_in gps_socket_addr_dest;
  struct sockaddr_in gps_socket_addr_src;

  struct
  {
    __u32 command;  // 0: nothing 1: add checkpoint 2: clear checkpoint
    __u32 latitude;
    __u32 longitude;
    __u32 direction;
    __u32 distance_from_previous;
  } gps_info;

  unsigned char rtb_point_catch = 0;
  /*double gps_lon;
   double gps_lat;
   double gps_direction;
   double PDOP, HDOP, vdop;
   int sat_inview, sat_used;*/
#endif

  /* Gps */
#ifdef GPS_SIMUL
  char gps_buffer[2048];
  int gps_timeout = 0;
#endif

  /* Rover */
  double rtb_old_direction = 0;
  double rtb_old_lat = 0;
  double rtb_old_lon = 0;
  double rtb_old_elv = 0;
  double angle_threshold = 0;
  double rover_angle_error = 0;
  double rover_tracking_angle = 0;
  RTB_point *rtb_point_temp = NULL;
  unsigned char rtb_active_flag = 0;
  unsigned char rtb_ctrl_timer_flag = 0;

// timer
  long rover_time_start_hs = 0;
  long rover_time_stop_hs = 0;
  long rover_elapsed_time_hs = 0;

  /* Air pump */
  long airpump_timeout_counter = 0;

  /* Generic Variable */
  int done = 0; // for the while in main loop
  int bytes_read; // to check how many bytes has been read
  int bytes_sent;

  int select_result = -1; // value returned frome select()
  int nfds = 0; // fd to pass to select()
  fd_set rd, wr, er; // structure for select()
  struct timeval select_timeout;
  long return_to_base_ctrl_timeout_counter = 0;
  long return_to_base_timeout_counter = 0;
  long automove_timeout_counter = 0;
  long homing_timeout_counter = 0;
  long stop_timeout_counter = 0;
  long arm_position_timeout_counter = 0;
  long arm_battery_timeout_counter = 0;
  long init_position_timeout_counter = 0;
  long current_timeout = 0;

  printf("Initializing. . .\n");

  while(eth_check_connection() != 1)
  {
    printf("Waiting for network. . .\n");
    sleep(1);
  }

  /* Peripheral initialization */

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  /* Init Segway Client */
  if(segway_open(&segway_socket, &segway_address, SEGWAY_ADDRESS, SEGWAY_PORT) == -1)
    perror("Init vehicle client");
  else
  {
    printf("Segway client\t[connected]\n");
    segway_init(segway_socket, &segway_address, &segway_status);
  }

  /* Init CCU Client */
  ccu_socket = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);

  if(ccu_socket < 0)
    perror("ccu_socket");
  else
    printf("Socket CCU\t[OK]\n");

  bzero(&ccu_socket_addr_src, sizeof(ccu_socket_addr_src));
  ccu_socket_addr_src.sin_family = AF_INET;
  ccu_socket_addr_src.sin_port = htons(CCU_PORT_ARM);
  ccu_socket_addr_src.sin_addr.s_addr = htonl(INADDR_ANY);

  if(bind(ccu_socket, (struct sockaddr *) &ccu_socket_addr_src, sizeof(ccu_socket_addr_src)) == -1)
    perror("ccu_socket");

  bzero(&ccu_socket_addr_dest, sizeof(ccu_socket_addr_dest));
  ccu_socket_addr_dest.sin_family = AF_INET;
  ccu_socket_addr_dest.sin_addr.s_addr = inet_addr(CCU_ADDRESS);
  ccu_socket_addr_dest.sin_port = htons(CCU_PORT_ARM);

  /* Init lms511 Client */
  if(laser_enable_flag)
  {
    if(lms511_open(&lms511_socket, &lms511_address, LMS511_ADDRESS, LMS511_PORT) == -1)
    {
      perror("Init lms511 client");
      close(lms511_socket);
      lms511_socket = -1;
    }
    else
    {
      printf("Laser Scanner\t[connected]\n");
      lms511_dist_flag = 5;
      lms511_init();
    }
  }
  else
    printf("Laser Scanner\t[disabled]\n");

  /* Init Arm Client */
  if(arm_open_server(&arm_udp_device, &arm_address, ARM_PORT) == -1)
    perror("init arm client");
  else
  {
    arm_command.arm_command_param.crc_uint = 0;
    arm_crc_initialize();

    printf("Arm\t[connected]\n");
  }

  /* Init rs485 */
  arm_rs485_device = arm_rs485_open(ARM_DEVICE_FILE, 115200, 'N', 8, 1);

  if(arm_rs485_device == -1)
  {
    perror("arm_rs485_open");
    fflush(stdout);
  }
  else
  {
    printf("Init rs485\t[OK]\n");
    fflush(stdout);
    /* Init Robotic Arm */
    arm_init(0, 500, 10, 1500, 200, 1500, 100, 300, 1023, arm_upper_limit, arm_lower_limit);

    // tuning motor 2 and 3
    arm_set_max_velocity(2, 700);
    arm_set_max_velocity(3, 700);

    // tuning motor 5
    arm_set_max_velocity(5, 1400);

    if(arm_set_command(5, "KP", 1000) <= 0)
      return -1;

    if(arm_set_command(5, "KL", 32767) <= 0)
      return -1;

    if(arm_set_command(5, "KD", 2000) <= 0)
      return -1;

    if(arm_set_command_without_value(5, "F") <= 0) // active settings
      return -1;

    // tuning motor 6
    arm_set_max_velocity(6, 700);

    if(arm_set_command(6, "KP", 1000) <= 0)
      return -1;

    if(arm_set_command(6, "KL", 32767) <= 0)
      return -1;

    if(arm_set_command(6, "KD", 2000) <= 0)
      return -1;

    if(arm_set_command_without_value(6, "F") <= 0) // active settings
      return -1;

  }

  /* Status client interface */
  scu_state_socket = socket(AF_INET, SOCK_DGRAM, 0);

  if(scu_state_socket < 0)
    perror("scu_state_socket");
  else
  {
    bzero(&scu_state_addr_src, sizeof(scu_state_addr_src));
    scu_state_addr_src.sin_family = AF_INET;
    scu_state_addr_src.sin_port = htons(SCU_STATUS_PORT);
    scu_state_addr_src.sin_addr.s_addr = htonl(INADDR_ANY);

    if(bind(scu_state_socket, (struct sockaddr *) &scu_state_addr_src, sizeof(scu_state_addr_src))
        == -1)
      perror("scu_state_socket");
    else
      printf("Status client\t[bind]\n");

    printf("Status client\t[listening]\n");
  }

  /* Init Segway info from joystick */
  segway_from_joystick_socket = socket(AF_INET, SOCK_DGRAM, 0);

  if(segway_from_joystick_socket < 0)
    perror("segway_from_joystick_socket");
  else
  {
    bzero(&segway_from_joystick_addr_src, sizeof(segway_from_joystick_addr_src));
    segway_from_joystick_addr_src.sin_family = AF_INET;
    segway_from_joystick_addr_src.sin_port = htons(SCU_SEGWAY_FROM_JOYSTICK_PORT);
    segway_from_joystick_addr_src.sin_addr.s_addr = htonl(INADDR_ANY);

    if(bind(segway_from_joystick_socket, (struct sockaddr *) &segway_from_joystick_addr_src,
        sizeof(segway_from_joystick_addr_src)) == -1)
      perror("segway_from_joystick_socket");

    printf("Socket Segway from Controller\t[connected]\n");
  }

  /* Init Gps */
#ifdef GPS_SIMUL
  gps_generate_init(2, 2, 4140.7277, 1230.5320, 0, 10.86, 0.0, 2, 2, &info);
#endif

#ifdef GPS
  if(gps_enable_flag)
  {
// Select UART2_TX and set it as output
    gpio_generic_set_value("/sys/kernel/debug/omap_mux/spi0_d0", 11);

// Select UART1_RX and set it as input pulled up
    gpio_generic_set_value("/sys/kernel/debug/omap_mux/spi0_sclk", 39);

    gps_device = rs232_open("/dev/ttyO2", 4800, 'N', 8, 1, 1, 0, GPS_INDEX);

    if(gps_device < 0)
      perror("com_open");
    else
    {
      printf("Gps Device\t[opened]\n");
      nmea_parser_init(&parser);

      //kalman_reset(0.0001, 0.0001, 1, 1, 0.01, 0, 0);

      gps_info.command = 0;

      gps_socket = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);

      if(gps_socket < 0)
        perror("gps_socket");

      bzero(&gps_socket_addr_src, sizeof(gps_socket_addr_src));
      gps_socket_addr_src.sin_family = AF_INET;
      gps_socket_addr_src.sin_port = htons(CCU_PORT_GPS);
      gps_socket_addr_src.sin_addr.s_addr = htonl(INADDR_ANY);

      if(bind(gps_socket, (struct sockaddr *) &gps_socket_addr_src, sizeof(gps_socket_addr_src))
          == -1)
        perror("gps_socket");
      else
        printf("Gps Info Socket\t[bind]\n");

      bzero(&gps_socket_addr_dest, sizeof(gps_socket_addr_dest));
      gps_socket_addr_dest.sin_family = AF_INET;
      gps_socket_addr_dest.sin_addr.s_addr = inet_addr(CCU_ADDRESS);
      gps_socket_addr_dest.sin_port = htons(CCU_PORT_GPS);
    }

    clock_gettime(CLOCK_REALTIME, &gps_timer_stop);
    rover_time_start_hs = (gps_timer_stop.tv_sec * 100 + (gps_timer_stop.tv_nsec / 10000000));
  }
#endif

  /* Init Rtb module */
  RTB_init();
  RTB_set_mode(RTB_recording);

  /* Init Air Pump */
  gpio_export(AIR_PUMP_GPIO);
  gpio_set_direction(AIR_PUMP_GPIO, 0);

  select_timeout.tv_sec = TIMEOUT_SEC;
  select_timeout.tv_usec = ARM_TIMEOUT_USEC;

  current_timeout = select_timeout.tv_usec;

  printf("Run main program. . .\n");

  while(!done)
  {
    fflush(stdout);

    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);

    // if segway_timer_enable_flag init timer
    if(segway_timer_enable_flag == 0)
    {
      clock_gettime(CLOCK_REALTIME, &segway_timer_start);
      segway_timer_enable_flag++;
    }
    else if(segway_buffer_tx_empty == 0)
    {
      clock_gettime(CLOCK_REALTIME, &segway_timer_stop);
      segway_elapsed_time_ns = (segway_timer_stop.tv_sec * 1000000000 + segway_timer_stop.tv_nsec)
          - (segway_timer_start.tv_sec * 1000000000 + segway_timer_start.tv_nsec);
    }

    if(request_timer_flag == 1)
    {
      clock_gettime(CLOCK_REALTIME, &request_timer_stop);
      request_elapsed_time_ns =
          (request_timer_stop.tv_sec * 1000000000 + request_timer_stop.tv_nsec)
              - (request_timer_start.tv_sec * 1000000000 + request_timer_start.tv_nsec);
    }

    if((scu_state_socket > 0))
    {
      if(scu_state_rqst_flag == 0)
      {
        FD_SET(scu_state_socket, &rd);
        nfds = max(nfds, scu_state_socket);
      }
      else if(arm_position_initialized == ((1 << MOTOR_NUMBER) - 1))
      {
        FD_SET(scu_state_socket, &wr);
        nfds = max(nfds, scu_state_socket);
      }
    }

    if(arm_udp_device > 0)
    {
      FD_SET(arm_udp_device, &rd);
      nfds = max(nfds, arm_udp_device);
    }

    if(arm_rs485_device > 0)
    {
      FD_SET(arm_rs485_device, &rd);
      nfds = max(nfds, arm_rs485_device);

      if(arm_rs485_buffer_tx_empty == 0)
      {
        FD_SET(arm_rs485_device, &wr);
        nfds = max(nfds, arm_rs485_device);
      }
    }
    else
    {
      arm_rs485_device = arm_rs485_open(ARM_DEVICE_FILE, 115200, 'N', 8, 1);

      if(arm_rs485_device > 0)
      {
        printf("Init rs485\t[OK]\n");
        fflush(stdout);
        // Init Robotic Arm
        arm_init(0, 500, 10, 1500, 200, 1500, 100, 300, 1023, arm_upper_limit, arm_lower_limit);

        // tuning motor 2 and 3
        arm_set_max_velocity(2, 700);
        arm_set_max_velocity(3, 700);

        // tuning motor 5
        arm_set_max_velocity(5, 1400);

        if(arm_set_command(5, "KP", 1000) <= 0)
          return -1;

        if(arm_set_command(5, "KL", 32767) <= 0)
          return -1;

        if(arm_set_command(5, "KD", 2000) <= 0)
          return -1;

        if(arm_set_command_without_value(5, "F") <= 0) // active settings
          return -1;

        // tuning motor 6
        arm_set_max_velocity(6, 700);

        if(arm_set_command(6, "KP", 1000) <= 0)
          return -1;

        if(arm_set_command(6, "KL", 32767) <= 0)
          return -1;

        if(arm_set_command(6, "KD", 2000) <= 0)
          return -1;

        if(arm_set_command_without_value(6, "F") <= 0) // active settings
          return -1;

      }
    }

    if((arm_battery_file != NULL) && battery_read_flag)
    {
      FD_SET(arm_battery_fd, &rd);
      nfds = max(nfds, arm_battery_fd);
    }

#ifdef GPS
    if(gps_enable_flag)
    {
      if(gps_device > 0)
      {
        FD_SET(gps_device, &rd);
        nfds = max(nfds, gps_device);
      }
    }
#endif

    if(lms511_socket > 0)
    {
      FD_SET(lms511_socket, &rd);
      nfds = max(nfds, lms511_socket);

      if(lms511_buffer_tx_empty == 0)
      {
        FD_SET(lms511_socket, &wr);
        nfds = max(nfds, lms511_socket);
      }
    }

    if(segway_socket > 0)
    {
      FD_SET(segway_socket, &rd);
      nfds = max(nfds, segway_socket);

      if((segway_buffer_tx_empty == 0)
          && (segway_elapsed_time_ns > (long) SEGWAY_TIMEOUT_USEC * 1000))
      {
        FD_SET(segway_socket, &wr);
        nfds = max(nfds, segway_socket);
      }
    }

    if((segway_from_joystick_socket > 0) && (rtb_active_flag == 0))
    {
      FD_SET(segway_from_joystick_socket, &rd);
      nfds = max(nfds, segway_from_joystick_socket);
    }

    select_result = select(nfds + 1, &rd, &wr, NULL, &select_timeout);

    if(select_result == -1 && errno == EAGAIN)
    {
      perror("select");
      continue;
    }

    if(select_result == -1)
    {
      perror("main:");

      return 1;
    }

    /* Manage SCU state request */
    if(scu_state_socket > 0)
    {
      if(FD_ISSET(scu_state_socket, &rd))
      {
        bytes_read = recvfrom(scu_state_socket, &scu_state_buffer, sizeof(scu_state_buffer), 0,
            (struct sockaddr *) &scu_state_addr_dest, &scu_state_addr_dest_len);

        if(bytes_read > 0)
        {
          return_to_base_timeout_counter = 0;  // reset timer

          switch(scu_state_buffer)
          {
            case SCU_RQST_STATE:
              if(rtb_active_flag)
              {
                /**** Release command to the extern control ****/
                rtb_ctrl_timer_flag = 0;

                switch(scu_state)
                {
                  case SCU_ARM_IDLE:
                    arm_command.arm_command_param.header_uint = ARM_RQST_PARK;
                    arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
                    break;

                  case SCU_ARM_MOVE:
                    arm_command.arm_command_param.header_uint = ARM_CMD_STOP;
                    arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
                    break;

                  case SCU_ARM_END_EFFECTOR_1:
                    arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_1;
                    arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
                    break;

                  case SCU_ARM_END_EFFECTOR_2:
                    arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_2;
                    arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
                    break;

                  case SCU_ARM_END_EFFECTOR_3:
                    arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_3;
                    arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
                    break;

                  case SCU_ARM_END_EFFECTOR_4:
                    arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_4;
                    arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
                    break;

                  case SCU_ARM_END_EFFECTOR_5:
                    arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_5;
                    arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
                    break;

                  case SCU_ARM_END_EFFECTOR_6:
                    arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_6;
                    arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
                    break;

                  case SCU_ARM_END_EFFECTOR_7:
                    arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_7;
                    arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
                    break;

                  case SCU_ARM_REST:
                    if(segway_status.list.operational_state == SEGWAY_TRACTOR)
                    {
                      bytes_sent = segway_configure_operational_mode(segway_socket, &segway_address,
                          SEGWAY_STANDBY_REQ);

                      if(bytes_sent == -1)
                        perror("segway_configure_operational_mode");
                    }
                    else
                    {
                      printf("Signal found: release control\n");
                      robotic_arm_selected = 1;
                      rtb_active_flag = 0;

                      scu_state_rqst_flag = 1;
                      scu_rqst_rtb_flag = 0;
                    }
                    break;

                  default:
                    break;
                }
              }
              else
                scu_state_rqst_flag = 1;
              break;

            case SCU_RQST_RTB:
              if(rtb_active_flag == 0)
                printf("Return to base request\n");

              scu_rqst_rtb_flag = 1;
              rtb_active_flag = 1;
              rtb_ctrl_timer_flag = 1;
              break;

            case SCU_RQST_RESET_BASE:
              printf("Reset base\n");

              RTB_set_mode(RTB_idle);
              RTB_set_mode(RTB_recording);

              gps_info.command = 3;

              if(gps_socket > 0)
                sendto(gps_socket, &gps_info, sizeof(gps_info), 0,
                    (struct sockaddr *) &gps_socket_addr_dest, sizeof(gps_socket_addr_dest));

              break;

            case SCU_RQST_LASER_ON:
              if(laser_enable_flag == 0)
              {
                printf("Enable laser\n");
                laser_enable_flag = 1;
                lms511_dist_flag = 5;

                if(lms511_socket < 0)
                {
                  if(lms511_open(&lms511_socket, &lms511_address, LMS511_ADDRESS, LMS511_PORT)
                      == -1)
                  {
                    perror("Init lms511 client");
                    close(lms511_socket);
                    lms511_socket = -1;
                  }
                  else
                  {
                    printf("Laser Scanner\t[connected]\n");
                    lms511_dist_flag = 5;
                    lms511_init();
                  }
                }
              }
              break;

            case SCU_RQST_LASER_OFF:
              if(laser_enable_flag == 1)
              {
                printf("Disable laser\n");
                laser_enable_flag = 0;
                lms511_dist_flag = 0;
              }
              break;
          }
        }
        else
          perror("status_read");
      }

      if(FD_ISSET(scu_state_socket, &wr))
      {
        scu_state_rqst_flag = 0;
        bytes_sent = sendto(scu_state_socket, &scu_state, sizeof(scu_state), 0,
            (struct sockaddr *) &scu_state_addr_dest, sizeof(scu_state_addr_dest));
      }
    }

#ifdef GPS
    /* Manage gps */
    if(gps_enable_flag)
    {
      if(gps_device > 0)
      {
        if(FD_ISSET(gps_device, &rd))
        {
          bytes_read = rs232_read(gps_device, GPS_INDEX);
          if((bytes_read > 0) || ((bytes_read < 0) && rs232_buffer_rx_full[GPS_INDEX]))
          {
            while((bytes_read = rs232_unload_rx_multifiltered(gps_device_buffer, gps_token, 1,
            GPS_INDEX)) > 0)
            {
              gps_device_buffer[bytes_read - 1] = *gps_token;
              gps_device_buffer[bytes_read] = '\0';

              if(gps_log_flag)
                gps_text_log(gps_device_buffer);

              nmea_token = strtok(gps_device_buffer, "\n");

              while(nmea_token != NULL)
              {
                sprintf(nmea_message, "%s\n", nmea_token);
                nmea_parse(&parser, nmea_message, (int) strlen(nmea_message), &info);
                nmea_token = strtok(NULL, "\n");
              }

              // If I have received all mandatory info (lat, lon, speed, direction) then update the rtb scu_state
              if((info.smask & GPRMC) && (info.smask & HCHDT) && (info.smask & GPGSA))
              {
                /*gps_lat = info.lat;
                 gps_lon = info.lon;
                 gps_direction = info.direction;
                 PDOP = info.PDOP;
                 HDOP = info.HDOP;
                 vdop = info.VDOP;
                 sat_inview = info.satinfo.inview;
                 sat_used = info.satinfo.inuse;*/

                info.smask &= !(GPRMC | GPGGA | GPGSA | GPVTG | HCHDT | GPGSV);

                /* Get time elapsed */
                clock_gettime(CLOCK_REALTIME, &gps_timer_stop);

                rover_time_stop_hs = (gps_timer_stop.tv_sec * 100
                    + (gps_timer_stop.tv_nsec / 10000000));

                if(rover_time_start_hs != rover_time_stop_hs)
                {
                  rover_elapsed_time_hs = rover_time_stop_hs - rover_time_start_hs;
                  rover_time_start_hs = rover_time_stop_hs;
                }

                // if I have odometry avaible then I can run kalman_estimate
                /*if((segway_status.list.operational_state == SEGWAY_TRACTOR) || (segway_status.list.operational_state == SEGWAY_STANDBY))
                 {
                 // Estimate parameter from kalman filter
                 //kalman_estimate(convert_to_float(segway_status.list.linear_vel_mps),
                 //                                 info.magnetic_sensor_heading_true,
                 //                                 rover_elapsed_time_hs * 10000);

                 if((info.sig == NMEA_SIG_BAD) || (info.fix == NMEA_FIX_BAD))
                 {
                 gps_fix_timer_hs = 0;

                 // If gps_generate is already init then generate gps information from odometry
                 if(gps_simulate_flag)
                 {
                 gps_generate(convert_to_float(segway_status.list.linear_vel_mps) * 3.6,
                 info.magnetic_sensor_heading_true,
                 rover_elapsed_time_hs * 10000, gps_device_buffer, &info);
                 }
                 else
                 {
                 gps_generate_init(NMEA_SIG_BAD, NMEA_FIX_BAD, rtb_old_lat, rtb_old_lon,
                 convert_to_float(segway_status.list.linear_vel_mps) * 3.6,
                 rtb_old_elv, info.magnetic_sensor_heading_true, 0, 0, &info);

                 gps_simulate_flag = 1;
                 }

                 if(show_gps_state_flag)
                 {
                 printf("\033[K");
                 printf("[Sig: Bad] [Fix: Bad] [lon %f lat%f]\r", GpsCoord2Double(info.lon), GpsCoord2Double(info.lat));
                 }
                 }
                 }*/

                // if I have a good signal form gps then update the kalman filter
                if(((info.sig != NMEA_SIG_BAD) || (info.fix != NMEA_FIX_BAD)) && (info.PDOP < 4)
                    && (info.HDOP < 4))
                {
                  if(gps_fix_timer_hs < GPS_FIX_TIMEOUT_SEC * 100)
                    gps_fix_timer_hs += rover_elapsed_time_hs;
                  else
                  {
                    // if it's the first time that I have gps fix, then traslate the previuse points
                    // calculate by odometry
                    if(gps_fix_flag == 0)
                    {
                      gps_fix_flag = 1;
                      gps_simulate_flag = 1;

                      gps_generate_init(NMEA_SIG_BAD, NMEA_FIX_BAD, info.lat, info.lon,
                          convert_to_float(segway_status.list.linear_vel_mps) * 3.6, rtb_old_elv,
                          info.magnetic_sensor_heading_true, 0, 0, &info);

                      // Check if I have to initialize the odometer data with real gps position. This condition
                      // is verified when it's time to compute odometer data for the first time. I don't traslate
                      // coord when I use odometer instead gps due to the signal loss
                      printf("Gps Traslate point\n");
                      RTB_traslate_point(GpsCoord2Double(info.lat), GpsCoord2Double(info.lon),
                          rtb_point_temp);

                      if(rtb_point_temp != NULL)
                      {
                        RTB_point *local_point = rtb_point_temp;

                        while(local_point->next != NULL)
                        {
                          gps_info.command = 1;

                          gps_info.latitude = convert_to_ieee754(local_point->y);
                          gps_info.longitude = convert_to_ieee754(local_point->x);

                          if(local_point->previous != NULL)
                            gps_info.distance_from_previous = local_point->distance_from_start
                                - local_point->previous->distance_from_start;
                          else
                            gps_info.distance_from_previous = 0;

                          local_point = local_point->next;

                          bytes_sent = sendto(gps_socket, &gps_info, sizeof(gps_info), 0,
                              (struct sockaddr *) &gps_socket_addr_dest,
                              sizeof(gps_socket_addr_dest));
                        }

                        free(rtb_point_temp);
                      }
                    }

                    if(show_gps_state_flag)
                    {
                      printf("\033[K");
                      switch(info.sig)
                      {
                        case 0:
                          printf("[Sig: Invalid] ");
                          break;
                        case 1:
                          printf("[Sig: Fix] ");
                          break;
                        case 2:
                          printf("[Sig: Differential] ");
                          break;
                        case 3:
                          printf("[Sig: Sensitive] ");
                          break;
                        default:
                          printf("[Sig: Invalid] ");
                          break;
                      }

                      switch(info.fix)
                      {
                        case 1:
                          printf("[Fix: not available] ");
                          break;
                        case 2:
                          printf("[Fix: 2d] ");
                          break;
                        case 3:
                          printf("[Fix: 3d] ");
                          break;
                        default:
                          printf("[Fix: unknonw] ");
                          break;
                      }

                      printf("[lon %f lat%f]\r", GpsCoord2Double(info.lon),
                          GpsCoord2Double(info.lat));
                    }

                    // Update kalman filter
                    // First I have to transform gps data in decimal format
                    /*info.lon = GpsCoord2Double(info.lon);
                     info.lat = GpsCoord2Double(info.lat);

                     kalman_update(&info.lon, &info.lat);

                     // Return to gps format
                     info.lon = Double2GpsCoord(info.lon);
                     info.lat = Double2GpsCoord(info.lat);*/
                  }
                }

#ifdef GPS_DEBUG
                if(it > 0)
                {
                  printf("\033[22A");
                }
                else
                it++;

                printf("Time: %i/%i/%i %i:%i:%i.%i                    \n", info.utc.day, info.utc.mon + 1, info.utc.year + 1900, info.utc.hour, info.utc.min, info.utc.sec, info.utc.hsec);
                if((info.smask & GPGGA) || (info.smask & GPRMC))
                {
                  //info.smask &= !GPGGA;
                  switch(info.sig)
                  {
                    case 0:
                    printf("Signal: INVALID                    \n");
                    break;
                    case 1:
                    printf("Signal: FIX                    \n");
                    break;
                    case 2:
                    printf("Signal: DIFFERENTIAL                    \n");
                    break;
                    case 3:
                    printf("Signal: SENSITIVE                    \n");
                    break;
                    default:
                    printf("Signal: INVALID                    \n");
                    break;
                  }

                  switch(info.fix)
                  {
                    case 1:
                    printf("Operating mode: FIX NOT AVAILABLE                    \n");
                    break;
                    case 2:
                    printf("Operating mode: 2D                    \n");
                    break;
                    case 3:
                    printf("Operating mode: 3D                    \n");
                    break;
                    default:
                    printf("Operating mode: UNKNOWN                    \n");
                    break;
                  }
                }
                else
                {
                  printf("Signal:\n");
                  printf("Operating mode:\n");
                }

                printf("Position Diluition of Precision: %f                    \n", info.PDOP);
                printf("Horizontal Diluition of Precision: %f                    \n", info.HDOP);
                printf("Vertical Diluition of Precisione: %f                    \n", info.VDOP);
                printf("Latitude: %f                    \n", info.lat);
                printf("Longitude: %f                    \n", info.lon);
                printf("Elevation: %f m                    \n", info.elv);
                printf("Speed: %f km/h                    \n", info.speed);
                printf("Direction: %f degrees                    \n", info.direction);

                printf("Magnetic variation degrees: %f                    \n", info.declination);
                printf("Magnetic sensor heading: %f                    \n", info.magnetic_sensor_heading);
                printf("Magnetic sensor heading true: %f               \n", info.magnetic_sensor_heading_true);
                printf("Magnetic sensor deviation: %f                    \n", info.magnetic_sensor_deviation);
                printf("Magnetic sensor variation: %f                    \n", info.magnetic_sensor_variation);
                printf("Rate turn: %f                    \n", convert_to_float(segway_status.list.inertial_z_rate_rps));
                printf("Pitch oscillation: %f                    \n", info.pitch_osc);
                printf("Roll oscillation: %f                    \n", info.roll_osc);

                printf("\nSatellite: \tin view: %i          \n\t\tin use: %i                    \n", info.satinfo.inview, info.satinfo.inuse);
#endif

                // Update the rtb map if I'm in recording mode
                if(rtb_active_flag == 0)
                {
                  if((RTBstatus.mode == RTB_recording)
                      && ((info.sig != NMEA_SIG_BAD) || (info.fix != NMEA_FIX_BAD))
                      && (info.PDOP < 4) && (info.HDOP < 4))
                  {
                    if(show_rtb_state_flag)
                    {
                      printf("\033[K");
                      printf("[Recording] [lon %f lat%f]\r", GpsCoord2Double(info.lon),
                          GpsCoord2Double(info.lat));
                    }

                    RTB_update(GpsCoord2Double(info.lon), GpsCoord2Double(info.lat),
                        (convert_to_float(segway_status.list.linear_vel_mps) * 3.6),
                        convert_to_float(segway_status.list.inertial_z_rate_rps), 0, NULL, 0,
                        &rtb_point_catch);
                  }
                  else
                    RTB_set_mode(RTB_recording);
                }

                // Send Gps info to ccu
                if((info.sig != NMEA_SIG_BAD) || (info.fix != NMEA_FIX_BAD))
                {
                  if(rtb_point_catch == 1)
                  {
                    if(RTBstatus.mode == RTB_recording)
                      gps_info.command = 1;
                    else
                      gps_info.command = 2;
                  }
                  else
                    gps_info.command = 0;

                  rtb_point_catch = 0;

                  // Send gps info to ccu
                  gps_info.latitude = convert_to_ieee754(GpsCoord2Double(info.lat));
                  gps_info.longitude = convert_to_ieee754(GpsCoord2Double(info.lon));
                  gps_info.direction = convert_to_ieee754(info.magnetic_sensor_heading_true);
                  gps_info.distance_from_previous = convert_to_ieee754(RTBstatus.distance);

                  bytes_sent = sendto(gps_socket, &gps_info, sizeof(gps_info), 0,
                      (struct sockaddr *) &gps_socket_addr_dest, sizeof(gps_socket_addr_dest));

                  if(bytes_sent < 0)
                    perror("sendto gps");
                }

                rtb_old_direction = info.magnetic_sensor_heading_true;
                rtb_old_lat = info.lat;
                rtb_old_lon = info.lon;
                rtb_old_elv = info.elv;
              }
              //info.smask &= !(GPRMC | GPGGA | GPVTG | HCHDT | GPGSV);
            }
          }
        }
      }
    }
#endif

    if(arm_udp_device > 0)
    {
      if(FD_ISSET(arm_udp_device, &rd))
      {
        bytes_read = recvfrom(arm_udp_device, &arm_command, sizeof(arm_command), 0, NULL, NULL);

        if(bytes_read > 0)
        {
          if(arm_crc_byte_buffer_crc_is_valid((__u8 *) arm_command.arm_command, bytes_read))
          {
            // se ho una richiesta in sospeso, aspetto ad aggiornare lo stato del braccio
            if((arm_request_position == 0) && (arm_request_trajectory == 0)
                && (arm_request_interpolation == 0) && (arm_request_error_position == 0))
              arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
          }
        }
        else
          perror("arm_read");
      }
    }

    if(arm_rs485_device > 0)
    {
      if(FD_ISSET(arm_rs485_device, &rd))
      {
        bytes_read = arm_rs485_read(arm_rs485_device);

        if(bytes_read == -2)
          printf("rs485 overrun\n");
        else if(bytes_read == -1)
          perror("rs485 read");

        if(arm_request_interpolation == 1)
        {
          bytes_read = arm_rs485_unload_interpolation((unsigned char *) arm_buffer);

          if(bytes_read == 6)
          {
            //printf("Interpolation [%i]\n", arm_query_link);
            request_timer_flag = 0;
            request_elapsed_time_ns = 0;

            arm_link[arm_query_link - 1].slots = arm_buffer[1] & 0x3f;
            arm_link[arm_query_link - 1].pending_g = arm_buffer[2] & 0x1;
            arm_link[arm_query_link - 1].ip_ready = (arm_buffer[2] >> 1) & 0x1;
            arm_link[arm_query_link - 1].invalid_time_delta = (arm_buffer[2] >> 2) & 0x1;
            arm_link[arm_query_link - 1].invalid_position_delta = (arm_buffer[2] >> 3) & 0x1;
            arm_link[arm_query_link - 1].md_ready = (arm_buffer[2] >> 4) & 0x1;
            arm_link[arm_query_link - 1].data_buffer_overflow = (arm_buffer[2] >> 5) & 0x1;
            arm_link[arm_query_link - 1].data_buffer_underflow = (arm_buffer[2] >> 6) & 0x1;
            arm_link[arm_query_link - 1].md_running = (arm_buffer[2] >> 7) & 0x1;

            /*printf("============================\n");
             printf("Motor %i\n", arm_query_link);
             printf("slots: %d\n", arm_link[arm_query_link - 1].slots);
             printf("pending_g: %d\n", arm_buffer[2] & 0x1);
             printf("ip_ready: %d\n", (arm_buffer[2] >> 1) & 0x1);

             if(arm_link[arm_query_link - 1].invalid_time_delta)
             printf("Warning: invalid time delta\n");

             if(arm_link[arm_query_link - 1].invalid_position_delta)
             printf("Warning: invalid position delta\n");

             printf("md_ready: %d\n", (arm_buffer[2] >> 4) & 0x1);
             printf("data_buffer_overflow: %d\n", (arm_buffer[2] >> 5) & 0x1);
             printf("data_buffer_underflow: %d\n", (arm_buffer[2] >> 6) & 0x1);
             printf("md_running: %d\n", (arm_buffer[2] >> 7) & 0x1);*/

            // mi segno a quanti motori ho richiesto lo stato
            arm_table_query |= (0x01 << (arm_query_link - 1));

            // mi segno i motori che sono ancora in movimento
            if(arm_link[arm_query_link - 1].md_running)
              arm_table_running |= (0x01 << (arm_query_link - 1));
            else
              arm_table_running &= ~(0x01 << (arm_query_link - 1));

            if(arm_table_query == ((1 << SMART_MOTOR_NUMBER) - 1))
            {
              //arm_query_position(0);

              arm_table_query = 0;
              //if(arm_link[arm_query_link - 1].md_running)
              //{
              if((generate_path(arm_link[arm_query_link - 1].slots, NULL) == -2)
                  && (arm_link[arm_query_link - 1].slots > 1))
              {
                generate_path(-1, NULL);

                if(arm_table_running == 0)
                {
                  // End of file reached
                  arm_stop(0);
                  automove_timer_flag = 0;
                  stop_timer_flag = 1;

                  if(scu_rqst_rtb_flag == 0)
                    scu_state_rqst_flag = 1;
                }
              }
            }

            arm_request_position = 0;
            arm_request_trajectory = 0;
            arm_request_interpolation = 0;
            arm_request_error_position = 0;

            arm_query_link = -1;
          }
          else if(bytes_read == 5)
          {
            request_timer_flag = 0;
            request_elapsed_time_ns = 0;

            //printf("Sync . . .\n");

            unsigned long sync_time = arm_buffer[1] | (arm_buffer[2] << 8) | (arm_buffer[3] << 16)
                | (arm_buffer[4] << 24);

            interpolation_write_sync(sync_time);

            arm_request_position = 0;
            arm_request_trajectory = 0;
            arm_request_interpolation = 0;
            arm_request_error_position = 0;

            arm_query_link = -1;
          }
        }
        else
        {
          //while((bytes_read = arm_rs485_unload_rx_filtered(arm_buffer, '\n')) > 0)
          while((bytes_read = arm_rs485_unload_rx_multifiltered(arm_buffer, uart_token, 2)) > 0)
          {
            //printf("position [%i]\n", arm_query_link);

            /**************** Filter message ******************************/
            // I expect at least one character with \r as trailer
            if(bytes_read < 2)
            {
              printf("Message too short: %x\n", arm_buffer[0]);
              continue;
            }

            // queste variabili vanno resesttate dopo aver controllato eventuali
            // errori sul bus
            request_timer_flag = 0;
            request_elapsed_time_ns = 0;
            arm_buffer[bytes_read] = 0;

            errno = 0;
            // convert sting in a long number
            arm_number_convertion_result = strtol(arm_buffer, &arm_token_number, 10);

            if((errno == ERANGE
                && (arm_number_convertion_result == LONG_MAX
                    || arm_number_convertion_result == LONG_MIN))
                || (errno != 0 && arm_number_convertion_result == 0))
            {
              //perror("strtol");
              printf("----------------------------------------> Not a number!!\n");

              //return 0;
              continue;
            }
            /*else if(arm_token_number == arm_buffer)
             {
             if((strncmp(arm_buffer, "$s0", 3) != 0) && (strncmp(arm_buffer, "$s1", 3) != 0)
             && (strncmp(arm_buffer, "$s2", 3) != 0))
             {
             printf("Not a number at all!\n %s\n", arm_buffer);

             //return 0;
             continue;
             }
             }
             else if(*arm_token_number != '\n')
             {
             printf("----------------------------------------> Not a compleate number!!: \n%s\n",
             arm_buffer);

             //return 0;
             continue;
             }*/

            if(arm_request_position == 1)
            {
              arm_link[arm_query_link - 1].actual_position = arm_number_convertion_result;

              if(arm_link[arm_query_link - 1].position_initialized == 0)
                arm_link[arm_query_link - 1].position_initialized = 1;

              arm_position_initialized |= (arm_link[arm_query_link - 1].position_initialized
                  << (arm_query_link - 1));

              if(show_arm_position_flag)
              {
                if(show_arm_ayz_init_flag != 0)
                {
                  printf("\033[%iA", MOTOR_NUMBER + 1);
                }
                else
                  show_arm_ayz_init_flag = 1;

                int i;
                for(i = 0; i < MOTOR_NUMBER; i++)
                  motor_step[i] = arm_link[i].actual_position;

                arm_ee_xyz(motor_step, &arm_x, &arm_y, &arm_z);
                printf("Actual Positions - x: %f m y: %f m z: %f m                \n", arm_x, arm_y,
                    arm_z);
                for(query_link_count = 0; query_link_count < MOTOR_NUMBER; query_link_count++)
                {
                  printf("Link%i: %ld step %f deg        \n", query_link_count + 1,
                      arm_link[query_link_count].actual_position,
                      (double) arm_link[query_link_count].actual_position
                          / (11 * arm_link[query_link_count].gear));
                }
              }
            }
            else if(arm_request_trajectory == 1)
            {
              arm_link[arm_query_link - 1].trajectory_status = arm_number_convertion_result;
            }
            else if(arm_request_error_position == 1)
            {
              //if(arm_number_convertion_result != 0)
              printf("REA[%d] = %ld\n", arm_query_link, arm_number_convertion_result);

              /*if((arm_number_convertion_result > ARM_ERROR_POSITION) || (arm_number_convertion_result < -ARM_ERROR_POSITION))
               {
               printf("Stop homing\n");

               arm_rs485_flush_buffer_tx();

               arm_set_command(2, "O", 1417778);
               arm_set_command(2, "p", 1417778);
               arm_set_command(2, "EPTR", 100);
               arm_set_command_without_value(2, "VST(p,1)");

               // load stop command
               arm_stop(0);
               arm_query_position(0);
               arm_query_trajectory(0);

               scu_state = SCU_ARM_STOP;

               if(scu_rqst_rtb_flag == 0)
               scu_state_rqst_flag = 1;

               stop_timer_flag = 1;

               homing_timer_flag = 0;
               }*/
            }

            arm_request_position = 0;
            arm_request_trajectory = 0;
            arm_request_interpolation = 0;
            arm_request_error_position = 0;

            arm_query_link = -1;
          }
        }

        if(bytes_read == -2)
          printf("Frame Error\n");
      }

      if((FD_ISSET(arm_rs485_device, &wr)) && (request_timer_flag == 0))
      {
        while((arm_rs485_buffer_tx_empty == 0) && (arm_query_link == -1))
        {
          bytes_sent = arm_rs485_write(arm_rs485_device, &arm_query_link, &arm_request_position,
              &arm_request_trajectory, &arm_request_interpolation, &arm_request_error_position);

          if(bytes_sent > 0)
          {
            if(arm_request_position || arm_request_trajectory || arm_request_interpolation
                || arm_request_error_position)
            {
              request_timer_flag = 1;
              //printf("Request position [%i]\n", arm_query_link);

              //clock_gettime(CLOCK_REALTIME, &arm485_timer_start);
              clock_gettime(CLOCK_REALTIME, &request_timer_start);
              //clock_gettime(CLOCK_REALTIME, &request_timer_stop);
              request_elapsed_time_ns = 0;
            }
          }
          else
          {
            printf("Error on rs485_write\n");
            return -1;
          }
        }
      }
    }

    if(arm_battery_file != NULL)
    {
      if(FD_ISSET(arm_battery_fd, &rd))
      {
        fscanf(arm_battery_file, "%i", (int *) &battery_level);

        battery_level_accum += battery_level;
        battery_count++;

        if(battery_count == ARM_BATTERY_FILTER_COUNT)
        {
          battery_count = 0;
          battery_level_med = (int) (battery_level_accum / ARM_BATTERY_FILTER_COUNT);
          battery_level_accum = 0;
        }

        if(ccu_socket > 0)
        {
          //printf("battery voltage read %i mV\n", (int) (battery_level * /*1.8 * 24.74*/ 40800 / 4096));
          // Il coefficiente di conversione tra le tensione della batteria e quelle lette dall'adc dopo il partitore è di
          // 22.92143832. Per calcolare la tensione della batteria, basta moltiplicare il valore ottenuto dall'adc per lo
          // step minimo di tensione (1.8 V / 4096) e moltiplicarlo per il coefficiente di conversione. Quindi, il
          // coefficiente moltiplicativo finale sarà di (1800 * 22.92143832 / 4096) = 10.0
          sprintf(arm_battery_buffer, "b%i", (int) (battery_level_med * 10.072));  //old: 40800

          bytes_sent = sendto(ccu_socket, arm_battery_buffer, strlen(arm_battery_buffer), 0,
              (struct sockaddr *) &ccu_socket_addr_dest, sizeof(ccu_socket_addr_dest));

          if(bytes_sent < 0)
            perror("sendto ccu");

          battery_read_flag = 0;

          close(arm_battery_fd);
          fclose(arm_battery_file);

          arm_battery_fd = -1;
          arm_battery_file = NULL;
        }
      }
    }

    if(segway_socket > 0)
    {
      if(FD_ISSET(segway_socket, &rd))
      {
        bytes_read = segway_read(segway_socket, &segway_status, segway_buffer);

        if(bytes_read <= 0)
          perror("segway_read");
        else
        {
          segway_check_counter = 0;

          if(segway_down_flag == 1)
          {
            printf("Segway Init\t[OK]\n");
            segway_down_flag = 0;
          }
        }
      }

      if(FD_ISSET(segway_socket, &wr))
      {
        if(segway_send(segway_socket, &segway_address) < 0)
          perror("segway_send");

        if(segway_timer_enable_flag > 0)
        {
          segway_timer_enable_flag = 0;
          clock_gettime(CLOCK_REALTIME, &segway_timer_stop);
          segway_elapsed_time_ns = 0;
        }
      }

      if(segway_from_joystick_socket > 0)
      {
        if(FD_ISSET(segway_from_joystick_socket, &rd))
        {
          bytes_read = segway_read(segway_from_joystick_socket, &segway_status, segway_buffer);

          if(bytes_read <= 0)
            perror("segway_read");
        }
      }
    }

    if(lms511_socket > 0)
    {
      if(FD_ISSET(lms511_socket, &rd))
      {
        char message_type;
        lms511_parse(lms511_socket, &message_type);
        if(message_type > 0)
        {
          if(lms511_info.data.spot_number > 0)
          {
            lms511_dist_flag = 0;

            if(show_laser_data_flag)
              lms511_min_dist = 8000;

            for(lms511_count = 0; lms511_count < lms511_info.spot_number; lms511_count++)
            {
              if(lms511_info.data.spot[lms511_count] == 0)
                lms511_info.data.spot[lms511_count] = 8000;

              if(show_laser_data_flag)
                lms511_min_dist = min(lms511_min_dist, lms511_info.data.spot[lms511_count]);

              if(lms511_info.data.spot[lms511_count] < 2000)
              {
                if(lms511_count == (lms511_dist_index + 1))
                {
                  lms511_dist_flag += 1;
                  if((lms511_dist_flag > 4) && (show_laser_data_flag == 0))
                    break;
                }
                else
                  lms511_dist_flag = 1;

                lms511_dist_index = lms511_count;
              }
            }

            if(show_laser_data_flag)
            {
              printf("\033[K");
              printf("Laser minimum distance: %i\r", lms511_min_dist);
            }
          }
        }
      }

      if(FD_ISSET(lms511_socket, &wr))
      {
        bytes_sent = lms511_send(lms511_socket, &lms511_address);

        if(bytes_sent <= 0)
        {
          perror("Error on lms511_send");
          lms511_dispose();
          close(lms511_socket);
          lms511_socket = -1;
        }
      }
    }

    /* Timeout region */
    if((select_timeout.tv_sec == 0) && (select_timeout.tv_usec == 0))
    {
      if((arm_position_initialized != ((1 << MOTOR_NUMBER) - 1))
          && (scu_state != SCU_ARM_OUT_OF_SERVICE))
      {
        init_position_timeout_counter++;
        if((init_position_timeout_counter * current_timeout) >= (long) INIT_POSITION_TIMEOUT_USEC)
        {
          init_position_timeout_counter = 0;
          for(query_link_count = 1; query_link_count <= MOTOR_NUMBER; query_link_count++)
          {
            if(arm_link[arm_query_link - 1].position_initialized == 0)
              arm_query_position(query_link_count);
          }
        }
      }
      else
        init_position_timeout_counter = 0;

      if(robotic_arm_selected == 0)
      {
        segway_socket_timeout++;
        if((segway_socket_timeout * current_timeout) >= SEGWAY_TIMEOUT_USEC)
        {
          segway_socket_timeout = 0;

          // Increase segway inactivity counter
          segway_check_counter++;

          if((segway_check_counter * SEGWAY_TIMEOUT_USEC) >= SEGWAY_INACTIVITY_TIMEOUT_US)
          {
            segway_check_counter = 0;

            // Send warning only once
            if(segway_down_flag == 0)
            {
              printf("Segway down!\n");
              segway_status.list.operational_state = UNKNOWN;
              segway_down_flag = 1;
            }
          }

          segway_status_update(&segway_status, segway_socket, &segway_address, JOY_MAX_VALUE);
        }
      }

      if(homing_timer_flag > 0)
      {
        homing_timeout_counter++;
        if((homing_timeout_counter * current_timeout) >= (long) HOMING_TIMEOUT_USEC)
        {
          homing_timeout_counter = 0;

          printf("homing timeout\n");

          arm_set_command(0, "c", 0);

          if(arm_set_command_without_value(2, "REA") <= 0)  // Report error position
            return -1;

          arm_query_position(0);
        }
      }
      else
        homing_timeout_counter = 0;

      if(automove_timer_flag > 0)
      {
        automove_timeout_counter++;
        if((automove_timeout_counter * current_timeout) >= (long) AUTOMOVE_TIMEOUT_USEC)
        {
          automove_timeout_counter = 0;

          if(automove_file_ptr != NULL)
          {
            generate_path(45, automove_file_ptr);
            automove_file_ptr = NULL;
          }

          interpolation_status_request(0);
          // }
        }
      }
      else
        automove_timeout_counter = 0;

      if(request_timer_flag == 1)
      {
        if(request_elapsed_time_ns >= (long) REQUEST_TIMEOUT_USEC * 1000)
        {
          request_timer_flag = 0;

          struct arm_rs485_frame arm_rs485_buffer;
          arm_rs485_get_last_message_write(&arm_rs485_buffer);
          printf("Timeout[%d] on %x%s\n. \tElapsed %ld ns\n\t# of timeout: %d\n", arm_query_link,
              arm_rs485_buffer.arm_command_param.command[0],
              &arm_rs485_buffer.arm_command_param.command[1], request_elapsed_time_ns,
              arm_link[arm_query_link - 1].timeout_counter + 1);

          request_elapsed_time_ns = 0;

          arm_request_position = 0;
          arm_request_trajectory = 0;
          arm_request_interpolation = 0;
          arm_request_error_position = 0;

          arm_link[arm_query_link - 1].timeout_counter++;

          if(arm_link[arm_query_link - 1].timeout_counter >= LINK_TIMEOUT_LIMIT)
          {
            arm_link[arm_query_link - 1].timeout_counter = LINK_TIMEOUT_LIMIT;

            if((arm_out_of_service_enable_flag) && (scu_state != SCU_ARM_OUT_OF_SERVICE))
            {
              printf("Arm\t[out of service link %i]\n", arm_query_link);
              scu_state = SCU_ARM_OUT_OF_SERVICE;

              if(show_arm_state_flag)
                printf("scu state\t[SCU_ARM_OUT_OF_SERVICE] in arm_rs485_device select(rd)\n");

              arm_stop(0);
              arm_set_command_without_value(0, "OFF");

              if(scu_rqst_rtb_flag == 0)
                scu_state_rqst_flag = 1;
            }

            if(arm_link[arm_query_link - 1].position_initialized == 0)
              arm_link[arm_query_link - 1].position_initialized = 1;

            arm_position_initialized |= (arm_link[arm_query_link - 1].position_initialized
                << (arm_query_link - 1));

            arm_link[arm_query_link - 1].trajectory_status = 0;
          }

          arm_query_link = -1;
        }
      }

      if(stop_timer_flag)
      {
        stop_timeout_counter++;

        if((stop_timeout_counter * current_timeout) >= (long) STOP_TIMEOUT_USEC)
        {
          stop_timeout_counter = 0;

          if(arm_check_trajectory())
          {
            stop_timer_flag = 0;
            //arm_rs485_flush_buffer_tx();

            arm_start_xyz();

            arm_set_command_without_value(0, "OFF");

            if(scu_state != SCU_ARM_AUTO_MOVE_ABORT)
              scu_state = scu_next_state;
            else
            {
              printf("Automove aborted\n");
              scu_state = scu_prev_state;
            }

            if(show_arm_state_flag)
              printf("scu state\t[%s] in stop_timer\n", translate_state(scu_state));

            if(scu_rqst_rtb_flag == 0)
              scu_state_rqst_flag = 1;
          }
          else
          {
            arm_query_position(0);
            arm_query_trajectory(0);
          }
        }
      }
      else
        stop_timeout_counter = 0;

      return_to_base_timeout_counter++;
      if((return_to_base_timeout_counter * current_timeout)
          >= ((long) (RETURN_TO_BASE_TIMEOUT_SEC * 1000000)))
      {
        if(rtb_active_flag == 0)
          printf("Return to base active!\n");

        rtb_active_flag = 1;
        rtb_ctrl_timer_flag = 1;

      }

      if(rtb_ctrl_timer_flag)
      {
        return_to_base_ctrl_timeout_counter++;
        if((return_to_base_ctrl_timeout_counter * current_timeout)
            >= ((long) (RETURN_TO_BASE_CTRL_TIMEOUT_USEC)))
        {
          return_to_base_ctrl_timeout_counter = 0;

          switch(scu_state)
          {
            case SCU_ARM_IDLE:
              arm_command.arm_command_param.header_uint = ARM_RQST_PARK;
              arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
              break;

            case SCU_ARM_MOVE:
              arm_command.arm_command_param.header_uint = ARM_CMD_STOP;
              arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
              break;

            case SCU_ARM_STOP:
              break;

            case SCU_ARM_END_EFFECTOR_1:
              arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_1;
              arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
              break;

            case SCU_ARM_END_EFFECTOR_2:
              arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_2;
              arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
              break;

            case SCU_ARM_END_EFFECTOR_3:
              arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_3;
              arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
              break;

            case SCU_ARM_END_EFFECTOR_4:
              arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_4;
              arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
              break;

            case SCU_ARM_END_EFFECTOR_5:
              arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_5;
              arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
              break;

            case SCU_ARM_END_EFFECTOR_6:
              arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_6;
              arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
              break;

            case SCU_ARM_END_EFFECTOR_7:
              arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_7;
              arm_status_update(&scu_state, &scu_next_state, &scu_prev_state, arm_command);
              break;

            case SCU_ARM_AUTO_MOVE:
              break;

            case SCU_ARM_AUTO_MOVE_ABORT:
              break;

            case SCU_ARM_OUT_OF_SERVICE:
              break;

            case SCU_ARM_REST:
              /* Check segway state */
              if(segway_status.list.operational_state != SEGWAY_TRACTOR)
              {
                if(segway_buffer_tx_empty == 1)
                {
                  segway_init(segway_socket, &segway_address, &segway_status);
                  segway_configure_operational_mode(segway_socket, &segway_address,
                      SEGWAY_TRACTOR_REQ);
                }
                break;
              }
              robotic_arm_selected = 0;
              /* Check LMS511 state */
              if(laser_enable_flag)
              {
                if(lms511_socket < 0)
                {
                  if(lms511_open(&lms511_socket, &lms511_address, LMS511_ADDRESS, LMS511_PORT)
                      == -1)
                  {
                    perror("Init lms511 client");
                    close(lms511_socket);
                    lms511_socket = -1;
                  }
                  else
                  {
                    printf("Laser Scanner\t[connected]\n");
                    lms511_dist_flag = 5;
                    lms511_init();
                  }
                }
                else if(lms511_info.state != LMS511_MEASURE)
                {
                  lms511_login_as_auth_client();
                  lms511_start_measure();
                  lms511_logout();

                  lms511_query_status();
                  break;
                }
                else
                  lms511_scan_request();
              }

              //printf("%i\n", info.smask);
              //printf("rtb_update: magnetic_sensor_heading_true %f vs %f lat %f vs %f lon %f vs %f\n", rtb_old_direction, info.magnetic_sensor_heading_true, rtb_old_lat, info.lat, rtb_old_lon, info.lon);
              /*gps_text_log(nmea_message_log);
               nmea_message_log[0] = '\0';*/
              /*gps_compare_log(GpsCoord2Double(gps_lat), GpsCoord2Double(gps_lon),
               GpsCoord2Double(info.lat), GpsCoord2Double(info.lon),
               convert_to_float(segway_status.list.linear_vel_mps), PDOP, HDOP, vdop, sat_inview, sat_used,
               gps_direction, info.magnetic_sensor_heading_true, rover_elapsed_time_hs * 10000);*/

              if((RTBstatus.mode == RTB_tracking) || (RTBstatus.mode == RTB_idle))
              {
                if((lms511_dist_flag <= 4) && (RTBstatus.mode != RTB_idle)
                    && ((info.sig != NMEA_SIG_BAD) || (info.fix != NMEA_FIX_BAD)) && (info.PDOP < 4)
                    && (info.HDOP < 4))
                {
                  if(show_rtb_state_flag)
                  {
                    printf("\033[K");
                    printf("[Tracking] [lon %f lat %f]\r", GpsCoord2Double(rtb_old_lon),
                        GpsCoord2Double(rtb_old_lat));
                  }

                  RTB_update(GpsCoord2Double(rtb_old_lon), GpsCoord2Double(rtb_old_lat),
                      (convert_to_float(segway_status.list.linear_vel_mps) * 3.6),
                      convert_to_float(segway_status.list.inertial_z_rate_rps), 0, NULL, 0,
                      &rtb_point_catch);
                  //printf("RTB_update\n");
                  //printf("Angle: %f Divergenza: %f\n",(RTBstatus.control_vector.angle_deg_north - info.direction), (RTBstatus.control_vector.angle_deg_north - info.direction)/180);

                  // if I'm in the right direction then speed value depends on angular error. The angle threshold depends
                  // on the actual distance from last point with a min value equal to RTB_CONTROL_ANGLE_MIN. This allow to
                  // increase precision at lower distance. Angle_threshold also depends on 1/RTB_CONTROL_GAIN. This allow to
                  // have a proportional factor to adjust the formula, that is equal to control heading value with a RTB_CONTROL_GAIN
                  // factor. I can't add this variable to the final heading formula because I can't normalize after
                  angle_threshold = ((RTBstatus.distance / RTB_SAVE_DIST_TRSH)
                      * (RTB_CONTROL_ANGLE_MAX - RTB_CONTROL_ANGLE_MIN) + RTB_CONTROL_ANGLE_MIN);

                  rover_tracking_angle = info.magnetic_sensor_heading_true + 180;

                  if(rover_tracking_angle > 360)
                    rover_tracking_angle -= 360;
                  else if(rover_tracking_angle < -360)
                    rover_tracking_angle += 360;

                  rover_angle_error = RTBstatus.control_vector.angle_deg_north
                      - rover_tracking_angle;

                  if((fabs(rover_angle_error) < angle_threshold)
                      && (fabs(rover_angle_error) > -angle_threshold))
                  {
                    // heading value normalized to 1
                    RTBstatus.control_values.heading = RTB_CONTROL_ANGLE_PROP_GAIN
                        * (rover_angle_error / angle_threshold);
                    RTBstatus.control_values.speed =
                        RTB_CONTROL_SPEED_GAIN
                            * (-max((1 - fabs(RTBstatus.control_values.heading)),
                                RTB_CONTRO_MIN_SPEED));

                    //printf("Right direction!\n!");
                    //printf("Angle Error: %f\n", rover_angle_error);
                    //printf("Velocity: %f, Heading: %f\n", RTBstatus.control_values.speed, RTBstatus.control_values.heading);
                  }
                  else
                  {
                    RTBstatus.control_values.speed = -RTB_CONTRO_MIN_SPEED;

                    // Choose angle to turn. If error > 180° then turn over 360
                    if(rover_angle_error >= 0)
                    {
                      if(rover_angle_error <= 180)
                        RTBstatus.control_values.heading = RTB_CONTROL_ANGLE_PROP_GAIN;
                      else
                        RTBstatus.control_values.heading = -RTB_CONTROL_ANGLE_PROP_GAIN;
                    }
                    else
                    {
                      if(rover_angle_error >= -180)
                        RTBstatus.control_values.heading = -RTB_CONTROL_ANGLE_PROP_GAIN;
                      else
                        RTBstatus.control_values.heading = RTB_CONTROL_ANGLE_PROP_GAIN;
                    }
                    //printf("Wrong direction\n");
                    //printf("Angle Error: %f\n", rover_angle_error);
                    //printf("Velocity: %f, Heading: %f\n", RTBstatus.control_values.speed, RTBstatus.control_values.heading);
                  }
                }
                else
                {
                  if(show_rtb_state_flag)
                  {
                    printf("\033[K");

                    if(lms511_dist_flag > 4)
                      printf("[Obstacle] [lon %f lat %f]\r", GpsCoord2Double(rtb_old_lon),
                          GpsCoord2Double(rtb_old_lat));
                    else
                      printf("[Idle] [lon %f lat %f]\r", GpsCoord2Double(rtb_old_lon),
                          GpsCoord2Double(rtb_old_lat));
                  }

                  if(((info.sig == NMEA_SIG_BAD) || (info.fix == NMEA_FIX_BAD)) || (info.PDOP > 4)
                      || (info.HDOP > 4))
                    printf("GPS signal bad\n");

                  RTBstatus.control_values.heading = 0;
                  RTBstatus.control_values.speed = 0;
                }
              }
              else
                RTB_set_mode(RTB_tracking);
              break;
          }
        }
      }

      if(airpump_enable_flag == 1)
        airpump_timeout_counter++;

      if(airpump_timeout_counter * current_timeout >= ((long) AIRPUMP_TIMEOUT_SEC * 1000000))
      {
        airpump_enable_flag = 0;
        airpump_timeout_counter = 0;

        gpio_set_value(AIR_PUMP_GPIO, 0);
      }

      if(arm_position_initialized == ((1 << MOTOR_NUMBER) - 1))
      {
        arm_position_timeout_counter++;
        if(arm_position_timeout_counter * current_timeout >= (long) ARM_POSITION_TIMEOUT_USEC)
        {
          arm_position_timeout_counter = 0;
          if(ccu_socket > 0)
          {
            sprintf(arm_position_for_ccu, "1%ld 2%ld 3%ld 4%ld 5%ld 6%ld ",
                arm_link[0].actual_position, arm_link[1].actual_position,
                arm_link[2].actual_position, arm_link[3].actual_position,
                arm_link[4].actual_position, arm_link[5].actual_position);

            bytes_sent = sendto(ccu_socket, arm_position_for_ccu, strlen(arm_position_for_ccu), 0,
                (struct sockaddr *) &ccu_socket_addr_dest, sizeof(ccu_socket_addr_dest));

            if(bytes_sent < 0)
              perror("sendto ccu");
          }
        }
      }

      arm_battery_timeout_counter++;
      if((arm_battery_timeout_counter * current_timeout)
          >= (long) ARM_BATTERY_TIMEOUT_SEC * 1000000)
      {
        arm_battery_timeout_counter = 0;

        if(arm_battery_fd == -1)
          arm_battery_fd = open("/sys/devices/platform/omap/tsc/ain5", O_RDONLY);

        if(arm_battery_fd < 0)
          printf("ADC file for battery\t[FAILED]\n");
        else if(arm_battery_file == NULL)
        {
          arm_battery_file = fdopen(arm_battery_fd, "r");

          if(arm_battery_file == NULL)
            perror("arm_batter_file");

          battery_read_flag = 1;
        }
      }

      select_timeout.tv_sec = TIMEOUT_SEC;
      select_timeout.tv_usec = ARM_TIMEOUT_USEC;

      current_timeout = select_timeout.tv_usec;
    }
  }  // end while(!= done)

  return 0;
}

void arm_status_update(unsigned char *arm_state, unsigned char *arm_next_state,
    unsigned char *arm_prev_state, struct arm_frame arm_message)
{
  unsigned char automatic_motion_flag = -1;

  static unsigned char arm_stop_flag = 0;
  static unsigned char arm_abort_flag = 0;

  static float x = 0;
  static float y = 0;
  static float z = 0;

  long motor_step[MOTOR_NUMBER];
  int motor_index;

  switch(*arm_state)
  {

    case SCU_ARM_IDLE:
      if(arm_stop_flag)
        arm_stop_flag = 0;

      if(arm_abort_flag)
        arm_abort_flag = 0;

      switch(arm_message.arm_command_param.header_uint)
      {
        case ARM_CMD_FIRST_TRIPLET:
        case ARM_CMD_SECOND_TRIPLET:
        case ARM_CMD_ACTUATOR:
          for(motor_index = 0; motor_index < 3; motor_index++)
            motor_step[motor_index] = arm_link[motor_index].actual_position;

          arm_ee_xyz(motor_step, &x, &y, &z);

          arm_start_xyz();
          arm_query_position(0);

          *arm_state = SCU_ARM_MOVE;
          *arm_next_state = SCU_ARM_IDLE;

          if(show_arm_state_flag)
            printf("scu state\t[SCU_ARM_MOVE] in arm_status_update after ARM_CMD_MOVEs\n");

          if(scu_rqst_rtb_flag == 0)
            scu_state_rqst_flag = 1;
          break;

        case ARM_CMD_STOP:
          break;

        case ARM_RQST_PARK:
          init_interpolation_mode(ARM_PARK_FILE);
          *arm_prev_state = *arm_state;
          *arm_state = SCU_ARM_AUTO_MOVE;
          *arm_next_state = SCU_ARM_REST;

          if(show_arm_state_flag)
            printf("scu state\t[SCU_ARM_AUTO_MOVE] in arm_status_update after ARM_RQST_PARK\n");

          automove_timer_flag = 1;

          if(scu_rqst_rtb_flag == 0)
            scu_state_rqst_flag = 1;

          printf("Arm is going to parking position. . .\n");
          break;

        case ARM_RQST_PARK_CLASSA:
          init_interpolation_mode(ARM_PARK_CLASSA_FILE);
          *arm_prev_state = *arm_state;
          *arm_state = SCU_ARM_AUTO_MOVE;
          *arm_next_state = SCU_ARM_IDLE;

          if(show_arm_state_flag)
            printf(
                "scu state\t[SCU_ARM_AUTO_MOVE] in arm_status_update after ARM_RQST_PARK_CLASSA\n");

          automove_timer_flag = 1;

          if(scu_rqst_rtb_flag == 0)
            scu_state_rqst_flag = 1;

          printf("Arm is going to parking classA position. . .\n");

          break;

        case ARM_RQST_STEADY:
          init_interpolation_mode(ARM_READY_FILE);
          *arm_prev_state = *arm_state;
          *arm_state = SCU_ARM_AUTO_MOVE;
          *arm_next_state = SCU_ARM_IDLE;

          if(show_arm_state_flag)
            printf("scu state\t[SCU_ARM_AUTO_MOVE] in arm_status_update after f\n");

          automove_timer_flag = 1;

          if(scu_rqst_rtb_flag == 0)
            scu_state_rqst_flag = 1;

          printf("Arm is going to ready position. . .\n");
          break;

        case ARM_RQST_DINAMIC:
          init_interpolation_mode(ARM_DINAMIC_FILE);
          *arm_prev_state = *arm_state;
          *arm_state = SCU_ARM_AUTO_MOVE;
          *arm_next_state = SCU_ARM_IDLE;

          if(show_arm_state_flag)
            printf("scu state\t[SCU_ARM_AUTO_MOVE] in arm_status_update after ARM_RQST_DINAMIC\n");

          automove_timer_flag = 1;

          if(scu_rqst_rtb_flag == 0)
            scu_state_rqst_flag = 1;

          printf("Arm is going to dinamic position. . .\n");
          break;

        case ARM_RQST_STEP:
          break;

        case ARM_RQST_GET_TOOL_1:
        case ARM_RQST_GET_TOOL_2:
        case ARM_RQST_GET_TOOL_3:
        case ARM_RQST_GET_TOOL_4:
        case ARM_RQST_GET_TOOL_5:
        case ARM_RQST_GET_TOOL_6:
        case ARM_RQST_GET_TOOL_7:
          *arm_prev_state = *arm_state;

          if(arm_message.arm_command_param.header_uint == ARM_RQST_GET_TOOL_1)
          {
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_GET_BOX1_FILE);
            init_interpolation_mode(ARM_GET_BOX1_FILE);
            *arm_next_state = SCU_ARM_END_EFFECTOR_1;
          }
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_GET_TOOL_2)
          {
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_GET_BOX2_FILE);
            init_interpolation_mode(ARM_GET_BOX2_FILE);
            *arm_next_state = SCU_ARM_END_EFFECTOR_2;
          }
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_GET_TOOL_3)
          {
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_GET_BOX3_FILE);
            init_interpolation_mode(ARM_GET_BOX3_FILE);
            *arm_next_state = SCU_ARM_END_EFFECTOR_3;
          }
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_GET_TOOL_4)
          {
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_GET_BOX4_FILE);
            init_interpolation_mode(ARM_GET_BOX4_FILE);
            *arm_next_state = SCU_ARM_END_EFFECTOR_4;
          }
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_GET_TOOL_5)
          {
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_GET_BOX5_FILE);
            init_interpolation_mode(ARM_GET_BOX5_FILE);
            *arm_next_state = SCU_ARM_END_EFFECTOR_5;
          }
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_GET_TOOL_6)
          {
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_GET_BOX6_FILE);
            init_interpolation_mode(ARM_GET_BOX6_FILE);
            *arm_next_state = SCU_ARM_END_EFFECTOR_6;
          }
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_GET_TOOL_7)
          {
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_GET_BOX7_FILE);
            init_interpolation_mode(ARM_GET_BOX7_FILE);
            *arm_next_state = SCU_ARM_END_EFFECTOR_7;
          }

          *arm_state = SCU_ARM_AUTO_MOVE;

          if(show_arm_state_flag)
            printf(
                "scu state\t[SCU_ARM_AUTO_MOVE] in arm_status_update after ARM_RQST_GET_TOOL_X\n");

          automove_timer_flag = 1;

          if(scu_rqst_rtb_flag == 0)
            scu_state_rqst_flag = 1;

          printf("Arm is going to get a tool. . .\n");
          break;

        case ARM_RQST_PUMP:
          if(airpump_enable_flag == -1)
          {
            airpump_enable_flag = 1;

            gpio_set_value(AIR_PUMP_GPIO, 1);

            printf("Arm is going to start pump\n");
          }
          else
            printf("Pump not avaible. . . \n");

          *arm_state = SCU_ARM_IDLE;
          *arm_next_state = SCU_ARM_IDLE;

          if(show_arm_state_flag)
            printf("scu state\t[SCU_ARM_IDLE] in arm_status_update after ARM_RQST_PUMP\n");

          break;

        case ARM_RQST_PUT_TOOL_1:
        case ARM_RQST_PUT_TOOL_2:
        case ARM_RQST_PUT_TOOL_3:
        case ARM_RQST_PUT_TOOL_4:
        case ARM_RQST_PUT_TOOL_5:
        case ARM_RQST_PUT_TOOL_6:
        case ARM_RQST_PUT_TOOL_7:
          break;

        case ARM_RQST_ABORT:
          break;

        case ARM_SET_ORIGIN:
          printf("Warning: predefined point set!\n");

          // store position
          arm_set_command(1, "O", -435209); //new: -435209.4888 old: -435300
          arm_set_command(1, "p", -435209);
          arm_set_command(1, "EPTR", 100);
          arm_set_command_without_value(1, "VST(p,1)");

          arm_set_command(2, "O", 1139225); //new: 1139225.309 old: 1143856
          arm_set_command(2, "p", 1139225);
          arm_set_command(2, "EPTR", 100);
          arm_set_command_without_value(2, "VST(p,1)");

          arm_set_command(3, "O", -290972); //new: -290971.8083 old: -291687
          arm_set_command(3, "p", -291687);
          arm_set_command(3, "EPTR", 100);
          arm_set_command_without_value(3, "VST(p,1)");

          arm_set_command(4, "O", 0);  //new: 0 old:-508
          arm_set_command(4, "p", 0);
          arm_set_command(4, "EPTR", 100);
          arm_set_command_without_value(4, "VST(p,1)");

          arm_set_command(5, "O", 251534);
          arm_set_command(5, "p", 251534);
          arm_set_command(5, "EPTR", 100);
          arm_set_command_without_value(5, "VST(p,1)");

          arm_set_command(6, "O", -252424);
          arm_set_command(6, "p", -252424);
          arm_set_command(6, "EPTR", 100);
          arm_set_command_without_value(6, "VST(p,1)");
          break;
      }
      break;

    case SCU_ARM_END_EFFECTOR_1:
    case SCU_ARM_END_EFFECTOR_2:
    case SCU_ARM_END_EFFECTOR_3:
    case SCU_ARM_END_EFFECTOR_4:
    case SCU_ARM_END_EFFECTOR_5:
    case SCU_ARM_END_EFFECTOR_6:
    case SCU_ARM_END_EFFECTOR_7:
      if(arm_stop_flag)
        arm_stop_flag = 0;

      if(arm_abort_flag)
        arm_abort_flag = 0;

      switch(arm_message.arm_command_param.header_uint)
      {
        case ARM_CMD_FIRST_TRIPLET:
        case ARM_CMD_SECOND_TRIPLET:
        case ARM_CMD_ACTUATOR:
          for(motor_index = 0; motor_index < 3; motor_index++)
            motor_step[motor_index] = arm_link[motor_index].actual_position;

          arm_ee_xyz(motor_step, &x, &y, &z);
          arm_start_xyz();

          arm_query_position(0);
          *arm_next_state = *arm_state;
          *arm_state = SCU_ARM_MOVE;

          if(show_arm_state_flag)
            printf("scu state\t[SCU_ARM_MOVE] in arm_status_update after ARM_CMD_MOVE\n");

          if(scu_rqst_rtb_flag == 0)
            scu_state_rqst_flag = 1;
          break;

        case ARM_CMD_STOP:
          break;

        case ARM_RQST_PARK:
        case ARM_RQST_PARK_CLASSA:
        case ARM_RQST_STEADY:
        case ARM_RQST_DINAMIC:
        case ARM_RQST_STEP:
        case ARM_RQST_GET_TOOL_1:
        case ARM_RQST_GET_TOOL_2:
        case ARM_RQST_GET_TOOL_3:
        case ARM_RQST_GET_TOOL_4:
        case ARM_RQST_GET_TOOL_5:
        case ARM_RQST_GET_TOOL_6:
        case ARM_RQST_GET_TOOL_7:
        case ARM_RQST_PUT_TOOL_1:
        case ARM_RQST_PUT_TOOL_2:
        case ARM_RQST_PUT_TOOL_3:
        case ARM_RQST_PUT_TOOL_4:
        case ARM_RQST_PUT_TOOL_5:
        case ARM_RQST_PUT_TOOL_6:
        case ARM_RQST_PUT_TOOL_7:
          /*arm_ee_xyz(&x, &y, &z);

           // tuning motor 1
           arm_set_max_velocity(1, 700);*/

          if(arm_message.arm_command_param.header_uint == ARM_RQST_PUT_TOOL_1)
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_PUT_BOX1_FILE);
            init_interpolation_mode(ARM_PUT_BOX1_FILE);
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_PUT_TOOL_2)
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_PUT_BOX2_FILE);
            init_interpolation_mode(ARM_PUT_BOX2_FILE);
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_PUT_TOOL_3)
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_PUT_BOX3_FILE);
            init_interpolation_mode(ARM_PUT_BOX3_FILE);
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_PUT_TOOL_4)
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_PUT_BOX4_FILE);
            init_interpolation_mode(ARM_PUT_BOX4_FILE);
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_PUT_TOOL_5)
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_PUT_BOX5_FILE);
            init_interpolation_mode(ARM_PUT_BOX5_FILE);
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_PUT_TOOL_6)
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_PUT_BOX6_FILE);
            init_interpolation_mode(ARM_PUT_BOX6_FILE);
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_PUT_TOOL_7)
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_PUT_BOX7_FILE);
            init_interpolation_mode(ARM_PUT_BOX7_FILE);

          /*if(automatic_motion_flag > 0)
           {*/
          *arm_prev_state = *arm_state;
          *arm_state = SCU_ARM_AUTO_MOVE;
          automove_timer_flag = 1;

          if(scu_rqst_rtb_flag == 0)
            scu_state_rqst_flag = 1;

          printf("Arm is going to put a tool. . .\n");

          if(show_arm_state_flag)
            printf("scu state\t[SCU_ARM_AUTO_MOVE] in arm_status_update after ARM_RQST_\n");

          *arm_next_state = SCU_ARM_IDLE;
          break;

        case ARM_RQST_ABORT:
          break;

        case ARM_SET_ORIGIN:
          break;
      }
      break;

    case SCU_ARM_REST:
      switch(arm_message.arm_command_param.header_uint)
      {
        case ARM_CMD_FIRST_TRIPLET:
        case ARM_CMD_SECOND_TRIPLET:
        case ARM_CMD_ACTUATOR:
          break;

        case ARM_CMD_STOP:
          break;

        case ARM_RQST_PARK:
          break;

        case ARM_RQST_PARK_CLASSA:
          init_interpolation_mode(ARM_PARK_CLASSA_FILE);
          *arm_prev_state = *arm_state;
          *arm_state = SCU_ARM_AUTO_MOVE;
          *arm_next_state = SCU_ARM_IDLE;
          automove_timer_flag = 1;

          if(scu_rqst_rtb_flag == 0)
            scu_state_rqst_flag = 1;

          printf("Arm is going to parking classA position. . .\n");

          if(show_arm_state_flag)
            printf(
                "scu state\t[SCU_ARM_AUTO_MOVE] in arm_status_update after ARM_RQST_PARK_CLASSA\n");
          break;

        case ARM_RQST_STEADY:
          init_interpolation_mode(ARM_GET_OUT_FROM_REST_FILE);

          *arm_prev_state = SCU_ARM_IDLE;
          *arm_state = SCU_ARM_AUTO_MOVE;
          *arm_next_state = SCU_ARM_IDLE;
          //automove_timer_flag = automatic_motion_flag;
          automove_timer_flag = 1;

          if(scu_rqst_rtb_flag == 0)
            scu_state_rqst_flag = 1;

          printf("Arm is going to ready position. . .\n");

          if(show_arm_state_flag)
            printf("scu state\t[SCU_ARM_AUTO_MOVE] in arm_status_update after ARM_RQST_STEADY\n");
          break;

        case ARM_RQST_DINAMIC:
          init_interpolation_mode(ARM_DINAMIC_FILE);
          *arm_prev_state = *arm_state;
          *arm_state = SCU_ARM_AUTO_MOVE;
          *arm_next_state = SCU_ARM_IDLE;
          automove_timer_flag = 1;

          if(scu_rqst_rtb_flag == 0)
            scu_state_rqst_flag = 1;

          printf("Arm is going to dinamic position. . .\n");

          if(show_arm_state_flag)
            printf("scu state\t[SCU_ARM_AUTO_MOVE] in arm_status_update after ARM_RQST_DINAMIC\n");
          break;

        case ARM_RQST_STEP:
          init_interpolation_mode(ARM_STEP_FILE);
          *arm_prev_state = *arm_state;
          *arm_state = SCU_ARM_AUTO_MOVE;
          *arm_next_state = SCU_ARM_IDLE;
          automove_timer_flag = 1;

          if(scu_rqst_rtb_flag == 0)
            scu_state_rqst_flag = 1;

          printf("Arm is going to step position. . .\n");

          if(show_arm_state_flag)
            printf("scu state\t[SCU_ARM_AUTO_MOVE] in arm_status_update after ARM_RQST_STEP\n");
          break;

        case ARM_RQST_GET_TOOL_1:
        case ARM_RQST_GET_TOOL_2:
        case ARM_RQST_GET_TOOL_3:
        case ARM_RQST_GET_TOOL_4:
        case ARM_RQST_GET_TOOL_5:
        case ARM_RQST_GET_TOOL_6:
        case ARM_RQST_GET_TOOL_7:
          *arm_prev_state = *arm_state;
          if(arm_message.arm_command_param.header_uint == ARM_RQST_GET_TOOL_1)
          {
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_GET_BOX1_FILE);
            init_interpolation_mode(ARM_GET_BOX1_FILE);
            *arm_next_state = SCU_ARM_END_EFFECTOR_1;
          }
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_GET_TOOL_2)
          {
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_GET_BOX2_FILE);
            init_interpolation_mode(ARM_GET_BOX2_FILE);
            *arm_next_state = SCU_ARM_END_EFFECTOR_2;
          }
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_GET_TOOL_3)
          {
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_GET_BOX3_FILE);
            init_interpolation_mode(ARM_GET_BOX3_FILE);
            *arm_next_state = SCU_ARM_END_EFFECTOR_3;
          }
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_GET_TOOL_4)
          {
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_GET_BOX4_FILE);
            init_interpolation_mode(ARM_GET_BOX4_FILE);
            *arm_next_state = SCU_ARM_END_EFFECTOR_4;
          }
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_GET_TOOL_5)
          {
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_GET_BOX5_FILE);
            init_interpolation_mode(ARM_GET_BOX5_FILE);
            *arm_next_state = SCU_ARM_END_EFFECTOR_5;
          }
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_GET_TOOL_6)
          {
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_GET_BOX6_FILE);
            init_interpolation_mode(ARM_GET_BOX6_FILE);
            *arm_next_state = SCU_ARM_END_EFFECTOR_6;
          }
          else if(arm_message.arm_command_param.header_uint == ARM_RQST_GET_TOOL_7)
          {
            //automatic_motion_flag = arm_automatic_motion_xyz_start(ARM_GET_BOX7_FILE);
            init_interpolation_mode(ARM_GET_BOX7_FILE);
            *arm_next_state = SCU_ARM_END_EFFECTOR_7;
          }

          /*if(automatic_motion_flag > 0)
           { */
          *arm_state = SCU_ARM_AUTO_MOVE;
          automove_timer_flag = 1;

          if(scu_rqst_rtb_flag == 0)
            scu_state_rqst_flag = 1;

          printf("Arm is going to get a tool. . .\n");

          if(show_arm_state_flag)
            printf(
                "scu state\t[SCU_ARM_AUTO_MOVE] in arm_status_update after ARM_RQST_GET_TOOL_\n");
          break;

        case ARM_RQST_PUMP:
          if(airpump_enable_flag == -1)
          {
            airpump_enable_flag = 1;

            gpio_set_value(AIR_PUMP_GPIO, 1);

            printf("Arm is going to start pump\n");
          }
          else
            printf("Pump not avaible. . . \n");

          *arm_state = SCU_ARM_IDLE;
          *arm_next_state = SCU_ARM_IDLE;

          if(show_arm_state_flag)
            printf("scu state\t[SCU_ARM_IDLE] in arm_status_update after ARM_RQST_PUMP\n");
          break;

        case ARM_RQST_PUT_TOOL_1:
        case ARM_RQST_PUT_TOOL_2:
        case ARM_RQST_PUT_TOOL_3:
        case ARM_RQST_PUT_TOOL_4:
        case ARM_RQST_PUT_TOOL_5:
        case ARM_RQST_PUT_TOOL_6:
        case ARM_RQST_PUT_TOOL_7:
          break;

        case ARM_RQST_ABORT:
          break;

        case ARM_SET_ORIGIN:
          break;
      }
      break;

    case SCU_ARM_MOVE:
      switch(arm_message.arm_command_param.header_uint)
      {
        case ARM_CMD_FIRST_TRIPLET:
          //y += arm_message.arm_command_param.value2 * ARM_JOINT_YZ_STEP_M;
          z += arm_message.arm_command_param.value3 * ARM_JOINT_YZ_STEP_M;

          arm_move_xyz(1, arm_message.arm_command_param.value1,
              arm_message.arm_command_param.value2 * ARM_JOINT_YZ_STEP_M, z);

          arm_query_position(1);
          arm_query_position(2);
          arm_query_position(3);
          arm_query_position(4);
          arm_query_position(5);
          arm_query_position(6);
          break;

        case ARM_CMD_SECOND_TRIPLET:
          arm_move_xyz(2, arm_message.arm_command_param.value1,
              arm_message.arm_command_param.value2, arm_message.arm_command_param.value3);
          arm_query_position(4);
          arm_query_position(5);
          arm_query_position(6);
          break;

        case ARM_CMD_ACTUATOR:
          arm_move_xyz(3, arm_message.arm_command_param.value1,
              arm_message.arm_command_param.value2, arm_message.arm_command_param.value3);
          arm_query_position(7);
          break;

        case ARM_CMD_STOP:
          // flush tx buffer
          arm_rs485_flush_buffer_tx();

          // load stop command
          arm_stop(0);
          arm_query_position(0);
          arm_query_trajectory(0);

          *arm_state = SCU_ARM_STOP;

          if(scu_rqst_rtb_flag == 0)
            scu_state_rqst_flag = 1;

          stop_timer_flag = 1;

          if(show_arm_state_flag)
            printf("scu state\t[SCU_ARM_STOP] in arm_status_update after ARM_CMD_STOP\n");
          break;

        case ARM_RQST_PARK:
          break;

        case ARM_RQST_PARK_CLASSA:
          break;

        case ARM_RQST_STEADY:
          break;

        case ARM_RQST_DINAMIC:
          break;

        case ARM_RQST_STEP:
          break;

        case ARM_RQST_GET_TOOL_1:
        case ARM_RQST_GET_TOOL_2:
        case ARM_RQST_GET_TOOL_3:
        case ARM_RQST_GET_TOOL_4:
        case ARM_RQST_GET_TOOL_5:
        case ARM_RQST_GET_TOOL_6:
        case ARM_RQST_GET_TOOL_7:
          break;

        case ARM_RQST_PUMP:
          break;

        case ARM_RQST_PUT_TOOL_1:
        case ARM_RQST_PUT_TOOL_2:
        case ARM_RQST_PUT_TOOL_3:
        case ARM_RQST_PUT_TOOL_4:
        case ARM_RQST_PUT_TOOL_5:
        case ARM_RQST_PUT_TOOL_6:
        case ARM_RQST_PUT_TOOL_7:
          break;

        case ARM_RQST_ABORT:
          break;

        case ARM_SET_ORIGIN:
          break;
      }
      break;

    case SCU_ARM_STOP:
      switch(arm_message.arm_command_param.header_uint)
      {
        case ARM_CMD_FIRST_TRIPLET:
          break;

        case ARM_CMD_SECOND_TRIPLET:
          break;

        case ARM_CMD_ACTUATOR:
          break;

        case ARM_CMD_STOP:
          break;

        case ARM_RQST_PARK:
          break;

        case ARM_RQST_PARK_CLASSA:
          break;

        case ARM_RQST_STEADY:
          break;

        case ARM_RQST_DINAMIC:
          break;

        case ARM_RQST_STEP:
          break;

        case ARM_RQST_GET_TOOL_1:
        case ARM_RQST_GET_TOOL_2:
        case ARM_RQST_GET_TOOL_3:
        case ARM_RQST_GET_TOOL_4:
        case ARM_RQST_GET_TOOL_5:
        case ARM_RQST_GET_TOOL_6:
        case ARM_RQST_GET_TOOL_7:
          break;

        case ARM_RQST_PUMP:
          break;

        case ARM_RQST_PUT_TOOL_1:
        case ARM_RQST_PUT_TOOL_2:
        case ARM_RQST_PUT_TOOL_3:
        case ARM_RQST_PUT_TOOL_4:
        case ARM_RQST_PUT_TOOL_5:
        case ARM_RQST_PUT_TOOL_6:
        case ARM_RQST_PUT_TOOL_7:
          break;

        case ARM_RQST_ABORT:
          break;

        case ARM_SET_ORIGIN:
          break;
      }
      break;

    case SCU_ARM_AUTO_MOVE_ABORT:
      switch(arm_message.arm_command_param.header_uint)
      {
        case ARM_CMD_FIRST_TRIPLET:
          break;

        case ARM_CMD_SECOND_TRIPLET:
          break;

        case ARM_CMD_ACTUATOR:
          break;

        case ARM_CMD_STOP:
          break;

        case ARM_RQST_PARK:
          break;

        case ARM_RQST_PARK_CLASSA:
          break;

        case ARM_RQST_STEADY:
          break;

        case ARM_RQST_DINAMIC:
          break;

        case ARM_RQST_STEP:
          break;

        case ARM_RQST_GET_TOOL_1:
        case ARM_RQST_GET_TOOL_2:
        case ARM_RQST_GET_TOOL_3:
        case ARM_RQST_GET_TOOL_4:
        case ARM_RQST_GET_TOOL_5:
        case ARM_RQST_GET_TOOL_6:
        case ARM_RQST_GET_TOOL_7:
          break;

        case ARM_RQST_PUMP:
          break;

        case ARM_RQST_PUT_TOOL_1:
        case ARM_RQST_PUT_TOOL_2:
        case ARM_RQST_PUT_TOOL_3:
        case ARM_RQST_PUT_TOOL_4:
        case ARM_RQST_PUT_TOOL_5:
        case ARM_RQST_PUT_TOOL_6:
        case ARM_RQST_PUT_TOOL_7:
          break;

        case ARM_RQST_ABORT:
          break;

        case ARM_SET_ORIGIN:
          break;
      }
      break;

    case SCU_ARM_AUTO_MOVE:
      switch(arm_message.arm_command_param.header_uint)
      {
        case ARM_CMD_FIRST_TRIPLET:
          break;

        case ARM_CMD_SECOND_TRIPLET:
          break;

        case ARM_CMD_ACTUATOR:
          break;

        case ARM_CMD_STOP:
          break;

        case ARM_RQST_PARK:
          break;

        case ARM_RQST_PARK_CLASSA:
          break;

        case ARM_RQST_STEADY:
          break;

        case ARM_RQST_DINAMIC:
          break;

        case ARM_RQST_STEP:
          break;

        case ARM_RQST_GET_TOOL_1:
        case ARM_RQST_GET_TOOL_2:
        case ARM_RQST_GET_TOOL_3:
        case ARM_RQST_GET_TOOL_4:
        case ARM_RQST_GET_TOOL_5:
        case ARM_RQST_GET_TOOL_6:
        case ARM_RQST_GET_TOOL_7:
          break;

        case ARM_RQST_PUMP:
          break;

        case ARM_RQST_PUT_TOOL_1:
        case ARM_RQST_PUT_TOOL_2:
        case ARM_RQST_PUT_TOOL_3:
        case ARM_RQST_PUT_TOOL_4:
        case ARM_RQST_PUT_TOOL_5:
        case ARM_RQST_PUT_TOOL_6:
        case ARM_RQST_PUT_TOOL_7:
          break;

        case ARM_RQST_ABORT:
          // flush tx buffer
          //arm_rs485_flush_buffer_tx();

          // load stop command
          arm_stop(0);
          arm_automatic_motion_abort();

          arm_query_position(0);
          arm_query_trajectory(0);
          *arm_state = SCU_ARM_AUTO_MOVE_ABORT;

          printf("Aborting automove. . .\n");

          if(show_arm_state_flag)
            printf(
                "scu state\t[SCU_ARM_AUTO_MOVE_ABORT] in arm_status_update after ARM_RQST_ABORT\n");

          automove_timer_flag = 0;
          homing_timer_flag = 0;

          if(scu_rqst_rtb_flag == 0)
            scu_state_rqst_flag = 1;

          stop_timer_flag = 1;
          break;

        case ARM_SET_ORIGIN:
          break;
      }
      break;
  }
}

void segway_status_update(union segway_union *segway_status, int socket,
    struct sockaddr_in *segway_address, long int joy_max_value)
{
  static int segway_previouse_state = 0;
  int bytes_sent = -1;

  switch(segway_status->list.operational_state)
  {
    case CCU_INIT:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        printf("Segway CCU Init\n");

        segway_previouse_state = segway_status->list.operational_state;
      }
      break;

    case PROPULSION_INIT:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        printf("Segway Propulsion Init\n");

        segway_previouse_state = segway_status->list.operational_state;
      }

      break;

    case CHECK_STARTUP:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        printf("Segway Check Startup Issue\n");

        segway_previouse_state = segway_status->list.operational_state;
      }
      break;

    case SEGWAY_STANDBY: //standby mode
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        printf("Segway in Standby Mode\n");

        segway_previouse_state = segway_status->list.operational_state;
      }
      break;

    case SEGWAY_TRACTOR:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        printf("Segway in Tractor Mode\n");

        segway_previouse_state = segway_status->list.operational_state;
      }

      bytes_sent = segway_motion_set(socket, segway_address, RTBstatus.control_values.speed,
          RTBstatus.control_values.heading, 1);

      if(bytes_sent < 0)
        perror("segway_motion_set");
      break;

    case DISABLE_POWER:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        printf("Segway Disable Power\n");

        segway_previouse_state = segway_status->list.operational_state;
      }
      break;

    default:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        printf("Segway Unknown State\n");

        segway_previouse_state = segway_status->list.operational_state;
      }
      break;
  } //end switch
}

void gps_compare_log(double latitude, double longitude, double latitude_odometry,
    double longitude_odometry, double velocity, double PDOP, double HDOP, double vdop,
    int sat_inview, int sat_used, double direction, double direction_bussola, long time_us)
{
  FILE *file = NULL;

// Init Log File
  file = fopen("gps_compare_log", "a");

  if(!file)
  {
    perror("logfile fopen:");
    return;
  }

  fprintf(file, "%f,%f,0,%f,%f,0,%f,%f,%f,%f,%d,%d,%f,%f,%ld\n", longitude, latitude,
      longitude_odometry, latitude_odometry, velocity, PDOP, HDOP, vdop, sat_inview, sat_used,
      direction, direction_bussola, time_us);

  fclose(file);
}

void gps_text_log(char *string)
{
  FILE *file = NULL;

// Init Log File
  file = fopen("gps_log.txt", "a");

  if(!file)
  {
    perror("logfile fopen:");
    return;
  }

  fprintf(file, "%s", string);

  fclose(file);
}

int eth_check_connection()
{
  FILE *file = NULL;
  char *line = NULL;
  size_t len = 0;
  ssize_t read;

  file = fopen("/sys/class/net/eth0/operstate", "r");

  if(file == NULL)
    return -1;

  read = getline(&line, &len, file);

  if(read != -1)
  {
    if(strncmp(line, "up", strlen("up")) == 0)
    {
      free(line);
      fclose(file);
      return 1;
    }
  }

  free(line);
  fclose(file);

  return 0;
}

int gpio_export(int pin_number)
{
  FILE *file = NULL;

  file = fopen("/sys/class/gpio/export", "a");

  if(file == NULL)
    return -1;

  fprintf(file, "%i", pin_number);

  fclose(file);
  return 1;
}

int gpio_set_value(int pin_number, int value)
{
  FILE *file = NULL;
  char file_path[64];

  sprintf(file_path, "/sys/class/gpio/gpio%i/value", pin_number);
  file = fopen(file_path, "a");

  if(file == NULL)
    return -1;

  fprintf(file, "%i", value);

  fclose(file);
  return 1;
}

int gpio_set_direction(int pin_number, int value)
{
  FILE *file = NULL;
  char file_path[64];

  sprintf(file_path, "/sys/class/gpio/gpio%i/direction", pin_number);
  file = fopen(file_path, "a");

  if(file == NULL)
    return -1;

  if(value)
    fprintf(file, "in");
  else
    fprintf(file, "out");

  fclose(file);
  return 1;
}

int gpio_generic_set_value(char *path, int value)
{
  FILE *file = NULL;

  file = fopen(path, "a");

  if(file == NULL)
    return -1;

  fprintf(file, "%i", value);

  fclose(file);
  return 1;
}
