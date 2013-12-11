#include <sys/types.h> 
#include <sys/socket.h> 
#include <sys/ioctl.h> 
#include <sys/stat.h> 
#include <fcntl.h> 
#include <net/if.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h>

#include <sys/time.h> 
#include <time.h> 
#include <errno.h>

#include <locale.h>

#include <string.h>
#include <math.h>

#include <netinet/in.h> 
#include <arpa/inet.h>

#include "segway_config.h" 
#include "joystick.h" 
#include "segway_udp_v2.h" 
#include "arm_udp.h"
#include "nmea/nmea.h"
#include "rover_rtb.h"
#include "gps_generate.h"
#include "rs232.h"

#define GPS
//#define GPS_DEBUG
//#define GPS_SIMUL

#define ARM_MAX_POSITION 0.1 // value in degree under which the link will be stopped

#define CCU_ADDRESS "192.168.1.102"
#define CCU_PORT_ARM 8000
#define SEGWAY_ADDRESS "192.168.1.40"
#define SEGWAY_PORT 55
#define ARM_ADDRESS "192.168.1.28"
#define ARM_PORT 8012
#define ACU_ARM_PORT 8015
#define ACU_STATUS_PORT 8016
#define ACU_SEGWAY_PORT 8017

#ifdef GPS
#define ACU_GPS_PORT 8017
#endif

#define RTB_CONTROL_ANGLE_MAX 45
#define RTB_CONTROL_ANGLE_MIN 20

#define ARM_PRESCALER 2
#define JOY_MAX_VALUE 32767 
#define JOYSTICK_ADDRESS "192.168.1.60"
#define JOYSTICK_STATUS_PORT 8014
#define TIMEOUT_SEC 0 
#define TIMEOUT_USEC_SEGWAY 100000
#define TIMEOUT_USEC_ARM 10000
#define TIMEOUT_SEC_RETURN_TO_BASE 30
#define TIMEOUT_USEC_STATUS 500000
#define LOG_FILE "/var/log/automatic_control_unit"

// ACU status
#define ACU_IDLE 0
#define ACU_HOME 1
#define ACU_ARM_HOMING 2
#define ACU_ARM_AUTO_MOVE 3
#define ACU_RETURN_TO_BASE 4
#define ACU_RETURN_TO_BASE_ABORT 5
#define ACU_ARM_AUTO_MOVE_ABORT 6
#define ACU_UNKNOWN 7

// Joystick command
#define JOYSTICK_NONE 0
#define JOYSTICK_REQUEST_HOMING 1
#define JOYSTICK_REQUEST_DINAMIC 2
#define JOYSTICK_ABORT_AUTO_MOVE 3
#define JOYSTICK_REQUEST_BOX 4
#define JOYSTICK_REQUEST_PARK 5
#define JOYSTICK_REQUEST_STEP 6

/* Macro */
#undef max 
#define max(x,y) ((x) > (y) ? (x) : (y))

#undef min
#define min(x,y) ((x) < (y) ? (x) : (y))

/* Prototype */
void segway_status_update(union segway_union *, int, struct sockaddr_in *, long int);
//void arm_status_update(int socket, struct sockaddr_in *address, struct wwvi_js_event *jse, long int joy_max_value) ;
void message_log(const char *, const char *); 
int copy_log_file(void);
void gps_compare_log(double latitude, double longitude, double latitude_odometry, double longitude_odometry,
                     double velocity, double pdop, double hdop, double vdop, int sat_inview, int sat_used, double direction, 
		     double direction_bussola, long time_us);

void gps_text_log(char *string);

//double GpsCoord2Double(double gps_coord);

/* Gps */
nmeaINFO info;

extern RTB_status RTBstatus;

int main() 
{
  /* Automatic Control Unit*/
  unsigned char previouse_status = ACU_IDLE; // it's used to return to the previouse status after an automatic move is completed
  unsigned char status = ACU_IDLE;
//  printf("ACU_IDLE\n");
  
  /* Robotic Arm */
  unsigned char robotic_arm_selected = 1;
  const unsigned char arm_encoder_factor = 11; // = (4000/360) each motor has an encorder with 4000 step
  long arm_direction = 0;

  /* Segway */
  int socket_segway = -1;
  struct sockaddr_in segway_address;
  union segway_union segway_status;
  unsigned char segway_prescaler_timeout = 0;
  unsigned char segway_check = 0;
  unsigned char segway_down = 0;

  /* CCU interface */
  int socket_ccu = -1;
  struct sockaddr_in socket_ccu_addr_dest;
  struct sockaddr_in socket_ccu_addr_src;
  __u8 ccu_buffer[(SEGWAY_PARAM*4) + 3];
  
  /* Segway info from ccu */
  int socket_segway_ccu = -1;
  struct sockaddr_in socket_segway_ccu_addr_src;
  
  /* Robotic Arm Client*/
  int socket_arm = -1;
  struct sockaddr_in arm_address;
  struct arm_frame arm_buffer_temp;
  unsigned char arm_request_index;
  char *arm_token_result;

  /* Status client interface*/
  int socket_status = -1;
  socklen_t socket_status_addr_dest_len = sizeof(struct sockaddr_in);
  struct sockaddr_in socket_status_addr_dest_temp;
  struct sockaddr_in socket_status_addr_dest;
  struct sockaddr_in socket_status_addr_src;
  unsigned char status_buffer;
  unsigned char status_return;
  char address_buffer[256];
  char address_buffer_temp[256];
  
  /* Gps */
#ifdef GPS
  int gps_device = -1;
  char gps_device_buffer[RS232_BUFFER_SIZE];
  char *token;
  char nmea_message[256];
  char nmea_message_log[2048];
  int it = 0;
    
  nmeaPARSER parser;
  char gps_simulate_flag = 0;
  char gps_simulate_init = 0;
  
  struct timespec gps_timer_stop;
  
  /*double gps_lon;
  double gps_lat;
  double gps_direction;
  double pdop, hdop, vdop;
  int sat_inview, sat_used;*/
#endif
  
  /* Gps */
#ifdef GPS_SIMUL
  char gps_buffer[2048];
  int gps_timeout = 0;
#endif

  /* Rover */
  double old_direction = 0;
  double old_lat = 0;
  double old_lon = 0;
  double old_elv = 0;
  double angle_treshold = 0;
  int rtb_status = RTB_idle;
  
  
  // timer
  //struct timespec rover_timer_start, rover_timer_stop;
  long rover_time_start_hs = 0;
  long rover_time_stop_hs = 0;
  long rover_elapsed_time_hs = 0;
  
  /* Generic Variable */
  int done = 0; // for the while in main loop
  int bytes_read; // to check how many bytes has been read
  int bytes_sent;

  int select_result = -1; // value returned frome select()
  int nfds = 0; // fd to pass to select()
  fd_set rd, wr, er; // structure for select()
  struct timeval select_timeout;
  long timeout_return_to_base = 0;
  long timeout_status = 0;
  long current_timeout = 0;
  
  message_log("stdof", "Initializing. . .");
  printf("Initializing. . .\n");

  /* Peripheral initialization */

  /* Init Segway Client */  
  if(segway_open(&socket_segway, &segway_address, SEGWAY_ADDRESS, SEGWAY_PORT) == -1)
  {
    message_log("init vehicle client", strerror(errno));
    perror("init vehicle client");
  }
  else
    segway_init(socket_segway, &segway_address, &segway_status);

  /* Init CCU Client */
  socket_ccu = socket(AF_INET, SOCK_DGRAM, 0);

  if(socket_ccu < 0)
    perror("socket_ccu");
 
  bzero(&socket_ccu_addr_src, sizeof(socket_ccu_addr_src));
  socket_ccu_addr_src.sin_family = AF_INET;
  socket_ccu_addr_src.sin_port = htons(CCU_PORT_ARM);
  socket_ccu_addr_src.sin_addr.s_addr = htonl(INADDR_ANY);

  if(bind(socket_ccu, (struct sockaddr *)&socket_ccu_addr_src, sizeof(socket_ccu_addr_src)) == -1)
    perror("socket_ccu");

  bzero(&socket_ccu_addr_dest, sizeof(socket_ccu_addr_dest));
  socket_ccu_addr_dest.sin_family = AF_INET;
  socket_ccu_addr_dest.sin_addr.s_addr = inet_addr(CCU_ADDRESS);
  socket_ccu_addr_dest.sin_port = htons(CCU_PORT_ARM);
  
  
  /* Init Arm Client */  
  if(arm_open(&socket_arm, &arm_address, ACU_ARM_PORT, ARM_ADDRESS, ARM_PORT) == -1)
  {
    message_log("init arm client", strerror(errno));
    perror("init arm client");
  }
  
  /* Init Robotic Arm */
  arm_init(0, 1000, 10, 32767, 2000, 1500, 100, 700, 1023);
  arm_init(2, 20000, 10, 1500, 35000, 1500, 100, 500, 1023);
  
  /* Status client interface */
  socket_status = socket(AF_INET, SOCK_DGRAM, 0);

  if(socket_status < 0)
    perror("socket_status");
 
  bzero(&socket_status_addr_src, sizeof(socket_status_addr_src));
  socket_status_addr_src.sin_family = AF_INET;
  socket_status_addr_src.sin_port = htons(ACU_STATUS_PORT);
  socket_status_addr_src.sin_addr.s_addr = htonl(INADDR_ANY);

  if(bind(socket_status, (struct sockaddr *)&socket_status_addr_src, sizeof(socket_status_addr_src)) == -1)
    perror("socket_status");

  bzero(&socket_status_addr_dest, sizeof(socket_status_addr_dest));
  socket_status_addr_dest.sin_family = AF_INET;
  socket_status_addr_dest.sin_addr.s_addr = inet_addr(JOYSTICK_ADDRESS);
  socket_status_addr_dest.sin_port = htons(JOYSTICK_STATUS_PORT);
  
  /* Init Segway info from ccu */
  socket_segway_ccu = socket(AF_INET, SOCK_DGRAM, 0);

  if(socket_segway_ccu < 0)
    perror("socket_segway_ccu");
 
  bzero(&socket_segway_ccu_addr_src, sizeof(socket_segway_ccu_addr_src));
  socket_segway_ccu_addr_src.sin_family = AF_INET;
  socket_segway_ccu_addr_src.sin_port = htons(ACU_SEGWAY_PORT);
  socket_segway_ccu_addr_src.sin_addr.s_addr = htonl(INADDR_ANY);

  if(bind(socket_segway_ccu, (struct sockaddr *)&socket_segway_ccu_addr_src, sizeof(socket_segway_ccu_addr_src)) == -1)
    perror("socket_segway_ccu");
  
  /* Init Gps */
#ifdef GPS_SIMUL
  gps_generate_init(2, 2, 4140.7277, 1230.5320, 0, 10.86, 0.0, 2, 2, &info);
#endif
  
#ifdef GPS
  // Select UART2_TX and set it as output
  sprintf(gps_device_buffer, "echo 11 > /sys/kernel/debug/omap_mux/spi0_d0");
  if(system(gps_device_buffer) < 0)
    perror("setting tx");
  
  // Select UART1_RX and set it as input pulled up
  sprintf(gps_device_buffer, "echo 39 > /sys/kernel/debug/omap_mux/spi0_sclk");
  if(system(gps_device_buffer) < 0)
    perror("setting rx");
  
  gps_device = com_open("/dev/ttyO2", 4800, 'N', 8, 1);
  
  if(gps_device < 0)
    perror("com_open");
  else
    nmea_parser_init(&parser);
  
  clock_gettime(CLOCK_REALTIME, &gps_timer_stop);
  rover_time_start_hs = (gps_timer_stop.tv_sec * 100 + (gps_timer_stop.tv_nsec / 10000000));
  
  nmea_message_log[0] = '\0';
#endif
  
  /* Init Rtb module */
  RTB_init();
  RTB_set_mode(RTB_recording);
    
  switch(status)
  {
    case ACU_HOME:
    case ACU_IDLE:
      select_timeout.tv_sec = TIMEOUT_SEC;
      select_timeout.tv_usec = TIMEOUT_USEC_STATUS;
      break;

    case ACU_ARM_HOMING:
    case ACU_ARM_AUTO_MOVE:
    case ACU_ARM_AUTO_MOVE_ABORT:
      select_timeout.tv_sec = TIMEOUT_SEC;
      select_timeout.tv_usec = TIMEOUT_USEC_ARM;
      break;
	
    case ACU_RETURN_TO_BASE:
      if(robotic_arm_selected)
      {
        select_timeout.tv_sec = TIMEOUT_SEC;
        select_timeout.tv_usec = TIMEOUT_USEC_ARM;
      }
      else
      {
        select_timeout.tv_sec = TIMEOUT_SEC;
        select_timeout.tv_usec = TIMEOUT_USEC_SEGWAY;
      }

      break;
  }

  current_timeout = select_timeout.tv_usec;
  
  message_log("stdof", "Run main program. . .");
  printf("Run main program. . .\n");
  /*
  long tetha0, tetha1, tetha2;
  // move link 3 on y-z axis
  double x,y,z;
  double x0, y0, z0;
  int i;

  arm_calc_xyz(&x, &y, &z, -59 * M_PI / (arm_encoder_factor * arm_link[0].gear * 180),
            2983 * M_PI / (arm_encoder_factor * arm_link[1].gear * 180),
             -838 * M_PI / (arm_encoder_factor * arm_link[2].gear * 180));
  arm_calc_tetha(112.333725, 318.253753, 835.046354, &tetha0, &tetha1, &tetha2);
  printf("Theta0: %ld Theta1: %ld Theata2: %ld\n", tetha0,tetha1, tetha2); 

  arm_calc_xyz(&x, &y, &z, tetha0 * M_PI / (arm_encoder_factor * arm_link[0].gear * 180),
            tetha1 * M_PI / (arm_encoder_factor * arm_link[1].gear * 180),
             tetha2 * M_PI / (arm_encoder_factor * arm_link[2].gear * 180));
  printf("x: %f y: %f z: %f\n\n", x, y, z);*/

 /*   x = -22.913913;
    y = -856.824226;
    z = 473.655255 + 200;
    for(i = 25; i <= 200; i += 25)
    {
      y += i;
    
      arm_calc_tetha(x, y, z, &tetha0, &tetha1, &tetha2);
      printf("Theta0: %ld Theta1: %ld Theata2: %ld\n", tetha0, tetha1, tetha2); 
      //arm_calc_xyz(&x, &y, &z, tetha0 * M_PI / (arm_encoder_factor * arm_link[0].gear * 180),
      //           tetha1 * M_PI / (arm_encoder_factor * arm_link[1].gear * 180),
      //           tetha2 * M_PI / (arm_encoder_factor * arm_link[2].gear * 180));
      //printf("x: %f y: %f z: %f\n\n", x, y, z);
    }*/

  /*arm_calc_xyz(&x, &y, &z, 0 * M_PI / (arm_encoder_factor * arm_link[0].gear * 180),
               679000 * M_PI / (arm_encoder_factor * arm_link[1].gear * 180),
               215000 * M_PI / (arm_encoder_factor * arm_link[2].gear * 180));
  printf("x: %f y: %f z: %f\n", x, y, z);
    
  arm_calc_tetha(x, y, z, &tetha0, &tetha1, &tetha2);
  printf("Theta0: %ld Theta1: %ld Theata2: %ld\n", tetha0, tetha1, tetha2); 
  arm_calc_xyz(&x, &y, &z, tetha0 * M_PI / (arm_encoder_factor * arm_link[0].gear * 180),
             tetha1 * M_PI / (arm_encoder_factor * arm_link[1].gear * 180),
             tetha2 * M_PI / (arm_encoder_factor * arm_link[2].gear * 180));
  printf("x: %f y: %f z: %f\n\n", x, y, z);
  
  arm_calc_xyz(&x, &y, &z, 20721 * M_PI / (arm_encoder_factor * arm_link[0].gear * 180),
               1088668 * M_PI / (arm_encoder_factor * arm_link[1].gear * 180),
               261131 * M_PI / (arm_encoder_factor * arm_link[2].gear * 180));
  printf("x: %f y: %f z: %f\n", x, y, z);
    
  arm_calc_tetha(x, y, z, &tetha0, &tetha1, &tetha2);
  printf("Theta0: %ld Theta1: %ld Theata2: %ld\n", tetha0, tetha1, tetha2); 
  arm_calc_xyz(&x, &y, &z, tetha0 * M_PI / (arm_encoder_factor * arm_link[0].gear * 180),
             tetha1 * M_PI / (arm_encoder_factor * arm_link[1].gear * 180),
             tetha2 * M_PI / (arm_encoder_factor * arm_link[2].gear * 180));
  printf("x: %f y: %f z: %f\n", x, y, z);*/
  /*arm_calc_tetha(0, 500 + 580 * cos(1.046), 580 * sin(1.046), &tetha0, &tetha1, &tetha2);
      printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(0, 500 + 580 * cos(M_PI_4), 580 * sin(M_PI_4), &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(0, 500 + 580 * cos(0.523), 580 * sin(0.523), &tetha0, &tetha1, &tetha2);
      printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(0, 1080, 0, &tetha0, &tetha1, &tetha2);
      printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(0, 500 + 580 * cos(-0.523), 580 * sin(-0.523), &tetha0, &tetha1, &tetha2);
      printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(0, 500 + 580 * cos(-M_PI_4), 580 * sin(-M_PI_4), &tetha0, &tetha1, &tetha2);
      printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(0, 500 + 580 * cos(-1.046), 580 * sin(-1.046), &tetha0, &tetha1, &tetha2);
      printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(0, 500, -580, &tetha0, &tetha1, &tetha2);
      printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  */
  /*arm_calc_tetha(0, 0, 1080, &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(0, 1080 * cos(1.046), 1080 * sin(1.046), &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(0, 1080 * cos(M_PI_4), 1080 * sin(M_PI_4), &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(0, 1080 * cos(0.523), 1080 * sin(0.523), &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(0, 1080, 0, &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(0, 1080 * cos(-0.523), 1080 * sin(-0.523), &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(0, 1080 * cos(-M_PI_4), 1080 * sin(-M_PI_4), &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(0, 1080 * cos(-1.046), 1080 * sin(-1.046), &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(0, 0, -1080, &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
*/  
  /*arm_calc_tetha(1080, 0, 0, &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(1080 * sin(1.046), 1080 * cos(1.046), 0, &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(1080 * sin(M_PI_4), 1080 * cos(M_PI_4), 0, &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(1080 * sin(0.523), 1080 * cos(0.523), 0, &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(0, 1080, 0, &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(1080 * sin(-0.523), 1080 * cos(-0.523), 0, &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(1080 * sin(-M_PI_4), 1080 * cos(-M_PI_4), 0, &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(1080 * sin(-1.046), 1080 * cos(-1.046), 0, &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  arm_calc_tetha(-1080, 0, 0, &tetha0, &tetha1, &tetha2);
  printf("Theta0: %f Theta1: %f Theata2: %f\n", (double)tetha0 / (11 * arm_link[0].gear), (double)tetha1 / (11 * arm_link[1].gear), (double)tetha2 / (11 * arm_link[2].gear)); 
  */
 
  while(!done)
  { 
    fflush(stdout);

    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);

    if((socket_status > 0))
    {
      FD_SET(socket_status, &rd);
      nfds = max(nfds, socket_status);
    }

    if(socket_segway_ccu > 0)
    {
      FD_SET(socket_segway_ccu, &rd);
      nfds = max(nfds, socket_segway_ccu);
    }
    
#ifdef GPS
    if(gps_device > 0)
    {
      FD_SET(gps_device, &rd);
      nfds = max(nfds, gps_device);
    }
#endif

    switch(status)
    {
      case ACU_HOME:
      case ACU_IDLE:
        //get data from gps
        //send status to joystick (outside switch)
        //read joystick status (outside switch)
        if((socket_arm > 0) && (robotic_arm_selected) && (arm_buffer_tx_empty == 0))
        {
          FD_SET(socket_arm, &wr);
          nfds = max(nfds, socket_arm);
        }
        
        if((socket_segway > 0) && (robotic_arm_selected == 0) && (segway_buffer_tx_empty == 0))
        {
          FD_SET(socket_segway, &wr);
          nfds = max(nfds, socket_segway);  
        }
        break;

      case ACU_ARM_HOMING:
      case ACU_ARM_AUTO_MOVE:
      case ACU_ARM_AUTO_MOVE_ABORT:
        //move arm to home position
        //send status to joystick (outside switch)
        //read joystick status (outside switch)
        if((socket_arm > 0))
        {
          FD_SET(socket_arm, &rd);
          nfds = max(nfds, socket_arm);  
 
          if(arm_buffer_tx_empty == 0)
          {
            FD_SET(socket_arm, &wr);
            nfds = max(nfds, socket_arm);
          }
        }
        break;

      case ACU_RETURN_TO_BASE:
      case ACU_RETURN_TO_BASE_ABORT:
        //arm homing
        //get data from gps
        //drive segway to home
        //read sick
        //send status to joystick (outside switch)
        //read joystick status (outside switch)
        if((socket_segway > 0) && (robotic_arm_selected == 0))
        {
          FD_SET(socket_segway, &rd);
          nfds = max(nfds, socket_segway);
  
          if(segway_buffer_tx_empty == 0)
          {
            FD_SET(socket_segway, &wr);
            nfds = max(nfds, socket_segway);  
          }
        }
        
        break;

      default:
        break;
    }
    
    select_result = select(nfds + 1, &rd, &wr, NULL, &select_timeout);

    if(select_result == -1 && errno == EAGAIN)
    {
      perror("select");
      continue;
    }

    if(select_result == -1)
    {
      message_log("stdof", strerror(errno));
      perror("main:");

      return 1;
    }

    /* Manage joystick status */
    if(socket_status > 0)
    {
      if(FD_ISSET(socket_status, &rd))
      {
        bytes_read = recvfrom(socket_status, &status_buffer, sizeof(status_buffer), 0, (struct sockaddr *)&socket_status_addr_dest_temp, &socket_status_addr_dest_len);

        if(strcmp(inet_ntop(AF_INET, &socket_status_addr_dest.sin_addr.s_addr, address_buffer, sizeof(address_buffer)), 
                  inet_ntop(AF_INET, &socket_status_addr_dest_temp.sin_addr.s_addr, address_buffer_temp, sizeof(address_buffer_temp))) != 0)
        {
          // When arrive a command from another source, it can be forwarded only if the arm is in a safe state
          if((status == ACU_HOME) || (status == ACU_IDLE))
            memcpy(&socket_status_addr_dest, &socket_status_addr_dest_temp, sizeof(struct sockaddr_in));
          else
            continue;
        }

        if(bytes_read > 0)
        {
          timeout_return_to_base = 0;  // reset timer
  
          if(status == ACU_RETURN_TO_BASE) // control segway
          {
            if(robotic_arm_selected == 0)
              previouse_status = ACU_HOME;
            else
              previouse_status = ACU_IDLE;

            status = ACU_RETURN_TO_BASE_ABORT;
//            printf("ACU_RETURN_TO_BASE_ABORT\n");
            continue;
          }
 
          if(previouse_status == ACU_RETURN_TO_BASE) // control arm
          {
            // Abort automatic movement ad return in idle state
            robotic_arm_selected = 1;
            previouse_status = ACU_IDLE;
            arm_start();
            status = ACU_ARM_AUTO_MOVE_ABORT;
//            printf("ACU_ARM_AUTO_MOVE_ABORT\n");
            continue;
          }
  
          switch(status_buffer)
          {
            case JOYSTICK_NONE:
              break;
      
            case JOYSTICK_REQUEST_HOMING:
              // init arm for homing
              if(status != ACU_ARM_HOMING)
              {
                arm_init(0, 1000, 10, 32767, 2000, 1500, 100, 700, 1023);
                arm_init(2, 20000, 10, 1500, 35000, 1500, 100, 500, 1023);

                status_return = arm_automatic_motion_start(ARM_HOME_FILE);
                if(status_return > 0)
                {
                  status = ACU_ARM_HOMING;
//                  printf("ACU_ARM_HOMING\n");
                  previouse_status = ACU_HOME;
                  printf("Start homing. . .\n");
                }
                else if(status_return == -1)
                  perror("arm_automatic_motion_start");
                else
                  printf("arm_motion_start error\n");
              }
              break;
  
            case JOYSTICK_REQUEST_DINAMIC:
              // init arm for dinamic position
              if(status != ACU_ARM_AUTO_MOVE)
              {
                arm_init(0, 1000, 10, 32767, 2000, 1500, 100, 700, 1023);
                arm_init(2, 20000, 10, 1500, 35000, 1500, 100, 500, 1023);

                status_return = arm_automatic_motion_start(ARM_DINAMIC_FILE);
                if(status_return > 0)
                {
                  status = ACU_ARM_AUTO_MOVE;
//                  printf("ACU_ARM_AUTO_MOVE\n");
                  previouse_status = ACU_IDLE;
                  printf("Arm is going to dinamic position. . .\n");
                }
                else if(status_return == -1)
                  perror("arm_automatic_motion_start");
                else
                  printf("arm_motion_start error\n");
              }
              break;

            case JOYSTICK_REQUEST_BOX:
              // init arm for box position
              if(status != ACU_ARM_AUTO_MOVE)
              {
                arm_init(0, 1000, 10, 32767, 2000, 1500, 100, 700, 1023);
                arm_init(2, 20000, 10, 1500, 35000, 1500, 100, 500, 1023);

                status_return = arm_automatic_motion_start(ARM_BOX_FILE);
                if(status_return > 0)
                {
                  status = ACU_ARM_AUTO_MOVE;
//                  printf("ACU_ARM_AUTO_MOVE\n");
                  previouse_status = ACU_IDLE;
                  printf("Arm is going to put an object into box. . .\n");
                }
                else if(status_return == -1)
                  perror("arm_automatic_motion_start");
                else
                  printf("arm_motion_start error\n");
              }
              break;
 
            case JOYSTICK_REQUEST_PARK:
              // init arm for homing
              if(status != ACU_ARM_AUTO_MOVE)
              {
                arm_init(0, 1000, 10, 32767, 2000, 1500, 100, 700, 1023);
                arm_init(2, 20000, 10, 1500, 35000, 1500, 100, 500, 1023);

                status_return = arm_automatic_motion_start(ARM_PARK_FILE);
                if(status_return > 0)
                {
                  status = ACU_ARM_AUTO_MOVE;
//                  printf("ACU_ARM_AUTO_MOVE\n");
                  previouse_status = ACU_IDLE;
                  printf("Arm is going to parking position. . .\n");
                }
                else if(status_return == -1)
                  perror("arm_automatic_motion_start");
                else
                  printf("arm_motion_start error\n");
              }
              break;
  
              case JOYSTICK_REQUEST_STEP:
              // init arm for homing
              if(status != ACU_ARM_AUTO_MOVE)
              {
                arm_init(0, 1000, 10, 32767, 2000, 1500, 100, 700, 1023);
                arm_init(2, 20000, 10, 1500, 35000, 1500, 100, 500, 1023);

                status_return = arm_automatic_motion_start(ARM_STEP_FILE);
                if(status_return > 0)
                {
                  status = ACU_ARM_AUTO_MOVE;
//                  printf("ACU_ARM_AUTO_MOVE\n");
                  previouse_status = ACU_IDLE;
                  printf("Arm is going to step position. . .\n");
                }
                else if(status_return == -1)
                  perror("arm_automatic_motion_start");
                else
                  printf("arm_motion_start error\n");
              }
              break;
			  
            case JOYSTICK_ABORT_AUTO_MOVE:
              //abort homing
              if((status == ACU_ARM_AUTO_MOVE) || (status == ACU_ARM_HOMING))
              {
                arm_start();
                status = ACU_ARM_AUTO_MOVE_ABORT;
//                printf("ACU_ARM_AUTO_MOVE\n");
              }
              break;
    
            default:
              break;
          }
        }
        else    
        {
          message_log("status_read", strerror(errno));
          perror("status_read");
        }

        continue;
      }
    }

    if(socket_segway_ccu > 0)
    {
      if(FD_ISSET(socket_segway_ccu, &rd))
      {
        bytes_read = segway_read(socket_segway_ccu, &segway_status, ccu_buffer);

        if(bytes_read <= 0)
          perror("segway_read");
 
        continue;
      }
    }
    
#ifdef GPS
    if(gps_device > 0)
    {
      if(FD_ISSET(gps_device, &rd))
      {
        bytes_read = rs232_read(gps_device);
        if((bytes_read > 0) || ((bytes_read < 0) && rs232_buffer_rx_full))
        {
          // load udp buffer only if I have received the whole message
          bytes_read = rs232_unload_rx_filtered(gps_device_buffer, '\n');

          if(bytes_read > 0)
          {
            gps_device_buffer[bytes_read] = '\0';
            //printf("rs232 buffer returned %s\n\n\n", gps_device_buffer);
            //printf("Length: %i\n", bytes_read);
            //printf("Gps Buffer %s\n", gps_device_buffer);
            token = strtok(gps_device_buffer, "\n");

            while(token != NULL)
            {
              //printf("Token %s\n", token);
              sprintf(nmea_message_log, "%s%s\n", nmea_message_log, token);
              sprintf(nmea_message, "%s\n", token);
              nmea_parse(&parser, nmea_message, (int)strlen(nmea_message), &info);
              token = strtok(NULL, "\n");
            }

            //printf("info.smask: %i\n", info.smask);
            // If I have received all mandatory info (lat, lon, speed, direction) then update the rtb status
            if((info.smask & GPRMC) && (info.smask & HCHDT) && (info.smask & GPGSV)/*|| 
               ((info.smask & GPGGA) && ((info.smask & GPVTG) || (info.smask & HCHDT)))*/)
            {
	      /*gps_lat = info.lat;
	      gps_lon = info.lon;
	      pdop = info.PDOP;
	      vdop = info.VDOP;
	      hdop = info.HDOP;
	      sat_inview = info.satinfo.inview;
	      sat_used = info.satinfo.inuse;
	      gps_direction = info.direction;*/

              // I can't generate gps message if there isn't data from segway
              if((segway_status.list.operational_state == SEGWAY_TRACTOR) || (segway_status.list.operational_state == SEGWAY_STANDBY))
              {
                // If bad signal then generate gps string by odometry
                //if((info.sig == NMEA_SIG_BAD) || (info.fix == NMEA_FIX_BAD))
                //{
                  clock_gettime(CLOCK_REALTIME, &gps_timer_stop);
      
                  rover_time_stop_hs = (gps_timer_stop.tv_sec * 100 + (gps_timer_stop.tv_nsec / 10000000));

                  if(rover_time_start_hs != rover_time_stop_hs)
                  {
                    rover_elapsed_time_hs = rover_time_stop_hs - rover_time_start_hs;
                    rover_time_start_hs = rover_time_stop_hs;
                  }
                  gps_timer_stop.tv_nsec = 0;
                  // If gps_generate already init then generate gps information
                  if(gps_simulate_flag)
                  {
                    gps_generate(convert_to_float(segway_status.list.linear_vel_mps) * 3.6, 
                                 info.magnetic_sensor_heading_true, 
                                 rover_elapsed_time_hs * 10000, gps_device_buffer, &info);
                  }
                  else
                  {
                     gps_generate_init(NMEA_SIG_BAD, NMEA_FIX_BAD, old_lat, old_lon, 
                                      convert_to_float(segway_status.list.linear_vel_mps) * 3.6,
                                      old_elv, info.magnetic_sensor_heading_true, 0, 0, &info);

		    //gps_generate_init(NMEA_SIG_BAD, NMEA_FIX_BAD, info.lat, info.lon, 
                    //                  convert_to_float(segway_status.list.linear_vel_mps) * 3.6,
                    //                 old_elv, info.magnetic_sensor_heading_true, 0, 0, &info);

                    gps_simulate_flag = 1;
                  }
                /*}
                else
                {
                  // If I found the signal
                  if(gps_simulate_flag)
                  {
                    gps_simulate_flag = 0;

                    // If the start position is unknown until now
                    if(gps_simulate_init == 0)
                      RTB_traslate_point(GpsCoord2Double(info.lat), GpsCoord2Double(info.lon));
                  }

                  if(gps_simulate_init == 0)
                    gps_simulate_init = 1;

                  rover_time_stop_hs = (info.utc.sec * 100 + info.utc.hsec);

                  if(rover_time_start_hs != rover_time_stop_hs)
                  {
                    rover_elapsed_time_hs = rover_time_stop_hs - rover_time_start_hs;
                    rover_time_start_hs = rover_time_stop_hs;
                  }
                }*/
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

              if((old_direction != info.magnetic_sensor_heading_true) || (old_lat != info.lat) || (old_lon != info.lon))
              {
                //printf("%i\n", info.smask);
                //printf("rtb_update: magnetic_sensor_heading_true %f vs %f lat %f vs %f lon %f vs %f\n", old_direction, info.magnetic_sensor_heading_true, old_lat, info.lat, old_lon, info.lon);
                /*gps_text_log(nmea_message_log);
                nmea_message_log[0] = '\0';
                gps_compare_log(GpsCoord2Double(gps_lat), GpsCoord2Double(gps_lon), 
				GpsCoord2Double(info.lat), GpsCoord2Double(info.lon), 
				convert_to_float(segway_status.list.linear_vel_mps), pdop, hdop, vdop, sat_inview, sat_used, 
				gps_direction, info.magnetic_sensor_heading_true, rover_elapsed_time_hs * 10000);*/
		
                info.smask &= !(GPRMC | GPGGA | GPVTG | HCHDT | GPGSV);
 
                //printf("if((info.smask & GPRMC) \n");

                // rover_elapsed_time_hs in hundreds of seconds
                rtb_status = RTB_update(GpsCoord2Double(info.lon), GpsCoord2Double(info.lat), (info.speed / 3.6), 
                              convert_to_float(segway_status.list.inertial_z_rate_rps));

                if(rtb_status == RTB_tracking)
                {
                  //printf("RTB_update\n");
                  //printf("Angle: %f Divergenza: %f\n",(RTBstatus.control_vector.angle_deg_north - info.direction), (RTBstatus.control_vector.angle_deg_north - info.direction)/180);
              
                  // if I'm in the right direction then activate speed value
                  angle_treshold = (RTBstatus.distance / RTB_SAVE_DIST_TRSH) * (RTB_CONTROL_ANGLE_MAX - RTB_CONTROL_ANGLE_MIN) + RTB_CONTROL_ANGLE_MIN;
		  
                  if((fabs(RTBstatus.control_vector.angle_deg_north - info.magnetic_sensor_heading_true) < angle_treshold) && 
                     (fabs(RTBstatus.control_vector.angle_deg_north - info.magnetic_sensor_heading_true) > -angle_treshold))
                  {
                    // Add a factor that depend on error
                    RTBstatus.control_values.speed = (1 - (fabs(RTBstatus.control_vector.angle_deg_north - info.magnetic_sensor_heading_true)) / 180) * cos((RTBstatus.control_vector.angle_deg_north - info.magnetic_sensor_heading_true) * M_PI / 180);

                    // Add a factor (90 / RTB_CONTROL_ANGLE) to the angle to obtain a fastest convergence
                    if((RTBstatus.control_vector.angle_deg_north - info.magnetic_sensor_heading_true) >= 0)
                    {
                      RTBstatus.control_values.heading = sin(min((RTBstatus.control_vector.angle_deg_north - info.magnetic_sensor_heading_true) * (180 / angle_treshold), 90) * M_PI / 180);
                      //printf("Heading factor: %f\n", min((RTBstatus.control_vector.angle_deg_north - info.magnetic_sensor_heading_true) * (180 / angle_treshold), 90));
                    }
                    else
                    {
                      RTBstatus.control_values.heading = sin(max((RTBstatus.control_vector.angle_deg_north - info.magnetic_sensor_heading_true) * (180 / angle_treshold), -90) * M_PI / 180);
                      //printf("Heading factor: %f\n", max((RTBstatus.control_vector.angle_deg_north - info.direction) * (180 / angle_treshold/2), -90));
                    }
                  }
                  else
                  {
                    RTBstatus.control_values.speed = 0;
                
                    // Choose angle to turn. If error > 180° then turn over 360
                    if((RTBstatus.control_vector.angle_deg_north - info.magnetic_sensor_heading_true) >= 0)
                    {
                       if((RTBstatus.control_vector.angle_deg_north - info.magnetic_sensor_heading_true) <= 180)
                        RTBstatus.control_values.heading = 1;
                      else 
                        RTBstatus.control_values.heading = -1;
                    }
                    else
                    {
                      if((RTBstatus.control_vector.angle_deg_north - info.magnetic_sensor_heading_true) >= -180)
                        RTBstatus.control_values.heading = -1;
                      else 
                        RTBstatus.control_values.heading = 1;
                    }
                  }
                }
                else
                {
                  RTBstatus.control_values.heading = 0;
                  RTBstatus.control_values.speed = 0;
                }

                old_direction = info.magnetic_sensor_heading_true;
                old_lat = info.lat;
                old_lon = info.lon;
                old_elv = info.elv;
              }
                     
              info.smask &= !(GPRMC | GPGGA | GPVTG | HCHDT | GPGSV);
            }
          }
        }
        continue;
      }
    }
#endif

    switch(status)
    {
      case ACU_HOME:
      case ACU_IDLE:
        //get data from gps
        //send status to joystick 
        //read joystick status
        
        /* I have to empty the buffer */
        if(socket_arm > 0)
        {
          if(FD_ISSET(socket_arm, &wr))
          {
            //printf("%i\n", rs232_buffer_tx_data_count);
            bytes_sent = arm_send(socket_arm, &arm_address);

            if(bytes_sent <= 0)
              printf("Error on arm_send\n");
  
            continue;
          }
        }
        break;

      case ACU_ARM_AUTO_MOVE_ABORT:
        if((socket_arm > 0))
        {
          if(FD_ISSET(socket_arm, &rd))
          {
            // the message would be an information such position or warning
            bytes_read = recvfrom(socket_arm, &arm_buffer_temp, sizeof(struct arm_frame), 0, NULL, NULL);
            
            if(bytes_read > 0)
            {
              // Every message from arm must ends with \r
              arm_token_result = strchr(arm_buffer_temp.param.arm_command, 13);
      
              if((query_link > -1) && (arm_token_result != NULL))
              {
                //printf("Received message from %i\n", query_link);
                *arm_token_result = '\0';  // translate token in null character
                arm_request_index = query_link;

                if(arm_link[arm_request_index - 1].request_trajectory_status == 1)
                {
                  query_link = -1;
                  arm_link[arm_request_index - 1].request_timeout = 0;
                  arm_link[arm_request_index - 1].request_trajectory_status = 0;
                  arm_link[arm_request_index - 1].trajectory_status = atoi(arm_buffer_temp.param.arm_command);
  
                  if(arm_request_index == MOTOR_NUMBER)
                  {
                    if(arm_link[arm_request_index - 1].trajectory_status > 0)
                    {
                      if(arm_link[arm_request_index -1].position_target > 0)
                        actuator_set_command(30000);
                      else
                        actuator_set_command(-30000);
                    }
                    else
                      link_homing_complete |= (int)pow(2, arm_request_index - 1);
                  }
                }
              }
            }
            else
              perror("arm_read");
  
            continue;      
          }
  
          if(FD_ISSET(socket_arm, &wr))
          {
            //printf("%i\n", rs232_buffer_tx_data_count);
            bytes_sent = arm_send(socket_arm, &arm_address);

            if(bytes_sent <= 0)
              printf("Error on pc_interface_send");
  
            continue;
          }
        }

        break;

      case ACU_ARM_HOMING:
      case ACU_ARM_AUTO_MOVE:
        //move arm to home position
        //send status to joystick 
        //read joystick status
        if((socket_arm > 0))
        {
          if(FD_ISSET(socket_arm, &rd))
          {
            // the message would be an information such position or warning
            bytes_read = recvfrom(socket_arm, &arm_buffer_temp, sizeof(struct arm_frame), 0, NULL, NULL);

            if(bytes_read > 0)
            {
              // Every message from arm must ends with \r
              arm_token_result = strchr(arm_buffer_temp.param.arm_command, 13);
      
              if((query_link > -1) && (arm_token_result != NULL))
              {
                //printf("Received message from %i\n", query_link);
                *arm_token_result = '\0';  // translate token in null character
                arm_request_index = query_link;

                if(arm_link[arm_request_index - 1].request_actual_position == 1)
                {
                  query_link = -1;
                  arm_link[arm_request_index - 1].request_timeout = 0;
                  arm_link[arm_request_index - 1].request_actual_position = 0;
                  arm_link[arm_request_index - 1].actual_position = atol(arm_buffer_temp.param.arm_command);
   
                  arm_direction = arm_link[arm_request_index - 1].actual_position - arm_link[arm_request_index -1].position_target;
  
                  if((arm_direction >= 0))
                  {
                    // if link is far from the target point, set the max velocity target
                    if(arm_link[arm_request_index - 1].actual_position >= (arm_link[arm_request_index -1].position_target + 10 * arm_encoder_factor * arm_link[arm_request_index -1].gear))
                      arm_link[arm_request_index - 1].velocity_target = -(long)arm_link[arm_request_index - 1].velocity_target_limit;
                    else if(arm_link[arm_request_index - 1].actual_position >= (arm_link[arm_request_index -1].position_target + 5 * arm_encoder_factor * arm_link[arm_request_index -1].gear))
                      arm_link[arm_request_index - 1].velocity_target = -(long)arm_link[arm_request_index - 1].velocity_target_limit/2;
                    else
                      arm_link[arm_request_index - 1].velocity_target = -(long)arm_link[arm_request_index - 1].velocity_target_limit/2;
        
                    //printf("velocity target for %i: %ld, velocity_target_limit: %ld\n",arm_request_index, arm_link[arm_request_index - 1].velocity_target, arm_link[arm_request_index - 1].velocity_target_limit);
                    if((arm_link[arm_request_index - 1].actual_position < (arm_link[arm_request_index -1].position_target + (long)(arm_encoder_factor * arm_link[arm_request_index -1].gear/2))) && 
                       ((link_homing_complete & (int)pow(2, arm_request_index - 1)) == 0))
                    {
                      //printf("Motor%i Actual position: %ld, Position target range: %ld\n", arm_request_index, arm_link[arm_request_index - 1].actual_position, arm_link[arm_request_index -1].position_target + 400);
                      if(arm_stop(arm_request_index))
                      {
                        //printf("link_homing_complete |= %i\n", (int)pow(2, arm_request_index - 1));
                        arm_link[arm_request_index - 1].velocity_target = 0;
                        link_homing_complete |= (int)pow(2, arm_request_index - 1);
                      }
                    }  
                    else
                    {
                      //printf("velocity target for %i: %ld\n",arm_request_index, arm_link[arm_request_index - 1].velocity_target);
                      arm_set_command(arm_request_index, "VT", (arm_link[arm_request_index - 1].velocity_target));
                      arm_set_command_without_value(arm_request_index, "G");
                      arm_set_command(arm_request_index, "c", 0);
                    }
                  }
                    
                  if((arm_direction < 0))
                  {
                    // if link is far from the target point, set the max velocity target
                    if(arm_link[arm_request_index - 1].actual_position <= (arm_link[arm_request_index -1].position_target - 10*arm_encoder_factor * arm_link[arm_request_index -1].gear))
                      arm_link[arm_request_index - 1].velocity_target = arm_link[arm_request_index - 1].velocity_target_limit;
                    else if(arm_link[arm_request_index - 1].actual_position <= (arm_link[arm_request_index -1].position_target - 5*arm_encoder_factor * arm_link[arm_request_index -1].gear))
                      arm_link[arm_request_index - 1].velocity_target = (long)arm_link[arm_request_index - 1].velocity_target_limit/2;
                    else
                      arm_link[arm_request_index - 1].velocity_target = (long)arm_link[arm_request_index - 1].velocity_target_limit/4;
   
                    //printf("velocity target for %i: %ld\n",arm_request_index, arm_link[arm_request_index - 1].velocity_target);
                    if((arm_link[arm_request_index - 1].actual_position > (arm_link[arm_request_index -1].position_target - (long)(arm_encoder_factor * arm_link[arm_request_index -1].gear / 2))) &&
                      ((link_homing_complete & (int)pow(2, arm_request_index - 1))  == 0))
                    {
                      //printf("Motor%i Arm direction: %ld, Actual position: %ld, Position target range: %ld\n", arm_request_index, arm_direction, arm_link[arm_request_index - 1].actual_position, arm_link[arm_request_index -1].position_target - 400);
                      if(arm_stop(arm_request_index))
                      {
                        //printf("link_homing_complete |= %i\n", (int)pow(2, arm_request_index - 1));
                        arm_link[arm_request_index - 1].velocity_target = 0;
                        link_homing_complete |= (int)pow(2, arm_request_index - 1);
                      }
                    }
                    else
                    {
                      arm_set_command(arm_request_index, "VT", (arm_link[arm_request_index - 1].velocity_target));
                      arm_set_command_without_value(arm_request_index, "G");
                      arm_set_command(arm_request_index, "c", 0);
                    }
                  }

                  if(socket_ccu_addr_dest.sin_port != htons(CCU_PORT_ARM))
                    socket_ccu_addr_dest.sin_port = htons(CCU_PORT_ARM);
  
                  sprintf((char *)ccu_buffer, "%i%ld ", (arm_request_index - 1), arm_link[arm_request_index - 1].actual_position);
                  bytes_sent = sendto(socket_ccu, ccu_buffer, bytes_read, 0, (struct sockaddr *)&socket_ccu_addr_dest, sizeof(socket_ccu_addr_dest));

                  if(bytes_sent < 0)
                    perror("sendto ccu");

                } 
                else if(arm_link[arm_request_index - 1].request_trajectory_status == 1)
                {
                  query_link = -1;
                  arm_link[arm_request_index - 1].request_timeout = 0;
                  arm_link[arm_request_index - 1].request_trajectory_status = 0;
                  arm_link[arm_request_index - 1].trajectory_status = atoi(arm_buffer_temp.param.arm_command);
  
                  if((arm_link[arm_request_index - 1].trajectory_status == 0) && (arm_request_index < MOTOR_NUMBER))
                    arm_set_command_without_value(arm_request_index, "OFF");

                  if(arm_request_index == MOTOR_NUMBER)
                  {
                    if(arm_link[arm_request_index - 1].trajectory_status > 0)
                    {
                      if(arm_link[arm_request_index -1].position_target > 0)
                        actuator_set_command(30000);
                      else
                        actuator_set_command(-30000);
                    }
                    else
                      link_homing_complete |= (int)pow(2, arm_request_index - 1);
                  }
                }
              }
            }
            else
              perror("arm_read");
  
            continue;      
          }
  
          if(FD_ISSET(socket_arm, &wr))
          {
            //printf("%i\n", rs232_buffer_tx_data_count);
            bytes_sent = arm_send(socket_arm, &arm_address);

            if(bytes_sent <= 0)
              printf("Error on pc_interface_send");
  
            continue;
          }
        }
        break;

      case ACU_RETURN_TO_BASE:

      case ACU_RETURN_TO_BASE_ABORT:
        //arm homing
        //drive segway to home
        //read sick
        //check joystick status
        if((socket_segway > 0) && (robotic_arm_selected == 0))
        {
          if(FD_ISSET(socket_segway, &rd))
          {
            bytes_read = segway_read(socket_segway, &segway_status, ccu_buffer);

            if(bytes_read <= 0)
              perror("segway_read");
            else
            {
              segway_check = 0;

              if(RTBstatus.control_values.heading == 0)
                segway_status.list.inertial_z_rate_rps = 0;

              if(RTBstatus.control_values.speed == 0)
                segway_status.list.linear_vel_mps = 0;

              if(segway_down == 1)
              {
                printf("Segway Init\t[OK]\n");
        
                segway_down = 0;
              }
            }
            continue;
          }
	 
          if(FD_ISSET(socket_segway, &wr))
          {
            segway_send(socket_segway, &segway_address);
            continue;
          }
        }
        break;

      default:
        status = ACU_IDLE;
//        printf("ACU_IDLE\n");
        continue;
        break;
    }

 
    timeout_return_to_base++;
    timeout_status++;

    // Send status info
    if((timeout_status * current_timeout) >= TIMEOUT_USEC_STATUS)
    {
      timeout_status = 0;
      if(socket_status > 0)
      {
        status_buffer = status;
        bytes_sent = sendto(socket_status, &status_buffer, sizeof(status_buffer), 0, (struct sockaddr *)&socket_status_addr_dest, sizeof(socket_status_addr_dest));

        if(bytes_sent < 0)
          perror("sendto joystick");
      }
    }
    
    // Joystick offline, return to base and clear time variable
    if((timeout_return_to_base * current_timeout) >= ((long)TIMEOUT_SEC_RETURN_TO_BASE * 1000000))
    {
      timeout_return_to_base = 0;
      if((status != ACU_RETURN_TO_BASE) && (previouse_status != ACU_RETURN_TO_BASE))
      {
        if(status != ACU_HOME)
        {
          arm_init(0, 1000, 10, 32767, 2000, 1500, 100, 700, 1023);
          arm_init(2, 20000, 10, 1500, 35000, 1500, 100, 500, 1023);

          status_return = arm_automatic_motion_start(ARM_HOME_FILE);

          printf("Return to base. . .\n");
  
          if(status_return > 0)
          {
            previouse_status = ACU_RETURN_TO_BASE;
            status = ACU_ARM_HOMING;
//            printf("ACU_ARM_HOMING\n");
            robotic_arm_selected = 0;
            printf("Start homing. . .\n");
#ifdef GPS_DEBUG
            gps_generate_init(2, 2, info.lat, info.lon, 0, info.elv, info.direction, 3, 3, &info);
#endif
          }
          else if(status_return == -1)
            perror("arm_automatic_motion_start");
          else
            printf("arm_motion_start error\n");
        }
        else
        {
          robotic_arm_selected = 0;
          status = ACU_RETURN_TO_BASE;
//          printf("ACU_RETURN_TO_BASE\n");
        }
      }
    }

    switch(status)
    {
      case ACU_HOME:
      case ACU_IDLE:
#ifdef GPS_SIMUL
        gps_timeout++;

        if(gps_timeout >= (1000000 / current_timeout)) // 1s
        {
	  //printf("Yaw rate: %f\n", convert_to_float(segway_status.list.inertial_z_rate_rps));
          gps_generate((float)convert_to_float(segway_status.list.linear_vel_mps) * 3.6, 
                       (convert_to_float(segway_status.list.inertial_z_rate_rps) * gps_timeout * current_timeout) / 1000000) * 180 / M_PI, 
                       gps_timeout * current_timeout, gps_buffer, &info);

          //printf("Generate gps command \n%s\n", gps_buffer);

          //printf("Yaw Rate: %f\n",((info.direction - old_direction)*100)/ rover_elapsed_time_hs);
          if((old_direction != info.direction) || (old_lat != info.lat) || (old_lon != info.lon))
          {
            RTB_update(GpsCoord2Double(info.lon), GpsCoord2Double(info.lat), (info.speed / 3.6), ((info.direction - old_direction)) / (gps_timeout * ((float)current_timeout)/1000000));
            old_direction = info.direction;
            old_lat = info.lat;
            old_lon = info.lon;
          }
          
          gps_timeout = 0;
        }
#endif
        select_timeout.tv_sec = TIMEOUT_SEC;
        select_timeout.tv_usec = TIMEOUT_USEC_STATUS;
        break;

      case ACU_ARM_AUTO_MOVE_ABORT:
#ifdef GPS_SIMUL
        gps_timeout++;

        if(gps_timeout >= (1000000 / current_timeout)) // 1s
        {
          gps_generate((float)convert_to_float(segway_status.list.linear_vel_mps) * 3.6, 
                       (convert_to_float(segway_status.list.inertial_z_rate_rps) * gps_timeout * current_timeout) / 1000000) * 180 / M_PI, 
                       gps_timeout * current_timeout, gps_buffer, &info);

          //printf("Generate gps command \n%s\n", gps_buffer);

          // If I have received all mandatory info (lat, lon, speed, direction) then update the rtb status
          if((old_direction != info.direction) || (old_lat != info.lat) || (old_lon != info.lon))
          {
            RTB_update(GpsCoord2Double(info.lon), GpsCoord2Double(info.lat), (info.speed / 3.6), ((info.direction - old_direction)) / (gps_timeout * ((float)current_timeout)/1000000));
            old_direction = info.direction;
            old_lat = info.lat;
            old_lon = info.lon;
          }
          
          gps_timeout = 0;
        }
#endif
        if(arm_stop(0))
        {
          arm_automatic_motion_abort();
          printf("Auto move aborted\n");
          status = ACU_IDLE;
//          printf("ACU_IDLE\n");
          arm_init(0, 1000, 10, 32767, 2000, 1500, 100, 700, 1023);
          arm_init(2, 20000, 10, 1500, 35000, 1500, 100, 500, 1023);
        }
        
        select_timeout.tv_sec = TIMEOUT_SEC;
        select_timeout.tv_usec = TIMEOUT_USEC_ARM;
        break;

      case ACU_ARM_HOMING:
      case ACU_ARM_AUTO_MOVE:
#ifdef GPS_SIMUL
        gps_timeout++;

        if(gps_timeout >= (1000000 / current_timeout)) // 1s
        {
          gps_generate((float)convert_to_float(segway_status.list.linear_vel_mps) * 3.6, 
                       (convert_to_float(segway_status.list.inertial_z_rate_rps) * gps_timeout * current_timeout) / 1000000) * 180 / M_PI, 
                       gps_timeout * current_timeout, gps_buffer, &info);

          //printf("Generate gps command \n%s\n", gps_buffer);
          // If I have received all mandatory info (lat, lon, speed, direction) then update the rtb status
          if((old_direction != info.direction) || (old_lat != info.lat) || (old_lon != info.lon))
          {
            RTB_update(GpsCoord2Double(info.lon), GpsCoord2Double(info.lat), (info.speed / 3.6), ((info.direction - old_direction)) / (gps_timeout * ((float)current_timeout)/1000000));
            old_direction = info.direction;
            old_lat = info.lat;
            old_lon = info.lon;
          }
       
          gps_timeout = 0;
        }
#endif
        if(arm_homing_check())
        {
          //printf("Arm homing check\n");
          if(arm_stop(0))
          {
            printf("Motion complete\n");

            arm_init(0, 1000, 10, 32767, 2000, 1500, 100, 700, 1023);
            arm_init(2, 20000, 10, 1500, 35000, 1500, 100, 500, 1023);
            timeout_return_to_base = 0;
            status = previouse_status;
	  }
        }

        select_timeout.tv_sec = TIMEOUT_SEC;
        select_timeout.tv_usec = TIMEOUT_USEC_ARM;
        break;

      case ACU_RETURN_TO_BASE:
#ifdef GPS_SIMUL
        gps_timeout++;

        if(gps_timeout >= (1000000 / current_timeout)) // 1s
        {
          gps_generate((float)convert_to_float(segway_status.list.linear_vel_mps) * 3.6, 
                       (convert_to_float(segway_status.list.inertial_z_rate_rps) * gps_timeout * current_timeout) / 1000000) * 180 / M_PI, 
                       gps_timeout * current_timeout, gps_buffer, &info);

          //printf("Generate gps command \n%s\n", gps_buffer);

          //gps_log(GpsCoord2Double(info.lat), GpsCoord2Double(info.lon));
          if(RTB_update(GpsCoord2Double(info.lon), GpsCoord2Double(info.lat), (info.speed / 3.6), 
             (info.direction - old_direction) / (gps_timeout * ((float)current_timeout)/1000000)) == RTB_tracking)
          {
	    printf("RTB_update\n");
            //printf("Angle: %f Divergenza: %f\n",(RTBstatus.control_vector.angle_deg_north - info.direction), (RTBstatus.control_vector.angle_deg_north - info.direction)/180);
              
            // if I'm in the right direction then activate speed value
            if((fabs(RTBstatus.control_vector.angle_deg_north - info.direction) < RTB_CONTROL_ANGLE) && (fabs(RTBstatus.control_vector.angle_deg_north - info.direction) > -RTB_CONTROL_ANGLE))
            {
              // Add a factor that depend on error
              RTBstatus.control_values.speed = (1 - (fabs(RTBstatus.control_vector.angle_deg_north - info.direction)) / 180) * cos((RTBstatus.control_vector.angle_deg_north - info.direction) * 3.14 / 180);

              // Add a factor (90 / RTB_CONTROL_ANGLE) to the angle to obtain a fastest convergence
              if((RTBstatus.control_vector.angle_deg_north - info.direction) >= 0)
              {
                RTBstatus.control_values.heading = sin(min((RTBstatus.control_vector.angle_deg_north - info.direction) * (90 / (RTB_CONTROL_ANGLE/2)), 90) * 3.14 / 180);
                printf("Heading factor: %f\n", min((RTBstatus.control_vector.angle_deg_north - info.direction) * (90 / (RTB_CONTROL_ANGLE/2)), 90));
              }
              else
              {
                RTBstatus.control_values.heading = sin(max((RTBstatus.control_vector.angle_deg_north - info.direction) * (90 / (RTB_CONTROL_ANGLE/2)), -90) * 3.14 / 180);
                printf("Heading factor: %f\n", max((RTBstatus.control_vector.angle_deg_north - info.direction) * (90 / (RTB_CONTROL_ANGLE/2)), -90));
              }
            }
            else
            {
              RTBstatus.control_values.speed = 0;
                
              // Choose angle to turn. If error > 180° then turn over 360
              if((RTBstatus.control_vector.angle_deg_north - info.direction) >= 0)
              {
                if((RTBstatus.control_vector.angle_deg_north - info.direction) <= 180)
                  RTBstatus.control_values.heading = 1;
                else 
                  RTBstatus.control_values.heading = -1;
              }
              else
              {
                if((RTBstatus.control_vector.angle_deg_north - info.direction) >= -180)
                  RTBstatus.control_values.heading = -1;
                else 
                  RTBstatus.control_values.heading = 1;
              }
            }
          }
          else
          {
            RTBstatus.control_values.heading = 0;
            RTBstatus.control_values.speed = 0;
          }
          
          gps_timeout = 0;
        }
#endif

        // Try to communicate with segway. If it is on tractor mode then I send the joystick
        // command 
        if(segway_prescaler_timeout >= 50)
        {
          segway_prescaler_timeout = 0;

          if((segway_status.list.operational_state < 3) || (segway_status.list.operational_state > 5))
          {
            printf("Segway Init. . .\n");
            segway_init(socket_segway, &segway_address, &segway_status);

            // Set max velocity
            //printf("segway_configure_max_vel\n");
            segway_configure_max_vel(socket_segway, &segway_address, 0.5/*MAX_VELOCITY*/);

            // Set max acceleration
            //printf("segway_configure_max_acc\n");
            segway_configure_max_acc(socket_segway, &segway_address, 0.5/*MAX_ACCELERATION*/);

            // Set max deceleration
            //printf("segway_configure_max_decel\n");
            segway_configure_max_decel(socket_segway, &segway_address, 0.5/*MAX_DECELERATION*/);
          }  
          else
            segway_configure_none(socket_segway, &segway_address, 0x00);

          segway_check++;

          if(segway_check > 3)
          {
            segway_check = 4;
        
            // Send warning only the first time
            if(segway_down == 0)
            {
              message_log("stdof", "Segway down!");
              printf("Segway down!\n");
              segway_status.list.operational_state = UNKNOWN;
            }
 
            segway_down = 1;
          }
        }
        else
          segway_prescaler_timeout++;

        segway_status_update(&segway_status, socket_segway, &segway_address, JOY_MAX_VALUE);
        
        if(segway_status.list.operational_state == SEGWAY_STANDBY)
        {
          // Request tractor mode
          bytes_sent = segway_configure_operational_mode(socket_segway, &segway_address, SEGWAY_TRACTOR_REQ);
      
          if(bytes_sent == -1)
          {
            //message_log("segway_configure_operational_mode to tractor", strerror(errno));
            perror("segway_configure_operational_mode");
          }
        }
  
        select_timeout.tv_sec = TIMEOUT_SEC;
        select_timeout.tv_usec = TIMEOUT_USEC_SEGWAY;
        break;

      case ACU_RETURN_TO_BASE_ABORT:
#ifdef GPS_SIMUL
        gps_timeout++;

        if(gps_timeout >= (1000000 / current_timeout)) // 1s
        {
          gps_generate((float)convert_to_float(segway_status.list.linear_vel_mps) * 3.6, 
                       (convert_to_float(segway_status.list.inertial_z_rate_rps) * gps_timeout * current_timeout) / 1000000) * 180 / M_PI, 
                       gps_timeout * current_timeout, gps_buffer, &info);

          //printf("Generate gps command \n%s\n", gps_buffer);

          // If I have received all mandatory info (lat, lon, speed, direction) then update the rtb status
          if((old_direction != info.direction) || (old_lat != info.lat) || (old_lon != info.lon))
          {
            RTB_update(GpsCoord2Double(info.lon), GpsCoord2Double(info.lat), (info.speed / 3.6), ((info.direction - old_direction)) / (gps_timeout * ((float)current_timeout)/1000000));
            old_direction = info.direction;
            old_lat = info.lat;
            old_lon = info.lon;
          }
          
          gps_timeout = 0;
        }
#endif
        // Stop segway and return in idle state
        if(socket_segway > 0)
        {
          if(segway_status.list.operational_state == SEGWAY_TRACTOR)
          {
            bytes_sent = segway_configure_operational_mode(socket_segway, &segway_address, SEGWAY_STANDBY_REQ);

            if(bytes_sent == -1)
            {
              message_log("segway_configure_operational_mode to standby", strerror(errno));
              perror("segway_configure_operational_mode");
            }
          }
          else
          {
            robotic_arm_selected = 1;
            RTB_set_mode(RTB_recording);
            status = ACU_HOME;
//            printf("ACU_HOME\n");
            printf("Signal found: continue mission\n");
          }
        }
        else
        {
          robotic_arm_selected = 1;
          RTB_set_mode(RTB_recording);
          status = ACU_HOME;
//          printf("ACU_HOME\n");
          printf("Signal found: continue mission\n");
        }

        select_timeout.tv_sec = TIMEOUT_SEC;
        select_timeout.tv_usec = TIMEOUT_USEC_SEGWAY;
        break;
    }

    current_timeout = select_timeout.tv_usec;
  }  // end while(!= done)

  return 0;
}

void segway_status_update(union segway_union *segway_status, int socket, struct sockaddr_in *segway_address, long int joy_max_value) 
{
  static int segway_previouse_state = 0;
  int bytes_sent = -1;

  switch(segway_status->list.operational_state)
  {
    case CCU_INIT:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        message_log("stdof", "Segway CCU Init");
        printf("Segway CCU Init\n");

        segway_previouse_state = segway_status->list.operational_state;
      }
      break;

    case PROPULSION_INIT:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        message_log("stdof", "Segway Propulsion Init");
        printf("Segway Propulsion Init\n");

        segway_previouse_state = segway_status->list.operational_state;
      }

      break;

    case CHECK_STARTUP:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        message_log("stdof", "Segway Check Startup Issue");
        printf("Segway Check Startup Issue\n");

        segway_previouse_state = segway_status->list.operational_state;
      }
      break;

    case SEGWAY_STANDBY: //standby mode
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        message_log("stdof", "Segway in Standby Mode");
        printf("Segway in Standby Mode\n");
  
        segway_previouse_state = segway_status->list.operational_state;
      }
      break;

    case SEGWAY_TRACTOR:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        message_log("stdof", "Segway in Tractor Mode");
        printf("Segway in Tractor Mode\n");

        segway_previouse_state = segway_status->list.operational_state;

        // Init RTB in track mode
        RTB_set_mode(RTB_tracking);
      }
      
      bytes_sent = segway_motion_set(socket, segway_address, 
                                     RTBstatus.control_values.speed,
                                     RTBstatus.control_values.heading, 1);
      
      if(bytes_sent < 0)
      {
        message_log("segway_motion_set", strerror(errno));
        perror("segway_motion_set");
      }
      break;

    case DISABLE_POWER:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        message_log("stdof", "Segway Disable Power");
        printf("Segway Disable Power\n");

        segway_previouse_state = segway_status->list.operational_state;
      }
      break;

    default:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        message_log("stdof", "Segway Uknown State");
        printf("Segway Unknown State\n");

        segway_previouse_state = segway_status->list.operational_state;
      }
      break;
  } //end switch
}

void message_log(const char *scope, const char *message) 
{
  char buffer[32];
  struct tm *ts;
  size_t last;
  time_t timestamp = time(NULL);
  FILE *file = NULL;

  // Init Log File
  file = fopen(LOG_FILE, "a");
 
  if(!file)
  {
    perror("logfile fopen:");
    return;
  }

  ts = localtime(&timestamp);
  last = strftime(buffer, 32, "%b %d %T", ts);
  buffer[last] = '\0';

  fprintf(file, "[%s]%s: %s\n", buffer, scope, message);

  fclose(file);
}

void gps_compare_log(double latitude, double longitude, double latitude_odometry, double longitude_odometry,
                     double velocity, double pdop, double hdop, double vdop, int sat_inview, int sat_used, double direction, 
		     double direction_bussola, long time_us)
{
  FILE *file = NULL;

  // Init Log File
  file = fopen("gps_compare_log", "a");
  
  if(!file)
  {
    perror("logfile fopen:");
    return;
  }
  
  fprintf(file, "%f,%f,0,%f,%f,0,%f,%f,%f,%f,%d,%d,%f,%f,%ld\n", longitude, latitude, longitude_odometry, latitude_odometry, 
	                                                  velocity, pdop, hdop, vdop, sat_inview, sat_used, direction, direction_bussola,
							  time_us);

  fclose(file);
}

void gps_text_log(char *string)
{
  FILE *file = NULL;

  // Init Log File
  file = fopen("gps_text_log", "a");
  
  if(!file)
  {
    perror("logfile fopen:");
    return;
  }
  
  fprintf(file, "%s\n", string);

  fclose(file);
}

int copy_log_file() {
  FILE *log = NULL;
  FILE *disk = NULL;
  char ch;

  disk = fopen("/media/usb/stdof_log.txt", "w");

  if(!disk)
    return -1;

  log = fopen(LOG_FILE, "r");

  if(!log)
  {
    fclose(disk);
    printf("log file:\n");
    return -1;
  }

  while((ch = fgetc(log)) != EOF)
  {
    if(feof(log))
      break;

    fputc(ch, disk);
  }
  
  fclose(log);
  fclose(disk);
  
  return 0;
}

/*double GpsCoord2Double(double gps_coord)
{
  double degree;
  double minute;
  
  degree = trunc(gps_coord / 100);
  minute = ((gps_coord - degree * 100) / 60);

  return (degree + minute); 
}*/