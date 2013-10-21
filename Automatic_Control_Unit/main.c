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

#define GPS_DEBUG

#define ARM_MAX_POSITION 0.1 // value in degree under which the link will be stopped

#define CCU_ADDRESS "192.168.1.102"
#define CCU_PORT_ARM 8000
#define SEGWAY_ADDRESS "192.168.1.40"
#define SEGWAY_PORT 55
#define ARM_ADDRESS "192.168.1.28"
#define ARM_PORT 8012
#define ACU_ARM_PORT 8015
#define ACU_STATUS_PORT 8016

#ifdef GPS_DEBUG
#define ACU_GPS_PORT 8017
#endif

#define ARM_PRESCALER 2
#define JOY_MAX_VALUE 32767 
#define JOYSTICK_ADDRESS "192.168.1.60"
#define JOYSTICK_STATUS_PORT 8014
#define TIMEOUT_SEC 0 
#define TIMEOUT_USEC_SEGWAY 100000
#define TIMEOUT_USEC_ARM 10000
#define TIMEOUT_SEC_RETURN_TO_BASE 60
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

/* Prototype */
void segway_status_update(union segway_union *, int, struct sockaddr_in *, long int);
//void arm_status_update(int socket, struct sockaddr_in *address, struct wwvi_js_event *jse, long int joy_max_value) ;
void message_log(const char *, const char *); 
int copy_log_file(void);

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
  
  /* Robotic Arm Client*/
  int socket_arm = -1;
  struct sockaddr_in arm_address;
  struct arm_frame arm_buffer_temp;
  unsigned char arm_request_index;
  char *arm_token_result;

  /* Status client interface*/
  int socket_status = -1;
  struct sockaddr_in socket_status_addr_dest;
  struct sockaddr_in socket_status_addr_src;
  unsigned char status_buffer;
  unsigned char status_return;

  /* Gps */
#ifdef GPS_DEBUG
  int socket_gps = -1;
  struct sockaddr_in gps_address;
  char gps_buffer_temp[2048];
#endif
  
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
  
  /* Init Gps */
#ifdef GPS_DEBUG
  socket_gps = socket(AF_INET, SOCK_DGRAM, 0);

  if(socket_gps < 0)
    perror("socket_gps");
 
  bzero(&gps_address, sizeof(gps_address));
  gps_address.sin_family = AF_INET;
  gps_address.sin_port = htons(ACU_GPS_PORT);
  gps_address.sin_addr.s_addr = htonl(INADDR_ANY);

  if(bind(socket_gps, (struct sockaddr *)&gps_address, sizeof(gps_address)) == -1)
    perror("gps_address");
#endif
    
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
#ifdef GPS_DEBUG
        if(socket_gps > 0)
        {
          FD_SET(socket_gps, &rd);
          nfds = max(nfds, socket_gps);
        }
#endif
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
#ifdef GPS_DEBUG
        if(socket_gps > 0)
        {
          FD_SET(socket_gps, &rd);
          nfds = max(nfds, socket_gps);
        }
#endif
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
        bytes_read = recvfrom(socket_status, &status_buffer, sizeof(status_buffer), 0, NULL, NULL);

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
              printf("Error on arm_send");
  
            continue;
          }
        }

#ifdef GPS_DEBUG
        if((socket_gps > 0))
        {
          if(FD_ISSET(socket_gps, &rd))
          {
            bytes_read = recvfrom(socket_gps, gps_buffer_temp, sizeof(gps_buffer_temp), 0, NULL, NULL);

            if(bytes_read > 0)
            {
              gps_buffer_temp[bytes_read] = '\0';
              printf("%s", gps_buffer_temp);
            }
            continue;
          }
        }
#endif
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
                *arm_token_result = '\0';
                arm_request_index = query_link;

                if(arm_link[arm_request_index - 1].request_trajectory_status == 1)
                {
                  arm_link[arm_request_index - 1].request_trajectory_status = 0;
                  arm_link[arm_request_index - 1].trajectory_status = atoi(arm_buffer_temp.param.arm_command);
                }
                
                query_link = -1;
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
        
                    //printf("velocity target for %i: %ld\n",arm_request_index, arm_link[arm_request_index - 1].velocity_target);
                    if((arm_link[arm_request_index - 1].actual_position < (arm_link[arm_request_index -1].position_target + (long)(arm_encoder_factor * arm_link[arm_request_index -1].gear/2))) && 
		      ((link_homing_complete & (int)pow(2, arm_request_index - 1)) == 0))
                    {
                      //printf("Motor%i Actual position: %ld, Position target range: %ld\n", arm_request_index, arm_link[arm_request_index - 1].actual_position, arm_link[arm_request_index -1].position_target + 400);
                      if(arm_stop(arm_request_index))
                      {
                        //printf("link_homing_complete |= %i\n", (int)pow(2, arm_request_index - 1));
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

      case ACU_RETURN_TO_BASE:
#ifdef GPS_DEBUG
        if((socket_gps > 0))
        {
          if(FD_ISSET(socket_gps, &rd))
          {
            bytes_read = recvfrom(socket_gps, gps_buffer_temp, sizeof(gps_buffer_temp), 0, NULL, NULL);

            if(bytes_read > 0)
            {
              gps_buffer_temp[bytes_read] = '\0';
              printf("%s", gps_buffer_temp);
            }
            continue;
          }
        }
#endif
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
          
              if(segway_down == 1)
              {
                printf("Segway Init\t[OK]\n");
        
                segway_configure_audio_command(socket_segway, &segway_address, 9);
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
        select_timeout.tv_sec = TIMEOUT_SEC;
        select_timeout.tv_usec = TIMEOUT_USEC_STATUS;
        break;

      case ACU_ARM_AUTO_MOVE_ABORT:
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
        if(arm_homing_check())
        {
          printf("Motion complete\n");

          arm_init(0, 1000, 10, 32767, 2000, 1500, 100, 700, 1023);
          arm_init(2, 20000, 10, 1500, 35000, 1500, 100, 500, 1023);
          timeout_return_to_base = 0;
          status = previouse_status;
        }

        select_timeout.tv_sec = TIMEOUT_SEC;
        select_timeout.tv_usec = TIMEOUT_USEC_ARM;
        break;

      case ACU_RETURN_TO_BASE:
        // Try to communicate with segway. If it is on tractor mode then I send the joystick
        // command 
        if(segway_prescaler_timeout >= 50)
        {
          segway_prescaler_timeout = 0;

          if((segway_status.list.operational_state < 3) || (segway_status.list.operational_state > 5))
          {
            //printf("Segway Init. . .\n");
            segway_init(socket_segway, &segway_address, &segway_status);
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
            message_log("segway_configure_operational_mode to tractor", strerror(errno));
            perror("segway_configure_operational_mode");
          }
        }
  
        select_timeout.tv_sec = TIMEOUT_SEC;
        select_timeout.tv_usec = TIMEOUT_USEC_SEGWAY;
        break;

      case ACU_RETURN_TO_BASE_ABORT:
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
            status = ACU_HOME;
//            printf("ACU_HOME\n");
            printf("Signal found: continue mission\n");
          }
        }
        else
        {
          robotic_arm_selected = 1;
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
      }
      
      bytes_sent = segway_motion_set(socket, segway_address, 0, joy_max_value, joy_max_value);
	      
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
