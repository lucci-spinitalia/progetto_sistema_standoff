#include <sys/types.h> 
#include <sys/ioctl.h> 
#include <sys/stat.h> 
#include <fcntl.h> 
#include <signal.h>

#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h>
#include <stdarg.h>

#include <sys/time.h> 
#include <time.h> 
#include <errno.h>

#include <locale.h>

#include <string.h>

#include <linux/joystick.h>

#include <sys/socket.h> 
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <net/if.h> 

#include "joystick.h" 
#include "segway_config.h" 
#include "segway_udp_v2.h" 
#include "socket_udp.h"
#include "arm_udp.h"

/* Led */
#define LED_READY 7
#define LED_RUN 48
#define LED_STDBY 49
#define LED_ARM 115

/* Joystick */
#define JOY_LOCAL_NAME "/dev/input/js0"
#define JOY_MAX_VALUE 32767 

/* Segway */
#define SEGWAY_ADDRESS "192.168.1.40"
#define SEGWAY_PORT 8080
#define SCU_SEGWAY_PORT 8017  // where to send segway info
#define CCU_SEGWAY_PORT 8003  // where to send segway info
#define SEGWAY_INACTIVITY_TIMEOUT_SEC 2

/* State request */
#define SCU_ADDRESS "192.168.1.28"
#define SCU_STATE_PORT 8016
#define CCU_STATE_PORT 8004  // where to send state info

/* CCU */
#define CCU_ADDRESS "192.168.1.102"

/* Arm */
#define ARM_ADDRESS "192.168.1.28"
#define ARM_PORT 8012

/* Timeout */
#define TIMEOUT_SEC 0 
#define SEGWAY_TIMEOUT_USEC 100000
#define ARM_TIMEOUT_USEC 50000
#define SCU_STATE_TIMEOUT_USEC 500000
#define LMS511_LED_TIMEOUT_SEC 1

#define SCU_STATE_TIMEOUT_LIMIT_SEC 5

// Standoff Control Unit status
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

/* Macro */
#undef max 
#define max(x,y) ((x) > (y) ? (x) : (y))
#define min(x,y) ((x) < (y) ? (x) : (y))

/* Signal */
struct sigaction signal_act;

/* Robotic Arm */
int arm_socket = -1;
unsigned char robotic_arm_selected = 0;
unsigned char step_request = 0;
unsigned char park_request_flag = 0;

/* Segway */
int segway_socket = -1;

/* CCU interface */
int ccu_socket = -1;

/* Joystick interface */
int joy_local = -1;

/* Status client interface*/
int scu_state_socket = -1;

/* Segway info for Scu */
int segway_scu_socket = -1;

/* Generic */
unsigned char joystick_rqst_rtb_flag = 0;
unsigned char joystick_rqst_reset_base_flag = 0;
unsigned char joystick_rqst_laser_flag = 0;
unsigned char scu_laser_state = 1;

/* Prototype */
int led_export(int pin_number);
int led_set_value(int pin_number, int value);
int led_set_direction(int pin_number, int value);
int eth_check_connection();
void segway_status_update(int socket, struct sockaddr_in *segway_address,
    union segway_union *segway_status, unsigned char scu_state, struct wwvi_js_event *jse,
    long int joy_max_value);
void arm_status_update(int socket, struct sockaddr_in *address, unsigned char arm_state,
    struct wwvi_js_event *jse, long int joy_max_value);

void signal_handler(int signum)
{
  // Garbage collection
  printf("Terminating program...\n");

  led_set_value(LED_READY, 0);
  led_set_value(LED_STDBY, 0);
  led_set_value(LED_RUN, 0);
  led_set_value(LED_ARM, 0);

  close(segway_socket);
  close(ccu_socket);
  close(arm_socket);
  close(joy_local);
  close(scu_state_socket);
  close(segway_scu_socket);

  exit(signum);
}

int main()
{
  /* Led */
  unsigned char led_ready_value = 0;
  unsigned char led_stdby_value = 0;
  unsigned char led_run_value = 0;

  /* Joystick interface */
  struct wwvi_js_event jse;

  /* Segway */
  struct sockaddr_in segway_address;
  union segway_union segway_status;
  unsigned char segway_check_counter = 0;
  unsigned char segway_down_flag = 1;

  struct timespec segway_timer_start, segway_timer_stop;
  long segway_elapsed_time_ns = 0;
  unsigned char segway_timer_enable_flag = 0;

  /* Status client interface*/
  struct sockaddr_in scu_state_addr_dest;
  unsigned char scu_state_buffer = -1;
  unsigned char scu_state_check_counter = 0;
  unsigned char scu_state_down_flag = 1;
  unsigned char scu_state = SCU_ARM_UNKNOWN;

  /* CCU interface */
  struct sockaddr_in ccu_socket_addr_dest;
  union segway_union ccu_segway_info;
  __u8 ccu_buffer[(SEGWAY_PARAM * 4) + 3];
  __u32 ccu_buffer_size = 0;

  /* Robotic Arm */
  struct sockaddr_in arm_address;

  /* Segway info for Scu */
  struct sockaddr_in segway_scu_socket_addr_dest;

  /* Generic Variable */
  int done = 0; // for the while in main loop
  int bytes_read; // to check how many bytes has been read
  int bytes_sent; // to check how many bytes has been sent

  unsigned char rtb_request_button = 0;
  unsigned char laser_request_button = 0;

  int select_result = -1; // value returned frome select()
  int nfds = 0; // fd to pass to select()
  fd_set rd, wr, er; // structure for select()
  struct timeval select_timeout;
  long current_timeout;
  long scu_state_timeout = 0;
  long segway_socket_timeout = 0;
  long arm_socket_timeout = 0;
  long lms511_led_timeout = 0;

  printf("Initializing stdof. . .\n");

  while(eth_check_connection() != 1)
  {
    printf("Waiting for network. . .\n");
    sleep(1);
  }

  /* Peripheral initialization */

  /* Signal */
  signal_act.sa_handler = signal_handler;
  sigemptyset(&signal_act.sa_mask);
  signal_act.sa_flags = 0;

  sigaction(SIGINT, &signal_act, NULL);
  sigaction(SIGTERM, &signal_act, NULL);

  /* Init Leds */
  // Init READY led
  if(led_export(LED_READY) <= 0)
    perror("export gpio");

  if(led_set_direction(LED_READY, 0) <= 0)
    perror("direction gpio");

  led_set_value(LED_READY, 0);

  if(led_export(LED_STDBY) <= 0)
    perror("export gpio");

  if(led_set_direction(LED_STDBY, 0) <= 0)
    perror("direction gpio");

  led_set_value(LED_STDBY, 0);

  if(led_export(LED_RUN) <= 0)
    perror("export gpio");

  if(led_set_direction(LED_RUN, 0) <= 0)
    perror("direction gpio");

  led_set_value(LED_RUN, 0);

  if(led_export(LED_ARM) <= 0)
    perror("export gpio");

  if(led_set_direction(LED_ARM, 0) <= 0)
    perror("direction gpio");

  led_set_value(LED_ARM, 0);

  /* Init Segway Client */
  if(segway_open(&segway_socket, &segway_address, SEGWAY_ADDRESS, SEGWAY_PORT) == -1)
    perror("init segway client");
  else
  {
    printf("Segway socket\t[connected]\n");
    memset(&segway_status, 0, sizeof(segway_status));
    segway_status.list.operational_state = UNKNOWN;
  }

  /* Init CCU Client */
  ccu_socket = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);

  if(ccu_socket < 0)
    perror("ccu_socket");
  else
    printf("CCU client\t[opened]\n");

  bzero(&ccu_socket_addr_dest, sizeof(ccu_socket_addr_dest));
  ccu_socket_addr_dest.sin_family = AF_INET;
  ccu_socket_addr_dest.sin_addr.s_addr = inet_addr(CCU_ADDRESS);
  ccu_socket_addr_dest.sin_port = htons(CCU_SEGWAY_PORT);

  /* Init Arm Client */
  if(arm_open(&arm_socket, &arm_address, ARM_ADDRESS, ARM_PORT) == -1)
    perror("init arm client");
  else
  {
    printf("Arm client\t[connected]\n");
    arm_crc_initialize();
  }

  /* Init Joystick */
  memset(&jse, 0, sizeof(struct wwvi_js_event));
  joy_local = open_joystick(JOY_LOCAL_NAME);

  if(joy_local < 0)
    perror("open_joystick(local)");
  else
    printf("Find Local Joystick\t[OK]\n");

  /* Status client interface */
  scu_state_socket = socket(AF_INET, SOCK_DGRAM, 0);

  if(scu_state_socket < 0)
    perror("scu_state_socket");
  else
    printf("Status client\t[opened]\n");

  bzero(&scu_state_addr_dest, sizeof(scu_state_addr_dest));
  scu_state_addr_dest.sin_family = AF_INET;
  scu_state_addr_dest.sin_addr.s_addr = inet_addr(SCU_ADDRESS);
  scu_state_addr_dest.sin_port = htons(SCU_STATE_PORT);

  /* Segway info for SCU interface */
  segway_scu_socket = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);

  if(segway_scu_socket < 0)
    perror("segway_scu_socket");
  else
    printf("Segway Info client to scu\t[opened]\n");

  bzero(&segway_scu_socket_addr_dest, sizeof(segway_scu_socket_addr_dest));
  segway_scu_socket_addr_dest.sin_family = AF_INET;
  segway_scu_socket_addr_dest.sin_addr.s_addr = inet_addr(SCU_ADDRESS);
  segway_scu_socket_addr_dest.sin_port = htons(SCU_SEGWAY_PORT);

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

    if(scu_state_socket > 0)
    {
      FD_SET(scu_state_socket, &rd);
      nfds = max(nfds, scu_state_socket);
    }

    if(joy_local > 0)
    {
      FD_SET(joy_local, &rd);
      nfds = max(nfds, joy_local);
    }
    else
    {
      /* Init Joystick */
      memset(&jse, 0, sizeof(struct wwvi_js_event));
      joy_local = open_joystick(JOY_LOCAL_NAME);

      if(joy_local > 0)
        printf("Find Local Joystick\t[OK]\n");
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

    if(arm_socket > 0)
    {
      //FD_SET(arm_socket, &rd);
      //nfds = max(nfds, arm_socket);  

      if(arm_buffer_tx_empty == 0)
      {
        FD_SET(arm_socket, &wr);
        nfds = max(nfds, arm_socket);
      }
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

    if(select_result > 0)
    {
      /* Manage SCU state answer */
      if(scu_state_socket > 0)
      {
        if(FD_ISSET(scu_state_socket, &rd))
        {
          bytes_read = recvfrom(scu_state_socket, &scu_state_buffer, sizeof(scu_state_buffer), 0,
              NULL, NULL);

          if(bytes_read > 0)
          {
            scu_state = scu_state_buffer;

            scu_state_check_counter = 0;

            if(scu_state_down_flag == 1)
            {
              scu_state_down_flag = 0;

              printf("Standoff Control Unit\t[online]\n");
              if(led_ready_value == 0)
              {
                led_ready_value = 1;
                led_set_value(LED_READY, 1);
              }
            }
          }
          else
            perror("status_read");
        }
      }

      /* Manage segway message */
      if(segway_socket > 0)
      {
        if(FD_ISSET(segway_socket, &rd))
        {
          bytes_read = segway_read(segway_socket, &segway_status, NULL);

          if(bytes_read <= 0)
            perror("segway_read");
          else
          {
            segway_check_counter = 0;

            if(segway_down_flag == 1)
            {
              printf("Segway Online\n");
              segway_down_flag = 0;
            }

            // Update LED status
            switch(segway_status.list.operational_state)
            {
              case SEGWAY_STANDBY: //standby 
                // Unset RUN led
                if(led_run_value == 1)
                {
                  led_set_value(LED_RUN, 0);
                  led_run_value = 0;
                }

                // Set STDBY led
                if(led_stdby_value == 0)
                {
                  printf("Segway in Standby Mode\n");
                  led_set_value(LED_STDBY, 1);
                  led_stdby_value = 1;
                }
                break;

              case SEGWAY_TRACTOR:
                // Unset STDBY led
                if(led_stdby_value == 1)
                {
                  led_set_value(LED_STDBY, 0);
                  led_stdby_value = 0;
                }

                // Set RUN led
                if(led_run_value == 0)
                {
                  led_set_value(LED_RUN, 1);
                  led_run_value = 1;
                  printf("Segway in Tractor Mode\n");
                }
                break;

              case CCU_INIT:
              case PROPULSION_INIT:
              case CHECK_STARTUP:
              case DISABLE_POWER:
              default:
                // Unset RUN led
                if(led_run_value == 1)
                {
                  led_set_value(LED_RUN, 0);
                  led_run_value = 0;
                  printf("Segway not ready...\n");
                }

                // Set STDBY led
                if(led_stdby_value == 1)
                {
                  led_set_value(LED_STDBY, 0);
                  led_stdby_value = 0;
                }
                break;
            } //end switch

            // Send info to ccu and scu
            memcpy(&ccu_segway_info, &segway_status, sizeof(segway_status));

            if((jse.stick_y < 1000) && (jse.stick_y > -1000))
              ccu_segway_info.list.linear_vel_mps = 0;

            if((jse.stick_x < 1000) && (jse.stick_x > -1000))
              ccu_segway_info.list.inertial_z_rate_rps = 0;

            segway_convert_param_message(ccu_segway_info, ccu_buffer, &ccu_buffer_size);

            if(ccu_socket_addr_dest.sin_port != htons(CCU_SEGWAY_PORT))
              ccu_socket_addr_dest.sin_port = htons(CCU_SEGWAY_PORT);

            if(ccu_socket > 0)
            {
              bytes_sent = sendto(ccu_socket, ccu_buffer, ccu_buffer_size, 0,
                  (struct sockaddr *) &ccu_socket_addr_dest, sizeof(ccu_socket_addr_dest));

              if(bytes_sent < 0)
                perror("sendto segway_info to ccu");
            }

            if(segway_scu_socket > 0)
            {
              bytes_sent = sendto(segway_scu_socket, ccu_buffer, ccu_buffer_size, 0,
                  (struct sockaddr *) &segway_scu_socket_addr_dest,
                  sizeof(segway_scu_socket_addr_dest));

              if(bytes_sent < 0)
                perror("sendto segway_info to scu");
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
      }

      /* Manage arm message */
      if(arm_socket > 0)
      {
        if(FD_ISSET(arm_socket, &wr))
        {
          if(arm_send(arm_socket, &arm_address) < 0)
            printf("Error on arm_send\n");
        }
      }

      /* Manage joystick message */
      if(joy_local > 0)
      {
        if(FD_ISSET(joy_local, &rd))
        {
          bytes_read = get_joystick_status(&joy_local, &jse);

          if(bytes_read <= 0)
          {
            perror("get_joystick_status");
            close(joy_local);
            joy_local = -1;
          }
        }
      }
    }

    if((select_timeout.tv_sec == 0) && (select_timeout.tv_usec == 0))
    {
      if(robotic_arm_selected == 0)
      {
        segway_socket_timeout++;
        if((segway_socket_timeout * current_timeout) >= SEGWAY_TIMEOUT_USEC)
        {
          segway_socket_timeout = 0;

          // Increase segway inactivity counter
          segway_check_counter++;

          if((segway_check_counter * SEGWAY_TIMEOUT_USEC)
              >= (long) SEGWAY_INACTIVITY_TIMEOUT_SEC * 1000000)
          {
            segway_check_counter = 0;

            // Send warning only once
            if(segway_down_flag == 0)
            {
              printf("Segway down!\n");
              segway_status.list.operational_state = UNKNOWN;

              //Update led
              if(led_stdby_value == 1)
              {
                led_set_value(LED_STDBY, 0);
                led_stdby_value = 0;
              }

              if(led_run_value == 1)
              {
                led_set_value(LED_RUN, 0);
                led_run_value = 0;
              }

              segway_down_flag = 1;
            }

            // send new segway status
            memcpy(&ccu_segway_info, &segway_status, sizeof(segway_status));

            if((jse.stick_y < 1000) && (jse.stick_y > -1000))
              ccu_segway_info.list.linear_vel_mps = 0;

            if((jse.stick_x < 1000) && (jse.stick_x > -1000))
              ccu_segway_info.list.inertial_z_rate_rps = 0;

            segway_convert_param_message(ccu_segway_info, ccu_buffer, &ccu_buffer_size);

            if(ccu_socket_addr_dest.sin_port != htons(CCU_SEGWAY_PORT))
              ccu_socket_addr_dest.sin_port = htons(CCU_SEGWAY_PORT);

            if(ccu_socket > 0)
            {
              bytes_sent = sendto(ccu_socket, ccu_buffer, ccu_buffer_size, 0,
                  (struct sockaddr *) &ccu_socket_addr_dest, sizeof(ccu_socket_addr_dest));

              if(bytes_sent < 0)
                perror("sendto segway info to ccu");
            }

            if(segway_scu_socket > 0)
            {
              bytes_sent = sendto(segway_scu_socket, ccu_buffer, ccu_buffer_size, 0,
                  (struct sockaddr *) &segway_scu_socket_addr_dest,
                  sizeof(segway_scu_socket_addr_dest));

              if(bytes_sent < 0)
                perror("sendto segway info to scu");
            }
          }

          if(joystick_rqst_rtb_flag == 0)
          {
            /*printf("status word 1: %lx\n",segway_status.list.fault_status_word1);
             printf("status word 2: %lx\n",segway_status.list.fault_status_word2);
             printf("status word 3: %lx\n",segway_status.list.fault_status_word3);
             printf("status word 4: %lx\n",segway_status.list.fault_status_word4);
             printf("mcu fault 0: %lx\n",segway_status.list.mcu_0_fault_status);
             printf("mcu fault 1: %lx\n",segway_status.list.mcu_1_fault_status);
             printf("mcu fault 2: %lx\n",segway_status.list.mcu_2_fault_status);
             printf("mcu fault 3: %lx\n",segway_status.list.mcu_3_fault_status);*/

            /*segway_config_decode_arch_fault(segway_status.list.fault_status_word1, char *fault_message);
             segway_config_decode_critical_fault(segway_status.list.fault_status_word1, char *fault_message);
             segway_config_decode_comm_fault(segway_status.list.fault_status_word2, char *fault_message);
             segway_config_decode_internal_fault(segway_status.list.fault_status_word2, char *fault_message);
             segway_config_decode_sensors_fault(segway_status.list.fault_status_word3, char *fault_message);
             segway_config_decode_bsa_fault(segway_status.list.fault_status_word3, char *fault_message);
             segway_config_decode_mcu_fault(segway_status.list.fault_status_word3, char *fault_message);*/

            segway_status_update(segway_socket, &segway_address, &segway_status, scu_state, &jse,
                JOY_MAX_VALUE);
          }

          // Check for rtb request
          if((jse.button[3]) && (jse.button[4]))
            rtb_request_button = 1;

          if((!jse.button[3]) && rtb_request_button)
          {
            rtb_request_button = 0;

            if(jse.stick_z > (JOY_MAX_VALUE - 5000))  //z positive
            {
              if(joystick_rqst_rtb_flag == 0)
              {
                printf("Requested return to base to scu\n");
                joystick_rqst_rtb_flag = 1;
              }
              else if(joystick_rqst_rtb_flag == 1)
              {
                printf("Aborting return to base\n");
                joystick_rqst_rtb_flag = 0;
              }
            }
            else if(jse.stick_z < -(JOY_MAX_VALUE - 5000))  //z negative
            {
              if(joystick_rqst_reset_base_flag == 0)
              {
                printf("Request reset base\n");
                joystick_rqst_reset_base_flag = 1;
              }
            }
          }

          // Check for laser enable/disable request
          if((jse.button[2]) && (jse.button[4]))
            laser_request_button = 1;

          if((!jse.button[2]) && laser_request_button)
          {
            laser_request_button = 0;

            if(jse.stick_z > (JOY_MAX_VALUE - 5000))  //z positive
            {
              if(joystick_rqst_laser_flag == 0)
              {
                if(scu_laser_state == 1)
                  printf("Request to disable laser scanner\n");
                else
                  printf("Request to enable laser scanner\n");

                joystick_rqst_laser_flag = 1;
              }
            }
          }
        }
      }

      if(robotic_arm_selected == 1)
      {
        arm_socket_timeout++;
        if((arm_socket_timeout * current_timeout) >= ARM_TIMEOUT_USEC)
        {
          arm_socket_timeout = 0;

          if(joystick_rqst_rtb_flag == 0)
            arm_status_update(scu_state_socket, &scu_state_addr_dest, scu_state, &jse,
                JOY_MAX_VALUE);

          // Check for rtb request
          if((jse.button[3]) && (jse.button[4]))
            rtb_request_button = 1;

          if((!jse.button[3]) && rtb_request_button)
          {
            rtb_request_button = 0;

            if(jse.stick_z > (JOY_MAX_VALUE - 5000))  //z positive
            {
              if(joystick_rqst_rtb_flag == 0)
              {
                printf("Requested return to base to scu\n");
                joystick_rqst_rtb_flag = 1;
              }
              else if(joystick_rqst_rtb_flag == 1)
              {
                printf("Aborting return to base\n");
                joystick_rqst_rtb_flag = 0;
              }
            }
            else if(jse.stick_z < -(JOY_MAX_VALUE - 5000))  //z negative
            {
              if(joystick_rqst_reset_base_flag == 0)
              {
                printf("Requested reset base\n");
                joystick_rqst_reset_base_flag = 1;
              }

            }
          }
        }
      }

      scu_state_timeout++;
      if((scu_state_timeout * current_timeout) >= SCU_STATE_TIMEOUT_USEC)
      {
        scu_state_timeout = 0;

        if(joystick_rqst_rtb_flag == 1)
          scu_state_buffer = SCU_RQST_RTB;
        else if(joystick_rqst_reset_base_flag == 1)
        {
          scu_state_buffer = SCU_RQST_RESET_BASE;
          joystick_rqst_reset_base_flag = 0;
        }
        else if(joystick_rqst_laser_flag == 1)
        {
          if(scu_laser_state == 0)
          {
            scu_state_buffer = SCU_RQST_LASER_ON;
            scu_laser_state = 1;

            if(scu_state_down_flag == 0)
            {
              led_set_value(LED_READY, 1);
              led_ready_value = 1;
            }
            else
            {
              led_set_value(LED_READY, 0);
              led_ready_value = 0;
            }
          }
          else
          {
            scu_state_buffer = SCU_RQST_LASER_OFF;
            scu_laser_state = 0;
          }

          joystick_rqst_laser_flag = 0;
        }
        else
          scu_state_buffer = SCU_RQST_STATE;

        if(scu_state_socket > 0)
        {
          bytes_sent = sendto(scu_state_socket, &scu_state_buffer, sizeof(scu_state_buffer), 0,
              (struct sockaddr *) &scu_state_addr_dest, sizeof(scu_state_addr_dest));

          if(bytes_sent < 0)
            perror("sendto scu_state request");
        }

        if(scu_state_down_flag == 0)
        {
          scu_state_check_counter++;
          if(scu_state_check_counter * SCU_STATE_TIMEOUT_USEC
              >= (long) SCU_STATE_TIMEOUT_LIMIT_SEC * 1000000)
          {
            scu_state_check_counter--;

            scu_state_down_flag = 1;
            scu_state = SCU_ARM_UNKNOWN;

            printf("Standoff Control Unit\t[offline]\n");
            // Unset led READY
            if(led_ready_value == 1)
            {
              led_ready_value = 0;
              led_set_value(LED_READY, 0);
            }
          }
        }
        else
          scu_state_check_counter = 0;

        // Send the new scu state to CCU
        if(ccu_socket > 0)
        {
          ccu_socket_addr_dest.sin_port = htons(CCU_STATE_PORT);
          bytes_sent = sendto(ccu_socket, &scu_state, sizeof(scu_state), 0,
              (struct sockaddr *) &ccu_socket_addr_dest, sizeof(ccu_socket_addr_dest));

          if(bytes_sent < 0)
            perror("sendto scu_state to ccu");
        }
      }

      if((scu_laser_state == 0) && (scu_state_down_flag == 0))
      {
        lms511_led_timeout++;
        if((lms511_led_timeout * current_timeout) >= (LMS511_LED_TIMEOUT_SEC * 1000000))
        {
          lms511_led_timeout = 0;

          if(led_ready_value == 0)
          {
            led_set_value(LED_READY, 1);
            led_ready_value = 1;
          }
          else
          {
            led_set_value(LED_READY, 0);
            led_ready_value = 0;
          }
        }
      }

      select_timeout.tv_sec = TIMEOUT_SEC;
      select_timeout.tv_usec = ARM_TIMEOUT_USEC;

      current_timeout = select_timeout.tv_usec;
    }
  }  // end while(!= done)
  return 0;
}

void segway_status_update(int socket, struct sockaddr_in *segway_address,
    union segway_union *segway_status, unsigned char scu_state, struct wwvi_js_event *jse,
    long int joy_max_value)
{
  static unsigned char change_actuator_request = 0;
  static unsigned char change_mode_request = 0;
  static unsigned char led_timer = 0;
  static unsigned char arm_led = 0;

  struct arm_frame arm_command;
  int bytes_sent = -1;

  switch(segway_status->list.operational_state)
  {
    case SEGWAY_STANDBY: //standby 
      if(scu_state != SCU_ARM_UNKNOWN)
      {
        // Change actuator
        if((jse->button[3]) && (jse->button[4] == 0))
          change_actuator_request = 1;

        if((!jse->button[3]) && change_actuator_request)
        {
          change_actuator_request = 0;

          robotic_arm_selected = 1;

          printf("Pass to robotic arm\n");

          // Set ARM led
          led_set_value(LED_ARM, 1);
          break;
        }

        if(jse->button[0])
          change_mode_request = 1;

        if(segway_buffer_tx_empty == 1)
        {
          if((!jse->button[0]) && change_mode_request)
          {
            change_mode_request = 0;

            bytes_sent = segway_configure_operational_mode(socket, segway_address,
                SEGWAY_TRACTOR_REQ);

            if(bytes_sent == -1)
              perror("segway_configure_operational_mode");
          }
          else
          {
            segway_configure_none(socket, segway_address, 0x00);
          }
        }
      }
      break;

    case SEGWAY_TRACTOR:
      switch(scu_state)
      {
        case SCU_ARM_IDLE:
        case SCU_ARM_REST:
        case SCU_ARM_OUT_OF_SERVICE:
          if(jse->button[2]) // Change actuator and move it into step position
            change_actuator_request = 1;

          if((!jse->button[2]) && change_actuator_request)
          {
            change_actuator_request = 0;

            if(step_request == 0)
            {
              //load step request
              step_request = 1;
              arm_command.arm_command_param.header_uint = ARM_RQST_STEP;

              if(arm_load_tx(arm_command) != 1)
                printf("Cannot load command for arm\n");
            }
            else
            {
              //load park request
              step_request = 0;
              arm_command.arm_command_param.header_uint = ARM_RQST_PARK;

              if(arm_load_tx(arm_command) != 1)
                printf("Cannot load command for arm\n");
            }

            break;
          }

          goto operate_with_segway;
          break;

        case SCU_ARM_AUTO_MOVE:
          // Update LED
          if(led_timer * SEGWAY_TIMEOUT_USEC >= 1000000) // 1 s
          {
            led_timer = 0;
            if(arm_led)
              arm_led = 0;
            else
              arm_led = 1;

            // Toggle ARM led
            led_set_value(LED_ARM, arm_led);
          }

          if(jse->button[0] || jse->button[1] || jse->button[2])
            change_mode_request = 1;

          if(!jse->button[0] && !jse->button[1] && !jse->button[2] && change_mode_request)
          {
            change_mode_request = 0;

            // send abort request
            arm_command.arm_command_param.header_uint = ARM_RQST_ABORT;

            if(arm_load_tx(arm_command) != 1)
              printf("Cannot load command for arm\n");
            break;
          }

          if(segway_buffer_tx_empty == 1)
          {
            bytes_sent = segway_motion_set(socket, segway_address, 0, 0, joy_max_value);

            if(bytes_sent < 0)
              perror("segway_motion_set");
          }
          break;

        case SCU_ARM_UNKNOWN:
          break;

        case SCU_ARM_END_EFFECTOR_1:
        case SCU_ARM_END_EFFECTOR_2:
        case SCU_ARM_END_EFFECTOR_3:
        case SCU_ARM_END_EFFECTOR_4:
        case SCU_ARM_END_EFFECTOR_5:
        case SCU_ARM_END_EFFECTOR_6:
        case SCU_ARM_END_EFFECTOR_7:

operate_with_segway:

          // Update LED
          if(arm_led == 1)
          {
            arm_led = 0;

            // Unset ARM led
            led_set_value(LED_ARM, arm_led);
          }

          if((jse->button[4]) && (segway_buffer_tx_empty == 1)) // If movement request
          {
            bytes_sent = segway_motion_set(socket, segway_address, jse->stick_y, jse->stick_x,
                joy_max_value);

            if(bytes_sent < 0)
              perror("segway_motion_set");

            break;
          }

          if(jse->button[1])  // Change mode to standby
            change_mode_request = 1;

          if((!jse->button[1]) && (step_request == 0) && change_mode_request
              && (segway_buffer_tx_empty == 1))
          {
            change_mode_request = 0;

            if(step_request == 0)
            {
              bytes_sent = segway_configure_operational_mode(socket, segway_address,
                  SEGWAY_STANDBY_REQ);

              if(bytes_sent == -1)
                perror("segway_configure_operational_mode");

              break;
            }
          }

          if(segway_buffer_tx_empty == 1)
          {
            bytes_sent = segway_motion_set(socket, segway_address, 0, 0, joy_max_value);

            if(bytes_sent < 0)
              perror("segway_motion_set");
          }
          break;

        default:

          printf("exe default status flag: %d\n", scu_state);
          // Update LED
          if(arm_led == 1)
          {
            arm_led = 0;

            // Unset ARM led
            led_set_value(LED_ARM, arm_led);
          }

          if(segway_buffer_tx_empty == 1)
          {
            bytes_sent = segway_motion_set(socket, segway_address, 0, 0, joy_max_value);

            if(bytes_sent < 0)
              perror("segway_motion_set");
          }
          break;
      }
      break;

    case CCU_INIT:
    case PROPULSION_INIT:
    case CHECK_STARTUP:
    case DISABLE_POWER:
    default:
      if(scu_state != SCU_ARM_UNKNOWN)
      {
        // Change actuator
        if((jse->button[3]) && (jse->button[4] == 0))
          change_actuator_request = 1;

        if((!jse->button[3]) && change_actuator_request)
        {
          change_actuator_request = 0;

          robotic_arm_selected = 1;
          printf("Pass to robotic arm\n");

          // Set ARM led
          led_set_value(LED_ARM, 1);
        }
        else
        {
          if(segway_buffer_tx_empty == 1)
            segway_init(socket, segway_address, segway_status);
        }
      }

      break;
  } //end switch
}

void arm_status_update(int scu_state_socket, struct sockaddr_in *address, unsigned char arm_state,
    struct wwvi_js_event *jse, long int joy_max_value)
{
  static unsigned char change_actuator_request = 0; // indicates when button has been released
  static unsigned char move_botton_request = 0;
  static unsigned char set_origin_button_request = 0;

  static unsigned char arm_led = 0;
  static unsigned char led_timer = 0;

  static unsigned char arm_stop_flag = 0;
  static unsigned char arm_abort_flag = 0;

  struct arm_frame arm_command;

  switch(arm_state)
  {
    case SCU_ARM_IDLE:
      if(arm_stop_flag)
        arm_stop_flag = 0;

      if(arm_abort_flag)
        arm_abort_flag = 0;

      // Update LED
      if(arm_led == 0)
      {
        arm_led = 1;

        // Set ARM led
        led_set_value(LED_ARM, arm_led);
      }

      if(jse->button[4] && jse->button[0])
        set_origin_button_request = 1;
      else if(jse->button[3] && !jse->button[0] && !jse->button[1] && !jse->button[2]
          && !jse->button[4])
        change_actuator_request = 1;
      else if((jse->button[0] || jse->button[1] || jse->button[2]) && !change_actuator_request)
      {
        move_botton_request = 1;

        if(jse->button[0])
        {
          arm_command.arm_command_param.header_uint = ARM_CMD_FIRST_TRIPLET;
          arm_command.arm_command_param.value1 = ((float) jse->stick_x / joy_max_value);
          arm_command.arm_command_param.value2 = ((float) jse->stick_y / joy_max_value);
          arm_command.arm_command_param.value3 = ((float) jse->stick_z / joy_max_value);
        }
        else if(jse->button[1])
        {
          arm_command.arm_command_param.header_uint = ARM_CMD_SECOND_TRIPLET;
          arm_command.arm_command_param.value1 = ((float) jse->stick_x / joy_max_value);
          arm_command.arm_command_param.value2 = ((float) jse->stick_y / joy_max_value);
          arm_command.arm_command_param.value3 = ((float) jse->stick_z / joy_max_value);
        }
        else if(jse->button[2])
        {
          arm_command.arm_command_param.header_uint = ARM_CMD_ACTUATOR;

          if(jse->stick_z > 5000)
            arm_command.arm_command_param.value1 = 1;
          else if(jse->stick_z < -5000)
            arm_command.arm_command_param.value1 = -1;
          else
            arm_command.arm_command_param.value1 = 0;
        }

        if(arm_load_tx(arm_command) != 1)
          printf("Cannot load command for arm\n");
      }

      if((!jse->button[3]) && change_actuator_request)  //button released
      {
        change_actuator_request = 0;

        if(jse->stick_z > (JOY_MAX_VALUE - 5000))  //z positive
        {
          arm_command.arm_command_param.header_uint = ARM_RQST_PARK;
          park_request_flag = 1;
        }
        else if(jse->stick_z < -(JOY_MAX_VALUE - 5000))  //z negative
          arm_command.arm_command_param.header_uint = ARM_RQST_PARK_CLASSA;
        else if((jse->stick_y > (JOY_MAX_VALUE - 5000)) && (jse->stick_x < (JOY_MAX_VALUE - 5000))
            && (jse->stick_x > -(JOY_MAX_VALUE - 5000))) // joy up
        {
          if(jse->button[4])
            arm_command.arm_command_param.header_uint = ARM_RQST_DINAMIC;
          else
            arm_command.arm_command_param.header_uint = ARM_RQST_GET_TOOL_1;
        }
        else if((jse->stick_y > (JOY_MAX_VALUE - 5000)) && (jse->stick_x > (JOY_MAX_VALUE - 5000))) // joy up-right
          arm_command.arm_command_param.header_uint = ARM_RQST_GET_TOOL_2;
        else if((jse->stick_y < (JOY_MAX_VALUE - 5000)) && (jse->stick_y > -(JOY_MAX_VALUE - 5000))
            && (jse->stick_x > (JOY_MAX_VALUE - 5000))) // joy right
          arm_command.arm_command_param.header_uint = ARM_RQST_GET_TOOL_3;
        else if((jse->stick_y < -(JOY_MAX_VALUE - 5000)) && (jse->stick_x > (JOY_MAX_VALUE - 5000))) // joy down-right
          arm_command.arm_command_param.header_uint = ARM_RQST_GET_TOOL_4;
        else if((jse->stick_y < -(JOY_MAX_VALUE - 5000)) && (jse->stick_x < (JOY_MAX_VALUE - 5000))
            && (jse->stick_x > -(JOY_MAX_VALUE - 5000))) // joy down
        {
          if(jse->button[4])
            arm_command.arm_command_param.header_uint = ARM_RQST_STEADY;
          else
            arm_command.arm_command_param.header_uint = ARM_RQST_GET_TOOL_5;
        }
        else if((jse->stick_y < -(JOY_MAX_VALUE - 5000))
            && (jse->stick_x < -(JOY_MAX_VALUE - 5000))) // joy down-left
          arm_command.arm_command_param.header_uint = ARM_RQST_GET_TOOL_6;
        else if((jse->stick_y < (JOY_MAX_VALUE - 5000)) && (jse->stick_y > -(JOY_MAX_VALUE - 5000))
            && (jse->stick_x < -(JOY_MAX_VALUE - 5000))) // joy left
          arm_command.arm_command_param.header_uint = ARM_RQST_GET_TOOL_7;
        else if((jse->stick_y > (JOY_MAX_VALUE - 5000)) && (jse->stick_x < -(JOY_MAX_VALUE - 5000))) // joy up-left
          arm_command.arm_command_param.header_uint = ARM_RQST_PUMP;
        else
        {
          robotic_arm_selected = 0;

          // Unset ARM led
          led_set_value(LED_ARM, 0);

          printf("Pass to vehicle\n");
          break;
        }

        if(arm_load_tx(arm_command) != 1)
          printf("Cannot load command for arm\n");
      }

      if((!jse->button[4]) && set_origin_button_request)  //button released
      {
        set_origin_button_request = 0;

        arm_command.arm_command_param.header_uint = ARM_SET_ORIGIN;
        if(arm_load_tx(arm_command) != 1)
          printf("Cannot load command for arm\n");
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

      // Update LED
      if(arm_led == 0)
      {
        arm_led = 1;

        // Set ARM led
        led_set_value(LED_ARM, arm_led);
      }

      // In this state I can move the arm, pass to veichle or put the end effector into the box
      if(jse->button[3] && !jse->button[0] && !jse->button[1] && !jse->button[2] && !jse->button[4])
        change_actuator_request = 1;
      else if(jse->button[0] || jse->button[1] || jse->button[2])
      {
        move_botton_request = 1;

        if(jse->button[0])
        {
          arm_command.arm_command_param.header_uint = ARM_CMD_FIRST_TRIPLET;
          arm_command.arm_command_param.value1 = ((float) jse->stick_x / joy_max_value);
          arm_command.arm_command_param.value2 = ((float) jse->stick_y / joy_max_value);
          arm_command.arm_command_param.value3 = ((float) jse->stick_z / joy_max_value);
        }
        else if(jse->button[1])
        {
          arm_command.arm_command_param.header_uint = ARM_CMD_SECOND_TRIPLET;
          arm_command.arm_command_param.value1 = ((float) jse->stick_x / joy_max_value);
          arm_command.arm_command_param.value2 = ((float) jse->stick_y / joy_max_value);
          arm_command.arm_command_param.value3 = ((float) jse->stick_z / joy_max_value);
        }
        else if(jse->button[2])
        {
          arm_command.arm_command_param.header_uint = ARM_CMD_ACTUATOR;

          if(jse->stick_z > 5000)
            arm_command.arm_command_param.value1 = 1;
          else if(jse->stick_z < -5000)
            arm_command.arm_command_param.value1 = -1;
          else
            break;
        }

        if(arm_load_tx(arm_command) != 1)
          printf("Cannot load command for arm\n");
      }

      if((!jse->button[3]) && change_actuator_request)  //button released
      {
        if((jse->stick_y > (JOY_MAX_VALUE - 5000)) || (jse->stick_y < -(JOY_MAX_VALUE - 5000))
            || (jse->stick_x > (JOY_MAX_VALUE - 5000)) || (jse->stick_x < -(JOY_MAX_VALUE - 5000))) // joy up
        {
          if(arm_state == SCU_ARM_END_EFFECTOR_1)
            arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_1;
          else if(arm_state == SCU_ARM_END_EFFECTOR_2)
            arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_2;
          else if(arm_state == SCU_ARM_END_EFFECTOR_3)
            arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_3;
          else if(arm_state == SCU_ARM_END_EFFECTOR_4)
            arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_4;
          else if(arm_state == SCU_ARM_END_EFFECTOR_5)
            arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_5;
          else if(arm_state == SCU_ARM_END_EFFECTOR_6)
            arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_6;
          else if(arm_state == SCU_ARM_END_EFFECTOR_7)
            arm_command.arm_command_param.header_uint = ARM_RQST_PUT_TOOL_7;

          if(arm_load_tx(arm_command) != 1)
            printf("Cannot load command for arm\n");
        }
        else
        {
          robotic_arm_selected = 0;

          // Unset ARM led
          led_set_value(LED_ARM, 0);

          printf("Pass to vehicle\n");
        }

        change_actuator_request = 0;
      }
      break;

    case SCU_ARM_REST:
      if(park_request_flag)
      {
        park_request_flag = 0;
        robotic_arm_selected = 0;

        // Unset ARM led
        led_set_value(LED_ARM, 0);

        printf("Pass to vehicle\n");
      }
      else
      {
        // Update LED
        if(arm_led == 0)
        {
          arm_led = 1;

          // Set ARM led
          led_set_value(LED_ARM, arm_led);
        }
        arm_command.arm_command_param.header_uint = ARM_RQST_STEADY;

        if(arm_load_tx(arm_command) != 1)
          printf("Cannot load command for arm\n");
      }
      break;

    case SCU_ARM_MOVE:
      // Update LED
      if(arm_led == 0)
      {
        arm_led = 1;

        // Set ARM led
        led_set_value(LED_ARM, arm_led);
      }

      if(arm_stop_flag)
      {
        arm_command.arm_command_param.header_uint = ARM_CMD_STOP;

        if(arm_load_tx(arm_command) != 1)
          printf("Cannot load command for arm\n");
      }
      else if(!jse->button[0] && !jse->button[1] && !jse->button[2] && move_botton_request)
      {
        move_botton_request = 0;

        arm_command.arm_command_param.header_uint = ARM_CMD_STOP;

        if(arm_load_tx(arm_command) != 1)
          printf("Cannot load command for arm\n");

        arm_stop_flag = 1;
      }
      else
      {
        if(jse->button[0])
        {
          arm_command.arm_command_param.header_uint = ARM_CMD_FIRST_TRIPLET;
          arm_command.arm_command_param.value1 = ((float) jse->stick_x / joy_max_value);
          arm_command.arm_command_param.value2 = ((float) jse->stick_y / joy_max_value);
          arm_command.arm_command_param.value3 = ((float) jse->stick_z / joy_max_value);
        }
        else if(jse->button[1])
        {
          arm_command.arm_command_param.header_uint = ARM_CMD_SECOND_TRIPLET;
          arm_command.arm_command_param.value1 = ((float) jse->stick_x / joy_max_value);
          arm_command.arm_command_param.value2 = ((float) jse->stick_y / joy_max_value);
          arm_command.arm_command_param.value3 = ((float) jse->stick_z / joy_max_value);
        }
        else if(jse->button[2])
        {
          arm_command.arm_command_param.header_uint = ARM_CMD_ACTUATOR;

          if(jse->stick_z > 5000)
            arm_command.arm_command_param.value1 = 1;
          else if(jse->stick_z < -5000)
            arm_command.arm_command_param.value1 = -1;
          else
            arm_command.arm_command_param.value1 = 0;
        }

        if(arm_load_tx(arm_command) != 1)
          printf("Cannot load command for arm\n");
      }
      break;

    case SCU_ARM_STOP:
      // Update LED
      if(arm_led == 0)
      {
        arm_led = 1;

        // Set ARM led
        led_set_value(LED_ARM, arm_led);
      }
      break;

    case SCU_ARM_AUTO_MOVE_ABORT:
      if(arm_abort_flag)
        arm_abort_flag = 0;

      // Update LED
      if(arm_led == 0)
      {
        arm_led = 1;

        // Set ARM led
        led_set_value(LED_ARM, arm_led);
      }
      break;

    case SCU_ARM_AUTO_MOVE:
      led_timer++;

      if(arm_abort_flag)
      {
        arm_command.arm_command_param.header_uint = ARM_RQST_ABORT;

        if(arm_load_tx(arm_command) != 1)
          printf("Cannot load command for arm\n");
      }
      else if(jse->button[0] || jse->button[1] || jse->button[2])
        move_botton_request = 1;

      if(!jse->button[0] && !jse->button[1] && !jse->button[2] && move_botton_request)
      {
        move_botton_request = 0;
        arm_command.arm_command_param.header_uint = ARM_RQST_ABORT;

        if(arm_load_tx(arm_command) != 1)
          printf("Cannot load command for arm\n");

        arm_abort_flag = 1;
      }

      if(led_timer * ARM_TIMEOUT_USEC >= 1000000) // 1 s
      {
        led_timer = 0;
        if(arm_led)
          arm_led = 0;
        else
          arm_led = 1;

        // Toggle ARM led
        led_set_value(LED_ARM, arm_led);
      }
      break;

    case SCU_ARM_OUT_OF_SERVICE:
      led_timer++;

      if(jse->button[3] && !jse->button[0] && !jse->button[1] && !jse->button[2] && !jse->button[4])
        change_actuator_request = 1;

      if((!jse->button[3]) && change_actuator_request)  //button released
      {
        change_actuator_request = 0;

        robotic_arm_selected = 0;

        // Unset ARM led
        led_set_value(LED_ARM, 0);

        printf("Pass to vehicle\n");
        break;
      }

      if(led_timer * ARM_TIMEOUT_USEC >= 250000) // 0.25 s
      {
        led_timer = 0;
        if(arm_led)
          arm_led = 0;
        else
          arm_led = 1;

        // Toggle ARM led
        led_set_value(LED_ARM, arm_led);
      }
      break;
  }
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

int led_export(int pin_number)
{
  FILE *file = NULL;

  file = fopen("/sys/class/gpio/export", "a");

  if(file == NULL)
    return -1;

  fprintf(file, "%i", pin_number);

  fclose(file);
  return 1;
}

int led_set_value(int pin_number, int value)
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

int led_set_direction(int pin_number, int value)
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
