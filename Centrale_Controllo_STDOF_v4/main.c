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

#include <libudev.h>

#include <locale.h>

#include <linux/can.h> 
#include <linux/can/raw.h> 
#include <string.h>

#include <linux/joystick.h>

#include <sys/mount.h>

#include <netinet/in.h> 
#include <arpa/inet.h>

#include "segway_config.h" 
#include "joystick.h" 
#include "segway_udp_v2.h" 
#include "socket_udp.h"
#include "arm_udp.h"

//#define SHOW_ARM_STATE

#define ACU_ADDRESS "192.168.1.28"
#define ACU_STATUS_PORT 8016
#define ACU_SEGWAY_PORT 8017

#define CCU_ADDRESS "192.168.1.102"
#define CCU_PORT_SEGWAY 8003
#define CCU_PORT_ARM 8000
#define SEGWAY_ADDRESS "192.168.1.40"
#define SEGWAY_PORT 55
#define ARM_ADDRESS "192.168.1.28"
#define ARM_PORT 8012
#define JOY_ARM_PORT 8013
#define JOY_STATUS_PORT 8014
#define JOY_LOCAL_NAME "/dev/input/js0"
#define JOY_MAX_VALUE 32767 
#define LED_READY 117
#define LED_RUN 48
#define LED_STDBY 49
#define LED_ARM 7
#define TIMEOUT_SEC 0 
#define TIMEOUT_USEC 100000
#define TIMEOUT_ARM_USEC 10000
#define TIMEOUT_USEC_STATUS 500000
#define LOG_FILE "/var/log/stdof_log.txt"

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

#define ARM_IDLE 0
#define ARM_MOVE 1
#define ARM_STOP 2
#define ARM_HOMING_REQUEST 3
#define ARM_AUTO_MOVE 4
#define ARM_AUTO_MOVE_ABORT 5
#define ARM_DINAMIC_REQUEST 6
#define ARM_REST 7
#define ARM_BOX_REQUEST 8
#define ARM_PARK_REQUEST 9
#define ARM_STEP_REQUEST 10
#define ARM_STEP 11

/* Macro */
#undef max 
#define max(x,y) ((x) > (y) ? (x) : (y))
#define min(x,y) ((x) < (y) ? (x) : (y))

/* Robotic Arm */
unsigned char robotic_arm_selected = 0;
unsigned char step_request = 0;
 
/* Automatic Control Unit */
unsigned char status_acu = ACU_UNKNOWN;
  
/* Prototype */
int monitor_input_devices(struct udev_monitor *, const char *, int *); 
void segway_status_update(union segway_union *, int, struct sockaddr_in *,  struct wwvi_js_event *, long int);
void arm_step_position(int socket_status, struct sockaddr_in *address, struct wwvi_js_event *jse, long int joy_max_value);
void arm_status_update(int socket, struct sockaddr_in *address, struct wwvi_js_event *jse, long int joy_max_value) ;
void message_log(const char *, const char *); 
int copy_log_file(void);

int main() 
{
  /* Led */
  char led_buffer[256];
  unsigned char led_ready_value = 0;

  /* Joystick interface */
  struct wwvi_js_event jse;
  int joy_local = -1;
  
  /* Udev */
  struct udev *udev;
  struct udev_monitor *udev_mon;
  int udev_fd = -1;

  /* Segway */
  int socket_segway = -1;
  struct sockaddr_in segway_address;
  union segway_union segway_status;
  unsigned char segway_check = 0;
  unsigned char segway_down = 1;

  struct timespec segway_timer_start, segway_timer_stop;
  long segway_elapsed_time = 0;
  unsigned char segway_timer = 0;

  /* CCU interface */
  int socket_ccu = -1;
  struct sockaddr_in socket_ccu_addr_dest;
  struct sockaddr_in socket_ccu_addr_src;
  union segway_union ccu_segway_status;
  __u8 ccu_buffer[(SEGWAY_PARAM*4) + 3];
  __u32 ccu_buffer_size = 0;
  int bytes_sent;
  
  /* Robotic Arm */
  int socket_arm = -1;
  struct sockaddr_in arm_address;
  struct arm_frame arm_buffer_temp;
  unsigned char arm_request_index;
  char *arm_token_result;

  /* Status client interface*/
  int socket_status = -1;
  struct sockaddr_in socket_status_addr_dest;
  struct sockaddr_in socket_status_addr_src;
  unsigned char status_buffer = -1;
  unsigned char acu_check = 0;
  unsigned char acu_down = 1;
  
  /* Segway info for Acu */
  int socket_segway_acu = -1;
  struct sockaddr_in socket_segway_acu_addr_dest;
  
  /* Generic Variable */
  int done = 0; // for the while in main loop
  int bytes_read; // to check how many bytes has been read

  int select_result = -1; // value returned frome select()
  int nfds = 0; // fd to pass to select()
  fd_set rd, wr, er; // structure for select()
  struct timeval select_timeout;
  long current_timeout;
  long timeout_status = 0;
  long timeout_segway = 0;
  
  //message_log("stdof", "Initializing stdof. . .");
  printf("Initializing stdof. . .\n");

  /* Peripheral initialization */

  /* Init Leds */
  // Init READY led
  sprintf(led_buffer, "echo %i > /sys/class/gpio/export", LED_READY);
  if(system(led_buffer) < 0)
    perror("export gpio");

  sprintf(led_buffer, "echo out > /sys/class/gpio/gpio%i/direction", LED_READY);
  if(system(led_buffer) < 0)
    perror("direction gpio");

  sprintf(led_buffer, "echo %i > /sys/class/gpio/export", LED_STDBY);
  if(system(led_buffer) < 0)
    perror("export gpio");

  sprintf(led_buffer, "echo out > /sys/class/gpio/gpio%i/direction", LED_STDBY);
  if(system(led_buffer) < 0)
    perror("direction gpio");

  sprintf(led_buffer, "echo %i > /sys/class/gpio/export", LED_RUN);
  if(system(led_buffer) < 0)
    perror("export gpio");

  sprintf(led_buffer, "echo out > /sys/class/gpio/gpio%i/direction", LED_RUN);
  if(system(led_buffer) < 0)
    perror("direction gpio");

  sprintf(led_buffer, "echo %i > /sys/class/gpio/export", LED_ARM);
  if(system(led_buffer) < 0)
    perror("export gpio");

  sprintf(led_buffer, "echo out > /sys/class/gpio/gpio%i/direction", LED_ARM);
  if(system(led_buffer) < 0)
    perror("direction gpio");

  /* Init Segway Client */  
  if(segway_open(&socket_segway, &segway_address, SEGWAY_ADDRESS, SEGWAY_PORT) == -1)
  {
    //message_log("init segway client", strerror(errno));
    perror("init segway client");
  }
  else
  {
    memset(&segway_status, 0, sizeof(segway_status));
    segway_status.list.operational_state = UNKNOWN;
  }

  /* Init CCU Client */
  socket_ccu = socket(AF_INET, SOCK_DGRAM, 0);

  if(socket_ccu < 0)
    perror("socket_ccu");
	  
  // Init source  
  bzero(&socket_ccu_addr_src, sizeof(socket_ccu_addr_src));
  socket_ccu_addr_src.sin_family = AF_INET;
  socket_ccu_addr_src.sin_port = htons(CCU_PORT_SEGWAY);
  socket_ccu_addr_src.sin_addr.s_addr = htonl(INADDR_ANY);

  if(bind(socket_ccu, (struct sockaddr *)&socket_ccu_addr_src, sizeof(socket_ccu_addr_src)) == -1)
    perror("socket_ccu");

  bzero(&socket_ccu_addr_dest, sizeof(socket_ccu_addr_dest));
  socket_ccu_addr_dest.sin_family = AF_INET;
  socket_ccu_addr_dest.sin_addr.s_addr = inet_addr(CCU_ADDRESS);
  socket_ccu_addr_dest.sin_port = htons(CCU_PORT_SEGWAY);
  
  
  /* Init Arm Client */  
  if(arm_open(&socket_arm, &arm_address, JOY_ARM_PORT, ARM_ADDRESS, ARM_PORT) == -1)
  {
    //message_log("init arm client", strerror(errno));
    perror("init arm client");
  }
  else
  {
    if(arm_init(0, 1000, 10, 32767, 2000, 1500, 100, 500, 500) < 0)
    {
      socket_arm = -1;
      //message_log("init arm", strerror(errno));
      perror("init arm");	  
    }
   
    //int arm_init(int index, long kp, long ki, long kd, long kv, long adt, long vt, long amps)
    arm_init(2, 20000, 10, 1500, 35000, 1500, 100, 500, 1023);
  }
  
  /* Init Joystick */
  memset(&jse, 0, sizeof(struct wwvi_js_event));
  joy_local = open_joystick(JOY_LOCAL_NAME);

  if(joy_local < 0)
  {
    //message_log("open_joystick(local)", strerror(errno));
    perror("open_joystick(local)");
  }
  else
  {
    //message_log("open_joystick", "Find Local Joystick\t[OK]");
    printf("Find Local Joystick\t[OK]\n");
  }
  
  /* Init Monitor for joystick or usb storage device */
  udev = udev_new();

  if(!udev)
  {
    //message_log("udev_new", "Init udev\t[FAIL]");
    printf("Can't create udev\n");
  }
  else
  {
    udev_mon = udev_monitor_new_from_netlink(udev, "udev");
    udev_monitor_filter_add_match_subsystem_devtype(udev_mon, "input", NULL); // for joystick
    udev_monitor_filter_add_match_subsystem_devtype(udev_mon, "block", "partition"); // for storage device
    udev_monitor_enable_receiving(udev_mon);
    udev_fd = udev_monitor_get_fd(udev_mon);

    //message_log("udev_new", "Init udev\t[OK]");
    printf("Init udev\t[OK]\n");
  }

  /* Status client interface */
  socket_status = socket(AF_INET, SOCK_DGRAM, 0);

  if(socket_status < 0)
    perror("socket_status");
 
  bzero(&socket_status_addr_src, sizeof(socket_status_addr_src));
  socket_status_addr_src.sin_family = AF_INET;
  socket_status_addr_src.sin_port = htons(JOY_STATUS_PORT);
  socket_status_addr_src.sin_addr.s_addr = htonl(INADDR_ANY);

  if(bind(socket_status, (struct sockaddr *)&socket_status_addr_src, sizeof(socket_status_addr_src)) == -1)
    perror("socket_status");

  bzero(&socket_status_addr_dest, sizeof(socket_status_addr_dest));
  socket_status_addr_dest.sin_family = AF_INET;
  socket_status_addr_dest.sin_addr.s_addr = inet_addr(ACU_ADDRESS);
  socket_status_addr_dest.sin_port = htons(ACU_STATUS_PORT);
  
  /* Segway info for ACU interface */
  socket_segway_acu = socket(AF_INET, SOCK_DGRAM, 0);

  if(socket_segway_acu < 0)
    perror("socket_segway_acu");
 
  /*bzero(&socket_segway_acu_addr_src, sizeof(socket_segway_acu_addr_src));
  socket_segway_acu_addr_src.sin_family = AF_INET;
  socket_segway_acu_addr_src.sin_port = htons(ACU_SEGWAY_PORT);
  socket_segway_acu_addr_src.sin_addr.s_addr = htonl(INADDR_ANY);

  if(bind(socket_segway_acu, (struct sockaddr *)&socket_segway_acu_addr_src, sizeof(socket_segway_acu_addr_src)) == -1)
    perror("bind socket_segway_acu");*/

  bzero(&socket_segway_acu_addr_dest, sizeof(socket_segway_acu_addr_dest));
  socket_segway_acu_addr_dest.sin_family = AF_INET;
  socket_segway_acu_addr_dest.sin_addr.s_addr = inet_addr(ACU_ADDRESS);
  socket_segway_acu_addr_dest.sin_port = htons(ACU_SEGWAY_PORT);
   
  // The segway requirements state that the minimum update frequency have to be 0.5Hz, so
  // the timeout on udev have to be < 2 secs.
  if(robotic_arm_selected == 0)
  {
    select_timeout.tv_sec = TIMEOUT_SEC;
    select_timeout.tv_usec = TIMEOUT_USEC;
  }
  else
  {
    select_timeout.tv_sec = TIMEOUT_SEC;
    select_timeout.tv_usec = TIMEOUT_ARM_USEC;
  }
  
  current_timeout = select_timeout.tv_usec;
      
  //message_log("stdof", "Run main program. . .");
  printf("Run main program. . .\n");
  
  while(!done)
  { 
    fflush(stdout);

    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);

    if(socket_status > 0)
    {
      FD_SET(socket_status, &rd);
      nfds = max(nfds, socket_status);
    }

    if(udev_fd > 0)
    {
      FD_SET(udev_fd, &rd);
      nfds = max(nfds, udev_fd);
    }

    switch(status_acu)
    {
      case ACU_HOME:
      case ACU_IDLE:
        // can perform all task
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

        if((socket_arm > 0) && (robotic_arm_selected))
        {
          FD_SET(socket_arm, &rd);
          nfds = max(nfds, socket_arm);  
	  
          if(arm_buffer_tx_empty == 0)
          {
            FD_SET(socket_arm, &wr);
            nfds = max(nfds, socket_arm);  
          }
        }
    
        if(joy_local > 0)
        {
          FD_SET(joy_local, &rd);
          nfds = max(nfds, joy_local);
        }
        break;
	      
      case ACU_ARM_AUTO_MOVE:
        // wait end
        //send abort message by selecting an arm link
        if(joy_local > 0)
        {
          FD_SET(joy_local, &rd);
          nfds = max(nfds, joy_local);
        }
        break;
	      
      case ACU_RETURN_TO_BASE:
        // read acu status and send its one. Wait for the idle state
        break;
	    
      default:
        // status unknown. Waiting for status info from ACU
        break;
    }
    
    if(segway_timer == 0)
    {
      clock_gettime(CLOCK_REALTIME, &segway_timer_start);
      segway_timer++;
    }
          
    select_result = select(nfds + 1, &rd, &wr, NULL, &select_timeout);

    if(select_result == -1 && errno == EAGAIN)
    {
      perror("select");
      continue;
    }

    if(select_result == -1)
    {
      //message_log("stdof", strerror(errno));
      perror("main:");

      return 1;
    }

    /* Monitor input devices */
    if(udev_fd > 0)
    {
      if(FD_ISSET(udev_fd, &rd))
      {
        monitor_input_devices(udev_mon, JOY_LOCAL_NAME, &joy_local);
        continue;
      }
    }

    /* Manage ACU status */
    if(socket_status > 0)
    {
      if(FD_ISSET(socket_status, &rd))
      {
        bytes_read = recvfrom(socket_status, &status_buffer, sizeof(status_buffer), 0, NULL, NULL);

        if(bytes_read > 0)
        {
          status_acu = status_buffer;

          acu_check = 0;

          if(acu_down == 1)
          {
            acu_down = 0;

            if(led_ready_value == 0)
            {
              led_ready_value = 1;
              sprintf(led_buffer, "echo 1 > /sys/class/gpio/gpio%i/value", LED_READY);
      
              if(system(led_buffer) < 0)
                perror("Set Ready led");
            }
          }
        }
        else
        {
          //message_log("status_read", strerror(errno));
          perror("status_read");
        }

        continue;
      }
    }
    
    /* Manage segway message */
    if(socket_segway > 0)
    {
      if(FD_ISSET(socket_segway, &rd))
      {
        bytes_read = segway_read(socket_segway, &segway_status, NULL);

        if(bytes_read <= 0)
        {
          //message_log("segway_read", strerror(errno));
          perror("segway_read");
        }
        else
        {
          segway_check = 0;
         
          if(segway_down == 1)
          {
            //message_log("segway_init", "Segway Init\t[OK]\n");
            printf("Segway Init\t[OK]\n");
        
            //segway_configure_audio_command(socket_segway, &segway_address, 9);
            segway_down = 0;
          }

          //printf("Max Vel %f               \n", convert_to_float(segway_status.list.vel_limit_mps));
          //printf("Max Turn Rate %f         \n", convert_to_float(segway_status.list.yaw_rate_limit_rps));
          //printf("Operational State: %i\n", segway_status.list.operational_state);
          //printf("Linear Velocity: %f             \n", (float)convert_to_float(segway_status.list.linear_vel_mps));
          //printf("Yaw Rate. %f                    \n", convert_to_float(segway_status.list.inertial_z_rate_rps));
          //printf("\033[2A");
          // Send info to ccu
          
          memcpy(&ccu_segway_status, &segway_status, sizeof(segway_status));

          if((jse.stick_y < 1000) && (jse.stick_y > -1000))
            ccu_segway_status.list.linear_vel_mps = 0;

          if((jse.stick_x < 1000) && (jse.stick_x > -1000))
            ccu_segway_status.list.inertial_z_rate_rps = 0;

          segway_convert_param_message(ccu_segway_status, ccu_buffer, &ccu_buffer_size);

          if(socket_ccu_addr_dest.sin_port != htons(CCU_PORT_SEGWAY))
            socket_ccu_addr_dest.sin_port = htons(CCU_PORT_SEGWAY);

          bytes_sent = sendto(socket_ccu, ccu_buffer, ccu_buffer_size, 0, (struct sockaddr *)&socket_ccu_addr_dest, sizeof(socket_ccu_addr_dest));
  
          if(bytes_sent < 0)
            perror("sendto ccu");

          bytes_sent = sendto(socket_segway_acu, ccu_buffer, ccu_buffer_size, 0, (struct sockaddr *)&socket_segway_acu_addr_dest, sizeof(socket_segway_acu_addr_dest));

          if(bytes_sent < 0)
            perror("sendto acu");
        }
        continue;
      }
      
      if(FD_ISSET(socket_segway, &wr))
      {
        if(segway_timer > 0)
        {
          clock_gettime(CLOCK_REALTIME, &segway_timer_stop);
          //debug_timer = 0;
      
          segway_elapsed_time = (segway_timer_stop.tv_sec * 1000000000 + segway_timer_stop.tv_nsec) - (segway_timer_start.tv_sec * 1000000000 + segway_timer_start.tv_nsec);
        }

        if(segway_elapsed_time > 10000000) // 10ms
        {
          if(segway_send(socket_segway, &segway_address) < 0)
            perror("segway_send");

          if(segway_timer > 0)
            segway_timer = 0;
        }

        continue;
      }
    }

    /* Manage arm message */
    if(socket_arm > 0)
    {
      if(FD_ISSET(socket_arm, &rd))
      {
        // the message would be an information such position or warning
        bytes_read = recvfrom(socket_arm, &arm_buffer_temp, sizeof(struct arm_frame), 0, NULL, NULL);

		//printf("Receive message from arm\n");
        if(bytes_read <= 0)
        {
          //message_log("arm_read", strerror(errno));
          perror("arm_read");
        }
        else
        {
          // Every message from arm must ends with \r
          arm_token_result = strchr(arm_buffer_temp.param.arm_command, 13);

          if(arm_token_result != NULL)
          {
            //printf("Received message from %i\n", query_link);
            *arm_token_result = '\0';  // translate token in null character

            if(query_link > -1)
            {
              arm_request_index = query_link;
   
              if(arm_link[arm_request_index - 1].request_actual_position == 1)
              {
                query_link = -1;
                arm_link[arm_request_index - 1].request_timeout = 0;
                arm_link[arm_request_index - 1].request_actual_position = 0;
                arm_link[arm_request_index - 1].actual_position = atol(arm_buffer_temp.param.arm_command);
 
                if(socket_ccu_addr_dest.sin_port != htons(CCU_PORT_ARM))
                  socket_ccu_addr_dest.sin_port = htons(CCU_PORT_ARM);
  
                bytes_read = sprintf((char *)ccu_buffer, "%i%ld ", (arm_request_index - 1), arm_link[arm_request_index - 1].actual_position);
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
              }
            }
          }
        }

        continue;
      }

      if(FD_ISSET(socket_arm, &wr))
      {
        if(arm_send(socket_arm, &arm_address) < 0)
        {
          if(led_ready_value == 1)
          {
            led_ready_value = 0;
            sprintf(led_buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_READY);
       
            if(system(led_buffer) < 0)
              perror("Set Ready led");
          }
        }
        else
        {
          if(led_ready_value == 0)
          {
            led_ready_value = 1;
            sprintf(led_buffer, "echo 1 > /sys/class/gpio/gpio%i/value", LED_READY);
      
            if(system(led_buffer) < 0)
              perror("Set Ready led");
          }
        }

        continue;
      }
    }
 
    if(joy_local > 0)
    {
      if(FD_ISSET(joy_local, &rd))
      {
        bytes_read = get_joystick_status(&joy_local, &jse);

        if(bytes_read <= 0)
        {
          //message_log("joystick", strerror(errno));
          perror("get_joystick_status");
        }  // end if(bytes_read <= 0)
        //else
        //{
          // update at the end of the while
        //}
        
        continue;
      }//end if(joy_selected == null)
    }
    
    // searching for the segway and send joystick command to it
  
      if(robotic_arm_selected == 0)
      {
        // Try to communicate with segway. If it is on tractor mode then I send the joystick
        // command 

        if((segway_status.list.operational_state < 3) || (segway_status.list.operational_state > 5))
        {
          //printf("Segway Init. . .\n");
		  // I have to check if ACU are not using the segway before send init command
          if(status_acu != ACU_UNKNOWN)
            segway_init(socket_segway, &segway_address, &segway_status);
        }
        else
          segway_configure_none(socket_segway, &segway_address, 0x00);

        segway_check++;

        if(segway_check > 9)
        {
          segway_check = 10;
        
          // Send warning only the first time
          if(segway_down == 0)
          {
            //message_log("stdof", "Segway down!");
            printf("Segway down!\n");
            segway_status.list.operational_state = UNKNOWN;
          }

		  // send new segway status
          memcpy(&ccu_segway_status, &segway_status, sizeof(segway_status));

          if((jse.stick_y < 1000) && (jse.stick_y > -1000))
            ccu_segway_status.list.linear_vel_mps = 0;

          if((jse.stick_x < 1000) && (jse.stick_x > -1000))
            ccu_segway_status.list.inertial_z_rate_rps = 0;

          segway_convert_param_message(ccu_segway_status, ccu_buffer, &ccu_buffer_size);

          if(socket_ccu_addr_dest.sin_port != htons(CCU_PORT_SEGWAY))
            socket_ccu_addr_dest.sin_port = htons(CCU_PORT_SEGWAY);

          bytes_sent = sendto(socket_ccu, ccu_buffer, ccu_buffer_size, 0, (struct sockaddr *)&socket_ccu_addr_dest, sizeof(socket_ccu_addr_dest));
  
          if(bytes_sent < 0)
            perror("sendto ccu");

          bytes_sent = sendto(socket_segway_acu, ccu_buffer, ccu_buffer_size, 0, (struct sockaddr *)&socket_segway_acu_addr_dest, sizeof(socket_segway_acu_addr_dest));

          if(bytes_sent < 0)
            perror("sendto acu");
			
          segway_down = 1;
        }

        segway_status_update(&segway_status, socket_segway, &segway_address, &jse, JOY_MAX_VALUE);
        //printf("Linear Velocity: %f\n", convert_to_float(segway_status.list.linear_vel_mps));
        //printf("Yaw Rate. %f\n", convert_to_float(segway_status.list.inertial_z_rate_rps));
        //printf("\033[2A");
        //printf("X: %3d   \tY: %3d   \tZ: %3d   \nbutton1: %3d   \tbutton2: %3d   \nbutton3: %3d   \tbutton4: %3d   \nbutton5: %3d   \n", 
        //        jse.stick_x, jse.stick_y, jse.stick_z, jse.button[0], jse.button[1], jse.button[2], jse.button[3], jse.button[4]);
        //printf("Y: %d\n", jse.stick_y);
        /*printf("Front Batt1 SOC: %f\n", convert_to_float(segway_status.list.front_base_batt_1_soc));
        printf("Front Batt2 SOC: %f\n", convert_to_float(segway_status.list.front_base_batt_2_soc));
        printf("Rear Batt1 SOC: %f\n", convert_to_float(segway_status.list.rear_base_batt_1_soc));
        printf("Rear Batt2 SOC: %f\n", convert_to_float(segway_status.list.rear_base_batt_2_soc));
        printf("Front Batt1 Temp: %f\n", convert_to_float(segway_status.list.front_base_batt_1_temp_degC));
        printf("Front Batt2 Temp: %f\n", convert_to_float(segway_status.list.front_base_batt_2_temp_degC));
        printf("Rear Batt1 Temp: %f\n", convert_to_float(segway_status.list.rear_base_batt_1_temp_degC));
        printf("Rear Batt2 Temp: %f\n", convert_to_float(segway_status.list.rear_base_batt_2_temp_degC));*/
        //printf("Config input bitmap: %ld\n", segway_status.list.fram_config_bitmap);
        //printf("Config input bitmap: %ld\n", segway_status.list.fram_config_bitmap);

        //printf("\033[2A");
        //printf("\033[8A");
      }
      else
      {
        if(step_request)
          arm_step_position(socket_status, &socket_status_addr_dest, &jse, JOY_MAX_VALUE);
        else
          arm_status_update(socket_status, &socket_status_addr_dest, &jse, JOY_MAX_VALUE);

        // Send segway state to acu
        timeout_segway++;
        if((timeout_segway * current_timeout) >= TIMEOUT_USEC)
        {
          timeout_segway = 0;

          ccu_segway_status.list.linear_vel_mps = 0;
          ccu_segway_status.list.inertial_z_rate_rps = 0;

          segway_convert_param_message(ccu_segway_status, ccu_buffer, &ccu_buffer_size);

          if(socket_ccu_addr_dest.sin_port != htons(CCU_PORT_SEGWAY))
            socket_ccu_addr_dest.sin_port = htons(CCU_PORT_SEGWAY);

          bytes_sent = sendto(socket_ccu, ccu_buffer, ccu_buffer_size, 0, (struct sockaddr *)&socket_ccu_addr_dest, sizeof(socket_ccu_addr_dest));
  
          if(bytes_sent < 0)
            perror("sendto ccu");

          bytes_sent = sendto(socket_segway_acu, ccu_buffer, ccu_buffer_size, 0, (struct sockaddr *)&socket_segway_acu_addr_dest, sizeof(socket_segway_acu_addr_dest));

          if(bytes_sent < 0)
            perror("sendto acu");
        }
      }
    
    timeout_status++;
    if((timeout_status * current_timeout) >= TIMEOUT_USEC_STATUS)
    {
      timeout_status = 0;
      //send data to acc
      //printf("Timeout on %ld of %ld\n", (timeout_status * current_timeout), TIMEOUT_USEC_STATUS);
      status_buffer = JOYSTICK_NONE;
      
      if(socket_status > 0)
      {
        bytes_sent = sendto(socket_status, &status_buffer, sizeof(status_buffer), 0, (struct sockaddr *)&socket_status_addr_dest, sizeof(socket_status_addr_dest));
      
        if(bytes_sent < 0)
          perror("sendto ACU on timeout");

      }
      
      acu_check++;
      if(acu_check >= 4)
      {
        acu_check = 5;
        acu_down = 1;
          
        // Unset led READY
        if(led_ready_value == 1)
        {
          led_ready_value = 0;
          sprintf(led_buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_READY);
      
          if(system(led_buffer) < 0)
            perror("Set Ready led");
        }
      }
    }
    
    // The segway requirements state that the minimum update frequency have to be 0.5Hz, so
    // the timeout on udev have to be < 2 secs.
    if(robotic_arm_selected == 0)
    {
      select_timeout.tv_sec = TIMEOUT_SEC;
      select_timeout.tv_usec = TIMEOUT_USEC;
    }
    else
    {
      select_timeout.tv_sec = TIMEOUT_SEC;
      select_timeout.tv_usec = TIMEOUT_ARM_USEC;
    }

    current_timeout = select_timeout.tv_usec;
  }  // end while(!= done)

  return 0;
}

int monitor_input_devices(struct udev_monitor *udev_mon,
                          const char *joystick_path, int *joy_local) 
{
  struct udev_device *udev_dev;
  const char *udev_node;
  const char *udev_subsystem;

  udev_dev = udev_monitor_receive_device(udev_mon);

  if(udev_dev)
  {
    udev_node = udev_device_get_devnode(udev_dev);
    udev_subsystem = udev_device_get_subsystem(udev_dev);

    if(udev_node && !strcmp(udev_subsystem, "input"))
    {
      if(!strcmp(udev_node, joystick_path))
      {
        if((!strcmp(udev_device_get_action(udev_dev),"add")) && (*joy_local < 0))
        {
          // add joystick
          *joy_local = open_joystick(joystick_path);

          if(*joy_local < 0)
            perror("open_joystick");

          //message_log("stdof", "Joystick added");
          printf("stdof: Joystick added\n");
        }
        else if((!strcmp(udev_device_get_action(udev_dev),"remove")) && (*joy_local > 0))
        {
          // remove joystick
          *joy_local = -1;

          //message_log("stdof", "Joystick removed");
          printf("stdof: Joystick removed\n");
        }
      }
    }  // end if(udev_node && !strcmp(udev_subsystem, "input"))
    else if(udev_node && !strcmp(udev_subsystem, "block"))
    {
      if(!strcmp(udev_device_get_action(udev_dev),"add"))
      {
        //message_log("stdof", "Find storage device");
        printf("udev: Find storage device %s\n", udev_node);

        // mount usb device
        if(mount(udev_node, "/media/usb", "vfat", MS_NOATIME, NULL))
        {
          //message_log("mount", strerror(errno));
          perror("mount");
          return -1;
        }
        else
        {
          //message_log("mount", "Drive mounted");
          printf("mount: Drive mounted\n");
        }

        if(copy_log_file() < 0)
        {
          //message_log("copy_log_file", strerror(errno));
          perror("copy_log_file");
        }
        else
        {
          //message_log("copy_log_file", "Log copied to usb device");
          printf("copy_log_file: Log copied to usb device\n");
        }

        // unmount usb device
        if(umount("/media/usb") < 0)
        {
          //message_log("umount", strerror(errno));
          perror("umount");
        }
        else
        {
          //message_log("umount", "Drive unmounted");
          printf("mount: Drive unmounted\n");
        }
      }
      else if(!strcmp(udev_device_get_action(udev_dev),"remove"))
      {
        //message_log("stdof", "Storage device removed");
        printf("udev: Storage device %s removed\n", udev_node);
      }
    }
    udev_device_unref(udev_dev);
  }

  return 0;
}

void segway_status_update(union segway_union *segway_status, int socket, struct sockaddr_in *segway_address, struct wwvi_js_event *jse, long int joy_max_value) 
{
  static int segway_previouse_state = 0;
  static unsigned char change_actuator_request = 0;
  static unsigned char change_mode_request = 0;
 
  char buffer[256];
  int bytes_sent = -1;

  switch(segway_status->list.operational_state)
  {
    case CCU_INIT:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        //message_log("stdof", "Segway CCU Init");
        printf("Segway CCU Init\n");

        // Unset RUN led
        sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_RUN);
        if(system(buffer) < 0)
          perror("Unset Standby led");
  
        // Set STDBY led
        sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_STDBY);
        if(system(buffer) < 0)
          perror("Set Standby led");

        segway_previouse_state = segway_status->list.operational_state;
      }

      // Change actuator
      if(jse->button[3])
      {
        change_actuator_request = 1;
      }
      if((!jse->button[3]) && change_actuator_request)
      {
        change_actuator_request = 0;

        if(arm_start() > 0)
          robotic_arm_selected = 1;

        break;
      }

      break;

    case PROPULSION_INIT:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        //message_log("stdof", "Segway Propulsion Init");
        printf("Segway Propulsion Init\n");

        // Unset RUN led
        sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_RUN);
        if(system(buffer) < 0)
          perror("Unset Standby led");
  
        // Set STDBY led
        sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_STDBY);
        if(system(buffer) < 0)
          perror("Set Standby led");

        segway_previouse_state = segway_status->list.operational_state;
      }

      // Change actuator
      if(jse->button[3])
        change_actuator_request = 1;

      if((!jse->button[3]) && change_actuator_request)
      {
        change_actuator_request = 0;

        if(arm_start() > 0)
          robotic_arm_selected = 1;

        break;
      }

      break;

    case CHECK_STARTUP:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        //message_log("stdof", "Segway Check Startup Issue");
        printf("Segway Check Startup Issue\n");

        // Unset RUN led
        sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_RUN);
        if(system(buffer) < 0)
          perror("Unset Standby led");
  
        // Set STDBY led
        sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_STDBY);
        if(system(buffer) < 0)
          perror("Set Standby led");

        segway_previouse_state = segway_status->list.operational_state;
      }

      // Change actuator
      if(jse->button[3])
        change_actuator_request = 1;

      if((!jse->button[3]) && change_actuator_request)
      {
        change_actuator_request = 0;

        if(arm_start() > 0)
          robotic_arm_selected = 1;

        break;
      }

      break;

    case SEGWAY_STANDBY: //standby 
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        //message_log("stdof", "Segway in Standby Mode");
        printf("Segway in Standby Mode\n");

        // Unset RUN led
        sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_RUN);
        if(system(buffer) < 0)
          perror("Unset Standby led");
  
        // Set STDBY led
        sprintf(buffer, "echo 1 > /sys/class/gpio/gpio%i/value", LED_STDBY);
        if(system(buffer) < 0)
          perror("Set Standby led");
  
        segway_previouse_state = segway_status->list.operational_state;
      }

      // Change actuator
      if(jse->button[3])
        change_actuator_request = 1;

      if((!jse->button[3]) && change_actuator_request)
      {
        change_actuator_request = 0;

        if(arm_start() > 0)
        {
          robotic_arm_selected = 1;

          // Set ARM led
          sprintf(buffer, "echo 1 > /sys/class/gpio/gpio%i/value", LED_ARM);
          if(system(buffer) < 0)
            perror("Set Arm led");

          //message_log("stdof", "Pass to robotic arm");
          printf("Pass to robotic arm\n");
        }
        break;
      }

      if(jse->button[0])
        change_mode_request = 1;

      if((!jse->button[0]) && change_mode_request)
      {
        change_mode_request = 0;

        bytes_sent = segway_configure_operational_mode(socket, segway_address, SEGWAY_TRACTOR_REQ);
      
        if(bytes_sent == -1)
        {
          //message_log("segway_configure_operational_mode to tractor", strerror(errno));
          perror("segway_configure_operational_mode");
        }

        break;
      }
      
      break;

    case SEGWAY_TRACTOR:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        //message_log("stdof", "Segway in Tractor Mode");
        printf("Segway in Tractor Mode\n");

        // Unset STDBY led
        sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_STDBY);
        if(system(buffer) < 0)
          perror("Unset Standby led");
  
        // Set RUN led
        sprintf(buffer, "echo 1 > /sys/class/gpio/gpio%i/value", LED_RUN);
        if(system(buffer) < 0)
          perror("Set Run led");

        segway_previouse_state = segway_status->list.operational_state;
      }

      if(jse->button[1])
        change_mode_request = 1;
      
      if((!jse->button[1]) && (step_request == 0) && change_mode_request)
      {
        change_mode_request = 0;
	
        bytes_sent = segway_configure_operational_mode(socket, segway_address, SEGWAY_STANDBY_REQ);
	
        if(bytes_sent == -1)
        {
          //message_log("segway_configure_operational_mode to standby", strerror(errno));
          perror("segway_configure_operational_mode");
        }
        break;
      } 
      
      // Change actuator and move it into step position
      if(jse->button[2])
        change_actuator_request = 1;

      if((!jse->button[2]) && change_actuator_request)
      {
        change_actuator_request = 0;

        if(arm_start() > 0)
        {
          robotic_arm_selected = 1;
          step_request = 1;
        }

        break;
      }
	  
      if(jse->button[4])
        bytes_sent = segway_motion_set(socket, segway_address, jse->stick_y, jse->stick_x, joy_max_value);
      else
        bytes_sent = segway_motion_set(socket, segway_address, 0, 0, joy_max_value);
     
      if(bytes_sent < 0)
      {
        //message_log("segway_motion_set", strerror(errno));
        perror("segway_motion_set");
      }
      
      break;

    case DISABLE_POWER:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        //message_log("stdof", "Segway Disable Power");
        printf("Segway Disable Power\n");

        // Unset RUN led
        sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_RUN);
        if(system(buffer) < 0)
          perror("Unset Standby led");
  
        // Set STDBY led
        sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_STDBY);
        if(system(buffer) < 0)
          perror("Set Standby led");

        segway_previouse_state = segway_status->list.operational_state;
      }

      // Change actuator
      if(jse->button[3])
        change_actuator_request = 1;

      if((!jse->button[3]) && change_actuator_request)
      {
        change_actuator_request = 0;

        if(arm_start() > 0)
         robotic_arm_selected = 1;

        break;
      }

      break;

    default:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        //message_log("stdof", "Segway Uknown State");
        printf("Segway Unknown State\n");

        // Unset RUN led
        sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_RUN);
        if(system(buffer) < 0)
          perror("Unset Standby led");
  
        // Set STDBY led
        sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_STDBY);
        if(system(buffer) < 0)
          perror("Set Standby led");

        segway_previouse_state = segway_status->list.operational_state;
      }
      

      // Change actuator
      if(jse->button[3])
        change_actuator_request = 1;

      if((!jse->button[3]) && change_actuator_request)
      {
        change_actuator_request = 0;

        if(arm_start() > 0)
          robotic_arm_selected = 1;
        break;
      }

      break;
  } //end switch
}

void arm_step_position(int socket_status, struct sockaddr_in *address, struct wwvi_js_event *jse, long int joy_max_value)
{
  char buffer[256];
  char status_buffer;
  int bytes_sent = -1;
  static unsigned char arm_state = ARM_STEP_REQUEST;  // arm's state
  static unsigned char homing_requested = 0;
  static unsigned char move_botton_request = 0;
  static unsigned char arm_led = 0;
  static unsigned char led_timer = 0;
  
  if(jse->button[0] || jse->button[1] || jse->button[2])
    move_botton_request = 1;
 
  if(!jse->button[0] && !jse->button[1] && !jse->button[2] && move_botton_request)
  {
    move_botton_request = 0;
    
    if(arm_state == ARM_AUTO_MOVE)
    {
#ifdef SHOW_ARM_STATE
      printf("ARM_AUTO_MOVE_ABORT\n");
#endif
      arm_state = ARM_AUTO_MOVE_ABORT;
    }
  }
  
  switch(arm_state)
  {
    case ARM_IDLE:
      break;
   
    case ARM_STEP:
#ifdef SHOW_ARM_STATE
	  printf("ARM_HOMING_REQUEST\n");
#endif
      arm_state = ARM_HOMING_REQUEST;
      break;

    case ARM_HOMING_REQUEST:
      if(status_acu != ACU_ARM_HOMING)
      {
        // Send homing message to ACU
        status_buffer = JOYSTICK_REQUEST_HOMING;
      
        if(socket_status > 0)
        {
          bytes_sent = sendto(socket_status, &status_buffer, sizeof(status_buffer), 0, (struct sockaddr *)address, sizeof(*address));
      
          if(bytes_sent < 0)
            perror("sendto ACU on HOMING_REQUEST");
          else
            homing_requested = 1;
        }
      }
      else
      {
#ifdef SHOW_ARM_STATE
        printf("ARM_AUTO_MOVE\n");
#endif
        arm_state = ARM_AUTO_MOVE;
      }
      break;

    case ARM_STEP_REQUEST:
      if(status_acu != ACU_ARM_AUTO_MOVE)
      {
        // Send homing message to ACU
        status_buffer = JOYSTICK_REQUEST_STEP;
      
        if(socket_status > 0)
        {
          bytes_sent = sendto(socket_status, &status_buffer, sizeof(status_buffer), 0, (struct sockaddr *)address, sizeof(*address));
      
          if(bytes_sent < 0)
            perror("sendto ACU on STEP_REQUEST");
          else
            step_request = 1;
        }
      }
      else
      {
#ifdef SHOW_ARM_STATE
        printf("ARM_AUTO_MOVE\n");
#endif
        arm_state = ARM_AUTO_MOVE;
      }
      break;

    case ARM_AUTO_MOVE_ABORT:
      if(status_acu == ACU_ARM_AUTO_MOVE)
      {
        // Send homing abort message to ACU_ADDRESS
        status_buffer = JOYSTICK_ABORT_AUTO_MOVE;
       
        if(socket_status > 0)
        {
          bytes_sent = sendto(socket_status, &status_buffer, sizeof(status_buffer), 0, (struct sockaddr *)address, sizeof(*address));
      
          if(bytes_sent < 0)
            perror("sendto ACU on HOMING ABORT");
        }
        break;
      }
      // don't put break here
    case ARM_AUTO_MOVE:
      led_timer++;
  
      if((status_acu != ACU_ARM_AUTO_MOVE) && (status_acu != ACU_ARM_HOMING))
      {
        led_timer = 0;

        if(homing_requested == 1)
        {
#ifdef SHOW_ARM_STATE
          printf("ARM_STEP_REQUEST\n");
#endif
          // Go into step_request modo cause I don't want homing
		  // when change from an actuator to another
          arm_state = ARM_STEP_REQUEST;
          homing_requested = 0;
          robotic_arm_selected = 0;
		  step_request = 0;

          // Unset ARM led
          sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_ARM);
          if(system(buffer) < 0)
            perror("Set Arm led");

          //message_log("stdof", "Pass to vehicle");
          printf("Pass to vehicle\n");
        }
        else if(step_request == 1)
        {
#ifdef SHOW_ARM_STATE
          printf("ARM_STEP\n");
#endif
          arm_state = ARM_STEP;
          robotic_arm_selected = 0;
  
          // Unset ARM led
          sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_ARM);
          if(system(buffer) < 0)
            perror("Set Arm led");

          //message_log("stdof", "Pass to vehicle with arm in step position");
          printf("Pass to vehicle with arm in step position\n");
        }
	  }
	  else
      {
        if(led_timer >= 100) // for 10ms timeout led_timer = 1 s
        {
          led_timer = 0;
          if(arm_led)
            arm_led = 0;
          else
            arm_led = 1;

          // Toggle ARM led
          sprintf(buffer, "echo %i > /sys/class/gpio/gpio%i/value", arm_led, LED_ARM);
          if(system(buffer) < 0)
            perror("Set Arm led");
         }
      }
      break;	  
  }
}

void arm_status_update(int socket_status, struct sockaddr_in *address, struct wwvi_js_event *jse, long int joy_max_value) 
{
  char buffer[256];
  char status_buffer;
  int bytes_sent = -1;
  static unsigned char arm_state = ARM_REST;  // arm's state
  static unsigned char change_actuator_request = 0; // indicates when button has been released
  static unsigned char move_botton_request = 0;
  static unsigned char set_origin_button_request = 0;
  static unsigned char homing_requested = 0;
  static unsigned char arm_led = 0;
  static unsigned char led_timer = 0;
  
  // Change actuator if press only the button 4
  if(jse->button[3] && !jse->button[0] && !jse->button[1] && !jse->button[2] && (arm_state == ARM_IDLE))
    change_actuator_request = 1;

  if((!jse->button[3]) && change_actuator_request)  //button released
  {
    if((jse->stick_y > (JOY_MAX_VALUE - 5000)) && (jse->stick_x < (JOY_MAX_VALUE - 5000)))
    {
      arm_state = ARM_BOX_REQUEST;
#ifdef SHOW_ARM_STATE
      printf("ARM_BOX_REQUEST\n");
#endif
    }
    else if((jse->stick_y < (-JOY_MAX_VALUE + 5000)) && (jse->stick_x < (JOY_MAX_VALUE - 5000)))
    {
      arm_state = ARM_PARK_REQUEST;
#ifdef SHOW_ARM_STATE
      printf("ARM_PARK_REQUEST\n");
#endif
    }
    else if(jse->stick_x > (JOY_MAX_VALUE - 5000))
    {
      arm_state = ARM_HOMING_REQUEST;
#ifdef SHOW_ARM_STATE
      printf("ARM_HOMING_REQUEST\n");
#endif
    }
    else
    {
      arm_state = ARM_IDLE;
#ifdef SHOW_ARM_STATE
      printf("ARM_IDLE\n");
#endif
      robotic_arm_selected = 0;
    
      // Unset ARM led
      sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_ARM);
      if(system(buffer) < 0)
        perror("Set Arm led");

      //message_log("stdof", "Pass to vehicle");
      printf("Pass to vehicle\n");
    }
    
    change_actuator_request = 0;
  }
  
  if((jse->button[0] || jse->button[1] || jse->button[2]) && !change_actuator_request)
  {
    move_botton_request = 1;
    
    if(arm_state == ARM_IDLE)
    {
#ifdef SHOW_ARM_STATE
      printf("ARM_MOVE\n");
#endif
      arm_start();
      arm_state = ARM_MOVE;
    }
  }
 
  if(!jse->button[0] && !jse->button[1] && !jse->button[2] && move_botton_request)
  {
    move_botton_request = 0;
    
    if(arm_state == ARM_AUTO_MOVE)
    {
#ifdef SHOW_ARM_STATE
      printf("ARM_AUTO_MOVE_ABORT\n");
#endif
      arm_state = ARM_AUTO_MOVE_ABORT;
    }
    else if(arm_state == ARM_MOVE)
    {
#ifdef SHOW_ARM_STATE
      printf("ARM_STOP\n");
#endif
      arm_state = ARM_STOP;
    }
  }
  
  if(jse->button[4] && jse->button[0] && jse->button[1])
    set_origin_button_request = 1;

  if((!jse->button[4]) && set_origin_button_request)  //button released
  {
    printf("Warning: predefined point set!\n");
    // store position
    arm_set_command(1, "O", -4856);
    arm_set_command(1, "p", -4856);
    arm_set_command(1, "EPTR", 100);
    arm_set_command_without_value(1, "VST(p,1)");

    arm_set_command(2, "O", 1024215);
    arm_set_command(2, "p", 1024215);
    arm_set_command(2, "EPTR", 100);
    arm_set_command_without_value(2, "VST(p,1)");
    
    arm_set_command(3, "O", 183759);
    arm_set_command(3, "p", 183759);
    arm_set_command(3, "EPTR", 100);
    arm_set_command_without_value(3, "VST(p,1)");
    
    arm_set_command(4, "O", 2109);
    arm_set_command(4, "p", 2109);
    arm_set_command(4, "EPTR", 100);
    arm_set_command_without_value(4, "VST(p,1)");
    
    arm_set_command(5, "O", 256000);
    arm_set_command(5, "p", 256000);
    arm_set_command(5, "EPTR", 100);
    arm_set_command_without_value(5, "VST(p,1)");
    
    arm_set_command(6, "O", 256000);
    arm_set_command(6, "p", 256000);
    arm_set_command(6, "EPTR", 100);
    arm_set_command_without_value(6, "VST(p,1)");

    set_origin_button_request = 0;
  }
  
  switch(arm_state)
  {
    case ARM_IDLE:
      break;
   
    case ARM_REST:
#ifdef SHOW_ARM_STATE
      printf("ARM_DINAMIC_REQUEST\n");
#endif
      arm_state = ARM_DINAMIC_REQUEST;
      break;

    case ARM_STEP:
#ifdef SHOW_ARM_STATE
	  printf("ARM_HOMING_REQUEST\n");
#endif
      arm_state = ARM_HOMING_REQUEST;
      break;

    case ARM_MOVE:
      bytes_sent = arm_move(*jse, JOY_MAX_VALUE);
       
      if(bytes_sent < 0)
      {
        //message_log("arm_load_tx", strerror(errno));
        perror("arm_load_tx");
      }
      break;
      
    case ARM_STOP:
      if(arm_stop(0))
      {
#ifdef SHOW_ARM_STATE
        printf("ARM_IDLE\n");
#endif
        arm_state = ARM_IDLE;
      }
      break;
      
    case ARM_HOMING_REQUEST:
      if(status_acu != ACU_ARM_HOMING)
      {
        // Send homing message to ACU
        status_buffer = JOYSTICK_REQUEST_HOMING;
      
        if(socket_status > 0)
        {
          bytes_sent = sendto(socket_status, &status_buffer, sizeof(status_buffer), 0, (struct sockaddr *)address, sizeof(*address));
      
          if(bytes_sent < 0)
            perror("sendto ACU on HOMING_REQUEST");
          else
            homing_requested = 1;
        }
      }
      else
      {
#ifdef SHOW_ARM_STATE
        printf("ARM_AUTO_MOVE\n");
#endif
        arm_state = ARM_AUTO_MOVE;
      }
      break;
      
      case ARM_DINAMIC_REQUEST:
      if(status_acu != ACU_ARM_AUTO_MOVE)
      {
        // Send homing message to ACU
        status_buffer = JOYSTICK_REQUEST_DINAMIC;
      
        if(socket_status > 0)
        {
          bytes_sent = sendto(socket_status, &status_buffer, sizeof(status_buffer), 0, (struct sockaddr *)address, sizeof(*address));
      
          if(bytes_sent < 0)
            perror("sendto ACU on DINAMIC_REQUEST");
        }
      }
      else
      {
#ifdef SHOW_ARM_STATE
        printf("ARM_AUTO_MOVE\n");
#endif
        arm_state = ARM_AUTO_MOVE;
      }
	  break;

    case ARM_BOX_REQUEST:
      if(status_acu != ACU_ARM_AUTO_MOVE)
      {
        // Send homing message to ACU
        status_buffer = JOYSTICK_REQUEST_BOX;
      
        if(socket_status > 0)
        {
          bytes_sent = sendto(socket_status, &status_buffer, sizeof(status_buffer), 0, (struct sockaddr *)address, sizeof(*address));
      
          if(bytes_sent < 0)
            perror("sendto ACU on BOX_REQUEST");
        }
      }
      else
      {
#ifdef SHOW_ARM_STATE
        printf("ARM_AUTO_MOVE\n");
#endif
        arm_state = ARM_AUTO_MOVE;
      }
      break;
  
    case ARM_PARK_REQUEST:
      if(status_acu != ACU_ARM_AUTO_MOVE)
      {
        // Send homing message to ACU
        status_buffer = JOYSTICK_REQUEST_PARK;
      
        if(socket_status > 0)
        {
          bytes_sent = sendto(socket_status, &status_buffer, sizeof(status_buffer), 0, (struct sockaddr *)address, sizeof(*address));
      
          if(bytes_sent < 0)
            perror("sendto ACU on PARK_REQUEST");
        }
      }
      else
      {
#ifdef SHOW_ARM_STATE
        printf("ARM_AUTO_MOVE\n");
#endif
        arm_state = ARM_AUTO_MOVE;
      }
      break;

    case ARM_STEP_REQUEST:
      if(status_acu != ACU_ARM_AUTO_MOVE)
      {
        // Send homing message to ACU
        status_buffer = JOYSTICK_REQUEST_STEP;
      
        if(socket_status > 0)
        {
          bytes_sent = sendto(socket_status, &status_buffer, sizeof(status_buffer), 0, (struct sockaddr *)address, sizeof(*address));
      
          if(bytes_sent < 0)
            perror("sendto ACU on STEP_REQUEST");
          else
            step_request = 1;
        }
      }
      else
      {
#ifdef SHOW_ARM_STATE
        printf("ARM_AUTO_MOVE\n");
#endif
        arm_state = ARM_AUTO_MOVE;
      }
      break;

    case ARM_AUTO_MOVE_ABORT:
      if((status_acu == ACU_ARM_AUTO_MOVE) || (status_acu == ACU_ARM_HOMING))
      {
        // Send homing abort message to ACU_ADDRESS
        status_buffer = JOYSTICK_ABORT_AUTO_MOVE;
       
        if(socket_status > 0)
        {
          bytes_sent = sendto(socket_status, &status_buffer, sizeof(status_buffer), 0, (struct sockaddr *)address, sizeof(*address));
      
          if(bytes_sent < 0)
            perror("sendto ACU on HOMING ABORT");
          else
            homing_requested = 0;
        }
        break;
      }
      // don't put break here
    case ARM_AUTO_MOVE:
      led_timer++;
  
      if((status_acu != ACU_ARM_AUTO_MOVE) && (status_acu != ACU_ARM_HOMING))
      {
        led_timer = 0;

        if(homing_requested == 1)
        {
#ifdef SHOW_ARM_STATE
          printf("ARM_REST\n");
#endif
          arm_state = ARM_REST;
          homing_requested = 0;
          robotic_arm_selected = 0;

          // Unset ARM led
          sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_ARM);
          if(system(buffer) < 0)
            perror("Set Arm led");

          //message_log("stdof", "Pass to vehicle");
          printf("Pass to vehicle\n");
        }
        else if(step_request == 1)
        {
#ifdef SHOW_ARM_STATE
          printf("ARM_STEP\n");
#endif
          arm_state = ARM_STEP;
          step_request = 0;
          robotic_arm_selected = 0;
  
          // Unset ARM led
          sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_ARM);
          if(system(buffer) < 0)
            perror("Set Arm led");

          //message_log("stdof", "Pass to vehicle with arm in step position");
          printf("Pass to vehicle with arm in step position\n");
        }
        else
        {
          // Set ARM led
          sprintf(buffer, "echo 1 > /sys/class/gpio/gpio%i/value", LED_ARM);
          if(system(buffer) < 0)
            perror("Set Arm led");

          //message_log("stdof", "Pass to robotic arm");
          printf("Pass to robotic arm\n");
#ifdef SHOW_ARM_STATE
          printf("ARM_IDLE\n");
#endif
          arm_state = ARM_IDLE;
        }

        return;
      }
      else
      {
        if(led_timer >= 100) // for 10ms timeout led_timer = 1 s
        {
          led_timer = 0;
          if(arm_led)
            arm_led = 0;
          else
            arm_led = 1;

          // Toggle ARM led
          sprintf(buffer, "echo %i > /sys/class/gpio/gpio%i/value", arm_led, LED_ARM);
          if(system(buffer) < 0)
            perror("Set Arm led");
         }
      }
      break;
      
  }
}

void message_log(const char *scope, const char *message) {
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
