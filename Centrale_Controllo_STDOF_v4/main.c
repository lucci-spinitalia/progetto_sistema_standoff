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

#define CCU_ADDRESS "192.168.1.102"
#define CCU_PORT 
#define SEGWAY_ADDRESS "192.168.1.40"
#define SEGWAY_PORT 55
#define ARM_ADDRESS "192.168.1.28"
#define ARM_PORT 8012
#define ARM_PRESCALER 2
#define JOY_PORT 8013
#define JOY_LOCAL_NAME "/dev/input/js0"
#define JOY_MAX_VALUE 32767 
#define LED_READY 117
#define LED_RUN 48
#define LED_STDBY 49
#define LED_ARM 7
#define TIMEOUT_SEC 0 
#define TIMEOUT_USEC 100000
#define TIMEOUT_ARM_USEC 10000
#define LOG_FILE "/var/log/stdof_log.txt"

/* Macro */
#undef max 
#define max(x,y) ((x) > (y) ? (x) : (y))
  
/* Robotic Arm */
unsigned char robotic_arm_selected = 0;
  
/* Prototype */
int monitor_input_devices(struct udev_monitor *, const char *, int *); 
void segway_status_update(union segway_union *, int, struct sockaddr_in *,  struct wwvi_js_event *, long int);
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
  unsigned char segway_prescaler_timeout = 0;
  unsigned char segway_check = 0;
  unsigned char segway_down = 0;

  /* Robotic Arm */
  int socket_arm = -1;
  struct sockaddr_in arm_address;
  struct arm_frame arm_buffer_temp;
  int arm_prescaler_count = 0;

  /* Generic Variable */
  int done = 0; // for the while in main loop
  int bytes_read; // to check how many bytes has been read

  int select_result = -1; // value returned frome select()
  int nfds = 0; // fd to pass to select()
  fd_set rd, wr, er; // structure for select()
  struct timeval select_timeout;

  message_log("stdof", "Initializing stdof. . .");
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
    message_log("init segway client", strerror(errno));
    perror("init segway client");
  }

  /* Init Arm Client */  
  if(arm_open(&socket_arm, &arm_address, JOY_PORT, ARM_ADDRESS, ARM_PORT) == -1)
  {
    message_log("init arm client", strerror(errno));
    perror("init arm client");
  }
  else
  {
    if(arm_init(0, 1000, 10, 2000, 400, 200000, 500) < 0)
	{
	  socket_arm = -1;
      message_log("init arm", strerror(errno));
      perror("init arm");	  
	} 
  }

  /* Init Joystick */
  memset(&jse, 0, sizeof(struct wwvi_js_event));
  joy_local = open_joystick(JOY_LOCAL_NAME);

  if(joy_local < 0)
  {
    message_log("open_joystick(local)", strerror(errno));
    perror("open_joystick(local)");
  }
  else
  {
    message_log("open_joystick", "Find Local Joystick\t[OK]");
    printf("Find Local Joystick\t[OK]\n");
  }
  
  /* Init Monitor for joystick or usb storage device */
  udev = udev_new();

  if(!udev)
  {
    message_log("udev_new", "Init udev\t[FAIL]");
    printf("Can't create udev\n");
  }
  else
  {
    udev_mon = udev_monitor_new_from_netlink(udev, "udev");
    udev_monitor_filter_add_match_subsystem_devtype(udev_mon, "input", NULL); // for joystick
    udev_monitor_filter_add_match_subsystem_devtype(udev_mon, "block", "partition"); // for storage device
    udev_monitor_enable_receiving(udev_mon);
    udev_fd = udev_monitor_get_fd(udev_mon);

    message_log("udev_new", "Init udev\t[OK]");
    printf("Init udev\t[OK]\n");
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
  
  message_log("stdof", "Run main program. . .");
  printf("Run main program. . .\n");
  
  while(!done)
  { 
    fflush(stdout);

    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);

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

    if(udev_fd > 0)
    {
      FD_SET(udev_fd, &rd);
      nfds = max(nfds, udev_fd);
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

    /* Monitor input devices */
    if(udev_fd > 0)
    {
      if(FD_ISSET(udev_fd, &rd))
      {
        monitor_input_devices(udev_mon, JOY_LOCAL_NAME, &joy_local);
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
          message_log("segway_read", strerror(errno));
          perror("segway_read");
        }
        else
        {
          segway_check = 0;
         
          if(segway_down == 1)
          {
            message_log("segway_init", "Segway Init\t[OK]\n");
            printf("Segway Init\t[OK]");
        
            segway_configure_audio_command(socket_segway, &segway_address, 9);
            segway_down = 0;
          }

          //printf("Operational State: %i\n", segway_status.list.operational_state);
        }
        continue;
      }
	  
	  if(FD_ISSET(socket_segway, &wr))
	  {
	    if(segway_send(socket_segway, &segway_address) < 0)
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

    /* Manage arm message */
    if(socket_arm > 0)
    {
      if(FD_ISSET(socket_arm, &rd))
      {
	    // the message would be an information such position or warning
		bytes_read = recvfrom(socket_arm, &arm_buffer_temp, sizeof(struct arm_frame), 0, NULL, NULL);

        if(bytes_read <= 0)
        {
          message_log("arm_read", strerror(errno));
          perror("arm_read");
        }
		else
		{
		  arm_link[query_link - 1].actual_position = atoi(arm_buffer_temp.frame);
		  query_link = -1;
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
          message_log("joystick", strerror(errno));
          perror("get_joystick_status");
        }  // end if(bytes_read <= 0)
        else
        {
          // update at the end of the while
          continue;
        }
      }//end if(joy_selected == null)
    }

    if(robotic_arm_selected == 0)
    {
      // Try to communicate with segway. If it is on tractor mode then I send the joystick
      // command 
      if(segway_prescaler_timeout < 50)
        segway_prescaler_timeout++;
      else
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

      segway_status_update(&segway_status, socket_segway, &segway_address, &jse, JOY_MAX_VALUE);
      //printf("X: %d\tY: %d\tZ: %d\nbutton1: %d\tbutton2: %d\nbutton3: %d\tbutton4: %d\nbutton5: %d\n", 
      //        jse.stick_x, jse.stick_y, jse.stick_z, jse.button[0], jse.button[1], jse.button[2], jse.button[3], jse.button[4]);
      //printf("Y: %d\n", jse.stick_y);
      //printf("Front Batt1 SOC: %f\n", convert_to_float(segway_status.list.front_base_batt_1_soc));
      //printf("Front Batt2 SOC: %f\n", convert_to_float(segway_status.list.front_base_batt_2_soc));
      //printf("Rear Batt1 SOC: %f\n", convert_to_float(segway_status.list.rear_base_batt_1_soc));
      //printf("Rear Batt2 SOC: %f\n", convert_to_float(segway_status.list.rear_base_batt_2_soc));  
    }
    else
    {
	  if(arm_prescaler_count >= ARM_PRESCALER)
	  {
	    arm_prescaler_count = 0;
        arm_status_update(socket_arm, &arm_address, &jse, JOY_MAX_VALUE);
	  }
      else
	    arm_prescaler_count++;
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

          message_log("stdof", "Joystick added");
	      printf("stdof: Joystick added\n");
	    }
        else if((!strcmp(udev_device_get_action(udev_dev),"remove")) && (*joy_local > 0))
        {
          // remove joystick
          *joy_local = -1;

          message_log("stdof", "Joystick removed");
          printf("stdof: Joystick removed\n");
        }
      }
    }  // end if(udev_node && !strcmp(udev_subsystem, "input"))
    else if(udev_node && !strcmp(udev_subsystem, "block"))
    {
      if(!strcmp(udev_device_get_action(udev_dev),"add"))
      {
        message_log("stdof", "Find storage device");
        printf("udev: Find storage device %s\n", udev_node);

        // mount usb device
        if(mount(udev_node, "/media/usb", "vfat", MS_NOATIME, NULL))
        {
          message_log("mount", strerror(errno));
          perror("mount");
          return -1;
        }
        else
        {
          message_log("mount", "Drive mounted");
          printf("mount: Drive mounted\n");
        }

        if(copy_log_file() < 0)
        {
          message_log("copy_log_file", strerror(errno));
          perror("copy_log_file");
        }
        else
        {
          message_log("copy_log_file", "Log copied to usb device");
          printf("copy_log_file: Log copied to usb device\n");
        }

        // unmount usb device
        if(umount("/media/usb") < 0)
        {
          message_log("umount", strerror(errno));
          perror("umount");
        }
        else
        {
          message_log("umount", "Drive unmounted");
          printf("mount: Drive unmounted\n");
        }
      }
      else if(!strcmp(udev_device_get_action(udev_dev),"remove"))
      {
        message_log("stdof", "Storage device removed");
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

  char buffer[256];
  int bytes_sent = -1;

  switch(segway_status->list.operational_state)
  {
    case CCU_INIT:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        message_log("stdof", "Segway CCU Init");
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

        // Set ARM led
        sprintf(buffer, "echo 1 > /sys/class/gpio/gpio%i/value", LED_ARM);
        if(system(buffer) < 0)
          perror("Set Arm led");

        message_log("stdof", "Pass to robotic arm");
        printf("Pass to robotic arm\n");
        break;
      }

      break;

    case PROPULSION_INIT:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        message_log("stdof", "Segway Propulsion Init");
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

        // Set ARM led
        sprintf(buffer, "echo 1 > /sys/class/gpio/gpio%i/value", LED_ARM);
        if(system(buffer) < 0)
          perror("Set Arm led");

        message_log("stdof", "Pass to robotic arm");
        printf("Pass to robotic arm\n");
        break;
      }

      break;

    case CHECK_STARTUP:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        message_log("stdof", "Segway Check Startup Issue");
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

        // Set ARM led
        sprintf(buffer, "echo 1 > /sys/class/gpio/gpio%i/value", LED_ARM);
        if(system(buffer) < 0)
          perror("Set Arm led");

        message_log("stdof", "Pass to robotic arm");
        printf("Pass to robotic arm\n");
        break;
      }

      break;

    case SEGWAY_STANDBY: //standby mode
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        message_log("stdof", "Segway in Standby Mode");
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
        robotic_arm_selected = 1;

        // Set ARM led
        sprintf(buffer, "echo 1 > /sys/class/gpio/gpio%i/value", LED_ARM);
        if(system(buffer) < 0)
          perror("Set Arm led");

        message_log("stdof", "Pass to robotic arm");
        printf("Pass to robotic arm\n");
        break;
      }
 
      if(jse->button[0])
      {
        bytes_sent = segway_configure_operational_mode(socket, segway_address, SEGWAY_TRACTOR_REQ);
      
        if(bytes_sent == -1)
        {
          message_log("segway_configure_operational_mode to tractor", strerror(errno));
          perror("segway_configure_operational_mode");
        }

        break;
      }
      break;

    case SEGWAY_TRACTOR:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        message_log("stdof", "Segway in Tractor Mode");
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
      {
        bytes_sent = segway_configure_operational_mode(socket, segway_address, SEGWAY_STANDBY_REQ);

        if(bytes_sent == -1)
        {
          message_log("segway_configure_operational_mode to standby", strerror(errno));
          perror("segway_configure_operational_mode");
        }
        break;
      } 
      
      if(jse->button[4])
        bytes_sent = segway_motion_set(socket, segway_address, jse->stick_y, jse->stick_x, joy_max_value);
      else
        bytes_sent = segway_motion_set(socket, segway_address, 0, 0, joy_max_value);
	      
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

        // Set ARM led
        sprintf(buffer, "echo 1 > /sys/class/gpio/gpio%i/value", LED_ARM);
        if(system(buffer) < 0)
          perror("Set Arm led");

        message_log("stdof", "Pass to robotic arm");
        printf("Pass to robotic arm\n");
        break;
      }

      break;

    default:
      if(segway_previouse_state != segway_status->list.operational_state)
      {
        message_log("stdof", "Segway Uknown State");
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

        // Set ARM led
        sprintf(buffer, "echo 1 > /sys/class/gpio/gpio%i/value", LED_ARM);
        if(system(buffer) < 0)
          perror("Set Arm led");

        message_log("stdof", "Pass to robotic arm");
        printf("Pass to robotic arm\n");
        break;
      }

      break;
  } //end switch
}

void arm_status_update(int socket, struct sockaddr_in *address, struct wwvi_js_event *jse, long int joy_max_value) 
{
  char buffer[256];
  int bytes_sent = -1;
  static unsigned char change_actuator_request = 0;
  static unsigned char link_button_pressed = 0;

  // Change actuator if press only the button 4
  if(jse->button[3] && !jse->button[0] && !jse->button[1] && !jse->button[2])
    change_actuator_request = 1;

  if((!jse->button[3]) && change_actuator_request)  //button released
  {
    change_actuator_request = 0;
    robotic_arm_selected = 0;
 	
    // Unset ARM led
    sprintf(buffer, "echo 0 > /sys/class/gpio/gpio%i/value", LED_ARM);
    if(system(buffer) < 0)
      perror("Set Arm led");

    message_log("stdof", "Pass to vehicle");
    printf("Pass to vehicle\n");
    return;
  }
      
  if(jse->button[0] || jse->button[1] || jse->button[2])
  {
    bytes_sent = arm_move(*jse, JOY_MAX_VALUE);
	link_button_pressed = 1;

    if(bytes_sent < 0)
    {
      message_log("arm_load_tx", strerror(errno));
      perror("arm_load_tx");
    }  
  }
  else if(!jse->button[0] && !jse->button[1] && !jse->button[2] && link_button_pressed) // link button release
  {
    if(arm_stop() > 0)
      link_button_pressed = 0;
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
