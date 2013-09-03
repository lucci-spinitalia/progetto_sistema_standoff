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

#include "./Segway/segway_config.h" 
#include "./Joystick/joystick.h" 
#include "./Socket_can/socket_can_interface.h" 
#include "./Segway/segway_udp.h" 
#include "./Socket_udp/socket_udp.h"

#define CCU_ADDRESS "192.168.178.60"
#define CCU_PORT 32000
#define CCU_POLL_PORT 32003
#define CAN_REMOTE_ADDRESS "192.168.178.60"
#define CAN_REMOTE_PORT 32004
#define SEGWAY_ADDRESS "192.168.1.40"
#define SEGWAY_PORT 55
#define JOY_LOCAL_NAME "/dev/input/js0"
#define JOY_REMOTE_PORT 32001
#define JOY_MAX_VALUE 32767 
#define TIMEOUT_SEC 0 
#define TIMEOUT_USEC 100000
#define LOG_FILE "/var/log/stdof_log.txt"

/* Macro */
#undef max 
#define max(x,y) ((x) > (y) ? (x) : (y))

/* Prototype */
int monitor_input_devices(struct udev_monitor *,
                          const char *, int *); 
void segway_status_update(union segway_union *, int, struct sockaddr_in *,  struct wwvi_js_event *, long int); 
void manage_remote_joystick(int *, const char *);  
void segway_message_handle(struct udp_frame *frame, union segway_union *segway_status); 
void message_log(const char *, const char *); 
int copy_log_file(void);

int main() 
{
  /* Polling Message Server */
/*  int socket_poll = -1;
  char poll_message[255];
  unsigned char poll_check = 0;
  struct sockaddr_in socket_poll_address;
*/
  /* SocketNet interface */
  int socket_client = -1;
  int socket_client_prescaler = 0;
  struct sockaddr_in client_address;
  socklen_t client_length = sizeof(client_address);
  char net_buffer[255];

  /* SocketCan interface */
  struct sockaddr_can addr;
  struct can_frame frame;
  struct ifreq ifr;
  int socket_can = -1;

  /* Remote Can client */
  int can_remote = -1;
  struct sockaddr_in can_remote_address;

  /* Joystick interface */
  struct wwvi_js_event jse;
  int joy_local = -1;
  int joy_remote = -1;
  struct sockaddr_in joy_remote_address;
  struct sockaddr_in sender_address;
  socklen_t sender_length = sizeof(sender_address);

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
  unsigned char joy_check = 0;

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

  /* Init Network */
  if(init_client(&socket_client, &client_address, CCU_ADDRESS, CCU_PORT) == -1)
  {
    message_log("init_client", strerror(errno));
    perror("init_client");
  }
  else
    printf("Init Client\t[OK]\n");

  /* Init Can Interface */
  /* The can is mandatory for this program */
  if(can_init(&socket_can, &addr, &ifr, 1, 0) == -1)
  {
    message_log("can_init", strerror(errno));
    perror("can_init");

    return 1;
  }
  else
  {
    message_log("can_init", "Init Can\t[OK]");
    printf("Init Can\t[OK]\n");
  }

  /* Init Segway */
  if(init_client(&socket_segway, &segway_address, SEGWAY_ADDRESS, SEGWAY_PORT) == -1)
  {
    message_log("segway_init", strerror(errno));
    perror("Segway client");
  }
  else if(segway_init(socket_segway, &segway_address, &segway_status) < 0)
  {
    message_log("segway_init", "Segway not found");
    printf("Init Segway\t[FAIL]\n");
  }
  else
  {
    message_log("segway_init", "Init Segway\t[OK]");
    printf("Init Segway\t[OK]\n");
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

  /* Init CCU poll server */
/*  if(init_server(&socket_poll, &socket_poll_address, CCU_POLL_PORT) == -1)
  {
    message_log("Init CCU poll Server", strerror(errno));
    perror("Init CCU poll Server");
  }
  else
  {
    message_log("Init CCU poll server", "CCU poll server\t[OK]");
    printf("Init CCU poll server\t[OK]\n");
  }
*/
  /* Init Remote Input Interface */
  if(init_server(&joy_remote, &joy_remote_address, JOY_REMOTE_PORT) == -1)
  {
    message_log("Init Remote Joystick", strerror(errno));
    perror("Init Remote Joystick");
  }
  else
  {
    message_log("Init Remote Joystick", "Remote Joystick\t[OK]");
    printf("Remote Joystick\t[OK]\n");
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
  select_timeout.tv_sec = TIMEOUT_SEC;
  select_timeout.tv_usec = TIMEOUT_USEC;
  
  message_log("stdof", "Run main program. . .");
  printf("Run main program. . .\n");

  usleep(20000);
  segway_configure_audio_command(socket_segway, &segway_address, 9);

  while(!done)
  {
    fflush(stdout);

    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);

    if(socket_segway > 0)
    {
      FD_SET(socket_segway, &rd);
      nfds = max(nfds, socket_segway);
    }
 
    if(socket_can > 0)
    {
      FD_SET(socket_can, &rd);
      nfds = max(nfds, socket_can);
    }
/*
    if(socket_poll > 0)
    {
      FD_SET(socket_poll, &rd);
      nfds = max(nfds, socket_poll);
    }
  */
    if(joy_local > 0)
    {
      FD_SET(joy_local, &rd);
      nfds = max(nfds, joy_local);
    }

    if(joy_remote > 0)
    {
      FD_SET(joy_remote, &rd);
      nfds = max(nfds, joy_remote);
    }

    if(udev_fd > 0)
    {
      FD_SET(udev_fd, &rd);
      nfds = max(nfds, udev_fd);
    }

    select_result = select(nfds + 1, &rd, NULL, NULL, &select_timeout);

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
        bytes_read = segway_read(socket_segway, &segway_status);

        if(bytes_read <= 0)
        {
          message_log("segway_read", strerror(errno));
          perror("segway_read");
        }
        else
        {
          segway_check = 0;        
          //printf("Operational State: %i\n", segway_status.list.operational_state);
        }
        continue;
      }
    }

    /* Manage ccu poll message */
/*    if(socket_poll > 0)
    {
      if(FD_ISSET(socket_poll, &rd))
      {
        bytes_read = recvfrom(socket_poll, poll_message, sizeof(poll_message), 0, NULL, NULL);

        if(bytes_read > 0)
          poll_check = 0;
      }
    }
*/
    /* Manage can message */
    if(socket_can > 0)
    {
      if(FD_ISSET(socket_can, &rd))
      {
        // Read a message back from the CAN bus
        bytes_read = read(socket_can, &frame, sizeof(frame));

        if(bytes_read < 0)
        {
          message_log("can raw socket read", strerror(errno));
          perror("can raw socket read");
        }
        else if(bytes_read < sizeof(struct can_frame))
        {
          message_log("can raw socket read", "Incomplete CAN frame");
          printf("read: incomplete CAN frame\n");
        }

        //can_message_handle(&frame, &segway_status);
        continue;  
      } // end if(FD_ISSET(socket_can, &rd))
    }

    /* Manage joystick command */
    if((joy_remote > 0) && (joy_local <= 0))
    {
      if(FD_ISSET(joy_remote, &rd))
      {
        bytes_read = recvfrom(joy_remote, &jse, sizeof(jse), 0, (struct sockaddr *)&sender_address, &sender_length);
        
        if(bytes_read > 0)
        {
          //update at the end of while
          joy_check = 0;

          //printf("X: %d\tY: %d\tZ: %d\nbutton1: %d\tbutton2: %d\n", jse.stick_x, jse.stick_y, jse.stick_z, jse.button[0], jse.button[1]);

          /* Send command to segway and update the status */
          //segway_status_update(&segway_status, socket_segway, &segway_address, &jse, JOY_MAX_VALUE);

          // The segway requirements state that the minimum update frequency have to be 0.5Hz, so
          // the timeout on udev have to be < 2 secs.
          //select_timeout.tv_sec = TIMEOUT_SEC;
          //select_timeout.tv_usec = TIMEOUT_USEC;
          continue;
        }
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

          //printf("X: %d\tY: %d\tZ: %d\nbutton1: %d\tbutton2: %d\n", jse.stick_x, jse.stick_y, jse.stick_z, jse.button[0], jse.button[1]);
          //printf("Message from parent: %d\n", jse.stick_z);

          /* Send command to segway and update the status */
          //segway_status_update(&segway_status, socket_segway, &segway_address, &jse, JOY_MAX_VALUE);

          // The segway requirements state that the minimum update frequency have to be 0.5Hz, so
          // the timeout on udev have to be < 2 secs.
          //select_timeout.tv_sec = TIMEOUT_SEC;
          //select_timeout.tv_usec = TIMEOUT_USEC;
          continue;
        }

      }//end if(joy_selected == null)
    }

    // Send a packet to the base
    if((socket_client > 0))
    {
      if(socket_client_prescaler < 10)
        socket_client_prescaler++;
      else
      {
        socket_client_prescaler = 0;
        sprintf(net_buffer, "Stand-off #1");
        if(sendto(socket_client, net_buffer, sizeof(net_buffer), 0, (struct sockaddr *)&client_address, client_length) == -1)
          perror("Description client");
      }
    }

    // Try to communicate with segway. If it is on tractor mode then I send the joystick
    // command 
    if(segway_prescaler_timeout < 50)
      segway_prescaler_timeout++;
    else
    {
      segway_prescaler_timeout = 0;

      if((segway_status.list.operational_state < 3) || (segway_status.list.operational_state > 5))
      {
        segway_configure_none(socket_segway, &segway_address, 0x00);
        segway_init(socket_segway, &segway_address, &segway_status);
      }
      else
      {
        segway_configure_none(socket_segway, &segway_address, 0x00);
      }

      segway_check++;

      if(segway_check > 3)
      {
        message_log("stdof", "Segway down!");
        printf("Segway down!\n");
        segway_status.list.operational_state = UNKNOWN;
      }
/*
      poll_check++;
      
      if(poll_check > 3)
      {
        poll_check = 4;
        printf("Connection lost!\n");
      }*/
    }  

    /*joy_check++;
 
    if(joy_check > 10)
    {
      joy_check = 0;

      jse.stick_y = 0; 
      jse.stick_x = 0;
    }*/

    segway_status_update(&segway_status, socket_segway, &segway_address, &jse, JOY_MAX_VALUE);
    //printf("X: %d\tY: %d\tZ: %d\nbutton1: %d\tbutton2: %d\n", jse.stick_x, jse.stick_y, jse.stick_z, jse.button[0], jse.button[1]);
    printf("Y: %d\n", jse.stick_y);

    // The segway requirements state that the minimum update frequency have to be 0.5Hz, so
    // the timeout on udev have to be < 2 secs.
    select_timeout.tv_sec = TIMEOUT_SEC;
    select_timeout.tv_usec = TIMEOUT_USEC;

  }  // end while(!= done)

  return 0;
}

int monitor_input_devices(struct udev_monitor *udev_mon,
                          const char *joystick_path, int *joy_local) {
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

      // I can only send a tractore request
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
      else if(jse->stick_z > 0)
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

void segway_message_handle(struct udp_frame *frame, union segway_union *segway_status) 
{
  /*int fault_return = 0;
  char fault_message[255];
 
  do
  {
    fault_return = segway_config_decode_arch_fault(segway_status->fault_status_word1, fault_message);
	      
    if(fault_return >= 0)
    {
      message_log("segway_config_fault:", fault_message);
      printf("Fault: %s\n", fault_message);
    }
  } while(fault_return > 0);
	    
  do
  {
    fault_return = segway_config_decode_critical_fault(segway_status->fault_status_word1, fault_message);

    if(fault_return >= 0)
    {
      message_log("segway_config_fault:", fault_message);
      printf("Fault: %s\n", fault_message);
    }
  } while(fault_return > 0);

  do
  {
    fault_return = segway_config_decode_comm_fault(segway_status->fault_status_word2, fault_message);

    if(fault_return >= 0)
    {
      message_log("segway_config_fault:", fault_message);
      printf("Fault: %s\n", fault_message);
    }
  } while(fault_return > 0);

  do
  {
    fault_return = segway_config_decode_internal_fault(segway_status->fault_status_word2, fault_message);

    if(fault_return >= 0)
    {
      message_log("segway_config_fault:", fault_message);
      printf("Fault: %s\n", fault_message);
    }
  } while(fault_return > 0);

  do
  {
    fault_return = segway_config_decode_sensors_fault(segway_status->fault_status_word3, fault_message);

    if(fault_return >= 0)
    {
      message_log("segway_config_fault:", fault_message);
      printf("Fault: %s\n", fault_message);
    }
  } while(fault_return > 0);

  do
  {
    fault_return = segway_config_decode_bsa_fault(segway_status->fault_status_word3, fault_message);

    if(fault_return >= 0)
    {
      message_log("segway_config_fault:", fault_message);
      printf("Fault: %s\n", fault_message);
    }
  } while(fault_return > 0);

  do
  {
    fault_return = segway_config_decode_mcu_fault(segway_status->fault_status_word4, fault_message);

    if(fault_return >= 0)
    {
      message_log("segway_config_fault:", fault_message);
      printf("Fault: %s\n", fault_message);
    }
  } while(fault_return > 0);

  do
  {
    fault_return = segway_config_decode_mcu_message(segway_status->mcu_0_fault_status, fault_message);

    if(fault_return >= 0)
    {
      message_log("segway mcu0", fault_message);
      printf("segway mcu0: %s\n", fault_message);
    }
  } while(fault_return > 0);

  do
  {
    fault_return = segway_config_decode_mcu_message(segway_status->mcu_1_fault_status, fault_message);

    if(fault_return >= 0)
    {
      message_log("segway mcu1", fault_message);
      printf("segway mcu1: %s\n", fault_message);
    }
  } while(fault_return > 0);

  do
  {
    fault_return = segway_config_decode_mcu_message(segway_status->mcu_2_fault_status, fault_message);

    if(fault_return >= 0)
    {
      message_log("segway mcu2", fault_message);
      printf("segway mcu2: %s\n", fault_message);
    }
  } while(fault_return > 0);

  do
  {
    fault_return = segway_config_decode_mcu_message(segway_status->mcu_3_fault_status, fault_message);

    if(fault_return >= 0)
    {
      message_log("segway mcu3", fault_message);
      printf("segway mcu3: %s\n", fault_message);
    }
  } while(fault_return > 0);*/
  //printf("Operational state: %08lx\n", segway_status->list.operational_state);
  /*printf("Linear velocity: %08lx\n", segway_status->linear_vel_mps);
  printf("Linear position: %08lx\n", segway_status->linear_pos_m);
  printf("Front Batt1 SOC: %08lx\n", segway_status->front_base_batt_1_soc);
  printf("Front Batt2 SOC: %08lx\n", segway_status->front_base_batt_2_soc);
  printf("Rear Batt1 SOC: %08lx\n", segway_status->rear_base_batt_1_soc);
  printf("Rear Batt2 SOC: %08lx\n", segway_status->rear_base_batt_2_soc);
  printf("Front Batt1 Temp: %08lx\n", segway_status->front_base_batt_1_temp_degC);
  printf("Front Batt2 Temp: %08lx\n", segway_status->front_base_batt_2_temp_degC);
  printf("Rear Batt1 Temp: %08lx\n", segway_status->rear_base_batt_1_temp_degC);
  printf("Rear Batt2 Temp: %08lx\n", segway_status->rear_base_batt_2_temp_degC);*/

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
