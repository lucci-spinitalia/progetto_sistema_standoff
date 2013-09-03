#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
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

#include "./Segway/segway.h"
#include "./Joystick/joystick.h"
#include "./Rs232/rs232.h"
#include "./Socket_can/socket_can_interface.h"
 
#define JOY_LOCAL_NAME "/dev/input/js0"
#define JOY_REMOTE_NAME "/home/root/remote_dev/js0"
#define JOY_MAX_VALUE 32767
#define CANTX_REMOTE_NAME "/home/root/remote_dev/can_tx"
#define CANRX_REMOTE_NAME "/home/root/remote_dev/can_rx"
#define TIMEOUT_SEC 0
#define TIMEOUT_USEC 100000
#define LOG_FILE "/var/log/stdof_log.txt"


/* Macro */
#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

/* Prototype */
int create_named_pipe(int *, const char *);
int monitor_input_devices(struct udev_monitor *, 
                          const char *, int *);
void segway_status_update(struct segway_struct *, int, struct wwvi_js_event *, long int);
void manage_remote_joystick(int *, const char *);
void manage_remote_can(int *, const char *, unsigned char);
void can_message_handle(struct can_frame *frame, struct segway_struct *segway_status);
void message_log(const char *, const char *);
int copy_log_file(void);

int main()
{
  /* SocketNet interface */
/*  int socket_client = -1;
  struct sockaddr_in client_address;
  socklen_t client_length = sizeof(client_address);
  unsigned char net_buffer[255];
*/
  /* SocketCan interface */
  struct sockaddr_can addr;
  struct can_frame frame;
  struct ifreq ifr;
  int socket_can = -1;
  int remote_cantx_pid = -1;
  int remote_canrx_pid = -1;
  int exec_can_pid = -1;
  int can_remote_rx[2];
  int can_remote_tx[2];
  int can_count = 0;

  /* Joystick interface */
  struct wwvi_js_event jse;
  int joy_local = -1;
  int pid = -1;
  int *joy_selected = NULL;
  int joy_remote[2];

  /* Udev   */
  struct udev *udev;
  struct udev_monitor *udev_mon;
  int udev_fd = -1;

  /* Segway */
  struct segway_struct segway_status;

  /* Generic Variable */
  int done = 0;  // for the while in main loop
  int bytes_read;  // to check how many bytes has been read
  int bytes_sent;  // to check how many byte has been write

  int select_result = -1;  // value returned frome select()
  int nfds = 0;  // fd to pass to select()
  fd_set rd, wr, er; // structure for select()
  struct timeval select_timeout;

  message_log("stdof", "Initializing stdof. . .");
  printf("Initializing stdof. . .\n");

  /* Peripheral initialization */

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

  /* Init SocketCan pipe */
  message_log("stdof", "Initializing remote can. . .");
  printf("Initializing remote can. . .\n");
  
  remote_cantx_pid = create_named_pipe(can_remote_tx, CANTX_REMOTE_NAME);

  if(remote_cantx_pid == -1)
  {
    close(can_remote_tx[0]);
    close(can_remote_tx[1]);

    message_log("Init Remote Can", strerror(errno));
    perror("Init Remote Can");
  }
  else if(remote_cantx_pid == 0)
    manage_remote_can(can_remote_tx, CANTX_REMOTE_NAME, 0);
  else if(remote_cantx_pid > 0)
  {
    // close unused read end
    close(can_remote_tx[0]);
  
    exec_can_pid = fork();
    if(exec_can_pid == -1)
    {
      message_log("Start can connection\n", strerror(errno));
      perror("Start can connection\n");
    }
    else if(exec_can_pid == 0)
    {
      int exec_return = -1;
      printf("Send can message over network. . .\n");
      fflush(stdout);

      exec_return = execlp("./can_tx.sh", "can_tx.sh", (char *) 0, (char *) 0);

      if(exec_return == -1)
        perror("exec:");

      printf("Stop sending can message over network. . .\n");
      fflush(stdout);

      exit(0);
    }
  }

  remote_canrx_pid = create_named_pipe(can_remote_rx, CANRX_REMOTE_NAME);
  
  if(remote_canrx_pid == -1)
  {
    close(can_remote_rx[0]);
    close(can_remote_rx[1]);

    message_log("Init Remote Can Rx", strerror(errno));
    perror("Init Remote Can Rx");
  }
  else if(remote_canrx_pid == 0)
    manage_remote_can(can_remote_rx, CANRX_REMOTE_NAME, 1);
  else if(remote_canrx_pid > 0)
  {
    //close unused write end
    close(can_remote_rx[1]);
  }

  /* Init Remote Input Interface */
  message_log("stdof", "Initializing remote input. . .");
  printf("Initializing remote input. . .\n");

  pid = create_named_pipe(joy_remote, JOY_REMOTE_NAME);

  if(pid == -1)
  {
    close(joy_remote[0]);
    close(joy_remote[1]);

    message_log("Init Remote Joystick", strerror(errno));
    perror("Init Remote Joystick");
  }
  else if(pid == 0)
    manage_remote_joystick(joy_remote, JOY_REMOTE_NAME);
  else if(pid > 0)
  {
    // close unused write end
    close(joy_remote[1]);
  }
  
  /* Init Segway */
  if(segway_init(socket_can, &segway_status) < 0)
  {
    message_log("segway_init", strerror(errno));
    perror("Segway_init");
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

  /* Init Monitor for joystick or usb storage device  */

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

  /* Init Network */
/*  if(net_init(&socket_client, &server_address, 9000) == -1)
  {
    message_log("net_init", strerror(errno));
    perror("net_init"),
  }
*/
  message_log("stdof", "Run main program. . .");
  printf("Run main program. . .\n");

  segway_configure_audio_command(socket_can, 9);

  while(!done)
  {
    fflush(stdout);

    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);

    if(can_remote_rx[0] > 0)
    {
      FD_SET(can_remote_rx[0], &rd);
      nfds = max(nfds, can_remote_rx[0]);
    }

    if(socket_can > 0)
    {
      FD_SET(socket_can, &rd);
      nfds = max(nfds, socket_can);
    }

    if(joy_local > 0)
    {
      FD_SET(joy_local, &rd);
      nfds = max(nfds, joy_local);
    }

    if(joy_remote[0] > 0)
    {
      FD_SET(joy_remote[0], &rd);
      nfds = max(nfds, joy_remote[0]);
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

    /* Manage remote can */
    if(can_remote_rx[0] > 0)
    {
      if(FD_ISSET(can_remote_rx[0], &rd))
      {
        bytes_read = read(can_remote_rx[0], &frame, sizeof(struct can_frame));
        
        if(bytes_read <= 0)
          perror("Can remote rx");
        else
        {       
          printf("[%x] [%i]", frame.can_id, frame.can_dlc);
    
          for(can_count = 0; can_count < frame.can_dlc; can_count++)
          {
            printf("[%x]", frame.data[can_count]);
          }
 
          printf("\n");
        }
      }
    }

    /* Manage joystick command */
    if((joy_local > 0) || (joy_remote[0] > 0))
    {
      joy_selected = NULL;

      /* The local solution win */
      if(joy_remote[0] > 0)
      {
        if(FD_ISSET(joy_remote[0], &rd))
          joy_selected = &joy_remote[0];
      }

      if(joy_local > 0)
      {
        if(FD_ISSET(joy_local, &rd))
          joy_selected = &joy_local;
      }

      if(joy_selected != NULL)
      {
        bytes_read = get_joystick_status(joy_selected, &jse);

        /* Select from local or remote joystick */

        if(bytes_read <= 0)
        {
          message_log("joystick", strerror(errno));
          perror("get_joystick_status");

          if(joy_selected == &joy_remote[0])
          {
            close(*joy_selected);

            *joy_selected = -1;
            
            pid = create_named_pipe(joy_remote, JOY_REMOTE_NAME);

            if(pid == -1)
            {
              close(joy_remote[0]);
              close(joy_remote[1]);

              message_log("Init Remote Joystick", strerror(errno));
              perror("Init Remote Joystick");
            }
            else if(pid == 0)
              manage_remote_joystick(joy_remote, JOY_REMOTE_NAME);
            else if(pid > 0)
            {
              // close unused write end
              close(joy_remote[1]);
            }
          }  // if(joy_selected == &joy_remote[0]) 
       
          continue;
        }  // end if(bytes_read <= 0)
 
        //printf("X: %d\tY: %d\tZ: %d\nbutton1: %d\tbutton2: %d\n", jse.stick_x, jse.stick_y, jse.stick_z, jse.button[0], jse.button[1]);
        //printf("Message from parent: %d\n", jse.stick_z);

        /* Send command to segway and update the status */
        segway_status_update(&segway_status, socket_can, &jse, JOY_MAX_VALUE);

        // The segway requirements state that the minimum update frequency have to be 0.5Hz, so
        // the timeout on udev have to be < 2 secs.
        select_timeout.tv_sec = TIMEOUT_SEC;
        select_timeout.tv_usec = TIMEOUT_USEC;

        continue;
      }//end if(joy_selected == null)
    }
    
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

        can_message_handle(&frame, &segway_status);
        write(can_remote_tx[1], &frame, sizeof(struct can_frame));

        // get interface name of the received CAN frame
        //  ifr.ifr_ifindex = addr.can_ifindex;
        //ioctl(socket_can, SIOCGIFNAME, &ifr);
        //printf("Received a CAN frame from interface %s\n", ifr.ifr_name);
        
        continue;
      } // end if(FD_ISSET(socket_can, &rd))
    }

    if(segway_status.operational_state == SEGWAY_TRACTOR)
    {
      // If z axis is > 0 and there's a joystick attached
      if((jse.stick_z > 0) && ((joy_local > 0) || (joy_remote[0] > 0)))
        bytes_sent = segway_motion_set(socket_can, jse.stick_y, jse.stick_x, JOY_MAX_VALUE);
      else
	bytes_sent = segway_motion_set(socket_can, 0, 0, JOY_MAX_VALUE);

      if(bytes_sent < 0)
      {
        message_log("segway_motion_set", strerror(errno));
        perror("segway_motion_set");
      }
    }

    // The segway requirements state that the minimum update frequency have to be 0.5Hz, so
    // the timeout on udev have to be < 2 secs.
    select_timeout.tv_sec = TIMEOUT_SEC;
    select_timeout.tv_usec = TIMEOUT_USEC;
  }  // end while(!= done)

  return 0;
}

int create_named_pipe(int *pipefd, const char *file_name)
{
  int pid = -1;

  //try to open pipe. If it fail then we have to create them
  *pipefd = open(file_name, O_RDONLY | O_NONBLOCK);

  if(*pipefd < 0)
  {
    if(mkfifo(file_name, S_IFIFO | S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH) == -1)
      return -1;
    else
    {
      close(*pipefd);
      *pipefd = -1;
    }
  }

  if(pipe(pipefd) != -1)
  {
    // When open a named pipe result in a blocking function if it's not open on the
    // other end. So we create a new process that handle the remote joystick 
    // stuff and link it to the main process by a kernel pipe
    pid = fork();

    if(pid < 0)
    {
      close(pipefd[0]);
      close(pipefd[1]);

      return -1;
    }
    else
      return pid;    
  }

  return -1;
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
    udev_node =  udev_device_get_devnode(udev_dev);
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

void segway_status_update(struct segway_struct *segway_status, int socket_can,  struct wwvi_js_event *jse, long int joy_max_value)
{
  static int segway_previouse_state = 0;
  int bytes_sent = -1;

  switch(segway_status->operational_state)
  {
    case CCU_INIT:
      if(segway_previouse_state != segway_status->operational_state)
      {
        message_log("stdof", "Segway CCU Init");
        printf("Segway CCU Init\n");

        segway_previouse_state = segway_status->operational_state;
      }
      break;

    case PROPULSION_INIT:
      if(segway_previouse_state != segway_status->operational_state)
      {
        message_log("stdof", "Segway Propulsion Init");
        printf("Segway Propulsion Init\n");

        segway_previouse_state = segway_status->operational_state;
      }
      break;

    case CHECK_STARTUP:
      if(segway_previouse_state != segway_status->operational_state)
      {
        message_log("stdof", "Segway Check Startup Issue");
        printf("Segway Check Startup Issue\n");

        segway_previouse_state = segway_status->operational_state;
      }
      break;

    case SEGWAY_STANDBY:  //standby mode
      if(segway_previouse_state != segway_status->operational_state)
      {
        message_log("stdof", "Segway in Standby Mode");
        printf("Segway in Standby Mode\n");

        segway_previouse_state = segway_status->operational_state;
      }

      // I can only send a tractore request
      if(jse->button[0])
      {
        bytes_sent = segway_configure_operational_mode(socket_can, SEGWAY_TRACTOR_REQ);
      
        if(bytes_sent == -1)
        { 
          message_log("segway_configure_operational_mode to tractor", strerror(errno));
          perror("segway_configure_operational_mode");
        }

        // get new state
        bytes_sent = segway_configure_none(socket_can, 0x00);

        break;
      }
      break;

    case SEGWAY_TRACTOR:
      if(segway_previouse_state != segway_status->operational_state)
      {
        message_log("stdof", "Segway in Tractor Mode");
        printf("Segway in Tractor Mode\n");

        segway_previouse_state = segway_status->operational_state;
      }

      if(jse->button[1])
      {
        bytes_sent = segway_configure_operational_mode(socket_can, SEGWAY_STANDBY_REQ);

        if(bytes_sent == -1)
        {
          message_log("segway_configure_operational_mode to standby", strerror(errno));
          perror("segway_configure_operational_mode");
        }

        // get new state
        bytes_sent = segway_configure_none(socket_can, 0x00);
	      
        break;
      }
            
      if(jse->stick_z > 0)
        bytes_sent = segway_motion_set(socket_can, jse->stick_y, jse->stick_x, joy_max_value);
      else
        bytes_sent = segway_motion_set(socket_can, 0, 0, joy_max_value);
	      
      if(bytes_sent < 0)
      {
        message_log("segway_motion_set", strerror(errno));
        perror("segway_motion_set");
      }

      break;

    case DISABLE_POWER:
      if(segway_previouse_state != segway_status->operational_state)
      {
        message_log("stdof", "Segway Disable Power");
        printf("Segway Disable Power\n");

        segway_previouse_state = segway_status->operational_state;
      }
      break;

    default:
      if(segway_previouse_state != segway_status->operational_state)
      {
        message_log("stdof", "Segway Uknown State");
        printf("Segway Unknown State\n");

        segway_previouse_state = segway_status->operational_state;
      }
      break;
  } //end switch 
}

void can_message_handle(struct can_frame *frame, struct segway_struct *segway_status)
{
  int fault_return = 0;
  char fault_message[255];

  switch(frame->can_id)
  {
    case 0x502: 
    case 0x503:
    case 0x504:
    case 0x505: 
    case 0x506:
    case 0x507:
    case 0x508:
    case 0x509:
    case 0x50a:
    case 0x50b:
    case 0x50c:
    case 0x50d:
    case 0x50e:
    case 0x50f:
    case 0x510:
    case 0x511:
    case 0x512:
    case 0x513:
    case 0x514:
    case 0x515:
    case 0x516:
    case 0x517:
    case 0x518:
    case 0x519:
    case 0x51a:
    case 0x51b:
    case 0x51c:
    case 0x51d:
    case 0x51e:
    case 0x51f:
    case 0x520:
    case 0x521:
    case 0x522:
    case 0x523:
    case 0x524:
    case 0x525:
    case 0x526:
    case 0x527:
    case 0x528:
    case 0x529:
    case 0x52a:
    case 0x52b:
      /* Update the segway flags */
      segway_config_update(frame);
	    
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
          } while(fault_return > 0);
          /*printf("Operational state: %08lx\n", segway_status->operational_state);
          printf("Linear velocity: %08lx\n", segway_status->linear_vel_mps);
          printf("Linear position: %08lx\n", segway_status->linear_pos_m);
          printf("Front Batt1 SOC: %08lx\n", segway_status->front_base_batt_1_soc);
          printf("Front Batt2 SOC: %08lx\n", segway_status->front_base_batt_2_soc);
          printf("Rear Batt1 SOC: %08lx\n", segway_status->rear_base_batt_1_soc);
          printf("Rear Batt2 SOC: %08lx\n", segway_status->rear_base_batt_2_soc);
          printf("Front Batt1 Temp: %08lx\n", segway_status->front_base_batt_1_temp_degC);
          printf("Front Batt2 Temp: %08lx\n", segway_status->front_base_batt_2_temp_degC);
          printf("Rear Batt1 Temp: %08lx\n", segway_status->rear_base_batt_1_temp_degC);
          printf("Rear Batt2 Temp: %08lx\n", segway_status->rear_base_batt_2_temp_degC);*/
          break;

    default:
      break;
  }  // end switch
}

/*int net_init(int *socket_net, struct sockaddr_in *addr, int portnumber)
{
  *socket_net = socket(AF_INET, SOCK_DGRAM, IPPROT_UDP);

  if(*socket_net < 0)
    return -1;

  bzero((char *)addr, sizeof(*addr));
  addr->sin_family = AF_INET;
  addr->sin_port = htons(portnumber);

  if(bind(*socket_net, (struct sockaddr *)addr, sizeof(*addr)) < 0)
    return -1;

  return 0;
}*/

void manage_remote_can(int *pipefd, const char *can_path, unsigned char direction)
{
  int pipe_file = -1;
  int bytes_write;
  struct can_frame frame;

  //close unused
  if(direction == 1)  // read from file and write on pipe
  {
    close(pipefd[0]);
    pipe_file = open(can_path, O_RDONLY);
  }
  else
  {
    close(pipefd[1]);
    pipe_file = open(can_path, O_WRONLY);
  }

  if(pipe_file < 0)
  {
    message_log("open can(remote)", strerror(errno));
    perror("open can(remote)");
  }
  else
  {
    message_log("open can(remote)", "Can remote session started");
    printf("Can remote session started\n");
  }

  fflush(stdout);

  if(direction == 1)
  {
    while(read(pipe_file, &frame, sizeof(struct can_frame)) > 0)
      bytes_write = write(pipefd[1], &frame, sizeof(struct can_frame));
  }
  else
  {
    while(read(pipefd[0], &frame, sizeof(struct can_frame)) > 0)
      bytes_write = write(pipe_file, &frame, sizeof(struct can_frame));
  }

  exit(0);
}

void manage_remote_joystick(int *pipefd, const char *joystick_path)
{
  int joy_remote = -1;
  int bytes_write;
  struct js_event jse;

  //close unused read end
  close(pipefd[0]);

  joy_remote = open(joystick_path, O_RDONLY);

  if(joy_remote < 0)
  {
    message_log("open joystick(remote)", strerror(errno));
    perror("open joystick(remote)");
  }
  else
  {
    message_log("open joystick(remote)", "Remote joystick found");
    printf("Remote joystick found\n");
  } 

  fflush(stdout);

  while(read(joy_remote, &jse, sizeof(jse)) > 0)
  {
    bytes_write = write(pipefd[1], &jse, sizeof(struct js_event)); 
  }

  exit(0);
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

int copy_log_file()
{
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
