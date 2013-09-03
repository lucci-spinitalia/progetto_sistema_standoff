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

#include "./Joystick/joystick.h"
#include "./Socket_udp/socket_udp.h"
 
#define JOY_LOCAL_NAME "/dev/input/js0"
#define CCU_PORT 32000
#define CCU_POLL 32003 
#define JOY_REMOTE_PORT 32001
#define JOY_MAX_VALUE 32767
#define TIMEOUT_SEC 0
#define TIMEOUT_USEC 50000
#define LOG_FILE "/var/log/stdof_log.txt"

/* Macro */
#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

/* Prototype */
int monitor_input_devices(struct udev_monitor *, const char *, int *,int *);
void message_log(const char *, const char *);
int copy_log_file(void);

int main()
{
  /* Polling Message */
/*  int socket_poll = -1;
  char poll_message[255];
  struct sockaddr_in socket_poll_address;
  int socket_poll_prescaler = 0;
*/
  /* SocketNet interface */
  int socket_server = -1;
  struct sockaddr_in server_address;
  char net_buffer[255];

  /* Joystick interface */
  struct wwvi_js_event jse;
  int joy_local = -1;
  int joy_remote = -1;
  struct sockaddr_in joy_remote_address;

  struct sockaddr_in stdof_address;
  socklen_t stdof_length = sizeof(stdof_address);

  /* Udev   */
  struct udev *udev;
  struct udev_monitor *udev_mon;
  int udev_fd = -1;

  /* Generic Variable */
  int done = 0;  // for the while in main loop
  int bytes_read;  // to check how many bytes has been read

  int select_result = -1;  // value returned frome select()
  int nfds = 0;  // fd to pass to select()
  fd_set rd, wr, er; // structure for select()
  struct timeval select_timeout;  

  message_log("stdof", "Initializing stdof. . .");
  printf("Initializing stdof. . .\n");

  /* Peripheral initialization */
  
  /* Init Poll Message */
  //sprintf(poll_message, "Centrale di controllo #1");

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

  /* Init Network Server */
  if(init_server(&socket_server, &server_address, CCU_PORT) == -1)
  {
    message_log("init_server", strerror(errno));
    perror("init_server");
  }
  else
  {
    message_log("net_init", "Init Net\t[OK]");
    printf("Init Net Server\t[OK]\n");
  }

  select_timeout.tv_sec = TIMEOUT_SEC;
  select_timeout.tv_usec = TIMEOUT_USEC;

  message_log("stdof", "Run main program. . .");
  printf("Run main program. . .\n");

  while(!done)
  {
    fflush(stdout);

    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);

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
	
    if(socket_server > 0)
    {
      FD_SET(socket_server, &rd);
      nfds = max(nfds, socket_server);
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
      perror("select");

      return 1;
    }

    /* Monitor input devices */
    if(udev_fd > 0)
    {
      if(FD_ISSET(udev_fd, &rd))
      {
        monitor_input_devices(udev_mon, JOY_LOCAL_NAME, &joy_local, &joy_remote);
      }
    }


    if(socket_server > 0)
    {
      if(FD_ISSET(socket_server, &rd))
      {
        bytes_read = recvfrom(socket_server, net_buffer, sizeof(net_buffer), 0, (struct sockaddr *) &stdof_address, &stdof_length);

        if(bytes_read > 0)
        {
          printf("Stand-off founded on %s:%d\n", inet_ntoa(stdof_address.sin_addr), ntohs(stdof_address.sin_port));

          memcpy(&joy_remote_address, &stdof_address, sizeof(stdof_address));
          if(init_client(&joy_remote, &joy_remote_address, inet_ntoa(joy_remote_address.sin_addr), JOY_REMOTE_PORT) == -1)
          {
            message_log("stdof", strerror(errno));
            perror("Send joy message");
          }
          else
          {
            message_log("stdof", "Send joystick message");
            printf("Send joystick message\n");
          }

          close(socket_server);
          socket_server = -1;
   
/*          memcpy(&socket_poll_address, &stdof_address, sizeof(stdof_address));
          if(init_client(&socket_poll, &socket_poll_address, inet_ntoa(socket_poll_address.sin_addr), CCU_POLL) == -1)
          {
            message_log("stdof", strerror(errno));
            perror("Send polling message");
          }
          else
          {
            message_log("stdof", "Send poll message");
            printf("Send polling message\n");
          }*/     
        }
      }
    } // end if(socket_server > 0)

    if(joy_remote > 0)
    {
      printf("X: %d\tY: %d\tZ: %d\nbutton1: %d\tbutton2: %d\n", jse.stick_x, jse.stick_y, jse.stick_z, jse.button[0], jse.button[1]);
      sendto(joy_remote, &jse, sizeof(jse), 0, (struct sockaddr *) &joy_remote_address, stdof_length);      
    }

    /* Manage joystick command */
    if((joy_local > 0))
    {
      if(FD_ISSET(joy_local, &rd))
      {
        bytes_read = get_joystick_status(&joy_local, &jse);

        if(bytes_read <= 0)
        {
          message_log("joystick", strerror(errno));
          perror("get_joystick_status");
    
          continue;
        }  // end if(bytes_read <= 0)
        
        if(joy_remote > 0)
        {
          printf("X: %d\tY: %d\tZ: %d\nbutton1: %d\tbutton2: %d\n", jse.stick_x, jse.stick_y, jse.stick_z, jse.button[0], jse.button[1]);
          sendto(joy_remote, &jse, sizeof(jse), 0, (struct sockaddr *) &stdof_address, stdof_length);      
        }

        select_timeout.tv_sec = TIMEOUT_SEC;
        select_timeout.tv_usec = TIMEOUT_USEC;
        continue;
      }//end if(joy_local > 0)
    }

/*    socket_poll_prescaler++;

    if(socket_poll_prescaler > 3)
    {
      socket_poll_prescaler = 0;

      if(socket_poll > 0)
        sendto(socket_poll, poll_message, strlen(poll_message), 0, (struct sockaddr *) &socket_poll_address, stdof_length);      
    }
*/
    select_timeout.tv_sec = TIMEOUT_SEC;
    select_timeout.tv_usec = TIMEOUT_USEC;

  } // end while(!= done)

  return 0;
}

int monitor_input_devices(struct udev_monitor *udev_mon, 
                          const char *joystick_path, int *joy_local, int *joy_remote)
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
          if(*joy_local > 0)
          {
            message_log("udev", "Cannot connect second joystick");
            printf("Cannot connect second joystick");
            return 0;
          }  
   
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
 
          close(*joy_remote);
          *joy_remote = -1;

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
