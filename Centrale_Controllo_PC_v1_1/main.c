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

#include "./Segway/segway.h"
#include "./Joystick/joystick.h"
#include "./Rs232/rs232.h"
 
#define JOY_LOCAL_NAME "/dev/input/js0"
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
void manage_remote_joystick();
void manage_remote_can(int *, const char *, unsigned char);
int net_init_client(int *, struct sockaddr_in *, const char *, int);
int net_init_server(int *, struct sockaddr_in *, const char *, int);
void message_log(const char *, const char *);
int copy_log_file(void);

int main()
{
  /* SocketNet interface */
  int socket_server = -1;
  int socket_client = -1;
  struct sockaddr_in server_address;
  struct sockaddr_in client_addr;
  struct sockaddr_in sender_address;
  socklen_t sender_length = sizeof(sender_address);
  char net_buffer[255];

  /* SocketCan interface */
  struct can_frame frame;
  int remote_cantx_pid = -1;
  int remote_canrx_pid = -1;
  int exec_can_pid = -1;
  int exec_return = -1;
  int can_remote_tx[2];
  int can_remote_rx[2];
  int can_count = 0;

  /* Joystick interface */
  struct wwvi_js_event jse;
  int joy_local = -1;

  /* Udev   */
  struct udev *udev;
  struct udev_monitor *udev_mon;
  int udev_fd = -1;
  int pid = -1;

  /* Segway */
  struct segway_struct segway_status;

  /* Pc interface */
  int rs232_device = -1;
  const unsigned char header = 0x024;
  unsigned char rs232_id;
  unsigned char rs232_state = 0;
  unsigned char rs232_data_count = 0;
  unsigned char rs232_data_read = 0;
  char rs232_buffer[255];
  
  /* Generic Variable */
  int done = 0;  // for the while in main loop
  int bytes_read;  // to check how many bytes has been read
  int bytes_sent;  // to check how many byte has been write

  int select_result = -1;  // value returned frome select()
  int nfds = 0;  // fd to pass to select()
  fd_set rd, wr, er; // structure for select()

  message_log("stdof", "Initializing stdof. . .");
  printf("Initializing stdof. . .\n");

  /* Peripheral initialization */
  
  /* Init SocketCan pipe */
  message_log("stdof", "Initilizing remote can. . .");
  printf("Initializing remote can. . .\n");

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

  remote_cantx_pid = create_named_pipe(can_remote_tx, CANTX_REMOTE_NAME);

  if(remote_cantx_pid == -1)
  {
    close(can_remote_tx[0]);
    close(can_remote_tx[1]);

    message_log("Init Remote Can Tx", strerror(errno));
    perror("Init Remote Can Tx");
  }
  else if(remote_cantx_pid == 0)
    manage_remote_can(can_remote_tx, CANTX_REMOTE_NAME, 0);
  else if(remote_cantx_pid > 0)
  {
    //close unused read end
    close(can_remote_tx[0]);
  
    exec_can_pid = fork();
    if(exec_can_pid == -1)
    {
      message_log("Start can connection\n", strerror(errno));
      perror("Start can connection\n");
    }
    else if(exec_can_pid == 0)
    {
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
    pid = fork();

    if(pid == -1)
    {
      message_log("Remote Joystick", strerror(errno));
      perror("Remote Joystick");
    }
    else if(pid == 0)
      manage_remote_joystick();
    
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

  /* Init RS232 Interface */
  rs232_device = com_open("/dev/ttyO2", 115200, 'N', 8, 1);
   
  if(rs232_device == -1)
  {
    message_log("com_open", strerror(errno));
    perror("com_open");
  }
  else
  {
    message_log("com_open", "Init RS232\t[OK]");
    printf("Init RS232\t[OK]\n");
  }
  
  /* Init Network Client */  
  if(net_init_client(&socket_client, &client_addr, "192.168.178.21", 9010) == -1)
  {
    message_log("net_init_client", strerror(errno));
    perror("net_init_client");
  }
  else
  {
    message_log("net_init", "Init Net\t[OK]");
    printf("Init Net Client\t[OK]\n");
  }

  /* Init Network Server */
  if(net_init_server(&socket_server, &server_address, "192.168.178.21",9011) == -1)
  {
    message_log("net_init_server", strerror(errno));
    perror("net_init_server");
  }
  else
  {
    message_log("net_init", "Init Net\t[OK]");
    printf("Init Net Server\t[OK]\n");
  }

  message_log("stdof", "Run main program. . .");
  printf("Run main program. . .\n");

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

    if(rs232_device > 0)
    {
      FD_SET(rs232_device, &rd);
      nfds = max(nfds, rs232_device);
    }
	
    if(socket_server > 0)
    {
      FD_SET(socket_server, &rd);
      nfds = max(nfds, socket_server);
    }
	
    select_result = select(nfds + 1, &rd, NULL, NULL, NULL);

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
/*    if(can_remote_rx[0] > 0)
    {
      if(FD_ISSET(can_remote_rx[0], &rd))
      {
        bytes_read = read(can_remote_rx[0], &frame, sizeof(struct can_frame));
        
        if(bytes_read < 0)
          perror("Can Remote Rx");
        else
        {
          //printf("Received from can:[%x] [%i]", frame.can_id, frame.can_dlc);
        
          //for(can_count = 0; can_count < frame.can_dlc; can_count++)
          //{
          //  printf("[%x]", frame.data[can_count]);
          //}

          //printf("\n");
         
          if(socket_client != -1)
            sendto(socket_client, &frame, sizeof(struct can_frame), 0,(struct sockaddr *)&client_addr, sizeof(client_addr));
        }
      }
    }*/

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
 
        if( rs232_device > 0)
        {
          //sprintf(rs232_buffer, "%x%x%x%x%d%d%d%d%d", 0x24, 0x06, 0x00, 0x05, jse.stick_x, jse.stick_y, jse.stick_z, jse.button[0], jse.button[1]);
	  rs232_buffer[0] = 0x24;
	  rs232_buffer[1] = (0x08 << 3) | 0x06;
	  rs232_buffer[2] = 0x00;
          rs232_buffer[3] = jse.stick_x >> 8;
	  rs232_buffer[4] = jse.stick_x;
          rs232_buffer[5] = jse.stick_y >> 8;
	  rs232_buffer[6] = jse.stick_y;
          rs232_buffer[7] = jse.stick_z >> 8;
          rs232_buffer[8] = jse.stick_z;
	  rs232_buffer[9] = jse.button[0];
	  rs232_buffer[10] = jse.button[1];
		  
	  //printf("%s\n", rs232_buffer);
          //can_frame_to_write.can_id = 0x632;
          //can_frame_to_write.can_dlc = bytes_read;
          //memcpy(can_frame_to_write.data, rs232_buffer, can_frame_to_write.can_dlc);
          //write(socket_can, &can_frame_to_write, sizeof(can_frame_to_write));
          write(rs232_device, rs232_buffer, 12);
        }
        //printf("X: %d\tY: %d\tZ: %d\nbutton1: %d\tbutton2: %d\n", jse.stick_x, jse.stick_y, jse.stick_z, jse.button[0], jse.button[1]);

        continue;
      }//end if(joy_local > 0)
    }
	
    /* Redirect stdof message */
    if(rs232_device > 0)
    {
      if(FD_ISSET(rs232_device, &rd))
      {
        bytes_read = read(rs232_device, rs232_buffer, 255);

        if(bytes_read > 0)
        {
         // manage_stdof_message();
		  
          continue;
        }
      }
    }

    if(socket_server > 0)
    {
      if(FD_ISSET(socket_server, &rd))
      {
        //bytes_read = read(socket_server, &frame, sizeof(struct can_frame));
        bytes_read = recvfrom(socket_server, &frame, sizeof(struct can_frame), 0,(struct sockaddr *) &sender_address, &sender_length);
 
        if(bytes_read > 0)
        {
          //printf("Recived from eth0: [%x] [%i]", frame.can_id, frame.can_dlc);
          /*for(can_count = 0; can_count < frame.can_dlc; can_count++)
            printf("[%x]", frame.data[can_count]);

          printf("\n");*/
        }
      }
    } // end if(socket_server > 0)
  } // end while(!= done)

  return 0;
}

int create_named_pipe(int *pipefd, const char *file_name)
{
  int pid = -1;

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

void manage_remote_can(int *pipefd, const char *can_path, unsigned char direction)
{
  int pipe_file = -1;
  int bytes_write;
  struct can_frame frame;

  //close unused pipe
  if(direction == 1)  //read from file and write on pipe
  {
    close(pipefd[0]);
    pipe_file = open(can_path, O_RDONLY);
  } 
  else
  {
    close(pipefd[1]);  //read from pipe and write on file
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

int monitor_input_devices(struct udev_monitor *udev_mon, 
                          const char *joystick_path, int *joy_local)
{
  struct udev_device *udev_dev;
  const char *udev_node;
  const char *udev_subsystem;
  int pid = -1;

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

          // send joystick command over network
          pid = fork();

          if(pid == -1)
          {
            message_log("Init Remote Joystick", strerror(errno));
            perror("Init Remote Joystick");
          }
         else if(pid == 0)
            manage_remote_joystick();
         
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
        bytes_sent = segway_motion_set(socket_can, -jse->stick_y, jse->stick_x, joy_max_value);
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

void manage_remote_joystick()
{
  int exec_return = -1;

  printf("Send joystick messages....\n");
  fflush(stdout);

  exec_return = execlp("./joystick.sh", "joystick.sh", (char *) 0, (char *) 0);

  if(exec_return == -1)
  {
    perror("exec:");
  }

  printf("Stop sending joystick messages!\n");
  fflush(stdout);

  exit(0);
}

/*void redirect_stdof_message()
{
  rs232_data_read = 0;

  while(rs232_data_read < bytes_read)
          { 
            switch(rs232_state)
            {
              case 0: // waiting for the header
	        printf("%d", rs232_state);
		fflush(stdout);

                if(rs232_buffer[rs232_data_read] == header)
                  rs232_state++;

                rs232_data_read++;
 
                if(rs232_data_read >= bytes_read)
	          break;

              case 1: //read first address byte
                printf("%d", rs232_state);
		fflush(stdout);

                can_frame_to_write.can_id = ((__u32)rs232_buffer[rs232_data_read] & 0x0007) << 8;
                rs232_state++;
                rs232_data_read++;
                
                if(rs232_data_read >= bytes_read)
                  break;

              case 2: //read second address byte
                printf("%d", rs232_state);
		fflush(stdout);

                can_frame_to_write.can_id |= rs232_buffer[rs232_data_read];
                rs232_state++;
                rs232_data_read++;
               
                if(rs232_data_read >= bytes_read) 
                  break;
 
              case 3: //read lenght byte
		printf("%d", rs232_state);
		fflush(stdout);

                can_frame_to_write.can_dlc = rs232_buffer[rs232_data_read];

                if(can_frame_to_write.can_dlc == 0)
                  rs232_state = 0;
                else
                  rs232_state++;

                rs232_data_read++;

                if(rs232_data_read >= bytes_read)
                  break;

              case 4: //read data and send over can bus
		printf("%d", rs232_state);
		fflush(stdout);

                if((bytes_read - rs232_data_read) <= can_frame_to_write.can_dlc)
                {
                  memcpy(can_frame_to_write.data, &rs232_buffer[rs232_data_read], bytes_read - rs232_data_read);
                  rs232_data_count = bytes_read - rs232_data_read;
                  rs232_data_read = bytes_read;
                }
                else
                {
                  memcpy(can_frame_to_write.data, &rs232_buffer[rs232_data_read], can_frame_to_write.can_dlc);
                  rs232_data_count += can_frame_to_write.can_dlc;
                  rs232_data_read += can_frame_to_write.can_dlc;
                }

                if(rs232_data_count == can_frame_to_write.can_dlc)
	        {
                  printf(":Write:%s:%d\n",can_frame_to_write.data, rs232_data_count);
                  fflush(stdout);

                  rs232_state = 0;
                  rs232_data_count = 0;

                  write(socket_can, &can_frame_to_write, sizeof(can_frame_to_write));
                }
                break;

              default:
                rs232_state = 0;
                rs232_data_count = 0;
                break;
            }
          }
          //can_frame_to_write.can_id = 0x632;
          //can_frame_to_write.can_dlc = bytes_read;
          //memcpy(can_frame_to_write.data, rs232_buffer, can_frame_to_write.can_dlc);
          //write(socket_can, &can_frame_to_write, sizeof(can_frame_to_write));
        }

    }
}*/

int net_init_client(int *socket_net, struct sockaddr_in *socket_addr, const char *addr, int portnumber)
{
  *socket_net = socket(AF_INET, SOCK_DGRAM, 0);

  if(*socket_net < 0)
    return -1;

  bzero((char *) socket_addr, sizeof(*socket_addr));
  socket_addr->sin_family = AF_INET;
  socket_addr->sin_port = htons(portnumber);

  if(inet_aton(addr, &socket_addr->sin_addr) == 0)
    return -1;

  return 0;
}

int net_init_server(int *socket_net, struct sockaddr_in *socket_addr, const char *addr, int portnumber)
{
  *socket_net = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  if(*socket_net < 0)
  {
    printf("socket error\n");
    return -1;
  }

  bzero((char *) socket_addr, sizeof(*socket_addr));
  socket_addr->sin_family = AF_INET;
  socket_addr->sin_port = htons(portnumber);
  socket_addr->sin_addr.s_addr = htonl(INADDR_ANY);

  if(bind(*socket_net, (struct sockaddr *)socket_addr, sizeof(*socket_addr)) == -1)
    return -1;

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
