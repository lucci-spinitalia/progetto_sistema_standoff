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

#include "../Segway/segway.h"
#include "../Joystick/joystick.h"
#include "../Rs232/rs232.h"

// PF stands for Protocol Family
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

#define JOY_MAX_VALUE 32767
#define UDEV_TIMEOUT_SEC 0
#define UDEV_TIMEOUT_USEC 100000
#define LOG_FILE "/var/log/stdof_log.txt"

#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

/*************************************************************
 * loopback: 0 = disabled, 1 = enabled
 * recv_own_msgs: 0 = disabled, 1 = enabled
 *
 */
int can_init(int *socket_can, struct sockaddr_can *addr, struct ifreq *ifr, int loopback, int recv_own_msgs)
{
  // Create the socket
  *socket_can = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  if(*socket_can == -1)
    return -1;

  // Filter rules
  /*struct can_filter rfilter[2];

  rfilter[0].can_id = 0x123;
  rfilter[0].can_mask = CAN_SFF_MASK;
  rfilter[1].can_id = 0x200;
  rfilter[1].can_mask = 0x700;

  setsockopt(skt, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));*/

  // Set loopback option
  setsockopt(*socket_can, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

  // Set if receive own message or not
  setsockopt(*socket_can, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));

  // Locate the interface you wish to use
  strcpy(ifr->ifr_name, "can0");
  ioctl(*socket_can, SIOCGIFINDEX, ifr);

  // Select that CAN interface, and bind the socket to it.
  addr->can_family = AF_CAN;
  addr->can_ifindex = ifr->ifr_ifindex;

  if(bind(*socket_can, (struct sockaddr*)addr, sizeof(struct sockaddr)) == -1)
    return -1;

  return 0;
}

int network_init(int *socket_net, struct sockaddr_in *addr, int port_number)
{
  *socket_net = socket(AF_INET, SOCK_STREAM, 0);

  if(socket_net < 0)
    return -1;

  bzero((char *)&addr, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port_number);

  if(bind(socket_net, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    return -1;

  listen(sockfd, 5);

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

int main()
{
  /* SocketCan interface */
  struct sockaddr_can addr;
  struct can_frame frame;
  struct ifreq ifr;
  int socket_can;


  /* SocketNet interface */
  int socket_net;
  int new_socket_net = 0;
  struct socklen_t client_len;
  struct sockaddr_in addr_net;
  struct sockaddr_in client_addr_net;
  struct client_addr;

  /* Joystick interface */
  struct wwvi_js_event jse;
  int dev_joy;

  /* Udev   */
  struct udev *udev;
  struct udev_monitor *udev_mon;
  struct udev_device *udev_dev;
  struct timeval udev_timeout;
  const char *udev_node;
  const char *udev_subsystem;
  int udev_fd;

  /* Segway */
  struct segway_struct segway_status;
  int segway_previouse_state = 0;
  char fault_message[100];
  int fault_return = 0;

  int done = 0;
  int bytes_read;
  int bytes_sent;

  /* Pc interface */
  int rs232_device;
  const unsigned char header = 0x024;
  unsigned char rs232_id;
  unsigned char rs232_state = 0;
  unsigned char rs232_data_count = 0;
  unsigned char rs232_data_read = 0;
  struct can_frame can_frame_to_write;
  unsigned char rs232_buffer[255];
  
  message_log("stdof", "Initializing stdof. . .");
  printf("Initializing stdof. . .\n");

  // Init RS232 Interface
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

  // Init Can Interface
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

  // Init Network Interface
  if(newtwork_init(&socket_net, &addr_net, 9000) == -1)
  {
    message_log("network_init", strerror(errno));
    perror("network_init");

    return 1;
  }
  else
  {
    client_len = sizeof(client_addr_net);

    message_log("network_init", "Init Network\t[OK]");
    printf("Init Network\t[OK]\n");
  }

  // Init Segway
  if(segway_init(socket_can, &segway_status) < 0)
  {
    message_log("segway_init", strerror(errno));
    perror("Segway_init\n");
  }
  else
  {
    message_log("segway_init", "Init Segway\t[OK]");
    printf("Init Segway\t[OK]\n");
  }

  // Init Joystick
  memset(&jse, 0, sizeof(struct wwvi_js_event));
  dev_joy = open_joystick();

  if(dev_joy < 0)
  {
    message_log("open_joystick", strerror(errno));
    perror("open_joystick");
  }
  else
  {
    message_log("open_joystick", "Find Joystick\t[OK]");
    printf("Find Joystick\t[OK]\n");
  }

  /* Monitor for joystick or usb storage device  */
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

    // The segway requirements state that the minimum update frequency have to be 0.5Hz, so
    // the timeout on udev have to be < 2 secs.
    udev_timeout.tv_sec = UDEV_TIMEOUT_SEC;
    udev_timeout.tv_usec = UDEV_TIMEOUT_USEC;

    message_log("udev_new", "Init udev\t[OK]");
    printf("Init udev\t[OK]\n");
  }

  message_log("stdof", "Run main program. . .");
  printf("Run main program. . .\n");

  while(!done)
  {
    int r, nfds = 0;
    fd_set rd, wr, er;

    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);

    if(rs232_device > 0)
    {
      FD_SET(rs232_device, &rd);
      nfds = max(nfds, rs232_device);
    }

    if(socket_can > 0)
    {
      FD_SET(socket_can, &rd);
      nfds = max(nfds, socket_can);
    }

    if(socket_net > 0)
    {
      FD_SET(socket_net, &rd);
      nfds = max(nfds, socket_net);
    }

    if(new_socket_net > 0)
    {
      FD_SET(new_socket_net, &rd);
      nfds = max(nfds, new_socket_net);
    }

    /* Read the usb joystick only if the remote connection is down */
    if(dev_joy > 0 && new_socket_net <= 0)
    {
      FD_SET(dev_joy, &rd);
      nfds = max(nfds, dev_joy);
    }
    
    if(udev_fd > 0)
    {
      FD_SET(udev_fd, &rd);
      nfds = max(nfds, udev_fd);
    }

    r = select(nfds + 1, &rd, NULL, NULL, &udev_timeout);

    if(r == -1 && errno == EAGAIN)
      continue;

    if(r == -1)
    {
      message_log("stdof", strerror(errno));
      perror("main:");

      return 1;
    }

    // Monitor input devices
    if(udev_fd > 0)
    {
      if(FD_ISSET(udev_fd, &rd))
      {
        udev_dev = udev_monitor_receive_device(udev_mon);

        if(udev_dev)
	{
	  udev_node =  udev_device_get_devnode(udev_dev);
	  udev_subsystem = udev_device_get_subsystem(udev_dev);

	  if(udev_node && !strcmp(udev_subsystem, "input")) 
          {
  	    if(!strcmp(udev_node, JOYSTICK_DEVNAME))
	    {
	      if((!strcmp(udev_device_get_action(udev_dev),"add")) && (dev_joy < 0))
	      {
		// add joystick
                dev_joy = open_joystick();

                if(dev_joy < 0)
                  perror("open_joystick");

                FD_SET(dev_joy, &rd);
                nfds = max(nfds, dev_joy);

                message_log("stdof", "Joystick added");
		printf("stdof: Joystick added\n");
	      }
	      else if((!strcmp(udev_device_get_action(udev_dev),"remove")) && (dev_joy > 0))
	      {
		// remove joystick
		FD_CLR(dev_joy, &rd);
		dev_joy = -1;

                message_log("stdof", "Joystick removed");
		printf("stdof: Joystick removed\n");
	      }
     	    }
	  }
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
		continue;
	      }
              else
	      {
                message_log("mount", "Drive mounted");
		printf("mount: Drive mounted\n");
	      }

              if(copy_log_file("/media/usb") < 0)
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

	continue;
      }
    }

    // Manage joystick command
    if(dev_joy > 0)
    {
      if(FD_ISSET(dev_joy, &rd))
      {
        if(get_joystick_status(&dev_joy, &jse) < 0)
	{
          message_log("get_joystick_status", "error");
          printf("get_joystick_status error");
	}

        switch(segway_status.operational_state)
        {
          case CCU_INIT:
            if(segway_previouse_state != segway_status.operational_state)
	    {
              message_log("stdof", "Segway CCU Init");
	      printf("Segway CCU Init\n");

	      segway_previouse_state = segway_status.operational_state;
	    }
            break;

          case PROPULSION_INIT:
            if(segway_previouse_state != segway_status.operational_state)
	    {
              message_log("stdof", "Segway Propulsion Init");
	      printf("Segway Propulsion Init\n");

	      segway_previouse_state = segway_status.operational_state;
	    }
            break;

          case CHECK_STARTUP:
            if(segway_previouse_state != segway_status.operational_state)
	    {
              message_log("stdof", "Segway Check Startup Issue");
	      printf("Segway Check Startup Issue\n");

	      segway_previouse_state = segway_status.operational_state;
	    }
            break;

          case SEGWAY_STANDBY:  //standby mode
	    if(segway_previouse_state != segway_status.operational_state)
	    {
              message_log("stdof", "Segway in Standby Mode");
	      printf("Segway in Standby Mode\n");

	      segway_previouse_state = segway_status.operational_state;
	    }

	    // I can only send a tractore request
	    if(jse.button[0])
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
	    if(segway_previouse_state != segway_status.operational_state)
	    {
              message_log("stdof", "Segway in Tractor Mode");
	      printf("Segway in Tractor Mode\n");

	      segway_previouse_state = segway_status.operational_state;
	    }

            if(jse.button[1])
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
            
            if(jse.stick_z > 0)
              bytes_sent = segway_motion_set(socket_can, jse.stick_y, jse.stick_x, JOY_MAX_VALUE);
            else
	      bytes_sent = segway_motion_set(socket_can, 0, 0, JOY_MAX_VALUE);
	      
            if(bytes_sent < 0)
	    {
              message_log("segway_motion_set", strerror(errno));
              perror("segway_motion_set");
	    }

            // The segway requirements state that the minimum update frequency have to be 0.5Hz, so
            // the timeout on udev have to be < 2 secs.
            udev_timeout.tv_sec = UDEV_TIMEOUT_SEC;
            udev_timeout.tv_usec = UDEV_TIMEOUT_USEC;

            break;

          case DISABLE_POWER:
            if(segway_previouse_state != segway_status.operational_state)
	    {
              message_log("stdof", "Segway Disable Power");
	      printf("Segway Disable Power\n");

	      segway_previouse_state = segway_status.operational_state;
	    }
            break;

          default:
            if(segway_previouse_state != segway_status.operational_state)
	    {
              message_log("stdof", "Segway Uknown State");
	      printf("Segway Unknown State\n");

	      segway_previouse_state = segway_status.operational_state;
	    }
            break;
        }
	continue;
      }
    }

    /* Manage RS232 */
    if(rs232_device > 0)
    {
      if(FD_ISSET(rs232_device, &rd))
      {
        bytes_read = read(rs232_device, rs232_buffer, 255);

        if(bytes_read > 0)
        {
          rs232_buffer[bytes_read] = 0;

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
        continue;
      }
    }

    /* Manage network connection request */
    if(socket_net > 0)
    {
      if(FD_ISSET(socket_net, &rd)
      {
        new_socket_net = accept(socket_net, (struct sockaddr *) client_addr_net, &client_addr_net);
       
        if(new_socket_net < 0) 
        {
          message_log("accept", "Connection error");
          perrror("socket_net");
        }
      }
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

          return 1;
        }

        if(bytes_read < sizeof(struct can_frame))
        {
          message_log("can raw socket read", "Incomplete CAN frame");
          fprintf(stderr, "read: incomplete CAN frame\n");

          return 1;
        }
      
        switch(frame.can_id)
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
            segway_config_update(&frame);
	    
	    /* do
            {
              fault_return = segway_config_decode_arch_fault(segway_status.fault_status_word1, fault_message);
	      
              if(fault_return >= 0)
	      {
                message_log("segway_config_fault:", fault_message);
                printf("Fault: %s\n", fault_message);
	      }
            } while(fault_return > 0);
	    
            do
            {
              fault_return = segway_config_decode_critical_fault(segway_status.fault_status_word1, fault_message);

              if(fault_return >= 0)
	      {
                message_log("segway_config_fault:", fault_message);
                printf("Fault: %s\n", fault_message);
	      }
            } while(fault_return > 0);

            do
            {
              fault_return = segway_config_decode_comm_fault(segway_status.fault_status_word2, fault_message);

              if(fault_return >= 0)
	      {
                message_log("segway_config_fault:", fault_message);
                printf("Fault: %s\n", fault_message);
	      }
            } while(fault_return > 0);

            do
            {
              fault_return = segway_config_decode_internal_fault(segway_status.fault_status_word2, fault_message);

              if(fault_return >= 0)
	      {
                message_log("segway_config_fault:", fault_message);
                printf("Fault: %s\n", fault_message);
	      }
            } while(fault_return > 0);

            do
            {
              fault_return = segway_config_decode_sensors_fault(segway_status.fault_status_word3, fault_message);

              if(fault_return >= 0)
	      {
                message_log("segway_config_fault:", fault_message);
                printf("Fault: %s\n", fault_message);
	      }
            } while(fault_return > 0);

            do
            {
              fault_return = segway_config_decode_bsa_fault(segway_status.fault_status_word3, fault_message);

              if(fault_return >= 0)
	      {
                message_log("segway_config_fault:", fault_message);
                printf("Fault: %s\n", fault_message);
	      }
            } while(fault_return > 0);

            do
            {
              fault_return = segway_config_decode_mcu_fault(segway_status.fault_status_word4, fault_message);

              if(fault_return >= 0)
	      {
                message_log("segway_config_fault:", fault_message);
                printf("Fault: %s\n", fault_message);
	      }
	    } while(fault_return > 0);

            do
            {
              fault_return = segway_config_decode_mcu_message(segway_status.mcu_0_fault_status, fault_message);

              if(fault_return >= 0)
	      {
                message_log("segway mcu0", fault_message);
                printf("segway mcu0: %s\n", fault_message);
	      }
	    } while(fault_return > 0);

            do
            {
              fault_return = segway_config_decode_mcu_message(segway_status.mcu_1_fault_status, fault_message);

              if(fault_return >= 0)
	      {
                message_log("segway mcu1", fault_message);
                printf("segway mcu1: %s\n", fault_message);
	      }
	    } while(fault_return > 0);

            do
            {
              fault_return = segway_config_decode_mcu_message(segway_status.mcu_2_fault_status, fault_message);

              if(fault_return >= 0)
	      {
                message_log("segway mcu2", fault_message);
                printf("segway mcu2: %s\n", fault_message);
	      }
	    } while(fault_return > 0);

            do
            {
              fault_return = segway_config_decode_mcu_message(segway_status.mcu_3_fault_status, fault_message);

              if(fault_return >= 0)
	      {
                message_log("segway mcu3", fault_message);
                printf("segway mcu3: %s\n", fault_message);
	      }
	    } while(fault_return > 0);
            printf("Operational state: %08lx\n", segway_status.operational_state);
            printf("Linear velocity: %08lx\n", segway_status.linear_vel_mps);
            printf("Linear position: %08lx\n", segway_status.linear_pos_m);
            printf("Front Batt1 SOC: %08lx\n", segway_status.front_base_batt_1_soc);
            printf("Front Batt2 SOC: %08lx\n", segway_status.front_base_batt_2_soc);
            printf("Rear Batt1 SOC: %08lx\n", segway_status.rear_base_batt_1_soc);
            printf("Rear Batt2 SOC: %08lx\n", segway_status.rear_base_batt_2_soc);
            printf("Front Batt1 Temp: %08lx\n", segway_status.front_base_batt_1_temp_degC);
            printf("Front Batt2 Temp: %08lx\n", segway_status.front_base_batt_2_temp_degC);
            printf("Rear Batt1 Temp: %08lx\n", segway_status.rear_base_batt_1_temp_degC);
            printf("Rear Batt2 Temp: %08lx\n", segway_status.rear_base_batt_2_temp_degC);*/
            break;

          default:
            // Send the can message with the header to uart interface
            write(rs232_device, &header, 1);
            rs232_id = frame.can_id >> 8;
            write(rs232_device, &(rs232_id), 1);
            write(rs232_device, &(frame.can_id), 1);
            write(rs232_device, &(frame.can_dlc), 1);
            write(rs232_device, frame.data, frame.can_dlc);

            break;
	}
    // get interface name of the received CAN frame
//  ifr.ifr_ifindex = addr.can_ifindex;
//ioctl(socket_can, SIOCGIFNAME, &ifr);
//printf("Received a CAN frame from interface %s\n", ifr.ifr_name);
        continue;
      }
    }

    if(segway_status.operational_state == SEGWAY_TRACTOR)
    {
      // If z axis is > 0 and there's a joystick attached
      if((jse.stick_z > 0) && (dev_joy > 0))
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
    udev_timeout.tv_sec = UDEV_TIMEOUT_SEC;
    udev_timeout.tv_usec = UDEV_TIMEOUT_USEC;
  }

  return 0;
}
