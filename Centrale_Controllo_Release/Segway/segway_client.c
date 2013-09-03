/* Segway client */

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <sys/select.h>
#include <unistd.h>

#include "segway_udp.h"
#include "segway_config.h"

int main(int argc, char**argv)
{
  int sockfd, n, i, pid;
  struct sockaddr_in servaddr, cliaddr;
  struct udp_frame frame;
  union segway_union segway_status;
  __u8 message[1000];
  unsigned char command_sent = 0;

  int select_result = -1;
  int nfds = 0;
  fd_set rd;
  struct timeval select_timeout;

  if(argc != 3)  
  {
    printf("usage: %s <IP address> <Port>\n", argv[0]);
    exit(1);
  }
  
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);

  if(sockfd < 0)
  {
    perror("socket");
    return -1;
  }

  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(argv[1]);
  servaddr.sin_port = htons(atoi(argv[2]));

  printf("Client started. . .\n");

  tk_crc_initialize();

  pid = fork();

  if(pid == 0)
  {
    segway_configure_feedback_bitmap1(sockfd, &servaddr,  0);
    segway_configure_feedback_bitmap2(sockfd, &servaddr,  0);
    segway_configure_feedback_bitmap3(sockfd, &servaddr,  0);

    //segway_configure_reset_params_to_default(sockfd, &servaddr, 0);
    if(segway_init(sockfd, &servaddr, &segway_status) == -1)
      printf("Segway not found from client\n");
    else
      printf("Segway found\n");

    exit(0);
  }
  else
  {
    for(;;)
    {
      FD_ZERO(&rd);
      select_timeout.tv_sec = 0;
      select_timeout.tv_usec = 100000;
 
      FD_SET(sockfd, &rd);
  
      select_result = select(sockfd + 1, &rd, NULL, NULL, NULL);

      if(select_result == -1)
        return -1;

      if(FD_ISSET(sockfd, &rd))
      {
        n = recvfrom(sockfd, message, 1000, 0, NULL, NULL);
        printf("Message from segway: ");

        for(i = 0; i < n; i++)
          printf("[%x]", message[i]);

        printf("\n");

        if(tk_crc_byte_buffer_crc_is_valid(message, n) == 0)
          printf("Bad crc\n");

        printf("Operational state: %f\n", convert_to_float(segway_status.list.operational_state));
        segway_config_update(message, &segway_status);
        printf("Operational state: %f\n", convert_to_float(segway_status.list.operational_state));
      }
    }
  }

/*  printf("Send sandard configuration command (pag. 14)\n");
  if(segway_configure_operational_mode(sockfd, &servaddr, 5) == -1)
    perror("segway_configure_operational_mode");
*/

  return 0;
}
