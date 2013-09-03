/* Segway server */

#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <error.h>

#include "segway_udp.h"

int main(int argc, char**argv)
{
  int sockfd, n, i;
  struct sockaddr_in servaddr, cliaddr;
  socklen_t len;
  struct udp_frame mesg;
  __u8 message[80];

  bzero(message, sizeof(message));
  message[35] = 0x3;
  tk_crc_compute_byte_buffer_crc(message, sizeof(message));

  sockfd = socket(AF_INET, SOCK_DGRAM, 0);

  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servaddr.sin_port = htons(32002);

  if(bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) == -1)
    perror("bind");

  printf("Segway server started. . .\n");

  tk_crc_initialize();

  for(;;)
  {
    fflush(stdout);
    len = sizeof(cliaddr);
    n = recvfrom(sockfd, &mesg, sizeof(struct udp_frame), 0, (struct sockaddr *)&cliaddr, &len);
//    n = recvfrom(sockfd, &message, 1000, 0, (struct sockaddr *)&cliaddr, &len);
    
    if(n <= 0)
      perror("recvfrom");

    if(tk_crc_byte_buffer_crc_is_valid(mesg.frame, sizeof(mesg.frame)) == 0)
      printf("Bad crc\n");

    switch(__builtin_bswap32(mesg.param.udp_id << 16))
    {
      case 0x500:
        if(sendto(sockfd, message, sizeof(message), 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr)) == -1)
          perror("sendto");

      /*  //printf("Motion message for segway:\tVel:%l\tYaw:%l", convert_to); 
        for(i = 0; i < sizeof(mesg.frame); i++)
          printf("[%x]", mesg.frame[i]);
  
        printf("\n");
        fflush(stdout);*/
        
         break;

      case 0x501:
        
        switch(mesg.param.data[3])
        {
          case 0x00:
            if(sendto(sockfd, message, sizeof(message), 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr)) == -1)
              perror("sendto");
            break;

          case 0x15:
            switch(mesg.param.data[7])
            {
              case 0x04:
                printf("---->Requested Standby mode \n");
                message[35] = 3;
                if(sendto(sockfd, message, sizeof(message), 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr)) == -1)
                  perror("sendto");
                break;
    
              case 0x05:
                printf("---->Requested Tractor mode \n");
                if(sendto(sockfd, message, sizeof(message), 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr)) == -1)
                  perror("sendto");
                message[35] = 4;
                break;          
            }
            break;
          default:
            if(sendto(sockfd, message, sizeof(message), 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr)) == -1)
              perror("sendto");

            printf("Configuration message for segway:"); 
            for(i = 0; i < sizeof(mesg.frame); i++)
              printf("[%x]", mesg.frame[i]);
  
            printf("\n");
            fflush(stdout);
        }
        break;

      default:
        printf("Message unkown\n");

        printf("Motion message for segway: ");
  
        for(i = 0; i < sizeof(mesg.frame); i++)
          printf("[%x]", mesg.frame[i]);

        printf("\n");
        fflush(stdout);

        break;
    }
  }
}
