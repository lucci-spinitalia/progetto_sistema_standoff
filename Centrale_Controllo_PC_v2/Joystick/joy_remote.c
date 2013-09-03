/* Sample Joystick client */

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "joystick.h"

int main(int argc, char**argv)
{
  int sockfd, n;
  struct sockaddr_in servaddr, cliaddr;
  char sendline[1000];
  char recvline[1000];

  if(argc != 3)  
  {
    printf("usage: %s <IP address> <Port>\n", argv[0]);
    exit(1);
  }
  
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);

  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(argv[1]);
  servaddr.sin_port = htons(atoi(argv[2]));

  struct wwvi_js_event jse;
  int joystick = -1;

  memset(&jse, 0, sizeof(struct wwvi_js_event));
  joystick = open_joystick("/dev/input/js0");

  if(joystick < 0)
    perror("joystick");
  else
    printf("joystick found");

  jse.button[1] = 1;
  jse.stick_x = 3000;
  jse.stick_z = 35000;

  sendto(sockfd, &jse, sizeof(jse), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
/*  while(fgets(sendline, 10000, stdin) != NULL)
  {
    sendto(sockfd, sendline, strlen(sendline), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));

    n = recvfrom(sockfd, recvline, 10000, 0, NULL, NULL);
    recvline[n] = 0;
    fputs(recvline, stdout);
  }
*/
  return 0;
}
