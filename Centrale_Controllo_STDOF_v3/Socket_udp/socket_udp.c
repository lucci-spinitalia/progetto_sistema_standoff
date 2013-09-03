#include <stdio.h>
#include <errno.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <net/if.h>
#include <sys/socket.h>
#include <strings.h>

#include "socket_udp.h"

#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

int init_server(int *net_socket, struct sockaddr_in *socket_addr, int portnumber)
{
  *net_socket = socket(AF_INET, SOCK_DGRAM,0);
  
  if(*net_socket < 0)
    return -1;
  
  bzero((char *)socket_addr, sizeof(*socket_addr));
  socket_addr->sin_family = AF_INET;
  socket_addr->sin_port = htons(portnumber);
  socket_addr->sin_addr.s_addr = htonl(INADDR_ANY);

  if(bind(*net_socket, (struct sockaddr *)socket_addr, sizeof(*socket_addr)) == -1)
    return -1;

  return 0;
}

int init_client(int *net_socket, struct sockaddr_in *socket_addr, const char *addr, int portnumber)
{
  *net_socket = socket(AF_INET, SOCK_DGRAM, 0);

  if(*net_socket < 0)
    return -1;

  bzero((char *)socket_addr, sizeof(*socket_addr));
  socket_addr->sin_family = AF_INET;
  socket_addr->sin_addr.s_addr = inet_addr(addr);
  socket_addr->sin_port = htons(portnumber);

  return 0;
}

/*
int main()
{
  // Server
  int socket_server = -1;
  struct sockaddr_in server_address;
  struct sockaddr_in sender_address;
  socklen_t sender_length = sizeof(sender_address);
  char net_buffer[255];
  
  // Client
  int socket_client = -1;
  struct sockaddr_in client_address;
  socklen_t client_length = sizeof(client_address);

  int select_result = -1;
  int nfds = 0;
  fd_set rd;

  int bytes_read;


  printf("------------ server_udp.o ------------\n");
  printf(" Start server on local host port 3200\n");
  printf(" and send standard message to the\n");
  printf(" Client's port 32000 on packet received\n");
  printf("--------------------------------------\n\n");
 
  if(init_server(&socket_server, &server_address, 32000) == -1)
    perror("init_server");
  else
    printf("Server started. . .\n");

  while(1)
  {
    FD_ZERO(&rd);

    if(socket_server > 0)
    {
      FD_SET(socket_server, &rd);
      nfds = max(nfds, socket_server);
    }

    select_result = select(nfds + 1, &rd, NULL, NULL, NULL);

    if((select_result == -1) && (errno != EAGAIN))
    {
      perror("select");
      return -1;
    }

    if(socket_server > 0)
    {
      if(FD_ISSET(socket_server, &rd))
      {
        bytes_read = recvfrom(socket_server, net_buffer, sizeof(net_buffer), 0, (struct sockaddr *)&sender_address, &sender_length);
        net_buffer[bytes_read] = '\0';

        if(bytes_read > 0)
        {
          printf("Received packet from: %s:%d\nData: %s\n\n", inet_ntoa(sender_address.sin_addr), ntohs(sender_address.sin_port), net_buffer);

          if(init_client(&socket_client, &client_address, inet_ntoa(sender_address.sin_addr), 32000) == -1)
            perror("init_server");
          else
            printf("Client Started. . .\n");
          
          sprintf(net_buffer, "Programma di prova\n");
          if(sendto(socket_client, net_buffer, sizeof(net_buffer), 0, (struct sockaddr *)&client_address, client_length) == -1)
            perror("sendto:");
        }
      }
    }
  } //end while
}*/
