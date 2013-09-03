#ifndef _SOCKET_UDP_H
#define _SOCKET_UDP_H

/* Protype */
extern int init_server(int *, struct sockaddr_in *, int);
extern int init_client(int *, struct sockaddr_in *, const char *, int);
#endif
