#ifndef __RS232_H_
#define __RS232_H_

extern int com_open(char *device_name, __u32 rate, char parity, int data_bits, int stop_bits);
extern void flush_device_input(int *device);
extern void flush_device_output(int *device);

#endif
