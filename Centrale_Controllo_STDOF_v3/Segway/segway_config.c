#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include "segway_config.h"

#define SEGWAY_PARAM 85

__u32 bitmap1;
__u32 bitmap2;
__u32 bitmap3;

void *bitmap_ptr_all[SEGWAY_PARAM];
int *bitmap[SEGWAY_PARAM]; //array of pointer to update the segway status struct
int bitmap_count;

__u32 convert_to_ieee754(float value)
{
  return (*((__u32*)&value));
}

float convert_to_float(__u32 value)
{
  return (*((float *)&value));
}

/**************************************************************************************
 * -1: nothing to read
 *  0: one message returned
 *  1: one message returned and one more to read
 *****************************************************************************************/
int segway_config_decode_arch_fault(__u32 fault_status_word1, char *fault_message)
{
  static int i = 0;
  __u32 fault;

  fault_status_word1 = (fault_status_word1 >> FSW_ARCH_FAULTS_SHIFT) & FSW_ARCH_FAULTS_MASK;

  if(fault_status_word1 == 0x00)
    return -1;

  while(i < 32)
  {
    fault = fault_status_word1 & (0x00000001 << i);
    
    i++;

    if(fault)
      break;
  }

  switch(fault)
  {
    case 0x000:
      return -1;
      break;

    case 0x001:
      strcpy(fault_message, "ARCHITECT_FAULT_SPI_RECEIVE");
      break;

    case 0x002:
      strcpy(fault_message, "ARCHITECT_FAULT_SPI_TRANSMIT");
      break;

    case 0x004:
      strcpy(fault_message, "ARCHITECT_FAULT_SPI_RECEIVE_OVERRUN");
      break;

    case 0x008:
      strcpy(fault_message, "ARCHITECT_FAULT_RX_BUFFER_OVERRUN");
      break;

    case 0x010:
      strcpy(fault_message, "ARCHITECT_FAULT_COMMANDED_SAFETY_SHUTDOWN");
      break;

    case 0x020:
      strcpy(fault_message, "ARCHITECT_FAULT_COMMANDED_DISABLE");
      break;

    case 0x040:
      strcpy(fault_message, "ARCHITECT_FAULT_KILL_SWITCH_ACTIVE");
      break;

    case 0x080:
      strcpy(fault_message, "ARCHITECT_FAULT_FRAM_CONFIG_INIT_FAILED");
      break;

    case 0x100:
      strcpy(fault_message, "ARCHITECT_FAULT_FRAM_CONFIG_SET_FAILED");
      break;

    case 0x200:
      strcpy(fault_message, "ARCHITECT_FAUILT_BAD_MODEL_IDENTIFIER");
      break;

    case 0x400:
      strcpy(fault_message, "ARCHITECT_FAULT_BAD_CCU_HW_REV");
      break;

    default:
      return -1;
      break;
  }
 
  if(i == 32)
    i = 0;

  if(fault_status_word1 & (0xFFFFFFFF << i))
    return 1;
  else
    i = 0;

  return 0;
}

int segway_config_decode_critical_fault(__u32 fault_status_word1, char *fault_message)
{
  static int i = 0;
  __u32 fault;

  fault_status_word1 = (fault_status_word1 >> FSW_CRITICAL_FAULTS_SHIFT) & FSW_CRITICAL_FAULTS_MASK;

  if(fault_status_word1 == 0x00)
    return -1;

  while(i < 32)
  {
    fault = fault_status_word1 & (0x00000001 << i);
    
    i++;

    if(fault)
      break;
  }

  switch(fault)
  {
    case 0x000:
      return -1;
      break;

    case 0x001:
      strcpy(fault_message, "CRITICAL_FAULT_INIT");
      break;

    case 0x002:
      strcpy(fault_message, "CRITICAL_FAULT_INIT_UIP_COMM");
      break;

    case 0x004:
      strcpy(fault_message, "CRITICAL_FAULT_INIT_PROPULSION");
      break;

    case 0x008:
      strcpy(fault_message, "CRITICAL_FAULT_INIT_TIMEOUT");
      break;

    case 0x010:
      strcpy(fault_message, "CRITICAL_FAULT_FORW_SPEED_LIMITER_HAZARD");
      break;

    case 0x020:
      strcpy(fault_message, "CRITICAL_FAULT_AFT_SPEED_LIMITER_HAZARD");
      break;

    case 0x040:
      strcpy(fault_message, "CRITICAL_FAULT_CHECK_STARTUP");
      break;

    case 0x080:
      strcpy(fault_message, "CRITICAL_FAULT_APP_VELOCITY_CTL_FAILED");
      break;

    case 0x100:
      strcpy(fault_message, "CRITICAL_FAULT_APP_POSITION_CTL_FAILED");
      break;

    case 0x200:
      strcpy(fault_message, "CRITICAL_FAULT_ABB_SHUTDOWN");
      break;

    default:
      return -1;
      break;
  }

  if(i == 32)
    i = 0;

  if(fault_status_word1 & (0xFFFFFFFF << i))
    return 1;
  else
    i = 0;

  return 0;
}

int segway_config_decode_comm_fault(__u32 fault_status_word2, char *fault_message)
{
  static int i = 0;
  __u32 fault;

  fault_status_word2 = (fault_status_word2 >> FSW_COMM_FAULTS_SHIFT) & FSW_COMM_FAULTS_MASK;

  if(fault_status_word2 == 0x00)
    return -1;

  while(i < 32)
  {
    fault = fault_status_word2 & (0x00000001 << i);
    
    i++;

    if(fault)
      break;
  }

  switch(fault)
  {
    case 0x000:
      return -1;
      break;

    case 0x001:
      strcpy(fault_message, "COMM_FAULT_UIP_MISSING_UIP_DATA");
      break;

    case 0x002:
      strcpy(fault_message, "COMM_FAULT_UIP_UNKNOWN_MESSAGE_RECEIVED");
      break;

    case 0x004:
      strcpy(fault_message, "COMM_FAULT_UIP_BAD_CHECKSUM");
      break;

    case 0x008:
      strcpy(fault_message, "COMM_FAULT_UIP_TRANSMIT");
      break;

    default:
      return -1;
      break;
  }

  if(i == 32)
    i = 0;

  if(fault_status_word2 & (0xFFFFFFFF << i))
    return 1;
  else
    i = 0;

  return 0;
}

int segway_config_decode_internal_fault(__u32 fault_status_word2, char *fault_message)
{
  static int i = 0;
  __u32 fault;

  fault_status_word2 = (fault_status_word2 >> FSW_INTERNAL_FAULTS_SHIFT) & FSW_INTERNAL_FAULTS_MASK;

  if(fault_status_word2 == 0x00)
    return -1;

  while(i < 32)
  {
    fault = fault_status_word2 & (0x00000001 << i);
    
    i++;

    if(fault)
      break;
  }

  switch(fault)
  {
    case 0x000:
      return -1;
      break;

    case 0x001:
      strcpy(fault_message, "INTERNAL_FAULT_HIT_DEFAULT_CONDITION");
      break;

    case 0x002:
      strcpy(fault_message, "INTERNAL_FAULT_HIT_SPECIALC_CASE");
      break;

    default:
      return -1;
      break;
  }

  if(i == 32)
    i = 0;

  if(fault_status_word2 & (0xFFFFFFFF << i))
    return 1;
  else
    i = 0;

  return 0;

}

int segway_config_decode_sensors_fault(__u32 fault_status_word3, char *fault_message)
{
  static int i = 0;
  __u32 fault;

  fault_status_word3 = (fault_status_word3 >> FSW_SENSORS_FAULTS_SHIFT) & FSW_SENSORS_FAULTS_MASK;

  if(fault_status_word3 == 0x00)
    return -1;

  while(i < 32)
  {
    fault = fault_status_word3 & (0x00000001 << i);
    
    i++;

    if(fault)
      break;
  }

  switch(fault)
  {
    case 0x000:
      return -1;
      break;

      /*case 0x001:
      strcpy(fault_message, "SENSOR_FAULT_2P5V_VREF_RANGE_FAULT");
      break;

    case 0x002:
      strcpy(fault_message, "SENSOR_FAULT_7P2V_VBAT_RANGE_FAULT");
      break;

    case 0x004:
      strcpy(fault_message, "SESOR_FAULT_7P2V_VBAT_WARNING");
      break;

    case 0x008:
      strcpy(fault_message, "SENSOR_FAULT_7P2V_BATT_INBALANCE_FAULT");
      break;

    case 0x010:
      strcpy(fault_message, "SENSOR_FAULT_7P2V_BATT_TEMPERATURE_FAULT");
      break;
      */
    case 0x020:
      strcpy(fault_message, "SENSOR_FAULT_DIGITAL_INPUT");
      break;

    case 0x040:
      strcpy(fault_message, "SENSOR_FAULT_RANGE");
      break;

    case 0x080:
      strcpy(fault_message, "SENSOR_FAULT_DEFAULT");
      break;

    default:
      return -1;
      break;
  }

  if(i == 32)
    i = 0;

  if(fault_status_word3 & (0xFFFFFFFF << i))
    return 1;
  else
    i = 0;

  return 0;
}

int segway_config_decode_bsa_fault(__u32 fault_status_word3, char *fault_message)
{
  static int i = 0;
  __u32 fault;

  fault_status_word3 = (fault_status_word3 >> FSW_BSA_FAULTS_SHIFT) & FSW_BSA_FAULTS_MASK;

  if(fault_status_word3 == 0x00)
    return -1;

  while(i < 32)
  {
    fault = fault_status_word3 & (0x00000001 << i);
    
    i++;

    if(fault)
      break;
  }

  switch(fault)
  {
    case 0x000:
      return -1;
      break;

    case 0x001:
      strcpy(fault_message, "BSA_FAULT_SIDE_A_MISSING_BSA_DATA");
      break;

    case 0x002:
      strcpy(fault_message, "BSA_FAULT_SIDE_B_MISSING_BSA_DATA");
      break;

    case 0x004:
      strcpy(fault_message, "BSA_FAULT_UNKNOWN_MESSAGE_RECEIVED");
      break;

    case 0x008:
      strcpy(fault_message, "BSA_FAULT_TRANSMIT_A_FAILED");
      break;

    case 0x010:
      strcpy(fault_message, "BSA_FAULT_TRANSMIT_B_FAILED");
      break;

    case 0x020:
      strcpy(fault_message, "BSA_FAULT_DEFAULT");
      break;

    case 0x040:
      strcpy(fault_message, "BSA_FAULT_SIDE_A_RATE_SENSOR_SATURATED");
      break;

    case 0x080:
      strcpy(fault_message, "BSA_FAULT_SIDE_B_RATE_SENSOR_SATURATED");
      break;

    case 0x100:
      strcpy(fault_message, "BSA_FAULT_SIDE_A_TILT_SENSOR_SATURATED");
      break;

    case 0x200:
      strcpy(fault_message, "BSA_FAULT_SIDE_B_TILT_SENSOR_SATURATED");
      break;

    default:
      return -1;
      break;
  }

  if(i == 32)
    i = 0;

  if(fault_status_word3 & (0xFFFFFFFF << i))
    return 1;
  else
    i = 0;

  return 0;
}

int segway_config_decode_mcu_fault(__u32 fault_status_word4, char *fault_message)
{
  static int i = 0;
  __u32 fault;

  if(fault_status_word4 == 0x00)
    return -1;

  while(i < 32)
  {
    fault = fault_status_word4 & (0x00000001 << i);
    
    i++;

    if(fault)
      break;
  }

  switch(fault)
  {
    case 0x000:
      return -1;
      break;

    case 0x001:
      strcpy(fault_message, "MCU_FAULT_MCU_0_IS_DEGRADED");
      break;

    case 0x002:
      strcpy(fault_message, "MCU_FAULT_MCU_0_IS_FAILED");
      break;

    case 0x004:
      strcpy(fault_message, "MCU_FAULT_MCU_0_REQUESTS_REDUCED_PERFORMANCE");
      break;

    case 0x008:
      strcpy(fault_message, "MCU_FAULT_MCU_0_REQUESTS_ZER_SPEED");
      break;

    case 0x010:
      strcpy(fault_message, "MCU_FAULT_MCU_1_IS_DEGRADED");
      break;

    case 0x020:
      strcpy(fault_message, "MCU_FAULT_MCU_1_IS_FAILED");
      break;

    case 0x040:
      strcpy(fault_message, "MCU_FAULT_MCU_1_REQUESTS_REDUCED_PERFORMANCE");
      break;

    case 0x080:
      strcpy(fault_message, "MCU_FAULT_MCU_1_REQUESTS_ZER_SPEED");
      break;

    case 0x100:
      strcpy(fault_message, "MCU_FAULT_MCU_2_IS_DEGRADED");
      break;

    case 0x200:
      strcpy(fault_message, "MCU_FAULT_MCU_2_IS_FAILED");
      break;

    case 0x400:
      strcpy(fault_message, "MCU_FAULT_MCU_2_REQUESTS_REDUCED_PERFORMANCE");
      break;

    case 0x800:
      strcpy(fault_message, "MCU_FAULT_MCU_2_REQUESTS_ZER_SPEED");
      break;

    case 0x1000:
      strcpy(fault_message, "MCU_FAULT_MCU_3_IS_DEGRADED");
      break;

    case 0x2000:
      strcpy(fault_message, "MCU_FAULT_MCU_3_IS_FAILED");
      break;

    case 0x4000:
      strcpy(fault_message, "MCU_FAULT_MCU_3_REQUESTS_REDUCED_PERFORMANCE");
      break;

    case 0x8000:
      strcpy(fault_message, "MCU_FAULT_MCU_3_REQUESTS_ZER_SPEED");
      break;

    case 0x10000:
      strcpy(fault_message, "MCU_FAULT_MISSING_MCU_0_DATA");
      break;

    case 0x20000:
      strcpy(fault_message, "MCU_FAULT_MISSING_MCU_1_DATA");
      break;

    case 0x40000:
      strcpy(fault_message, "MCU_FAULT_MISSING_MCU_2_DATA");
      break;

    case 0x80000:
      strcpy(fault_message, "MCU_FAULT_MISSING_MCU_3_DATA");
      break;

    case 0x100000:
      strcpy(fault_message, "MCU_FAULT_UNKNOWN_MESSAGE_RECEIVED");
      break;

    default:
      return -1;
      break;
  }

  if(i == 32)
    i = 0;

  if(fault_status_word4 & (0xFFFFFFFF << i))
    return 1;
  else
    i = 0;

  return 0;
}


int segway_config_decode_mcu_message(__u32 mcu_fault_status, char *fault_message)
{
  static int i = 0;
  __u32 fault;

  if(mcu_fault_status == 0x00)
    return -1;

  while(i < 32)
  {
    fault = mcu_fault_status & (0x00000001 << i);
    
    i++;

    if(fault)
      break;
  }

  switch(fault)
  {
    case 0x0:
      return -1;
      break;

    case 0x1:
      strcpy(fault_message, "MCU_TRANS_BATTERY_TEMP_WARNING");
      break;

    case 0x2:
      strcpy(fault_message, "MCU_TRANS_BATTERY_COLD_REGEN");
      break;

    case 0x4:
      strcpy(fault_message, "MCU_UNKNOWN");
      break;

    case 0x8:
      strcpy(fault_message, "MCU_UNKNOWN");
      break;

    case 0x10:
      strcpy(fault_message, "MCU_TRANS_LOW_BATTERY");
      break;

    case 0x20:
      strcpy(fault_message, "MCU_TRANS_BATT_OVERVOLTAGE");
      break;

    case 0x40:
      strcpy(fault_message, "MCU_CRITICAL_BATT_OVERVOLTAGE");
      break;

    case 0x80:
      strcpy(fault_message, "MCU_CRITICAL_EMPTY_BATTERY");
      break;

    case 0x100:
      strcpy(fault_message, "MCU_CRITICAL_BATTERY_TEMP");
      break;

    case 0x200:
      strcpy(fault_message, "MCU_COMM_CU_BCU_LINK_DOWN");
      break;

    case 0x400:
      strcpy(fault_message, "MCU_COMM_INITIALIZATION_FAILED");
      break;

    case 0x800:
      strcpy(fault_message, "MCU_COMM_FAILED_CAL_EEPROM");
      break;

    case 0x1000:
      strcpy(fault_message, "MCU_POWER_SUPPLY_TRANSIENT_FAULT");
      break;

    case 0x2000:
      strcpy(fault_message, "MCU_POWR_SUPPLY_12V_FAULT");
      break;

    case 0x4000:
      strcpy(fault_message, "MCU_POWER_SUPPLY_5V_FAULT");
      break;

    case 0x8000:
      strcpy(fault_message, "MCU_POWER_SUPPLY_3V_FAULT");
      break;

    case 0x10000:
      strcpy(fault_message, "MCU_JUNCTION_TEMP_FAULT");
      break;

    case 0x20000:
      strcpy(fault_message, "MCU_MOTOR_WINDING_TEMP_FAULT");
      break;

    case 0x40000:
      strcpy(fault_message, "MCU_MOTOR_DRIVE_FAULT");
      break;

    case 0x80000:
      strcpy(fault_message, "MCU_MOTOR_DRIVE_HALL_FAULT");
      break;

    case 0x100000:
      strcpy(fault_message, "MCU_MOTOR_DRIVE_AMP_FAULT");
      break;

    case 0x200000:
      strcpy(fault_message, "MCU_MOTOR_DRIVE_AMP_ENABLE_FAULT");
      break;

    case 0x400000:
      strcpy(fault_message, "MCU_MOTOR_DRIVE_AMP_OVERCURRENT_FAULT");
      break;

    case 0x800000:
      strcpy(fault_message, "MCU_MOTOR_DRIVE_VOLTAGE_FEEDBACK_FAULT");
      break;

    case 0x1000000:
      strcpy(fault_message, "MCU_FRAME_FAULT");
      break;

    case 0x2000000:
      strcpy(fault_message, "MCU_BATTERY_FAULT");
      break;

    case 0x8000000:
      strcpy(fault_message, "MCU_MOTOR_STUCK_RELAY_FAULT");
      break;

    case 0x10000000:
      strcpy(fault_message, "MCU_ACTUATOR_POWER_CONSISTENCY_FAULT");
      break;

    case 0x20000000:
      strcpy(fault_message, "MCU_ACTUATOR_HALT_PROCESSOR_FAULT");
      break;

    case 0x40000000:
      strcpy(fault_message, "MCU_ACTUATOR_DEGRADED_FAULT");
      break;

    default:
      return -1;
      break;
  }
 
  if(i == 32)
    i = 0;

  if(mcu_fault_status & (0xFFFFFFFF << i))
    return 1;
  else
    i = 0;

  return 0;
}

