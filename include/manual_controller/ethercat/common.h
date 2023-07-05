#ifndef DELTA_ASDA_ETHERCAT_COMMON_H
#define DELTA_ASDA_ETHERCAT_COMMON_H
// STL
#include <string>
#include <map>
// soem
#include "ethercat.h"


namespace delta { namespace asda { namespace ethercat {

/* Mode of Operation */
enum mode_of_operation_t : int8
{
  PROFILE_POSITION = 1,              // Profile Position Mode
  PROFILE_VELOCITY = 3,              // Profile Velocity Mode
  PROFILE_TORQUE = 4,                // Torque Profile Mode
  HOMING = 6,                        // Homing Mode
  INTERPOLATED_POSITION = 7,         // Interpolated Position Mode
  CYCLIC_SYNCHRONOUS_POSITION = 8,   // Cyclic Synchronous Position Mode
  CYCLIC_SYNCHRONOUS_VELOCITY = 9,   // Cyclic Synchronous Velocity Mode
  CYCLIC_SYNCHRONOUS_TORQUE = 10,    // Cyclic Synchronous Torque Mode
};
// In SM synchronization mode, PP, PV, TQ, HM and Q modes are supported.
// In DC synchronization mode, CSP, CSV and HM modes are supported.


/* Interpolation Sub-Mode */
enum interpolation_sub_mode_t : int16
{
  LINEAR_INTERPOLATION = 0,           // Linear Interpolation
};


/* Error Codes */
// static std::map<uint16, std::string> error_codes
// {
//   { 0x7500, "EtherCAT communication error"},
//   { 0xFF01, "Over Current"},
//   { 0xFF02, "Over Voltage"},
//   { 0xFF03, "Over Temperature"},
//   { 0xFF04, "Open Motor Winding"},
//   { 0xFF05, "Internal Voltage Bad"},
//   { 0xFF06, "Position Limit"},
//   { 0xFF07, "Bad Encoder"},
//   { 0xFF08, "reserved"},
//   { 0xFF09, "reserved"},
//   { 0xFF0A, "Excess regen"},
//   { 0xFF0B, "Safe Torque Off"},
//   { 0xFF31, "CW Limit"},
//   { 0xFF32, "CCW Limit"},
//   { 0xFF33, "CW Limit and CCW Limit"},
//   { 0xFF34, "Current Foldback"},
//   { 0xFF35, "Move while Disabled"},
//   { 0xFF36, "Under Voltage"},
//   { 0xFF37, "Blank Q segment"},
//   { 0xFF41, "Save Failed"},
//   { 0xFFFF, "Other Error"},
// };


// TODO: Improve The Error Messages
/* Error Codes */
static std::map<uint16, std::string> error_codes
{
  { 8976, "Overcurrent"},
  { 12560, "Overvoltage"},
  { 12576, "Undervoltage"},
  { 28962, "Motor error"},
  { 12816, "Regenerator error"},
  { 12848, "Overload"},
  { 33792, "Overspeed"},
  { 34304, "Abnormal pulse control command"},
  { 34321, "Excessive deviation"},
  { 0, "Reserved"},
  { 29445, "Encoder Error"},
  { 25376, "Adjustment error"},
  { 21569, "Emergency stop activated"},
  { 21571, "Reverse limit switch error"},
  { 21570, "Forward limit switch error"},
  { 16912, "IGBT temperature error"},
  { 21296, "Memory error"},
  { 29446, "Encoder output error"},
  { 29968, "Serial communication error"},
  { 29984, "Serial communication time out"},
  { 12592, "Input power phase loss"},
  { 12849, "Early warning for overload"},
  { 29445, "Encoder initial magnetic field error or Encoder internal error or Unreliable internal data of the encoder or Encoder data error"},
  // { 29445, "Encoder internal error"},
  // { 29445, "Unreliable internal data of the encoder"},
  // { 29445, "Encoder data error"},
  { 28961, "Motor protection error"},
  { 13056, "U,V,W wiring error"},
  { 34320, "Full-closed loop excessive deviation"},
  { 21760, "DSP firmware upgrade"},
  { 25360, "CANopen Data Initial Error"},
  { 21572, "Forward software limit"},
  { 21573, "Reverse software limit"},
  { 33056, "EtherCAT connection error (Servo Off)"},
  { 33072, "Node guarding or Heartbeat error (Servo Off)"},
  { 33280, "Sub-index error occurs when accessing CANopen PDO object or Data type (size) error occurs when accessing CANopen PDO object or Data range error occurs when accessing CANopen PDO object or CANopen PDO object is read-only and write-protected or CANopen PDO object does not support PDO or CANopen PDO object is write-protected when Servo On or Error occurs when reading CANopen PDO object from EEPROM or Error occurs when writing CANopen PDO object into EEPROM or EEPROM invalid address range or EEPROM checksum error or EEPROM zone error"},
  // { 0x82000123, "Data type (size) error occurs when accessing CANopen PDO object"},
  // { 0x82000124, "Data range error occurs when accessing CANopen PDO object"},
  // { 0x82000125, "CANopen PDO object is read-only and write-protected"},
  // { 0x82000126, "CANopen PDO object does not support PDO"},
  // { 0x82000127, "CANopen PDO object is write-protected when Servo On"},
  // { 0x82000128, "Error occurs when reading CANopen PDO object from EEPROM"},
  // { 0x82000129, "Error occurs when writing CANopen PDO object into EEPROM"},
  // { 0x82000130, "EEPROM invalid address range"},
  // { 0x82000131, "EEPROM checksum error"},
  // { 0x82000132, "EEPROM zone error"},
  { 25360, "CANopen load/save 1010/1011 error"},
  { 25088, "CANopen SYNC failed (Servo Off) or CANopen SYNC signal error (Servo Off) or CANopen SYNC time out (Servo Off) or SYNC period error (Servo Off)"},
  // { 0x620003E2, "CANopen SYNC signal error (Servo Off)"},
  // { 0x620003E3, "CANopen SYNC time out (Servo Off)"},
  // { 0x620003E4, "CANopen IP command failed (Servo Off)"},
  // { 0x620003E5, "SYNC period error (Servo Off)"},
  { 36864, "Safe torque off (Servo Off) or STO_A lost (Servo Off) or STO_B lost (Servo Off) or STO_error (Servo Off)"},
  // { 0x90000501, "STO_A lost (Servo Off)"},
  // { 0x90000502, "STO_B lost (Servo Off)"},
  // { 0x90000503, "STO_error (Servo Off)"},
};


template<class T>
int writeSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T value)
{
  int wkc = 0;

  T data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOwrite(slave, index, sub_index, FALSE, size_of_data, &data, EC_TIMEOUTRXM);

  return wkc;
}


template<class T>
int writeSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T *value)
{
  int wkc = 0;

  T *data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOwrite(slave, index, sub_index, TRUE, size_of_data, data, EC_TIMEOUTRXM);

  return wkc;
}


template<class T>
int readSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T &value)
{
  int wkc = 0;

  T data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOread(slave, index, sub_index, FALSE, &size_of_data, &data, EC_TIMEOUTRXM);

  value = data;

  return wkc;
}


template<class T>
int readSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T *value)
{
  int wkc = 0;

  T *data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOread(slave, index, sub_index, TRUE, &size_of_data, data, EC_TIMEOUTRXM);

  *value = *data;

  return wkc;
}


inline void print_ec_state(uint16 slave_idx)
{
  switch (ec_slave[slave_idx].state)
  {
    case EC_STATE_NONE:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "NONE");
      break;
    case EC_STATE_INIT:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "INIT");
      break;
    case EC_STATE_PRE_OP:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "PRE_OP");
      break;
    case EC_STATE_BOOT:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "BOOT");
      break;
    case EC_STATE_SAFE_OP:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "SAFE_OP");
      break;
    case EC_STATE_OPERATIONAL:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "OPERATIONAL");
      break;
    //case EC_STATE_ACK:
    //  ROS_INFO("%s: ESM: %s", ec_slave[slave].name, "EC_STATE_ACK");
    //  break;
    case EC_STATE_PRE_OP + EC_STATE_ERROR:
      printf("%s: EC_STATE: %s + %s\n", ec_slave[slave_idx].name, "PRE_OP", "ERROR");
      break;
    case EC_STATE_SAFE_OP + EC_STATE_ERROR:
      printf("%s: EC_STATE: %s + %s\n", ec_slave[slave_idx].name, "SAFE_OP", "ERROR");
      break;
    case EC_STATE_OPERATIONAL + EC_STATE_ERROR:
      printf("%s: EC_STATE: %s + %s\n", ec_slave[slave_idx].name, "OPERATIONAL", "ERROR");
      break;
  }
}


inline void print_sm(uint16 slave_idx, int sm)
{
  uint16 A = ec_slave[slave_idx].SM[sm].StartAddr;
  uint16 L = ec_slave[slave_idx].SM[sm].SMlength;
  uint32 F = ec_slave[slave_idx].SM[sm].SMflags;
  uint8 Type = ec_slave[slave_idx].SMtype[sm];
  printf("SM%d A:%4.4x L:%4d F:%8.8x Type:%d\n", sm, A, L, F, Type);
}


inline void print_fmmu(uint16 slave_idx, int fmmu)
{
  uint32 Ls = ec_slave[slave_idx].FMMU[fmmu].LogStart;
  uint16 Ll = ec_slave[slave_idx].FMMU[fmmu].LogLength;
  uint8 Lsb = ec_slave[slave_idx].FMMU[fmmu].LogStartbit;
  uint8 Leb = ec_slave[slave_idx].FMMU[fmmu].LogEndbit;
  uint16 Ps = ec_slave[slave_idx].FMMU[fmmu].PhysStart;
  uint8 Psb = ec_slave[slave_idx].FMMU[fmmu].PhysStartBit;
  uint8 Ty = ec_slave[slave_idx].FMMU[fmmu].FMMUtype;
  uint8 Act = ec_slave[slave_idx].FMMU[fmmu].FMMUactive;
  printf("FMMU%d Ls:%.8x Ll:%4.2d Lsb:%d Leb:%d Ps:%.4x Psb:%d Ty:%.2d Act:%.2d\n", fmmu, Ls, Ll, Lsb, Leb, Ps, Psb, Ty, Act);
}


} } } // namespace
#endif
