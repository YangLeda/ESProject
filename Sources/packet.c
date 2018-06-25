/*! @file
 *
 *  @brief Routines of packet encoding and decoding for the serial port.
 *
 *  This contains the functions implementations of the "Tower to PC Protocol" 5-byte packets.
 *
 *  @author Leda Yang
 *  @date 2018-06-26
 */

#include "packet.h"
#include "Flash.h"

// Stores the potential packet
TPacket Packet;

// Count of received bytes in a potential packet
static uint8_t NbBytesInPacket;

// Acknowledgment bit mask
const uint8_t PACKET_ACK_MASK = 0x80;

/*! @brief Initializes the packets by calling the initialization routines of the supporting software modules.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz
 *  @return bool - TRUE if the packet module was successfully initialized.
 */
bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  return UART_Init(baudRate, moduleClk);
}

/*! @brief Attempts to get a packet from the received data.
 *
 *  @return bool - TRUE if a valid packet was received.
 */
bool Packet_Get(void)
{
  // Try to build the packet according to current number of bytes in the potential packet
  switch (NbBytesInPacket)
  {
    case 0:
      UART_InChar(&Packet_Command);
      NbBytesInPacket++;
      break;
    case 1:
      UART_InChar(&Packet_Parameter1);
      NbBytesInPacket++;
      break;
    case 2:
      UART_InChar(&Packet_Parameter2);
      NbBytesInPacket++;
      break;
    case 3:
      UART_InChar(&Packet_Parameter3);
      NbBytesInPacket++;
      break;
    case 4:
      UART_InChar(&Packet_Checksum);
      if (Packet_Checksum == (Packet_Command ^ Packet_Parameter1 ^ Packet_Parameter2 ^ Packet_Parameter3))
      {
        NbBytesInPacket = 0;
        return TRUE;
      }
      // Remove the first byte
      Packet_Command = Packet_Parameter1;
      Packet_Parameter1 = Packet_Parameter2;
      Packet_Parameter2 = Packet_Parameter3;
      Packet_Parameter3 = Packet_Checksum;
      return FALSE;
  }
  return FALSE;
}

/*! @brief Builds a packet and places it in the transmit FIFO buffer.
 *
 *  @return bool - TRUE if a valid packet was sent.
 */
bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  EnterCritical();

  // Send each byte
  UART_OutChar(command);
  UART_OutChar(parameter1);
  UART_OutChar(parameter2);
  UART_OutChar(parameter3);
  UART_OutChar(command ^ parameter1 ^ parameter2 ^ parameter3);

  ExitCritical();
}

/*! @brief Handle the Special - Get startup values command or send startup packets when power on.
 *
 *  @return bool - TRUE if handle successful
 *               FALSE if handle failed
 */
static bool Handle_Startup_Packet(void)
{
  if (Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    Packet_Put(0x04, 0, 0, 0);
    return TRUE;
  }
  return FALSE;
}

/*! @brief Handle the Timing mode command.
 *
 *  @return bool - TRUE if handle successful
 *  	           FALSE if handle failed
 */
static bool Handle_Timing_Mode_Packet(void)
{
  if (Packet_Parameter1 == 0 && Packet_Parameter2 == 0x00) // Get timing mode
  {
    Packet_Put(0x10, *TimingMode, 0, 0);
    return TRUE;
  }
  else if (Packet_Parameter1 == 1 && Packet_Parameter2 == 0x00) // Set to definite mode
  {
    Flash_Write8((uint8_t*)TimingMode, 1);
    return TRUE;
  }
  else if (Packet_Parameter1 == 2 && Packet_Parameter2 == 0x00) // Set to inverse mode
  {
    Flash_Write8((uint8_t*)TimingMode, 2);
    return TRUE;
  }
  return FALSE;
}

/*! @brief Handle the Number of raises packet.
 *
 *  @return bool - TRUE if handle successful
 *               FALSE if handle failed
 */
static bool Handle_Raises_Packet(void)
{
  if (Packet_Parameter1 == 0 && Packet_Parameter2 == 0x00) // Get
  {
    Packet_Put(0x11, NumOfRaise->l, 0, 0);
    return TRUE;
  }
  else if (Packet_Parameter1 == 1 && Packet_Parameter2 == 0x00) // Reset to 0
  {
    Flash_Write16((uint16*) NumOfRaise, 0);
    return TRUE;
  }
  return FALSE;
}

/*! @brief Handle the Handle the Number of raises packet.
 *
 *  @return bool - TRUE if handle successful
 *               FALSE if handle failed
 */
static bool Handle_Lowers_Packet(void)
{
  if (Packet_Parameter1 == 0 && Packet_Parameter2 == 0x00) // Get
  {
    Packet_Put(0x12, NumOfLower->l, 0, 0);
    return TRUE;
  }
  else if (Packet_Parameter1 == 1 && Packet_Parameter2 == 0x00) // Reset to 0
  {
    Flash_Write16((uint16*) NumOfLower, 0);
    return TRUE;
  }
  return FALSE;
}

/*! @brief Handle the Handle the Voltage packet.
 *
 *  @return bool - TRUE if handle successful
 *               FALSE if handle failed
 */
static bool Handle_RMS_Packet(void)
{
  if (Packet_Parameter1 == 0) // Get all
  {
    Packet_Put(0x18, 1, AnalogThreadData[0].rms, AnalogThreadData[0].rms >> 8);
    Packet_Put(0x18, 2, AnalogThreadData[1].rms, AnalogThreadData[1].rms >> 8);
    Packet_Put(0x18, 3, AnalogThreadData[2].rms, AnalogThreadData[2].rms >> 8);
    return TRUE;
  }
  else if (Packet_Parameter1 == 1) // Get channel 0
  {
    Packet_Put(0x18, 1, AnalogThreadData[0].rms, AnalogThreadData[0].rms >> 8);
    return TRUE;
  }
  else if (Packet_Parameter1 == 2) // Get channel 1
  {
    Packet_Put(0x18, 2, AnalogThreadData[1].rms, AnalogThreadData[1].rms >> 8);
    return TRUE;
  }
  else if (Packet_Parameter1 == 3) // Get channel 2
  {
    Packet_Put(0x18, 3, AnalogThreadData[2].rms, AnalogThreadData[2].rms >> 8);
    return TRUE;
  }
  return FALSE;
}

/*! @brief Handle the Handle the Frequency packet.
 *
 *  @return bool - TRUE if handle successful
 *               FALSE if handle failed
 */
static bool Handle_Frequency_Packet(void)
{
  Packet_Put(0x17, AnalogThreadData[0].frequency, AnalogThreadData[0].frequency >> 8, 0);
  return TRUE;
}

/*! @brief Handle the Handle Spectrum packet.
 *
 *  @return bool - TRUE if handle successful
 *               FALSE if handle failed
 */
static bool Handle_Spectrum_Packet(void)
{
  if (Packet_Parameter1 >= 0 && Packet_Parameter1 <= 7)
  {
    OS_SemaphoreSignal(SpectrumSem);
    return TRUE;
  }
  return FALSE;
}

/*! @brief Routine to handle the packet.
 *
 *  @return void
 */
void Handle_Packet(void)
{
  uint8_t command = Packet_Command & ~PACKET_ACK_MASK;
  bool successFlag;

  // Switch to handle packet functions
  switch (command)
  {
    case 0x04:
      successFlag = Handle_Startup_Packet();
      break;
    case 0x10:
      successFlag = Handle_Timing_Mode_Packet();
      break;
    case 0x11:
      successFlag = Handle_Raises_Packet();
      break;
    case 0x12:
      successFlag = Handle_Lowers_Packet();
      break;
    case 0x17:
      successFlag = Handle_Frequency_Packet();
      break;
    case 0x18:
      successFlag = Handle_RMS_Packet();
      break;
    case 0x19:
      successFlag = Handle_Spectrum_Packet();
      break;
    default:
      successFlag = FALSE; // Incorrect command byte
      break;
   }

  // Put acknowledgment packet if required
  if (Packet_Command != command)
  {
    if (successFlag) // handle successful
      Packet_Put(Packet_Command, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
    else // handle failed
      Packet_Put(command, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
  }

  return;
}
