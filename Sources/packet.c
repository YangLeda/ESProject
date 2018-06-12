/*! @file
 *
 *  @brief Routines of packet encoding and decoding for the serial port.
 *
 *  This contains the functions implementations of the "Tower to PC Protocol" 5-byte packets.
 *
 *  @author Leda Yang, Jiayi Li
 *  @date 2018-05-30
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
  PacketPutSem = OS_SemaphoreCreate(1);
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
  OS_SemaphoreWait(PacketPutSem, 0);

  // Send each byte
  UART_OutChar(command);
  UART_OutChar(parameter1);
  UART_OutChar(parameter2);
  UART_OutChar(parameter3);
  UART_OutChar(command ^ parameter1 ^ parameter2 ^ parameter3);

  OS_SemaphoreSignal(PacketPutSem);
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
    Packet_Put(0x04, 0, 0, 0); // Special - Startup
    Packet_Put(0x09, 'v', 1, 0); // Special - Version number
    Packet_Put(0x0B, 1, NvTowerNb->s.Lo, NvTowerNb->s.Hi); // Tower Number
    Packet_Put(0x0D, 1, NvTowerMd->s.Lo, NvTowerMd->s.Hi); // Tower Mode
    return TRUE;
  }
  else
    return FALSE;
}

/*! @brief Handle the Special - Get version command.
 *
 *  @return bool - TRUE if handle successful
 *  	           FALSE if handle failed
 */
static bool Handle_Version_Packet(void)
{
  if (Packet_Parameter1 == 'v' && Packet_Parameter2 == 'x' && Packet_Parameter3 == 0xD)
    return Packet_Put(0x09, 'v', 1, 0);
  else
    return FALSE;
}

/*! @brief Handle the Tower number command.
 *
 *  @return bool - TRUE if handle successful
 *  	           FALSE if handle failed
 */
static bool Handle_Number_Packet(void)
{
  if (Packet_Parameter1 == 1 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0) // Get
    return Packet_Put(0x0B, 1, NvTowerNb->s.Lo, NvTowerNb->s.Hi);
  else if (Packet_Parameter1 == 2) // Set
    return Flash_Write16((uint16*) NvTowerNb, Packet_Parameter23);
  else
    return FALSE;
}

/*! @brief Handle the Flash - Read byte command.
 *
 *  @return bool - TRUE if handle successful
 *  	           FALSE if handle failed
 */
static bool Handle_FlashRead_Packet(void)
{
  if (Packet_Parameter1 >= 0 && Packet_Parameter1 <= 0x07 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
    return Packet_Put(0x08, Packet_Parameter1, 0, _FB(FLASH_DATA_START + (uint32_t) Packet_Parameter1)); // Put Flash byte in packet
  else
    return FALSE;
}

/*! @brief Handle the Flash - Program byte command.
 *
 *  @return bool - TRUE if handle successful
 *  	           FALSE if handle failed
 */
static bool Handle_FlashProgram_Packet(void)
{
  if (Packet_Parameter1 >= 0 && Packet_Parameter1 <= 0x07 && Packet_Parameter2 == 0) // Program Flash
    return Flash_Write8((uint8*) (FLASH_DATA_START + Packet_Parameter1), Packet_Parameter3);
  else if (Packet_Parameter1 == 0x08 && Packet_Parameter2 == 0) // Erase the entire Flash sector
    return Flash_Erase();
  else
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
    case 0x09:
      successFlag = Handle_Version_Packet();
      break;
    case 0x0B:
      successFlag = Handle_Number_Packet();
      break;
    case 0x08:
      successFlag = Handle_FlashRead_Packet();
      break;
    case 0x07:
      successFlag = Handle_FlashProgram_Packet();
      break;
    case 0x0D:
      //successFlag = Handle_Mode_Packet();
      break;
    case 0x0C:
      //successFlag = Handle_Time_Set_Packet();
      break;
    case 0x0A:
      //successFlag = Handle_Protocol_Mode_Packet();
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
