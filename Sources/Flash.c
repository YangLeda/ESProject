/*! @file
 *
 *  @brief Routines for erasing and writing to the Flash.
 *
 *  This contains the functions needed for accessing the internal Flash.
 *
 *  @author Leda Yang
 *  @date 2018-06-26
 */

#include "Flash.h"

// Struct to encapsulate FCCOB
typedef struct {
  uint8_t r0; // FCMD, a code that defines the FTFE command
  uint8_t r1; // Flash address [23:16]
  uint8_t r2; // Flash address [15:8]
  uint8_t r3; // Flash address [7:0]
  uint8_t data[8]; // Data in the 8 byte phase
} TFCCOB;

// An array to track what bytes have been allocated. However, lost when power off.
// An possible alternative is to check 0xFF on a byte. However, 0xFF could be user data.
// Another possible alternative is to store the map in Flash. However, this Lab is limited in the phrase.
static uint8_t AllocationMap[8];

/*! @brief Private function to launch an FCCOB.
 *
 *  @param commonCommandObject An set up FCCOB.
 *  @return bool - TRUE if the command is successfully launched.
 */
static bool LaunchCommand(TFCCOB* commonCommandObject)
{
  // FCCOB availability check, CCIF returns to 1 when command completes
  while (!(FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK));

  // Access error and protection violation check
  if (!((FTFE_FSTAT & FTFE_FSTAT_ACCERR_MASK) | (FTFE_FSTAT & FTFE_FSTAT_FPVIOL_MASK)))
  {
    // Write to the FCCOB registers
    FTFE_FCCOB0 = commonCommandObject->r0;
    FTFE_FCCOB1 = commonCommandObject->r1;
    FTFE_FCCOB2 = commonCommandObject->r2;
    FTFE_FCCOB3 = commonCommandObject->r3;
    FTFE_FCCOB4 = commonCommandObject->data[3];
    FTFE_FCCOB5 = commonCommandObject->data[2];
    FTFE_FCCOB6 = commonCommandObject->data[1];
    FTFE_FCCOB7 = commonCommandObject->data[0];
    FTFE_FCCOB8 = commonCommandObject->data[7];
    FTFE_FCCOB9 = commonCommandObject->data[6];
    FTFE_FCCOBA = commonCommandObject->data[5];
    FTFE_FCCOBB = commonCommandObject->data[4];

    // Clear CCIF to launch the command
    FTFE_FSTAT |= FTFE_FSTAT_CCIF_MASK;

    // FCCOB availability check, CCIF returns to 1 when command completes
    while (!(FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK));
    return TRUE;
  }
  return FALSE;
}

/*! @brief Private function to write a phrase.
 *
 *  @param address The address of the phase to be written.
 *  @param phase The phase of the data to be written.
 *  @return bool - TRUE if the command is successfully launched.
 */
static bool WritePhrase(const uint8_t phrase[])
{
  // Erase before programming
  if (Flash_Erase())
  {
    // Set tfcoob for program phrase command
    TFCCOB tfccob;
    tfccob.r0 = 0x07; // program phrase command
    tfccob.r1 = 0x08; // Flash address [23:16]
    tfccob.r2 = 0x00; // Flash address [15:8]
    tfccob.r3 = 0x00; // Flash address [7:0]
    // Set bytes to write
    tfccob.data[0] = phrase[0];
    tfccob.data[1] = phrase[1];
    tfccob.data[2] = phrase[2];
    tfccob.data[3] = phrase[3];
    tfccob.data[4] = phrase[4];
    tfccob.data[5] = phrase[5];
    tfccob.data[6] = phrase[6];
    tfccob.data[7] = phrase[7];
    // Launch the command
    return LaunchCommand(&tfccob);
  }
  return FALSE;
}

/*! @brief Private function to read the phrase in Flash into an array.
 *
 *  @param buffer[] Pointer to an array to store data.
 *  @return void
 */
static void ReadFlash(uint8_t buffer[])
{
  uint8_t* dataPointer = (uint8_t*) FLASH_DATA_START; // Pointer to read data
  uint8_t i; // Loop to get the byte data
  for (i = 0; i < 8; i++)
  {
    buffer[i] = *dataPointer;
    dataPointer++;
  }
}

/*! @brief Enables the Flash module.
 *
 *  @return bool - TRUE if the Flash was setup successfully.
 */
bool Flash_Init(void)
{
  // Init allocation map
  uint8_t i;
  for (i = 0; i < 8; i++)
    AllocationMap[i] = 0;

  return TRUE;
}

/*! @brief Allocates space for a non-volatile variable in the Flash memory.
 *
 *  @param variable is the address of a pointer to a variable that is to be allocated space in Flash memory.
 *         The pointer will be allocated to a relevant address:
 *         If the variable is a byte, then any address.
 *         If the variable is a half-word, then an even address.
 *         If the variable is a word, then an address divisible by 4.
 *         This allows the resulting variable to be used with the relevant Flash_Write function which assumes a certain memory address.
 *         e.g. a 16-bit variable will be on an even address
 *  @param size The size, in bytes, of the variable that is to be allocated space in the Flash memory. Valid values are 1, 2 and 4.
 *  @return bool - TRUE if the variable was allocated space in the Flash memory.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_AllocateVar(volatile void** variable, const uint8_t size)
{
  // Checking addresses to find empty location
  uint8_t checkingAddressOff;
  for (checkingAddressOff = 0; checkingAddressOff < 8; checkingAddressOff += size)
  {
    // Check if is empty
    uint8_t sum = 0;
    uint8_t i;
    for (i = checkingAddressOff; i < checkingAddressOff + size; i++)
      sum += AllocationMap[i];
    if (sum == 0) // Is empty
    {
      *variable = (void*) (FLASH_DATA_START + checkingAddressOff);
      for (i = checkingAddressOff; i < checkingAddressOff + size; i++) // Set the allocation map
	AllocationMap[i] = 1;
      return TRUE;
    }
  }
  return FALSE; // No spare space
}

/*! @brief Writes a 32-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 32-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 4-byte boundary or if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write32(volatile uint32_t* const address, const uint32_t data)
{
  // Off set position of the address in Flash
  uint32_t off = (uint32_t) address - FLASH_DATA_START;
  if (off % 4 != 0)
    return FALSE; // Address is not evenly divisible by 4
  // Store the data in unions for easy access
  uint32union_t word;
  uint16union_t halfWord0;
  uint16union_t halfWord1;
  word.l = data;
  halfWord0.l = word.s.Lo;
  halfWord1.l = word.s.Hi;
  // Read current content in Flash into a buffer
  uint8_t buffer[8];
  ReadFlash(buffer);
  // Edit the buffer
  buffer[off] = halfWord0.s.Lo;
  buffer[off+1] = halfWord0.s.Hi;
  buffer[off+2] = halfWord1.s.Lo;
  buffer[off+3] = halfWord1.s.Hi;
  // Write back to Flash
  return WritePhrase(buffer);
}

/*! @brief Writes a 16-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 16-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 2-byte boundary or if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write16(volatile uint16_t* const address, const uint16_t data)
{
  // Off set position of the address in Flash
  uint32_t off = (uint32_t) address - FLASH_DATA_START;
  if (off % 2 != 0)
    return FALSE; // Address is not even
  // Store the data a union for easy access
  uint16union_t halfWord;
  halfWord.l = data;
  // Read current content in Flash into a buffer
  uint8_t buffer[8];
  ReadFlash(buffer);
  // Edit the buffer
  buffer[off] = halfWord.s.Lo;
  buffer[off+1] = halfWord.s.Hi;
  // Write back to Flash
  return WritePhrase(buffer);
}

/*! @brief Writes an 8-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 8-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write8(volatile uint8_t* const address, const uint8_t data)
{
  // Off set position of the address in Flash
  uint32_t off = (uint32_t) address - FLASH_DATA_START;
  // Read current content in Flash into a buffer
  uint8_t buffer[8];
  ReadFlash(buffer);
  // Edit the buffer
  buffer[off] = data;
  // Write back to Flash
  return WritePhrase(buffer);
}

/*! @brief Erases the entire Flash sector.
 *
 *  @return bool - TRUE if the Flash "data" sector was erased successfully.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Erase(void)
{
  // Clear allocation map
  uint8_t i;
  for (i = 0; i < 8; i++)
    AllocationMap[i] = 0;

  // Set tfcoob for sector erase command
  TFCCOB tfccob;
  tfccob.r0 = 0x09; // Erase flash sector command
  tfccob.r1 = 0x08; // Flash address [23:16] in the flash sector to be erased
  tfccob.r2 = 0x00; // Flash address [15:8] in the flash sector to be erased
  tfccob.r3 = 0x00; // Flash address [7:0] in the flash sector to be erased

  // Launch the command
  return LaunchCommand(&tfccob);
}
