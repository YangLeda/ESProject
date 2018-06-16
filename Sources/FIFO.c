/*! @file
 *
 *  @brief Routines of a FIFO buffer.
 *
 *  This contains the methods for accessing a byte-wide FIFO.
 *
 *  @author Leda Yang, Jiayi Li
 *  @date 2018-05-30
 */

#include "FIFO.h"

/*! @brief Initialize the FIFO before first use.
 *
 *  @param FIFO A pointer to the FIFO that needs initializing.
 *  @return void
 */
void FIFO_Init (TFIFO * const FIFO)
{
  FIFO->Start = FIFO->End = 0;
  FIFO->NbBytes = 0;
  // Init semaphore to space available
  FIFO->SpaceAvailableSem = OS_SemaphoreCreate(FIFO_SIZE);
  // Init semaphore to no item available
  FIFO->ItemsAvailableSem = OS_SemaphoreCreate(0);
  // Init semaphore to buffer available
  FIFO->BufferAccessSem = OS_SemaphoreCreate(1);
}

/*! @brief Put one character into the FIFO.
 *
 *  @param FIFO A pointer to a FIFO struct where data is to be stored.
 *  @param data A byte of data to store in the FIFO buffer.
 *  @return bool - TRUE if data is successfully stored in the FIFO.
 *  @note Assumes that FIFO_Init has been called.
 */
void FIFO_Put (TFIFO * const FIFO, const uint8_t data)
{
  OS_SemaphoreWait(FIFO->SpaceAvailableSem, 0);
  OS_SemaphoreWait(FIFO->BufferAccessSem, 0);

  FIFO->Buffer[FIFO->End] = data; // Put data into FIFO
  if (FIFO->End == FIFO_SIZE - 1)
    FIFO->End = 0; // Wrap
  else
    FIFO->End ++;
  FIFO->NbBytes ++;

  OS_SemaphoreSignal(FIFO->BufferAccessSem);
  OS_SemaphoreSignal(FIFO->ItemsAvailableSem);
}

/*! @brief Get one character from the FIFO.
 *
 *  @param FIFO A pointer to a FIFO struct with data to be retrieved.
 *  @param dataPtr A pointer to a memory location to place the retrieved byte.
 *  @return bool - TRUE if data is successfully retrieved from the FIFO.
 *  @note Assumes that FIFO_Init has been called.
 */
void FIFO_Get (TFIFO * const FIFO, uint8_t * const dataPtr)
{
  OS_SemaphoreWait(FIFO->ItemsAvailableSem, 0);
  OS_SemaphoreWait(FIFO->BufferAccessSem, 0);

  *dataPtr = FIFO->Buffer[FIFO->Start]; // Get the data
  if (FIFO->Start == FIFO_SIZE - 1)
    FIFO->Start = 0; // Wrap
  else
    FIFO->Start ++;
  FIFO->NbBytes --;

  OS_SemaphoreSignal(FIFO->BufferAccessSem);
  OS_SemaphoreSignal(FIFO->SpaceAvailableSem);
}
