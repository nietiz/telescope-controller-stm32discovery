#include "commands.h"

#define MAX_COMMANDS            10


typedef struct
{
  uint8_t strCmd[MAX_COMMAND_LENGTH];
} command;


int head = 0;
int tail = 0;
int queueCount = 0;
command commandQueue[MAX_COMMANDS];


void PushCommand(uint8_t* buffer, int len)
{
  uint8_t* buf = commandQueue[head].strCmd;
  for(int i = 0; i < len; i++)
    buf[i] = buffer[i];
  head++;
  if(head == MAX_COMMANDS)
    head = 0;
  queueCount++;
  if(queueCount == MAX_COMMANDS)
  {
    queueCount = MAX_COMMANDS;
    tail = head;
  }
}
int GetCommandCount()
{
  return queueCount;
}
uint8_t* PopCommand()
{
  uint8_t* cmd = 0;
  if(queueCount > 0)
  {
    cmd = commandQueue[tail].strCmd;
    tail++;
    if(tail == MAX_COMMANDS)
      tail = 0;
    queueCount--;
  }
  return cmd;
}




/*

static uint8_t bufferAck[5] = "ACK\n\r";
static uint8_t bufferNak[5] = "NAK\n\r";


  uint8_t result = ParseCommand(Buf, Len);
  if(result == 0)
  {
    VCP_DataTx(bufferAck, 5);
  }
  else
  {
    VCP_DataTx(bufferNak, 5);
  }
*/