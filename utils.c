#include "utils.h"


// convert string to int;
// string format: ±0....
// i.e string must be zero padded and precede by sign; length is the number of
// chars included the sign
int ParseInt(uint8_t* buffer, int start, int length)
{
  int num=0;
  for(int i=1;i<length;i++)
  {
    num *= 10;
    num += (buffer[i+start] - '0');
  }
  if(buffer[start] == '-')
    num = -num;
  return num;
}


// convert int to string;
// string format: ±0....
// i.e string will be zero padded and preceded by sign; length is the number of
// chars included the sign
void IntToString(uint8_t* buffer, int start, int length, int number)
{
  if(number < 0)
  {
    buffer[start] = '-';
    number = -number;
  }
  else
    buffer[start] = '+';
  
  start++;
  length--;
  for(int i=0;i<length;i++)
  {
    buffer[start+length-i-1] = (number % 10) + '0';
    number /= 10;
  }
}