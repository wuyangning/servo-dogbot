#include "Arduino.h"
#include "command.h"

char command[64];
byte command_length;

void init_command()
{

  for (byte i=0; i<64; i++)
    command[i] = 0;
  command_length = 0;
}

void cli_loop()
{
  char c;
  while(Serial.available() > 0)
  {
    c = Serial.read();
    if (c != cm_end)
    {
      command[command_length] = c;
      command_length++;
    }
  }
 
  if (command_length && (c == cm_end) )
  {
    process_command();
    init_command();
  }
  
}



/*-----------------------------------------------------------------------------------------*/
void process_command()
{

  byte code = 0;
  
  
  if (command_seen(cm_start) )
  { 
    
    code = (int)key_value(cm_start);
    switch (code)
    {
      
      case 0:
        pwmWrite(servo_num[int(key_value('n'))],int(key_value('p')));
        Serial.println("done");
      break;

    }
   
  }
  

  
  

}
/*-----------------------------------------------------------------------------------------*/


double key_value(char key)
{
  char temp[10] = "";

  for (byte i=0; i<command_length; i++)
  {
    if (command[i] == key)
    {
      i++;      
      int k = 0;
      while (i < command_length && k < 10)
      {
        if (command[i] == cm_sep || command[i] == cm_end)
          break;

        temp[k] = command[i];
        i++;
        k++;
      }
    return strtod(temp, NULL);
    }
  }
  
  return 0;
}


bool command_seen(char key)
{
  for (byte i=0; i<command_length; i++)
  {
    if (command[i] == key)
      return true;
  }
  
  return false;
}