#include <Arduino.h>
#include <command.h>
#include <motion.h>





void setup() 
{
  Serial.begin(115200);
  setup_motion();


  trajy_para param={0.5,1,40,0,20,20,0};
  trajy_planning(param);

  delay(5000);
  gait_transit(0,20);


}

void loop() 
{

  gait_loop();

  //cli_loop();


}





