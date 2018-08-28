#include "SBUS2.h"
#include "SBUS_usart.h"

#define CURRENT_SLOT      3
#define RPM_SLOT          2
#define ERROR_SLOT        6
#define TEMPRATURE_SLOT   1
#define NUMBER_OF_POOLS   14
#define CURRENT_CAL       100

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  
  SBUS2_Setup(CURRENT_SLOT, TEMPRATURE_SLOT, RPM_SLOT, ERROR_SLOT, CURRENT_CAL, NUMBER_OF_POOLS);
                  
  while(1){
    SBUS2_loop();
  }
  
}
