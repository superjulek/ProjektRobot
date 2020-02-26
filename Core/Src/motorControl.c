#include "motorControl.h"
#include  "usart.h"

extern PR, PL, napiecie[2], natezenie[2];
extern uint8_t tekst[30];
extern uint16_t dl_tekst;

void wyswietlMoc(void){
}
void doPrzodu(){
    	 HAL_GPIO_WritePin(RF_GPIO_Port, RF_Pin, GPIO_PIN_SET);
    	 HAL_GPIO_WritePin(LF_GPIO_Port, LF_Pin, GPIO_PIN_SET);
    	 HAL_GPIO_WritePin(RB_GPIO_Port, RB_Pin, GPIO_PIN_RESET);
    	 HAL_GPIO_WritePin(LB_GPIO_Port, LB_Pin, GPIO_PIN_RESET);

     }
void doTylu(){
    	 HAL_GPIO_WritePin(RF_GPIO_Port, RF_Pin, GPIO_PIN_RESET);
    	 HAL_GPIO_WritePin(LF_GPIO_Port, LF_Pin, GPIO_PIN_RESET);
    	 HAL_GPIO_WritePin(RB_GPIO_Port, RB_Pin, GPIO_PIN_SET);
    	 HAL_GPIO_WritePin(LB_GPIO_Port, LB_Pin, GPIO_PIN_SET);

     }

void wLewo(){
    	 HAL_GPIO_WritePin(RF_GPIO_Port, RF_Pin, GPIO_PIN_SET);
    	 HAL_GPIO_WritePin(LF_GPIO_Port, LF_Pin, GPIO_PIN_RESET);
    	 HAL_GPIO_WritePin(RB_GPIO_Port, RB_Pin, GPIO_PIN_RESET);
    	 HAL_GPIO_WritePin(LB_GPIO_Port, LB_Pin, GPIO_PIN_SET);

     }

void wPrawo(){
    	 HAL_GPIO_WritePin(RF_GPIO_Port, RF_Pin, GPIO_PIN_RESET);
    	 HAL_GPIO_WritePin(LF_GPIO_Port, LF_Pin, GPIO_PIN_SET);
    	 HAL_GPIO_WritePin(RB_GPIO_Port, RB_Pin, GPIO_PIN_SET);
    	 HAL_GPIO_WritePin(LB_GPIO_Port, LB_Pin, GPIO_PIN_RESET);

     }

void stoj(){
    	 HAL_GPIO_WritePin(RF_GPIO_Port, RF_Pin, GPIO_PIN_RESET);
    	 HAL_GPIO_WritePin(LF_GPIO_Port, LF_Pin, GPIO_PIN_RESET);
    	 HAL_GPIO_WritePin(RB_GPIO_Port, RB_Pin, GPIO_PIN_RESET);
    	 HAL_GPIO_WritePin(LB_GPIO_Port, LB_Pin, GPIO_PIN_RESET);

     }

