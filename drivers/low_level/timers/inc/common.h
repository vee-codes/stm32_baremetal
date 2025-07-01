#ifndef COMMON_H
#define COMMON_H


void delay_ms(int delay){
	for(int i=0;i<delay;i++){
		while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
	}
}

void delay_sec(int delay){
    delay_ms(1000*delay);
}


#endif
