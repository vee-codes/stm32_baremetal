void systick_config(uint32_t ticks_ms, uint32_t clock_speed){
	uint32_t load_val = (ticks_ms * clock_speed / MS_TO_SEC)- 1U;

	// clip to max valid value
	if (load_val > SYS_LOAD_MAX){
		load_val = SYS_LOAD_MAX;
	}

	SysTick->LOAD = load_val;
	// Write to current value register clears field to 0
	SysTick->VAL = 0;
	// Enable and set to processor clock
	SysTick->CTRL |= (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);

}