void spi_send_msg(uint8_t reg_addr, uint8_t data){
    GPIOA->ODR &= ~(GPIO_ODR_OD4);

    uint8_t tmp[2] = {reg_addr, data};
    for(int i =0;i<2;i++){
        while(!(SPI1->SR & SPI_SR_TXE));
        *((volatile uint8_t *)&SPI1->DR) = tmp[i];

    }
    while(SPI1->SR & SPI_SR_BSY);
    GPIOA->ODR |= (GPIO_ODR_OD4);
}