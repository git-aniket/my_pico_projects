/**
 * @file NRF24.cpp
 * @author Aniket Mazumder (mazumder_aniket@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-08-05
 * Some of the code from: 
 * https://github.com/guser210/NRFDemo
 * has been used for this demonstration.
 * Descriptions about this repo can be found at:
 * https://www.youtube.com/watch?v=V4ziwen24Ps&t=2319s
 * 
 * Modified by Aniket Mazumder
 * @copyright Copyright Aniket Mazumder(c) 2021
 * 
 */
#include "NRF24.h"


NRF24::NRF24(spi_inst_t* port, uint8_t pin_cs, uint8_t pin_ce, uint8_t pin_miso, uint8_t pin_mosi, uint8_t pin_sck){
    this->PORT=port;
    this->PIN_CE=pin_ce;
    this->PIN_CS=pin_cs;
    this->PIN_MOSI=pin_mosi;
    this->PIN_MISO=pin_miso;
    this->PIN_SCK=pin_sck;
    
    //SPI initialisation at 1MHz.
    spi_init(this->PORT, 1000*1000);

    //set functions for the SPI pins
    gpio_set_function(this->PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(this->PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(this->PIN_MOSI, GPIO_FUNC_SPI);
    
    //set the CS and CE pins
    gpio_init(this->PIN_CS);
    gpio_init(this->PIN_CE);
    gpio_set_dir(this->PIN_CS, GPIO_OUT);
    gpio_set_dir(this->PIN_CE, GPIO_OUT);

	//configure the radio settings for power, data length etc
	this->config_radio();

    //device is active low, so drive CS high to deactivate
    ce_low();
    cs_high();
}


uint8_t NRF24::read_register(uint8_t reg){
    uint8_t result=0;
    //take only the 5 LSB's of the register
    reg=0b00011111 & reg;

    cs_low();
    //First write to spi the register to read from device
    spi_write_blocking(this->PORT, &reg, 1);
    //store data from register into result
    spi_read_blocking(this->PORT,NOP, &result, 1);
    cs_high();

    return result;
}


void NRF24::write_register(uint8_t reg, uint8_t* data, uint8_t size){
    //take only the 5 LSB's of the register
    reg=0b00100000|(0b00011111&reg);
    cs_low();
    spi_write_blocking(this->PORT,&reg,1);
    spi_write_blocking(this->PORT,(uint8_t*)data, size);
    cs_high();
}

void NRF24::write_register(uint8_t reg, uint8_t data){
    write_register(reg, &data, 1);
}



void NRF24::config_radio(){
    //power on reset sleep
    sleep_ms(100);
    //set PWR_UP and EN_CRC =1
	write_register(CONFIG, CONFIG|(1<<PWR_UP)|(1<<EN_CRC));
    //sleep for 1.5ms
    sleep_us(1500);
    //configure for no-acknowledgement
    write_register(EN_AA, 0);
    //enable data pipes 0 and 1
    write_register(EN_RXADDR, (EN_RXADDR)|(1<<ERX_P0)|(1<<ERX_P1));
    //setup of auto-retransmission to 4000uS and 15 retransmits
    write_register(SETUP_RETR,0xff);
    //select communication channel 60 or any channel without interference
    write_register(RF_CH,60);
    //select RF_power
    write_register(RF_SETUP,RF_SETUP|(RF_PWR_HIGHEST<<1));
    //select and name data pipe0
    write_register(RX_ADDR_P0, (uint8_t*)"pipec",5);
    write_register(TX_ADDR,(uint8_t*)"pipec",5);
    //select number of bytes in RX payload
    write_register(RX_PW_P0,32);
}


void NRF24::setmode_TX(){
    uint8_t reg=read_register(CONFIG);
    reg&=~(1<<PRIM_RX);
    write_register(CONFIG,reg);
    ce_high();
    sleep_us(130);
}


void NRF24::setmode_RX(){
    uint8_t reg=read_register(CONFIG);
    reg|=(1<<PRIM_RX);
    write_register(CONFIG,reg);
    ce_high();
    sleep_us(130);
}


void NRF24::send_message(char* msg){
    uint8_t reg=W_TX_PAYLOAD;
    cs_low();
	//write the write_payload command to the device
    spi_write_blocking(this->PORT, &reg,1);
	//write the message to the device
    spi_write_blocking(this->PORT, (uint8_t*)msg,strlen(msg));
    cs_high();

	//pulse CE
    ce_high();
    sleep_us(20);
    ce_low();
}


void NRF24::receive_message(char* msg){
    uint8_t reg=R_RX_PAYLOAD;
    cs_low();
	//write the read_payload command to the device
    spi_write_blocking(this->PORT,&reg, 1);
	//read the message from the device
    spi_read_blocking(this->PORT,0xff, (uint8_t*)msg,32);
    cs_high();
}


bool NRF24::fifo_rx_full(){
    bool status=true;
    uint8_t result=read_register(FIFO_STATUS);
    result=result&(1<<RX_FULL);
    return status;
}


bool NRF24::fifo_tx_empty(){
    bool status=true;
    uint8_t result=read_register(FIFO_STATUS);
    result=result&(1<<TX_EMPTY);
    return result;
}




