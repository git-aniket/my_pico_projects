/**
 * @file NRF24.h
 * @author Aniket Mazumder (mazumder_aniket@hotmail.com)
 * @brief 
 * Class file for NRF24. Include this in main file to setup 
 * sender/reciever NRFradio's along with SPI at 1MHz.
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
#ifndef __NRF24__
#define __NRF24__

#include "hardware/spi.h"
#include "stdint.h"
#include "pico/stdlib.h"
#include "nRF24L01.h"
#include "stdbool.h"
#include "string.h"

class NRF24
{
    public:
	NRF24(spi_inst_t* port,uint8_t pin_cs,uint8_t pin_ce, uint8_t pin_miso, uint8_t pin_mosi,uint8_t pin_sck);
	~NRF24(){};

	//methods
	uint8_t read_register(uint8_t reg);
	void write_register(uint8_t reg, uint8_t data);
	void write_register(uint8_t reg, uint8_t *data, uint8_t size);
	void config_radio();

	void setmode_RX();
	void setmode_TX();
	void send_message(char* msg);
	void receive_message(char* msg);
	bool fifo_tx_empty();
	bool fifo_rx_full();

    private:
	//variables
	spi_inst_t* PORT;
	uint8_t PIN_CE;
	uint8_t PIN_CS;
	uint8_t PIN_MISO;
	uint8_t PIN_MOSI;
	uint8_t PIN_SCK;
	
	//methods
	void cs_low(){
	    gpio_put(this->PIN_CS,0);
	}

	void cs_high(){
	    gpio_put(this->PIN_CS,1);
	}

	void ce_low(){
	    gpio_put(this->PIN_CE,0);
	}

	void ce_high(){
	    gpio_put(this->PIN_CE,1);
	}

};
#endif
