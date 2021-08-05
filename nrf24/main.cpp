/**
 * @file main.cpp
 * @author Aniket Mazumder (mazumder_aniket@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-08-05
 * 
 * @copyright Copyright Aniket Mazumder(c) 2021
 * 
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "string.h"
#include "hardware/spi.h"
#include "NRF24.h"

#define SPI_PORT_0 spi0
#define PIN_MISO_0 16//SPI_RX pin21
#define PIN_SCK_0  18//pin 24
#define PIN_MOSI_0 19//SPI_TX pin 25 
#define PIN_CS_0   17//pin 22
#define PIN_CE_0 20//pin 26

#define SPI_PORT_1 spi1
#define PIN_MISO_1 12//SPI_RX
#define PIN_SCK_1  14 
#define PIN_MOSI_1 15//SPI_TX
#define PIN_CS_1   13
#define PIN_CE_1 10

#define BUFF_SIZE 32
#define LED_PIN PICO_DEFAULT_LED_PIN

int main()
{
    stdio_init_all();

    //set LED_PIN
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN,GPIO_OUT);

    NRF24 sender(spi0,PIN_CS_0,PIN_CE_0,PIN_MISO_0,PIN_MOSI_0,PIN_SCK_0);
    NRF24 receiver(spi1,PIN_CS_1,PIN_CE_1,PIN_MISO_1,PIN_MOSI_1,PIN_SCK_1);

    sender.setmode_TX();
    receiver.setmode_RX();

    char buff_send[BUFF_SIZE]={'\0'};
    char buff_receive[BUFF_SIZE]={'\0'};

    while(1)
    {
        //printf("%d %d\n", sender.read_register(CONFIG),receiver.read_register(CONFIG));

        sprintf(buff_send,"Hello Aniket.\n");
        if(sender.fifo_tx_empty())
        {
            sender.send_message(buff_send);
            printf("sent: %s\n", buff_send);
        }

        //Check if the data buffer is filled with 32 bytes
        if(receiver.fifo_rx_full())
        {
            receiver.receive_message(buff_receive);
            printf("Received: %s\n",buff_receive);
        }

	    sleep_ms(100);
    }

    return 0;
}