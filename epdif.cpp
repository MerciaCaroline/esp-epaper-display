/**
 *  @filename   :   epdif.cpp
 *  @brief      :   Implements EPD interface functions
 *                  Users have to implement all the functions in epdif.cpp
 *  @author     :   Yehui from Waveshare
 *
 *  Copyright (C) Waveshare     August 10 2017
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documnetation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to  whom the Software is
 * furished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "epdif.h"

/**
 * @brief Escreve um valor lógico (HIGH/LOW) em um pino GPIO.
 * 
 * @param pin Número do pino GPIO.
 * @param value Valor lógico a ser escrito (0 ou 1).
 */
void EpdIf::DigitalWrite(unsigned int pin, int value) {
    // ESP_LOGI("EPDIF", "Set Pin %i: %i", pin, value);
    gpio_set_level((gpio_num_t)pin, value);
}

/**
 * @brief Lê o valor lógico de um pino GPIO configurado como entrada.
 * 
 * @param pin Número do pino GPIO.
 * @return Valor lógico do pino (0 ou 1).
 */
int EpdIf::DigitalRead(unsigned int pin) {
    int level = gpio_get_level((gpio_num_t)pin);
    return level;
}

/**
 * @brief Aguarda um tempo em milissegundos.
 * 
 * Pode ser implementado usando delay do sistema ou timers.
 * 
 * @param delaytime Tempo em milissegundos para aguardar.
 */
void EpdIf::DelayMs(unsigned int delaytime) {
    vTaskDelay(delaytime / portTICK_PERIOD_MS);
}

/**
 * @brief Envia um byte via SPI.
 * 
 * Deve realizar a transferência SPI para comunicar com o display.
 * 
 * @param data Byte a ser enviado.
 */
void EpdIf::SpiTransfer(unsigned char data) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 8;        // transaction length is in bits
    t.tx_data[0] = data;
    t.tx_data[1] = data;
    t.tx_data[2] = data;
    t.tx_data[3] = data;
    ret = spi_device_transmit(spi_handle, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

void EpdIf::AddDevice(int cs_pin = CS_PIN) {

    // Adição do dispositivo SPI (EPD)
    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.command_bits = 0;
    devcfg.address_bits = 0;
    devcfg.dummy_bits = 0;
    devcfg.clock_speed_hz = 2*1000*1000;
    devcfg.mode = 0;
    devcfg.spics_io_num = cs_pin;
    devcfg.queue_size = 1;

    esp_err_t ret;
    //Attach the EPD to the SPI bus
    #if ESP_IDF_VERSION_MAJOR >= 4
        ret=spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);
    #else
        ret=spi_bus_add_device(SPI_HOST, &devcfg, &spi_handle);
    #endif

    assert(ret==ESP_OK);
}

/**
 * @brief Inicializa o barramento SPI e configura o dispositivo SPI.
 * 
 * @return int,  0 em sucesso, código de erro caso contrário.
 */
int EpdIf::SpiInit() {
    // Liberação de recursos SPI existentes
    if(spi_handle) {
        spi_bus_remove_device(spi_handle);
        
    }

    #if ESP_IDF_VERSION_MAJOR < 4
        spi_bus_free(SPI_HOST); // This generates error on ESP-IDF >= v4.2
    #endif

    esp_err_t ret;
    // Configuração do barramento SPI
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.mosi_io_num = MOSI_PIN;
    buscfg.sclk_io_num = CLK_PIN;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    //Initialize the SPI bus
    #if ESP_IDF_VERSION_MAJOR >= 4
        ret=spi_bus_initialize(SPI2_HOST, &buscfg, 0);
    #else
        ret=spi_bus_initialize(SPI_HOST, &buscfg, 0);
    #endif
    switch (ret) {
        case ESP_ERR_INVALID_ARG:
            ESP_LOGE("EPDIF", "INVALID ARG");
            break;
        case ESP_ERR_INVALID_STATE:
            ESP_LOGE("EPDIF", "INVALID STATE");
            break;
        case ESP_ERR_NO_MEM:
            ESP_LOGE("EPDIF", "INVALID NO MEMORY");
            break;
        case ESP_OK:
            ESP_LOGE("EPDIF", "All OK");
    }
    assert(ret==ESP_OK);

    return 0;
}

/**
 * @brief Inicializa a interface SPI e os pinos GPIO necessários.
 * 
 * @param spi_initialize Flag para inicializar ou não o SPI, pode ja ter sido inicializado
 * @return 0 em sucesso, código de erro caso contrário.
 */
int EpdIf::IfInit(bool spi_initialize) {

    // Configuração dos GPIOs de controle
    // Pinos de saída (DC, RST)
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((uint64_t)1<<(uint64_t)DC_PIN) | ((uint64_t)1<<(uint64_t)RST_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    //Pino de entrada (BUSY)
    gpio_config_t i_conf;
    memset(&i_conf, 0, sizeof(i_conf));
    i_conf.intr_type = GPIO_INTR_DISABLE;
    i_conf.mode = GPIO_MODE_INPUT;
    i_conf.pin_bit_mask = ((uint64_t)1<<(uint64_t)BUSY_PIN);
    i_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    i_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&i_conf));

    if (!spi_initialize) {
        ESP_LOGI("EPDIF", "Skipping SPI initialization.");
    }
    else{
        SpiInit(); // Initialize SPI bus
        ESP_LOGI("EPDIF", "SPI initialized. Cs pin: %d, MOSI: %d, CLK: %d", CS_PIN, MOSI_PIN, CLK_PIN);
    }

    AddDevice(); // Add EPD device to SPI bus

    return 0;
}

