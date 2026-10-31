// ****************************************************************
// * Authors: T. LUM and R. MARTIN
// * CPEG222 Demo 11 PDM microphone, 10/15/26
// * NucleoF466RE CMSIS STM32F4xx example
// * Read temperature, pressure, and humidity from the BME280 sensor 
// * and display the values on the OLED.
// ****************************************************************
#include "stm32f4xx.h" // CMSIS STM32F4xx header file
#include "SysClock180MHz.h" // System clock configuration for 180 MHz
#include "UART3.h" // UART3 configuration for communication
#include <stdint.h>
#include <math.h>   // For sqrtf()
#include <stdio.h>  // For snprintf

#define LED_PORT        GPIOD
#define LED1_PIN         0  // Assuming LED1 is connected to GPIOD pin 0
#define LED2_PIN         1  // Assuming LED2 is connected to GPIOD pin 1
#define LED3_PIN         2  // Assuming LED3 is connected to GPIOD pin 2
#define LED4_PIN         3  // Assuming LED4 is connected to GPIOD pin 3
#define LED5_PIN         4  // Assuming LED5 is connected to GPIOD pin 4
#define LED6_PIN         5  // Assuming LED6 is connected to GPIOD pin 5
#define LED7_PIN         6  // Assuming LED7 is connected to GPIOD pin 6
#define LED8_PIN         7  // Assuming LED8 is connected to GPIOD pin 7
#define PDM_BUFFER_SIZE  128  // Raw 16-bit word buffer size
#define PDM_BITS_PER_WORD 16U
#define PDM_DECIMATION_FACTOR 64U
#define PCM_BUFFER_SIZE \
    ((PDM_BUFFER_SIZE * PDM_BITS_PER_WORD) / PDM_DECIMATION_FACTOR)

uint16_t pdm_raw_buffer[PDM_BUFFER_SIZE];
int16_t  pcm_audio_buffer[PCM_BUFFER_SIZE];

// Integrate 64 PDM bits and decimate to one signed PCM sample.
int16_t PDM_Decimate64(const uint16_t *pdm_words) {
    uint32_t ones = 0;
    for (uint32_t word = 0; word < PDM_DECIMATION_FACTOR / PDM_BITS_PER_WORD; word++) {
        uint16_t pdm_word = pdm_words[word];
        for (uint32_t bit = 0; bit < PDM_BITS_PER_WORD; bit++) {
            ones += (pdm_word >> bit) & 1U;
        }
    }

    int32_t centered = (int32_t)(ones * 2U) - (int32_t)PDM_DECIMATION_FACTOR;
    return (int16_t)((centered * 32767) / (int32_t)PDM_DECIMATION_FACTOR);
}

void PDM_Microphone_Init(void) {
    // 1. Enable Clocks for GPIOB and SPI2 (SPI2 is on APB1)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // 2. Configure PB13 (CLK) and PB15 (DATA) for Alternate Function (AF5 for SPI2)
    GPIOB->MODER   &= ~((3U << (13 * 2)) | (3U << (15 * 2)));
    GPIOB->MODER   |=  ((2U << (13 * 2)) | (2U << (15 * 2))); // AF Mode
    GPIOB->OSPEEDR |=  ((3U << (13 * 2)) | (3U << (15 * 2))); // High Speed

    // Pins 8-15 use AFR[1]; pin n occupies bits ((n - 8) * 4).
    GPIOB->AFR[1]  &= ~((0xFU << ((13 - 8) * 4)) | (0xFU << ((15 - 8) * 4)));
    GPIOB->AFR[1]  |=  ((0x5U << ((13 - 8) * 4)) | (0x5U << ((15 - 8) * 4))); // AF5

    // 3. Configure SPI2 as master, receive-only on the MOSI pin (PB15)
    // using 1-line bidirectional mode, with software NSS held high.
    SPI2->CR1 &= ~SPI_CR1_SPE; // Disable SPI2 before configuration

    // PCLK1 / 16 = 2.8125 MHz when PCLK1 is 45 MHz (180 MHz SYSCLK, APB1 /4).
    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1 |
                SPI_CR1_BIDIMODE | SPI_CR1_DFF | SPI_CR1_SSM | SPI_CR1_SSI;

    // STM32F4 selects 16-bit frames with SPI_CR1_DFF; SPI_CR2 has no DS field.
    SPI2->CR2 = SPI_CR2_RXDMAEN;                             // Enable RX DMA Request; frame size uses CR1.DFF
}

void PDM_To_PCM(void) {
    for (uint32_t i = 0; i < PCM_BUFFER_SIZE; i++) {
        uint32_t pdm_index = i * (PDM_DECIMATION_FACTOR / PDM_BITS_PER_WORD);
        pcm_audio_buffer[i] = PDM_Decimate64(&pdm_raw_buffer[pdm_index]);
    }
}

void DMA1_Stream3_Init(void) {
    // Enable DMA1 Clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // SPI2_RX is DMA1 Stream 3, Channel 0 on STM32F446.
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream3->CR & DMA_SxCR_EN) {
    }

    DMA1->LIFCR = DMA_LIFCR_CFEIF3 | DMA_LIFCR_CDMEIF3 |
                  DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 |
                  DMA_LIFCR_CTCIF3;

    DMA1_Stream3->PAR  = (uint32_t)&(SPI2->DR);
    DMA1_Stream3->M0AR = (uint32_t)pdm_raw_buffer;
    DMA1_Stream3->NDTR = PDM_BUFFER_SIZE;
    DMA1_Stream3->FCR  = 0;
    DMA1_Stream3->CR   = DMA_SxCR_CIRC | DMA_SxCR_MINC |  // CHSEL = 0
                         DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 |
                         DMA_SxCR_TCIE;

    // Enable DMA1 Stream 3
    DMA1_Stream3->CR |= DMA_SxCR_EN;

    // Start SPI after DMA is ready so received data has a destination.
    SPI2->CR1 |= SPI_CR1_SPE;
}

int main(void) {
    SystemClock_Config();// Configure system clock to 180 MHz
    UART3_Init();
    uart3_send_string("\nCPEG222 PDM Microphone Test F26!\r\n");

    // 1. Initialize System & Peripherals
    PDM_Microphone_Init();
    DMA1_Stream3_Init();
    // 2. Initialize LED pins
    // Enable GPIOD clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    // Configure LED pins as outputs
    LED_PORT->MODER &= ~(0xFFFF); // Clear mode bits for pins 0-7
    LED_PORT->MODER |= (0x5555);  // Set mode to output for pins 0-7

    while (1) {
        // Task 1: Poll and clear the         DMA1 Stream 3 transfer-complete flag.
        if (DMA1->LISR & DMA_LISR_TCIF3) { // Check if the transfer complete flag for DMA1 Stream 3 is set
            DMA1->LIFCR = DMA_LIFCR_CTCIF3; // Clear the transfer complete flag for DMA1 Stream 3
            // Task 2: Integrate each 64-bit PDM group into one PCM sample.
            PDM_To_PCM();
            // Task 3: Calculate RMS audio magnitude.
            uint64_t sum_squares = 0;
            for (uint32_t i = 0; i < PCM_BUFFER_SIZE; i++) {
                int32_t sample = pcm_audio_buffer[i];
                sum_squares += (uint64_t)(sample * sample);
            }
            float rms =  sqrtf((float)sum_squares / PCM_BUFFER_SIZE);
            rms = rms/8000; //rms / 32767.0f; // Normalize RMS to range [0, 1] based on 16-bit signed PCM
            int ledState = (1 << (int)(rms*8)) - 1; // Create a bitmask with 'magnitude' number of bits set
            LED_PORT->ODR = (LED_PORT->ODR & ~0xFF) | (ledState & 0xFF); // Update only the lower 8 bits for LEDs
            char buffer[90];
            snprintf(buffer, sizeof(buffer), "  rms : %.2f      \r\n", rms);   
            uart3_send_string(buffer);
        }
    }
    return 0;
}
