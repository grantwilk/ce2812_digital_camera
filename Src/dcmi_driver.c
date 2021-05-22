/**
  * @file dcmi_driver.c
  * @author Grant Wilk
  * @created 2/11/2020
  * @modified 2/11/2020
  * @desc digital camera interface driver
  */

# include "stm32f446xx.h"
# include "stm32f4xx_hal.h"
# include "dcmi_driver.h"
# include "uart_print.h"

# define GPIO_AF01_TIM2 0x01U

# define DCMI_CR_BSM_Pos 17U
# define DCMI_CR_BSM_EOB 0x01

# define DMA_SxCR_DIR_P2M 0x00U
# define DMA_SxCR_DIR_M2P 0x01U
# define DMA_SxCR_DIR_M2M 0x02U

# define DMA_SxCR_CHSEL_CH1 0x01U
# define DMA_SxCR_SIZE_WORD 0x02U

# define IMG_WIDTH 640 // px
# define IMG_HEIGHT 480 // px
# define IMG_PIXEL_WIDTH 1 // bytes

# define IMG_CROP_WIDTH 256 // px
# define IMG_CROP_HEIGHT 256 // px
# define IMG_CROP_VOFFSET 0 // ((IMG_HEIGHT - IMG_CROP_HEIGHT) / 2) // px
# define IMG_CROP_HOFFSET 0 // ((IMG_WIDTH - IMG_CROP_WIDTH) / 2) // px

# define IMG_BUFF_BYTES (IMG_CROP_WIDTH * IMG_CROP_HEIGHT) // bytes
# define IMG_BUFF_WORDS (IMG_BUFF_BYTES / 4) // 32-bit words

static uint8_t img_buffer[IMG_BUFF_BYTES];
static uint8_t frame_complete = 0;


/**
 * Initializes the DCMI
 */
void dcmi_init(void) {

    // enable GPIOA-GPIOC in RCC
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);

    // enable DCMI, DMA2, and TIM2 in RCC
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // set mode of DCMI-tied GPIO pins to alternate function mode
    GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE6 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
    GPIOA->MODER |= (GPIO_MODE_AF_PP << GPIO_MODER_MODE4_Pos) |
                    (GPIO_MODE_AF_PP << GPIO_MODER_MODE6_Pos) |
                    (GPIO_MODE_AF_PP << GPIO_MODER_MODE9_Pos) |
                    (GPIO_MODE_AF_PP << GPIO_MODER_MODE10_Pos);

    GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7 | GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOB->MODER |= (GPIO_MODE_AF_PP << GPIO_MODER_MODE6_Pos) |
                    (GPIO_MODE_AF_PP << GPIO_MODER_MODE7_Pos) |
                    (GPIO_MODE_AF_PP << GPIO_MODER_MODE8_Pos) |
                    (GPIO_MODE_AF_PP << GPIO_MODER_MODE9_Pos);

    GPIOC->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7 | GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE11);
    GPIOC->MODER |= (GPIO_MODE_AF_PP << GPIO_MODER_MODE6_Pos) |
                    (GPIO_MODE_AF_PP << GPIO_MODER_MODE7_Pos) |
                    (GPIO_MODE_AF_PP << GPIO_MODER_MODE8_Pos) |
                    (GPIO_MODE_AF_PP << GPIO_MODER_MODE9_Pos) |
                    (GPIO_MODE_AF_PP << GPIO_MODER_MODE11_Pos);

    // set alternate functions of DCMI-tied GPIO pins to DCMI
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL6);
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10);
    GPIOA->AFR[0] |= (GPIO_AF13_DCMI << GPIO_AFRL_AFSEL4_Pos) | (GPIO_AF13_DCMI << GPIO_AFRL_AFSEL6_Pos);
    GPIOA->AFR[1] |= (GPIO_AF13_DCMI << GPIO_AFRH_AFSEL9_Pos) | (GPIO_AF13_DCMI << GPIO_AFRH_AFSEL10_Pos);

    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9);
    GPIOB->AFR[0] |= (GPIO_AF13_DCMI << GPIO_AFRL_AFSEL6_Pos) | (GPIO_AF13_DCMI << GPIO_AFRL_AFSEL7_Pos);
    GPIOB->AFR[1] |= (GPIO_AF13_DCMI << GPIO_AFRH_AFSEL8_Pos) | (GPIO_AF13_DCMI << GPIO_AFRH_AFSEL9_Pos);

    GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
    GPIOC->AFR[1] &= ~(GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL11);
    GPIOC->AFR[0] |= (GPIO_AF13_DCMI << GPIO_AFRL_AFSEL6_Pos) | (GPIO_AF13_DCMI << GPIO_AFRL_AFSEL7_Pos);
    GPIOC->AFR[1] |= (GPIO_AF13_DCMI << GPIO_AFRH_AFSEL8_Pos) |
                     (GPIO_AF13_DCMI << GPIO_AFRH_AFSEL9_Pos) |
                     (GPIO_AF13_DCMI << GPIO_AFRH_AFSEL11_Pos);

    // set mode of TIM2_CH1-tied GPIO pin to alternate function, then set alternate function to TIM2_CH1
    GPIOA->MODER &= ~(GPIO_MODER_MODE0);
    GPIOA->MODER |= (GPIO_MODE_AF_PP << GPIO_MODER_MODE0_Pos);
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL0);
    GPIOA->AFR[0] |= (GPIO_AF01_TIM2 << GPIO_AFRL_AFSEL0_Pos);

    // configure TIM2 to oscillate TIM2_CH1 at 8MHz
    TIM2->CCMR1 |= (0b011 << TIM_CCMR1_OC1M_Pos);
    TIM2->CCER |= TIM_CCER_CC1E;
    TIM2->ARR = 1;
    TIM2->CR1 |= TIM_CR1_CEN;

    // configure DMA
    DMA2_Stream1->CR |= (DMA_SxCR_CHSEL_CH1 << DMA_SxCR_CHSEL_Pos); // Channel Selection = 1, channel 1
    DMA2_Stream1->CR |= DMA_SxCR_PL_Msk; // Priority Level = 3, very high

    DMA2_Stream1->CR |= (DMA_SxCR_SIZE_WORD << DMA_SxCR_MSIZE_Pos); // Memory Data Size = 2, word (32-bit)
    DMA2_Stream1->CR |= (DMA_SxCR_SIZE_WORD << DMA_SxCR_PSIZE_Pos); // Peripheral Data Size = 2, word (32-bit)
    DMA2_Stream1->CR |= DMA_SxCR_MINC; // Memory Increment Mode = 1, memory pointer is incremented after each transfer

    DMA2_Stream1->CR |= (DMA_SxCR_DIR_P2M << DMA_SxCR_DIR_Pos); // Data Transfer Direction = 0, peripheral-to-memory

    DMA2_Stream1->PAR = (uint32_t) &(DCMI->DR); // Peripheral Address Register = DCMI Data Register (DCMI_DR)
	DMA2_Stream1->M0AR = (uint32_t) img_buffer; // Memory Image Buffer

    // configure DCMI control register (CR)
	DCMI->CR |= DCMI_CR_OEBS; // Odd/Even Byte Select = 1, interface captures second data (odd bytes)
    DCMI->CR |= (DCMI_CR_BSM_EOB << DCMI_CR_BSM_Pos); // Byte Select Mode = 1, every other byte
    DCMI->CR |= DCMI_CR_PCKPOL; // Pixel Clock Polarity = 1, rising edge active
    DCMI->CR |= DCMI_CR_VSPOL; // Vertical Synchronization Polarity = 1, DCMI_VSYNC active high
    DCMI->CR |= DCMI_CR_CROP; // Crop Feature = 1, only the data inside the crop window will be captured
    DCMI->CR |= DCMI_CR_CM; // Capture Mode = 1, snapshot mode (single frame)

    // configure DCMI crop
    DCMI->CWSTRTR |= (IMG_CROP_VOFFSET << DCMI_CWSTRT_VST_Pos);
    DCMI->CWSTRTR |= (IMG_CROP_HOFFSET << DCMI_CWSTRT_HOFFCNT_Pos);
    DCMI->CWSIZER |= (IMG_CROP_HEIGHT << DCMI_CWSIZE_VLINE_Pos);
    DCMI->CWSIZER |= (IMG_CROP_WIDTH << DCMI_CWSIZE_CAPCNT_Pos);

    // other DCMI control register configurations
    // commented out because these are the reset values of the registers
    // DCMI->CR &= ~(DCMI_CR_HSPOL); // Horizontal Synchronization Polarity = 0, DCMI_HSYNC active low
    // DCMI->CR &= ~(DCMI_CR_EDM_0 | DCMI_CR_EDM_1); // Extended Data Mode = 0, interface captures 8-bit data
    // DCMI->CR &= ~(DCMI_CR_ESS); // Embedded Synchronization Select = 0, Hardware synchronization data capture

    // enable the DCMI after ~ALL~ DCMI parameters are configured
    DCMI->CR |= DCMI_CR_ENABLE;

    // enable DCMI interrupts in NVIC
    NVIC->ISER[2] = (1 << 14);

}

/**
 * Captures an image
 * @return a pointer to the image buffer
 */
void dcmi_capture() {

    // reset frame complete flag
    frame_complete = 0;

    // clear the DMA transfer complete interrupts
    DMA2->LIFCR = DMA_LIFCR_CTCIF1;
    DMA2->LIFCR = DMA_LIFCR_CHTIF1;

    // set data to transfer
    DMA2_Stream1->NDTR = (IMG_BUFF_WORDS); // Number of Data Transfer = # words in an image

    // enable frame complete interrupts
    DCMI->IER |= DCMI_IER_FRAME_IE;

    // enable the DMA stream
    DMA2_Stream1->CR |= DMA_SxCR_EN;

    // enable DCMI to capture a frame only ~ALL~ DMA parameters are configured (in dcmi_init)
    DCMI->CR |= DCMI_CR_CAPTURE;

    // poll until frame complete flag is set high
    while (!(frame_complete));

    // terminate the DMA transfer if it hasn't already stopped
    DMA2_Stream1->CR &= ~(DMA_SxCR_EN);

    // disable frame complete interrupts
    DCMI->ICR = DCMI_ICR_FRAME_ISC;

}

/**
 * Returns a pointer to the image buffer
 * @return a pointer to the image buffer
 */
uint8_t * dcmi_get_img_buf() {
    return img_buffer;
}

/**
 * Returns the size of the image buffer in bytes
 * @return the size of the image buffer in bytes
 */
uint32_t dcmi_get_img_buf_size() {
    return IMG_BUFF_BYTES;
}

/**
 * DCMI IRQ Handler
 */
void DCMI_IRQHandler(void) {
    // set frame complete flag if frame complete interrupt received
    if (DCMI->RISR & DCMI_RISR_FRAME_RIS) {
        frame_complete = 1;
        // clear interrupt from DCMI
        DCMI->ICR = DCMI_ICR_FRAME_ISC;
    }
}