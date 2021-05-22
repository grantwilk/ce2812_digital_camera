/**
  * @file dcmi_driver.h
  * @author Grant Wilk
  * @created 2/11/2020
  * @modified 2/11/2020
  * @desc 
  */

#include <stm32f446xx.h>

#ifndef DCMI_DRIVER_H
#define DCMI_DRIVER_H
#endif

/**
 * Initializes the DCMI
 */
void dcmi_init(void);

/**
 * Captures an image
 * @return a pointer to the image buffer
 */
void dcmi_capture();

/**
 * Returns a pointer to the image buffer
 * @return a pointer to the image buffer
 */
uint8_t * dcmi_get_img_buf();

/**
 * Returns the size of the image buffer in bytes
 * @return the size of the image buffer in bytes
 */
uint32_t dcmi_get_img_buf_size();