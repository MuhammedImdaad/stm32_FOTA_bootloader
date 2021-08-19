/*
 * gsm.h
 *
 *  Created on: Mar 18, 2021
 *      Author: Imdaad
 */

#ifndef INC_GSM_H_
#define INC_GSM_H_


#define TIME_FOR_MODULE_INIT 10000
#define TX_TIMEOUT 1000
#define RX_TIMEOUT1 2000
#define RX_TIMEOUT2 8000
#define RX_TIMEOUT_READ 6000
#define MAX_ERROR_COUNT 3
#define FILE_URL "http://agri.senzmate.com:8082/user_app_L0.bin"
#define MAX_DOWNLOAD_SIZE (NO_PAGES_AT_ONCE*FLASH_PAGE_SIZE)
#define MAX_FILE_SIZE (FLASH_SIZE-32*1024)/2     //80*1024

void Power_Toggle(void);
void AT_Command(uint8_t *p_string);
void GSM_Init(void);
void HTTP_Read(uint16_t Raddress, uint16_t Rsize, uint16_t header);


#endif /* INC_GSM_H_ */
