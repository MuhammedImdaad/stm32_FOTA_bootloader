/*
 * gsm.c
 *
 *  Created on: Mar 18, 2021
 *      Author: Imdaad
 */

#include "main.h"
#include "gsm.h"


extern UART_HandleTypeDef huart1;
extern uint32_t firmwareSize;
extern uint8_t downloadedData[MAX_DOWNLOAD_SIZE+100];
extern uint16_t DOWNLOAD_SIZE;

uint8_t downloadState = 0;
uint8_t errorCount = 0;
/* turn on/off GSM module
 */
void Power_Toggle(void)
{
//	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	  HAL_Delay(2000);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	  HAL_Delay(TIME_FOR_MODULE_INIT);
}




/* custom function to send AT commands at ease
 */
void AT_Command(uint8_t *p_string)
{
  uint16_t length = 0;

  while (p_string[length] != '\0')
  {
    length++;
  }
  HAL_UART_Transmit(&huart1, p_string, length, TX_TIMEOUT);
}




/* Initialize the GSM module through a state machine. After running this http get request can be send.
 */
void GSM_Init(void){ //uint8_t Raddress, uint16_t Rsize

	uint8_t ongoing = 1;
	uint16_t size = 100;
	uint8_t gsmreply[size];
	char httpParaUrl[200];
	memset(httpParaUrl, 0, 200);
	memset(gsmreply, 0, size);
	while (ongoing) {
		switch (downloadState) {
		case 0:
			AT_Command((uint8_t*)"AT\r\n");
			HAL_UART_Receive(&huart1, gsmreply, size, RX_TIMEOUT1);
			if (strstr((char*)gsmreply, "OK")){
				downloadState = 1;
				errorCount=0;
			}
			else{
				downloadState = 0;
				errorCount++;
				if (errorCount>=MAX_ERROR_COUNT){
					Power_Toggle();
					Error_Handler();
				}
			}
			memset(gsmreply, 0, size);
			break;
		case 1:
			AT_Command((uint8_t*)"AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*)gsmreply, size, RX_TIMEOUT1);
			if (strstr((char*)gsmreply, "OK")){
				downloadState = 2;
				errorCount=0;
			}
			else{
				downloadState = 1;
				errorCount++;
				if (errorCount>=MAX_ERROR_COUNT){
					Power_Toggle();
					Error_Handler();
				}
			}
			memset(gsmreply, 0, size);
			break;
		case 2:
			AT_Command((uint8_t*)"AT+SAPBR=3,1,\"APN\",\"PPWAP\"\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*)gsmreply, size, RX_TIMEOUT1);
			if (strstr((char*)gsmreply, "OK")){
				downloadState = 3;
				errorCount=0;
			}
			else{
				downloadState = 2;
				errorCount++;
				if (errorCount>=MAX_ERROR_COUNT){
					Power_Toggle();
					Error_Handler();
				}
			}
			memset(gsmreply, 0, size);
			break;
		case 3:
			AT_Command((uint8_t*)"AT+SAPBR=1,1\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*)gsmreply, size, RX_TIMEOUT1);
			if (strstr((char*)gsmreply, "OK")){
				downloadState = 4;
				errorCount=0;
			}
			else{
				downloadState = 3;
				errorCount++;
				if (errorCount>=MAX_ERROR_COUNT){
					Power_Toggle();
					Error_Handler();
				}
			}
			memset(gsmreply, 0, size);
			break;
		case 4:
			AT_Command((uint8_t*)"AT+HTTPINIT\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*)gsmreply, size, RX_TIMEOUT1);
			if (strstr((char*)gsmreply, "OK")){
				downloadState = 5;
				errorCount=0;
			}
			else{
				downloadState = 4;
				errorCount++;
				if (errorCount>=MAX_ERROR_COUNT){
					Power_Toggle();
					Error_Handler();
				}
			}
			memset(gsmreply, 0, size);
			break;
		case 5:
			AT_Command((uint8_t*)"AT+HTTPPARA=\"CID\",1\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*)gsmreply, size, RX_TIMEOUT1);
			if (strstr((char*)gsmreply, "OK")){
				downloadState = 6;
				errorCount=0;
			}
			else{
				downloadState = 5;
				errorCount++;
				if (errorCount>=MAX_ERROR_COUNT){
					Power_Toggle();
					Error_Handler();
				}
			}
			memset(gsmreply, 0, size);
			break;
		case 6:
			sprintf(httpParaUrl, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", FILE_URL);
			AT_Command((uint8_t*)httpParaUrl);
			HAL_UART_Receive(&huart1, (uint8_t*)gsmreply, size, RX_TIMEOUT1);
			if (strstr((char*)gsmreply, "OK")){
				downloadState = 7;
				errorCount=0;
			}
			else{
				downloadState = 6;
				errorCount++;
				if (errorCount>=MAX_ERROR_COUNT){
					Power_Toggle();
					Error_Handler();
				}
			}
			memset(gsmreply, 0, size);
			break;
		case 7:
			AT_Command((uint8_t*)"AT+HTTPACTION=0\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*)gsmreply, size, RX_TIMEOUT2);
			/* GSM module will send the firmware size with the ACK
			 */
			if (strstr((char*)gsmreply, "200"))
				{
				errorCount=0;
				uint8_t y=0;
				while (gsmreply[y] != ',')
					y++;
				y++;
				while (gsmreply[y] != ',')
					y++;
				y++;
				uint8_t sizeIn = 0;
				char sizeBuff[5];
				while (gsmreply[y] != '\r') {
					sizeBuff[sizeIn] = gsmreply[y];
					sizeIn++;
					y++;
				}
				firmwareSize = atoi(sizeBuff);
				memset(gsmreply, 0, size);
				if ((firmwareSize==0) || (firmwareSize>MAX_FILE_SIZE) || (firmwareSize%4!=0)) {
					Power_Toggle();
					Error_Handler();
				}
				else ongoing=0;
				}
			else{
				errorCount++;
				if (errorCount>=MAX_ERROR_COUNT){
					Power_Toggle();
					Error_Handler();
				}
			}
			break;
		}
	}
}




/* custom function to get the firmware data
 */
void HTTP_Read(uint16_t Raddress, uint16_t Rsize, uint16_t header)
{
	char httpRequest[35];
	uint8_t val;
	uint8_t ok[4] = {'O', 'K', '\r', '\n'};
	sprintf(httpRequest, "AT+HTTPREAD=%d,%d\r\n", Raddress, Rsize);
	memset(downloadedData, 0, MAX_DOWNLOAD_SIZE+100);
	AT_Command((uint8_t*)httpRequest);
	HAL_UART_Receive(&huart1, (uint8_t*)downloadedData, DOWNLOAD_SIZE+100, RX_TIMEOUT_READ);
	for (int i=0; i<4; i++){
		val = downloadedData[header+Rsize+i+2];
		if (val == ok[i]){
			continue;
		}
		else{
			Power_Toggle();
			Error_Handler();
		}
	}

}
