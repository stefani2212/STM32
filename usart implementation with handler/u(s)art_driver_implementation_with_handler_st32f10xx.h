/*
	The specific MCU registers can be found in STM32f10xx headers. 
  @did not include them due to the fact that the files are just a sample of the initial project
  @for additional info. & for serial driver testing i recommend using the STM32CubeMX for generating project specific headers
	
	*/

/*  
	USART CHANNEL DEFINITION 
	*/
#define Un0		(1u)
#define Un1		(2u)
#define Un2		(3u)
#define Un3		(4u)
#define Un4		(5u)

/* 
	FUNCTION PROTOTYPES 
	*/
void Usart_Init(uint8_t Channel, uint32_t u32PbClk_, uint32_t u32BaudRate_);
void Usart_Read(uint8_t Channel, uint8_t* lpBuffer, uint8_t nNumberOfBytesToRead, uint8_t* lpNumberOfBytesRead);
void Usart_Write(uint8_t Channel, const uint8_t* lpBuffer, uint8_t nNumberOfBytesToWrite, uint8_t* lpNumberOfBytesWritten);
void Un_RxISR(uint8_t Channel, uint8_t Data);
void Un_TxISR(uint8_t Channel);	
uint8_t Uart_GetRxQueueLevel(uint8_t Channel, uint8_t* lpLevel);
uint8_t Uart_GetTxQueueLevel(uint8_t Channel, uint8_t* lpLevel);

void Uart_EnableWakeup(UInt8 Channel);
void Uart_DisableWakeup(UInt8 Channel);