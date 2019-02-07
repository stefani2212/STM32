/*!
@as reference were used the specific headers for STMF10xx	 
@USART_DRIVER_IMPLEMENTATION_STM32F10xx
 */

#define UART_TOTAL_NUMBER_OF_CHANNELS  (5u)
 
  #define UART_TX_BUF_SIZE_MASK                   (UART_TX_BUF_SIZE - 1u)
#if (UART_TX_BUF_SIZE & UART_TX_BUF_SIZE_MASK)
#error UART_TX_BUF_SIZE is not power of 2
#endif

#define UART_RX_BUF_SIZE_MASK                   (UART_RX_BUF_SIZE - 1u)
#if (UART_RX_BUF_SIZE & UART_RX_BUF_SIZE_MASK)
#error UART_RX_BUF_SIZE is not power of 2
#endif

/* Local Prototypes */
typedef enum {
	FALSE,
	TRUE
}BOOL;

/*! \struct Un_DataType
    \brief The Uart data type instance definition.
 */
 typedef struct 
{
	uint8_t u8RxBuff[UART_RX_BUF_SIZE];   /**< The RX data buffer */
    uint8_t u8TxBuff[UART_TX_BUF_SIZE];   /**< The TX data buffer */
    uint16_t u16TxHead;   /**< Holds the head within TX buffer */
    uint16_t u16TxTail;   /**< Holds the tail within TX buffer */
    uint16_t u16RxHead;   /**< Holds the head within RX buffer */
    uint16_t u16RxTail;   /**< Holds the tail within RX buffer */
    uint8_t u8Wake;       /**< Non zero if the Wakeup notification is to be sent to the EcuM */

    uint32_t u32PbClk;    /**< Holds the peripheral bus clock */
    uint32_t u32BaudRate; /**< Holds the peripheral baud rate */
    BOOL bRts;                  /**< Holds the actual RTS status */
    void (*pfRTSCtl)(BOOL);     /**< A pointer to function that control the RTS status */

    uint8_t u8BootByteLast;   /**< Used for enter boot request bytes sequence identifier. Holds the latest received byte. */
    uint8_t u8BootByteCount;  /**< Used for enter boot request bytes sequence identifier. Holds the number of bytes the sequence is expected. */
} Un_DataType;

/*! \var static Un_DataType Un_Inst[ UART_TOTAL_NUMBER_OF_CHANNELS ]
    \brief The Uart instances declaration.
 */
static Un_DataType Un_Inst[ UART_TOTAL_NUMBER_OF_CHANNELS ];

/***************************************************************************//**
 * \fn void USART1_Handler(void)
 * \brief   The USART ISR.
 * \details
 * \return  This function has no return value.
 * \callgraph
 * \callergraph
 ******************************************************************************/
 void USART1_Handler(void)
{
	USART_TypeDef	*pUART = USART1;//init pointer to USART1 channel
	BOOL lb_InvalidInvocation = TRUE;//acts as a validation flag for channels handlers
	
	/* Set up ECU on wake-up mode in order to perform operations without reset */
		if ( 0u != Un_Inst[Un0].u8Wake ){
	}
	/* USART parity error interrupt occurred ------------------------------------- */
		if( 0u != ((pUART->ISR & USART_ISR_PE)&& (0ul != (pUART->CR1 & LL_USART_CR1_PEIE)) )){
			(void)pUART->RDR;              /* Discard received data */
			pUART->ICR = USART_ICR_PECF;   /* Clear flag            */
			lb_InvalidInvocation = FALSE;
		}
	/* UART frame error interrupt occurred -------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_FE)) && (0ul != (pUART->CR3 & USART_CR3_EIE)) ){
        (void)pUART->RDR;              /* Discard received data */
        pUART->ICR = USART_ICR_FECF;   /* Clear flag            */
        lb_InvalidInvocation = FALSE;
    }
	/* UART noise error interrupt occurred -------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_NE)) && (0ul != (pUART->CR3 & USART_CR3_EIE)) ){
        (void)pUART->RDR;              /* Discard received data */
        pUART->ICR = USART_ICR_NCF;    /* Clear flag            */
        lb_InvalidInvocation = FALSE;
    }
	/* UART Over-Run interrupt occurred ----------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_ORE)) && (0ul != (pUART->CR3 & USART_CR3_EIE)) ){
        (void)pUART->RDR;              /* Discard received data */
        pUART->ICR = USART_ICR_ORECF;  /* Clear flag            */
        lb_InvalidInvocation = FALSE;
    }
	/* UART RX-NotEmpty interrupt occurred ----------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_RXNE)) && (0ul != (pUART->CR1 & USART_CR1_RXNEIE)) ){
        Un_RxISR( Un0, (uint8_t)pUART->RDR );
        lb_InvalidInvocation = FALSE;
    }
	 /* UART TX-Empty interrupt occurred ----------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_TXE)) && (0ul != (pUART->CR1 & USART_CR1_TXEIE)) ) {
        pUART->CR1 &= ~USART_CR1_TXEIE;
        Un_TxISR( Un0 );
        lb_InvalidInvocation = FALSE;
    }
	if ( FALSE != lb_InvalidInvocation ) {
        LL_APB1_GRP2_ForceReset( LL_APB1_GRP2_PERIPH_USART1 );
        _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop();
        _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop();
        LL_APB1_GRP2_ReleaseReset( LL_APB1_GRP2_PERIPH_USART1 );

        /* (Re)Initialize it accordingly to latest settings */
        if ( 0ul != Un_Inst[Un0].u32BaudRate ) {
            Usart_Init( Un0, Un_Inst[Un0].u32PbClk, Un_Inst[Un0].u32BaudRate );
        }
    }
}
 
 /***************************************************************************//**
 * \fn void USART2_Handler(void)
 * \brief   The USART2 ISR.
 * \details
 * \return  This function has no return value.
 * \callgraph
 * \callergraph
 ******************************************************************************/
 void USART2_Handler(void)
{
	USART_TypeDef *pUART = USART2;//init pointer to USART2 channel
	BOOL lb_InvalidInvocation = TRUE;//acts as a validation flag for channels handlers
	
	/* Set up ECU on wake-up mode in order to perform operations without reset */
		if ( 0u != Un_Inst[Un1].u8Wake ){
	}
	/* USART parity error interrupt occurred ------------------------------------- */
	if( 0u != ((pUART->ISR & USART_ISR_PE)&& (0ul != (pUART->CR2 & LL_USART_CR1_PEIE)) )){
		(void)pUART->RDR;              /* Discard received data */
		pUART->ICR = USART_ICR_PECF;   /* Clear flag            */
		lb_InvalidInvocation = FALSE;
	}
	/* UART frame error interrupt occurred -------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_FE)) && (0ul != (pUART->CR3 & USART_CR3_EIE)) ){
        (void)pUART->RDR;              /* Discard received data */
        pUART->ICR = USART_ICR_FECF;   /* Clear flag            */
        lb_InvalidInvocation = FALSE;
    }
	/* UART noise error interrupt occurred -------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_NE)) && (0ul != (pUART->CR3 & USART_CR3_EIE)) ){
        (void)pUART->RDR;              /* Discard received data */
        pUART->ICR = USART_ICR_NCF;    /* Clear flag            */
        lb_InvalidInvocation = FALSE;
    }
	/* UART Over-Run interrupt occurred ----------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_ORE)) && (0ul != (pUART->CR3 & USART_CR3_EIE)) )
    {
        (void)pUART->RDR;              /* Discard received data */
        pUART->ICR = USART_ICR_ORECF;  /* Clear flag            */
        lb_InvalidInvocation = FALSE;
    }
	/* UART RX-NotEmpty interrupt occurred ----------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_RXNE)) && (0ul != (pUART->CR2 & USART_CR1_RXNEIE)) )
    {
        Un_RxISR( Un1, (uint8_t)pUART->RDR );
        lb_InvalidInvocation = FALSE;
    }
	 /* UART TX-Empty interrupt occurred ----------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_TXE)) && (0ul != (pUART->CR2 & USART_CR1_TXEIE)) ) {
        pUART->CR2 &= ~USART_CR1_TXEIE;
        Un_TxISR( Un1 );
       // lb_InvalidInvocation = FALSE;
    }
	if ( FALSE != lb_InvalidInvocation ) 
    {
        LL_APB1_GRP2_ForceReset( LL_APB1_GRP1_PERIPH_USART2 );
        _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop();
        _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop();
        LL_APB1_GRP2_ReleaseReset( LL_APB1_GRP1_PERIPH_USART2 );

        /* (Re)Initialize it accordingly to latest settings */
        if ( 0ul != Un_Inst[Un1].u32BaudRate ) {
            Usart_Init( Un1, Un_Inst[Un1].u32PbClk, Un_Inst[Un1].u32BaudRate );
        }
    }
}
 
/***************************************************************************//**
 * \fn void USART3_Handler(void)
 * \brief   The USART3 ISR.
 * \details
 * \return  This function has no return value.
 * \callgraph
 * \callergraph
 ******************************************************************************/
 void USART3_Handler(void)
{
	USART_TypeDef *pUART = USART3;//init pointer to USART3 channel
	BOOL lb_InvalidInvocation = TRUE;//acts as a validation flag for channels handlers
	
	/* Set up ECU on wake-up mode in order to perform operations without reset */
		if ( 0u != Un_Inst[Un2].u8Wake ){
	}
	/* USART parity error interrupt occurred ------------------------------------- */
	if( 0u != ((pUART->ISR & USART_ISR_PE)&& (0ul != (pUART->CR2 & LL_USART_CR1_PEIE)) )){
		(void)pUART->RDR;              /* Discard received data */
		pUART->ICR = USART_ICR_PECF;   /* Clear flag            */
		lb_InvalidInvocation = FALSE;
	}
	/* UART frame error interrupt occurred -------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_FE)) && (0ul != (pUART->CR3 & USART_CR3_EIE)) ){
        (void)pUART->RDR;              /* Discard received data */
        pUART->ICR = USART_ICR_FECF;   /* Clear flag            */
        lb_InvalidInvocation = FALSE;
    }
	/* UART noise error interrupt occurred -------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_NE)) && (0ul != (pUART->CR3 & USART_CR3_EIE)) ){
        (void)pUART->RDR;              /* Discard received data */
        pUART->ICR = USART_ICR_NCF;    /* Clear flag            */
        lb_InvalidInvocation = FALSE;
    }
	/* UART Over-Run interrupt occurred ----------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_ORE)) && (0ul != (pUART->CR3 & USART_CR3_EIE)) )
    {
        (void)pUART->RDR;              /* Discard received data */
        pUART->ICR = USART_ICR_ORECF;  /* Clear flag            */
        lb_InvalidInvocation = FALSE;
    }
	/* UART RX-NotEmpty interrupt occurred ----------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_RXNE)) && (0ul != (pUART->CR2 & USART_CR1_RXNEIE)) )
    {
        Un_RxISR( Un2, (uint8_t)pUART->RDR );
        lb_InvalidInvocation = FALSE;
    }
	 /* UART TX-Empty interrupt occurred ----------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_TXE)) && (0ul != (pUART->CR2 & USART_CR1_TXEIE)) ) {
        pUART->CR2 &= ~USART_CR1_TXEIE;
        Un_TxISR( Un2 );
       // lb_InvalidInvocation = FALSE;
    }
	 /* UART TX-Empty interrupt occurred ----------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_TXE)) && (0ul != (pUART->CR2 & USART_CR1_TXEIE)) ) {
        pUART->CR2 &= ~USART_CR1_TXEIE;
        Un_TxISR( Un1 );
       // lb_InvalidInvocation = FALSE;
    }
	if ( FALSE != lb_InvalidInvocation ) 
    {
        LL_APB1_GRP2_ForceReset( LL_APB1_GRP1_PERIPH_USART3 );
        _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop();
        _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop();
        LL_APB1_GRP2_ReleaseReset( LL_APB1_GRP1_PERIPH_USART3 );

        /* (Re)Initialize it accordingly to latest settings */
        if ( 0ul != Un_Inst[Un2].u32BaudRate ) {
            Usart_Init( Un2, Un_Inst[Un2].u32PbClk, Un_Inst[Un2].u32BaudRate );
        }
    }
}

/***************************************************************************//**
 * \fn void USART4_Handler(void)
 * \brief   The USART4 ISR.
 * \details
 * \return  This function has no return value.
 * \callgraph
 * \callergraph
 ******************************************************************************/
 void USART4_Handler(void)
{
	USART_TypeDef *pUART = USART4;//init pointer to USART3 channel
	BOOL lb_InvalidInvocation = TRUE;//acts as a validation flag for channels handlers
	
	/* Set up ECU on wake-up mode in order to perform operations without reset */
		if ( 0u != Un_Inst[Un3].u8Wake ){
	}
	/* USART parity error interrupt occurred ------------------------------------- */
	if( 0u != ((pUART->ISR & USART_ISR_PE)&& (0ul != (pUART->CR2 & LL_USART_CR1_PEIE)) )){
		(void)pUART->RDR;              /* Discard received data */
		pUART->ICR = USART_ICR_PECF;   /* Clear flag            */
		lb_InvalidInvocation = FALSE;
	}
	/* UART frame error interrupt occurred -------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_FE)) && (0ul != (pUART->CR3 & USART_CR3_EIE)) ){
        (void)pUART->RDR;              /* Discard received data */
        pUART->ICR = USART_ICR_FECF;   /* Clear flag            */
        lb_InvalidInvocation = FALSE;
    }
	/* UART noise error interrupt occurred -------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_NE)) && (0ul != (pUART->CR3 & USART_CR3_EIE)) ){
        (void)pUART->RDR;              /* Discard received data */
        pUART->ICR = USART_ICR_NCF;    /* Clear flag            */
        lb_InvalidInvocation = FALSE;
    }
	/* UART Over-Run interrupt occurred ----------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_ORE)) && (0ul != (pUART->CR3 & USART_CR3_EIE)) )
    {
        (void)pUART->RDR;              /* Discard received data */
        pUART->ICR = USART_ICR_ORECF;  /* Clear flag            */
        lb_InvalidInvocation = FALSE;
    }
	/* UART RX-NotEmpty interrupt occurred ----------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_RXNE)) && (0ul != (pUART->CR2 & USART_CR1_RXNEIE)) )
    {
        Un_RxISR( Un3, (uint8_t)pUART->RDR );
        lb_InvalidInvocation = FALSE;
    }
	 /* UART TX-Empty interrupt occurred ----------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_TXE)) && (0ul != (pUART->CR2 & USART_CR1_TXEIE)) ) {
        pUART->CR2 &= ~USART_CR1_TXEIE;
        Un_TxISR( Un3 );
       // lb_InvalidInvocation = FALSE;
    }
	if ( FALSE != lb_InvalidInvocation ) 
    {
        LL_APB1_GRP2_ForceReset( LL_APB1_GRP1_PERIPH_USART4 );
        _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop();
        _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop();
        LL_APB1_GRP2_ReleaseReset( LL_APB1_GRP1_PERIPH_USART4 );

        /* (Re)Initialize it accordingly to latest settings */
        if ( 0ul != Un_Inst[Un3].u32BaudRate ) {
            Usart_Init( Un3, Un_Inst[Un3].u32PbClk, Un_Inst[Un3].u32BaudRate );
        }
    }
}

/***************************************************************************//**
 * \fn void USART5_Handler(void)
 * \brief   The USART5 ISR.
 * \details
 * \return  This function has no return value.
 * \callgraph
 * \callergraph
 ******************************************************************************/
 void USART5_Handler(void)
{
	USART_TypeDef *pUART = USART5;//init pointer to USART3 channel
	BOOL lb_InvalidInvocation = TRUE;//acts as a validation flag for channels handlers
	
	/* Set up ECU on wake-up mode in order to perform operations without reset */
		if ( 0u != Un_Inst[Un4].u8Wake ){
	}
	/* USART parity error interrupt occurred ------------------------------------- */
	if( 0u != ((pUART->ISR & USART_ISR_PE)&& (0ul != (pUART->CR2 & LL_USART_CR1_PEIE))) ){
		(void)pUART->RDR;              /* Discard received data */
		pUART->ICR = USART_ICR_PECF;   /* Clear flag            */
		lb_InvalidInvocation = FALSE;
	}
	/* UART frame error interrupt occurred -------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_FE)) && (0ul != (pUART->CR3 & USART_CR3_EIE)) ){
        (void)pUART->RDR;              /* Discard received data */
        pUART->ICR = USART_ICR_FECF;   /* Clear flag            */
        lb_InvalidInvocation = FALSE;
    }
	/* UART noise error interrupt occurred -------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_NE)) && (0ul != (pUART->CR3 & USART_CR3_EIE)) ){
        (void)pUART->RDR;              /* Discard received data */
        pUART->ICR = USART_ICR_NCF;    /* Clear flag            */
        lb_InvalidInvocation = FALSE;
    }
	/* UART Over-Run interrupt occurred ----------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_ORE)) && (0ul != (pUART->CR3 & USART_CR3_EIE)) )
    {
        (void)pUART->RDR;              /* Discard received data */
        pUART->ICR = USART_ICR_ORECF;  /* Clear flag            */
        lb_InvalidInvocation = FALSE;
    }
	/* UART RX-NotEmpty interrupt occurred ----------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_RXNE)) && (0ul != (pUART->CR2 & USART_CR1_RXNEIE)) )
    {
        Un_RxISR( Un4, (uint8_t)pUART->RDR );
        lb_InvalidInvocation = FALSE;
    }
	 /* UART TX-Empty interrupt occurred ----------------------------------------- */
    if ( (0ul != (pUART->ISR & USART_ISR_TXE)) && (0ul != (pUART->CR2 & USART_CR1_TXEIE)) ) {
        pUART->CR2 &= ~USART_CR1_TXEIE;
        Un_TxISR( Un4 );
       // lb_InvalidInvocation = FALSE;
    }
	if ( FALSE != lb_InvalidInvocation ) 
    {
        LL_APB1_GRP2_ForceReset( LL_APB1_GRP1_PERIPH_USART5 );
        _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop();
        _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop();
        LL_APB1_GRP2_ReleaseReset( LL_APB1_GRP1_PERIPH_USART5 );

        /* (Re)Initialize it accordingly to latest settings */
        if ( 0ul != Un_Inst[Un4].u32BaudRate ) {
            Usart_Init( Un4, Un_Inst[Un4].u32PbClk, Un_Inst[Un4].u32BaudRate );
        }
    }
}

/***************************************************************************//**
 * \fn void Usart_Init(uint32_t Channel, uint32_t u32PbClk, uint32_t u32BaudRate)
 * \brief 
 *   Initialization routine of the module.
 * \details
 *   Upon system start-up, call this function to initialize the module before 
 *   any other module function call.
 * \return
 *   This function has no return value.
 * \callgraph
 * \callergraph
 ******************************************************************************/
void Usart_Init(uint8_t Channel, uint32_t u32PbClk_, uint32_t u32BaudRate_)
{
	USART_TypeDef *pUART = NULL;
	if ( UART_TOTAL_NUMBER_OF_CHANNELS > Channel )
	{
		Un_Inst[Channel].u32PbClk = u32PbClk_;
		Un_Inst[Channel].u32BaudRate = u32BaudRate_;
	switch( Channel)
	{
		case Un0:/* Channel_1 */
	pUART = USART1;
	/* Enable channel specific clock */
	LL_APB1_GRP2_EnableClock((uint32_t)USART1);
	/* Enable UART_INIT_BIT*/
	pUART -> CR1 |= USART_CR1_UE ;
	/* UART_BAUD_RATE Register Initialization */
	pUART -> BRR |= u32PbClk_ / u32BaudRate_ ;
	/* Enable Rx */
	pUART -> CR1 |= USART_CR1_RE ;
	/* Enable Tx */
	pUART -> CR1 |= USART_CR1_TE ;
	/* Enable Rx_INTerrupt */
	
	/* Enable Tx_INTerrupt */
	/* Attach the RTS pin */
	/* */
	NVIC_SetPriority(USART1_IRQn,0);
	NVIC_EnableIRQ(USART1_IRQn);
	
		/*case Un1:
		case Un2:
		case Un3:
		case Un4:*/
		
				
	}
	/*Initialize Tx & Rx Buffer*/
	Un_Inst[Channel].u16RxTail = Un_Inst[Channel].u16TxTail = 0;
	Un_Inst[Channel].u16RxHead = Un_Inst[Channel].u16TxHead = 0;
	}
}
/***************************************************************************//**
 * \fn void Uart_Read(uint8_t Channel, uint8_t* lpBuffer, uint8_t nNumberOfBytesToRead, uint8_t* lpNumberOfBytesRead)
 * \brief
 *	 Reading Data
 * \details
 *
 * \param[in] Channel - Identifies the peripheral instance.
 *
 * \param[in] lpBuffer - The buffer where data shall be read.
 *
 * \param[in] nNumberOfBytesToRead - The number of bytes to read.
 *
 * \param[out] lpNumberOfBytesRead - The number of bytes successfully read.
 *
 * \return
 * 	 This function has no return value.
 * \callgraph
 * \callergraph
 ******************************************************************************/
void Usart_Read(uint8_t Channel, uint8_t* lpBuffer, uint8_t nNumberOfBytesToRead, uint8_t* lpNumberOfBytesRead)
{
	uint8_t *pBuff = NULL_PTR;
	uint16_t *pHead = NULL_PTR;
	uint16_t *pTail = NULL_PTR;

	/* Check if given channel is correct. Each channel have one structure assigned so pick the correct one  */
	if ( UART_TOTAL_NUMBER_OF_CHANNELS > Channel )
	{	// Check whether invalid data was requested or not
		if ( (NULL_PTR != lpBuffer) && (NULL_PTR != lpNumberOfBytesRead) )
		{
			pHead = &Un_Inst[Channel].u16RxHead;
			pTail = &Un_Inst[Channel].u16RxTail;
			pBuff = Un_Inst[Channel].u8RxBuff;
			// Init number of elements extracted from LIFO structure
			*lpNumberOfBytesRead = 0u;
			// Extract elements from storage buffer and write them to lpBuffer
			while ( (*pHead != *pTail) && (0u < nNumberOfBytesToRead) )
			{
				*pTail = (uint8_t)((*pTail + 1u) & UART_RX_BUF_SIZE_MASK);	//new tail
				*lpBuffer++ = pBuff[*pTail];	//update buffer
				nNumberOfBytesToRead--;
				(*lpNumberOfBytesRead)++;
			}
		}
	}
}

/***************************************************************************//**
 * \fn void Uart_Write(uint8_t Channel, const uint8_t* lpBuffer, uint8_t nNumberOfBytesToWrite, uint8_t* lpNumberOfBytesWritten)
 * \brief
 *	 Writing Data
 * \details
 *
 * \param[in] Channel - Identifies the peripheral instance.
 *
 * \param[in] lpBuffer - The buffer where data shall be written.
 *
 * \param[in] nNumberOfBytesToWrite - The number of bytes to write.
 *
 * \param[out] lpNumberOfBytesWritten - The number of bytes successfully written.
 *
 * \return
 * 	 This function has no return value.
 * \callgraph
 * \callergraph
 ******************************************************************************/
void Usart_Write(uint8_t Channel, const uint8_t* lpBuffer, uint8_t nNumberOfBytesToWrite, uint8_t* lpNumberOfBytesWritten)
{
	USART_TypeDef *pUART = NULL_PTR;
	uint8_t lb_Cond = 1u;
	uint8_t lu8_Head = 0u;
	uint8_t *pBuff = NULL_PTR;
	uint16_t *pHead = NULL_PTR;
	uint16_t *pTail = NULL_PTR;
	uint8_t *pStatusReg = NULL_PTR;
	uint8_t *pTxDataReg = NULL_PTR;
	uint8_t lu8_Un_STATUS_TXBL_MASK = 0ul;
	volatile uint8_t lu8_SimpleDelay = 0ul;
	/*Check if given channel is correct. Each channel have one structure assigned so pick the correct one*/
	if ( UART_TOTAL_NUMBER_OF_CHANNELS > Channel )
	{
		if ( NULL_PTR != lpNumberOfBytesWritten )
		{
			*lpNumberOfBytesWritten = 0u;
			if ( 0u == nNumberOfBytesToWrite )
			{
				return;
			}
			pHead = &Un_Inst[Channel].u16TxHead;
			pTail = &Un_Inst[Channel].u16TxTail;
			pBuff = Un_Inst[Channel].u8TxBuff;

			do
			{
				lu8_Head = (uint8_t)((*pHead + 1u) & UART_TX_BUF_SIZE_MASK);
				if ( *pTail != lu8_Head )
				{
					*pHead = lu8_Head; /* Update It */
					pBuff[lu8_Head] = *lpBuffer++;
					(*lpNumberOfBytesWritten)++;
					nNumberOfBytesToWrite--;
				}
				else
				{
					lb_Cond = 0u;
				}
			} while ( (0u != lb_Cond) && (0u < nNumberOfBytesToWrite) );

			switch ( Channel )
			{
				/*check registers*/
				case Un0:
					pUART = USART1;
					pStatusReg = (uint8_t*)&pUART->ISR;
					pTxDataReg = (uint8_t*)&pUART->TDR;
					lu8_Un_STATUS_TXBL_MASK = USART_ISR_TXE;	//flag
					break;

				case Un1:
					pUART = USART2;
					pStatusReg = (uint8_t*)&pUART->ISR;
					pTxDataReg = (uint8_t*)&pUART->TDR;
					lu8_Un_STATUS_TXBL_MASK = USART_ISR_TXE;	//flag
					break;

				case Un2:
					pUART = USART3;
					pStatusReg = (uint8_t*)&pUART->ISR;
					pTxDataReg = (uint8_t*)&pUART->TDR;
					lu8_Un_STATUS_TXBL_MASK = USART_ISR_TXE;	//flag
					break;
				
				case Un3:
					pUART = USART4;
					pStatusReg = (uint8_t*)&pUART->ISR;
					pTxDataReg = (uint8_t*)&pUART->TDR;
					lu8_Un_STATUS_TXBL_MASK = USART_ISR_TXE;	//flag
					break;
				
				case Un4:
					pUART = USART5;
					pStatusReg = (uint8_t*)&pUART->ISR;
					pTxDataReg = (uint8_t*)&pUART->TDR;
					lu8_Un_STATUS_TXBL_MASK = USART_ISR_TXE;	//flag
					break;
				
				default:
					return;
					break;
			}
			/*************************************************************************************************/
			/*************/
			/* Drive RTS */
			/* ********* */
			if ( FALSE == Un_Inst[Channel].bRts )
			{
				Un_Inst[Channel].bRts = TRUE;
				if ( NULL_PTR != Un_Inst[Channel].pfRTSCtl )
				{
					Un_Inst[Channel].pfRTSCtl( TRUE );

					/* Output enable time to high/low level tPZH = 120 ns   */
					/* Simple delay, MAX 120 ns                             */
					/* Fcpu = 16MHz, Tcpu = 20.8 ns                         */
					lu8_SimpleDelay = 10u;
					while ( 0u < lu8_SimpleDelay )
					{
						lu8_SimpleDelay--;
					}
				}
			}

			/*see if any data is available then grb a byte from the buffer*/
			while ( (*pHead != *pTail)
			        && (0ul != (*pStatusReg & lu8_Un_STATUS_TXBL_MASK)) )
			{
				*pTail = (uint8_t)((*pTail + 1u) & UART_TX_BUF_SIZE_MASK);

				/* Wait until space is available in the FIFO */
				while ( !(*pStatusReg & USART_ISR_TXE) );
				/* Read UART_S1 register*/
				(void)*pStatusReg;
				/* Send the character */
				*pTxDataReg = pBuff[*pTail];
			}

			/* ******************************************* */
			/* Activate TX Empty or TX Complete interrupt? */
			/* ******************************************* */
			/*checks if data is available*/

			if ( *pHead != *pTail )
			{
				pUART->CR1 |= USART_CR1_TXEIE;
			}
			else
			{
				pUART->CR1 |= USART_CR1_TCIE;
			}
		}
	}
}

/***************************************************************************//**
 * \fn void Un_RxISR(uint8_t Channel, uint8_t Data)
 * \brief
 * This function shall be called from interrupt.
 * It reads from serial port 1 byte and then push it back to the buffer within data structure.
 * \details
 *
 * \param[in] Channel - Identifies the peripheral instance.
 *
 * \param[in] Data - Defines the data character passed from interrupt routine.
 *
 * \return
 *   This function has no return value.
 * \callgraph
 * \callergraph
 ******************************************************************************/
void Un_RxISR(uint8_t Channel, uint8_t Data){
	uint8_t *pBuff = NULL_PTR;
	uint16_t *pHead = NULL_PTR;
	uint16_t *pTail = NULL_PTR;
	uint8_t lu8_Head = 0u;

	// Check if given channel is correct. Each channel have one structure assigned so pick the correct one
	if ( UART_TOTAL_NUMBER_OF_CHANNELS > Channel )
	{
		//Assign initial values
		pHead = &Un_Inst[Channel].u16RxHead;
		pTail = &Un_Inst[Channel].u16RxTail;
		pBuff = Un_Inst[Channel].u8RxBuff;

		//Head Ptr updated
		lu8_Head = (uint8_t)((*pHead + 1u) & UART_RX_BUF_SIZE_MASK);

		//if Tail != Head ,update head with new value and update value in the Buffer
		if ( *pTail != lu8_Head )
		{
			*pHead = lu8_Head;
			pBuff[lu8_Head] = Data;
		}
	}
}

/***************************************************************************//**
 * \fn void Un_TxISR(uint8_t Channel)
 * \brief
 *	 This is the function used to send data via UART.
 * \details
 *
 * \param[in] Channel - Identifies the peripheral instance.
 *
 * \return
 *   This function has no return value.
 * \callgraph
 * \callergraph
 ******************************************************************************/
void Un_TxISR(uint8_t Channel)
{
	USART_TypeDef *pUART = NULL_PTR;
	uint8_t *pBuff = NULL_PTR;
	uint16_t *pHead = NULL_PTR;
	uint16_t *pTail = NULL_PTR;
	uint8_t *pStatusReg = NULL_PTR;
	uint8_t *pTxDataReg = NULL_PTR;
	uint8_t lu8_Un_STATUS_TXBL_MASK = 0ul;

	if ( UART_TOTAL_NUMBER_OF_CHANNELS > Channel )
	{
		pHead = &Un_Inst[Channel].u16TxHead;
		pTail = &Un_Inst[Channel].u16TxTail;
		pBuff = Un_Inst[Channel].u8TxBuff;
		/* ********************************************** */
		/* Get The Address Of STATUS And TXDATA Registers */
		/* ********************************************** */
		switch ( Channel )
		{
			case Un0:
				pUART = USART1;
				pStatusReg = (uint8_t*)&pUART->ISR; //used extract TDRE field from status register
				pTxDataReg = (uint8_t*)&pUART->TDR; //tx data write register
				lu8_Un_STATUS_TXBL_MASK = USART_ISR_TXE;
				break;
			case Un1:
				pUART = USART2;
				pStatusReg = (uint8_t*)&pUART->ISR; //used extract TDRE field from status register
				pTxDataReg = (uint8_t*)&pUART->TDR; //tx data write register
				lu8_Un_STATUS_TXBL_MASK = USART_ISR_TXE;
				break;
			case Un2:
				pUART = USART3;
				pStatusReg = (uint8_t*)&pUART->ISR; //used extract TDRE field from status register
				pTxDataReg = (uint8_t*)&pUART->TDR; //tx data write register
				lu8_Un_STATUS_TXBL_MASK = USART_ISR_TXE;
				break;
			default:
				return;
				break;
		}

		/* *********************************************************************** */
		/* Arm hardware TX Fifo until it is full or no more bytes are to be TXmited */
		/* *********************************************************************** */
		while ( (*pHead != *pTail)
		        && (0ul != (*pStatusReg & lu8_Un_STATUS_TXBL_MASK)) ) // transmit data
		{
			*pTail = (uint8_t)((*pTail + 1u) & UART_TX_BUF_SIZE_MASK);
			*pTxDataReg = pBuff[*pTail];
		}

		/* ***************************************************************************** */
		/* If there are more bytes to be TXmited - enable TX Empty, otherwise TX Complete */
		/* ***************************************************************************** */
		if ( *pHead != *pTail )
		{
			pUART->CR1 |= USART_CR1_TXEIE;
		}
		else
		{
			pUART->CR1 |= USART_CR1_TCIE;
		}
	}
}
/***************************************************************************//**
 * \fn uint8_t Uart_GetRxQueueLevel(uint8_t Channel, uint8_t* lpLevel)
 * \brief
 *	 This is the function used to determine how much space has the Rx Queue left
 * \details
 *
 * \param[in] Channel - Identifies the peripheral instance.
			  *lpLevel - Idicates the space left and converts it into an integer
 *
 * \return
 *   This function returns the remaining space left in the buffer
 * \callgraph
 * \callergraph
 ******************************************************************************/
uint8_t Uart_GetRxQueueLevel(uint8_t Channel, uint8_t* lpLevel){
	 return *lpLevel = (uint8_t*) Un_Inst[Channel].u16RxHead - Un_Inst[Channel].u16RxTail ;
}
/***************************************************************************//**
 * \fn uint8_t Uart_GetTxQueueLevel(uint8_t Channel, uint8_t* lpLevel)
 * \brief
 *	 This is the function used to determine how much space has the Tx Queue left
 * \details
 *
 * \param[in] Channel - Identifies the peripheral instance.
			  *lpLevel - Idicates the space left and converts it into an integer
 *
 * \return
 *   This function returns the remaining space left in the buffer
 * \callgraph
 * \callergraph
 ******************************************************************************/
uint8_t Uart_GetTxQueueLevel(uint8_t Channel, uint8_t* lpLevel){
	return *lpLevel = (uint8_t*) Un_Inst[Channel].u16TxHead - Un_Inst[Channel].u16TxTail ;
}	

/**
  * @}
  */