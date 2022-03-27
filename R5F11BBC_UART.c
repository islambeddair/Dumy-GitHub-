/**********************************************************************/
/*                                                                    */
/*  FILE        : R5F11BBC_UART.h                   			   */
/*  DATE        : Tue, Dec 22, 2021                                     */
/*  Last Update : Tue, Dec 22, 2021                            		   */
/*  DESCRIPTION : UART Source					                      */
/*  CPU GROUP   : R5F11BBC                                            */
/*  Author      : Osama ElMorady						          	  */
/*                          						                  */
/***********************************************************************/


/***********************************************************************************************************************
*                                            Module's Inclusion files                                                  *
***********************************************************************************************************************/
#include "core/R5F11B/LIB/BIT_MATH.h"
#include <core/R5F11B/LIB/STD_TYPES.h>

#include <core/R5F11B/iodefine.h>
#include "core/R5F11B/interrupt_handlers.h"

//#include <core/R5F11B/UART/R5F11BBC_UART_config.h>
//#include <core/R5F11B/UART/R5F11BBC_UART_private.h>
#include <core/R5F11B/UART/R5F11BBC_UART.h>



#define R5F11BLE	(1)
#define R5F11BBC	(0)

#define MCU_SERIES	R5F11BLE


/***********************************************************************************************************************/
#define  IIC_WAITTIME    14U
/***********************************************************************************************************************/


/***********************************************************************************************************************/
static uint16_t w_count ;
/***********************************************************************************************************************/
static void (* FPTR_UART_SetCallBack_ST[UART_No] )(void)  ;
static void (* FPTR_UART_SetCallBack_SR[UART_No] )(void)  ;
/***********************************************************************************************************************/
//static const uint8_t BaudRates_Map[UART_N_BaudRates] =
//		{
//			/* 2+2*reg_value = Division  @ 1 MHz */
//			103	,			/* 4800 ->   /208 */
//			51 ,			/* 9600 ->   /104 */
//			25 ,			/* 19200 ->  /52  */
//			12 ,			/* 38400 ->  /26  */
//			3  ,			/* 115200 ->  /8  */
//		}   ;

static const uint8_t BaudRates_Map[UART_N_BaudRates] =
		{
			/* 2+2*reg_value = Division  @ 2 MHz */
			207	,			/* 4800 ->   /416 */
			103 ,			/* 9600 ->   /208 */
			51 ,			/* 19200 ->  /104  */
			25 ,			/* 38400 ->  /52  */
			8  ,			/* 115200 ->  /18  */
		}   ;

/***********************************************************************************************************************/






/*****************************************************************************************************************/
/*********************************************** UART__Init(void) *************************************************/
/*****************************************************************************************************************/
void Uart_vInit( UART_Config_t * ch_config)
{

	switch (ch_config -> Channel_No)
	{
	/* UART0 --> SAU0 --> Ch0  */
	case UART0 :

		/* enables input clock supply */
		SAU0EN = 1U;

		NOP(); 	NOP(); 	NOP(); 	NOP();

		/* Array Unit Clock Selection */
//		SPS0 = (uint16_t) 0x0005 ;			// 1 MHz
		SPS0 = (uint16_t) 0x0004 ;			// 2 MHz

		/* Make sure channel0 , channel1  disabled */
		ST0 |= (uint16_t) 0x01 |  (uint16_t) 0x02  ;

		/*  interrupt Priority */
		STPR00 = 0U ;
		STPR10 = 0U ;
		SRPR00 = 0U ;
		SRPR10 = 0U ;

	    /*  interrupt status */
	    STMK0 = ! ch_config -> TR_type ;	/* 1 -> Disable interrupt */
	    SRMK0 = ! ch_config -> TR_type ;
	    /* clear INTIIC11 interrupt flag */
	    STIF0 = 0U;
	    SRIF0 = 0U;

	    // Clear error flags
	    SIR00 =0x07  ;

	    /********************************************************/
	    /* UART  selection */ /* Falling edge as start Bit */
	    SMR00 = (uint16_t) 0x0020 |(uint16_t) 0x0002 /* | (uint16_t) 0x0100 */  ;

									/* Tx	 					, Parity , 											  Direction (LSB)  								,  Stop bit(1) 		, 8-Bit Data Length */
	    SCR00 =(uint16_t) 0x0004 | (uint16_t)0x8000 |(uint16_t) ((ch_config ->Parity_status &0b11) <<8) | (uint16_t)((ch_config->Data_order_status & 1) <<  7) | (uint16_t)0x0010 |  (uint16_t)0x0003 ;


		/* Set the clock division for baud rate */
		SDR00 = (uint16_t) BaudRates_Map [ch_config -> BaudRate] << 9 ;
		/********************************************************/
	    /* UART  selection */ /* Falling edge as start Bit */  /* Valid edge for RXD */
	    SMR01 = (uint16_t) 0x0020 |(uint16_t) 0x0002  | (uint16_t) 0x0100 ;

	    							/* Rx 					, Parity , 											  Direction (LSB)  								,  Stop bit(1) 		, 8-Bit Data Length */
	    SCR01 =(uint16_t) 0x0004 | (uint16_t)0x4000 |(uint16_t) ((ch_config ->Parity_status &0b11) <<8) | (uint16_t)((ch_config->Data_order_status & 1) <<  7) | (uint16_t)0x0010 |  (uint16_t)0x0003 ;


		/* Set the clock division for baud rate */
		SDR01 = (uint16_t) BaudRates_Map [ch_config -> BaudRate] << 9 ;
		/********************************************************/
		/* No data Invertion */
		SOL0 &= (uint16_t) ~0x0001 ;

		/* Disable UART Output Communication */
	    SOE0 &= (uint16_t) ~0x0001 ;

	    /* Initial Output Level output Level : Tx , Rx */
	    SO0 |= (uint16_t) 0x0100 | (uint16_t) 0x0001;			// Pulled High


#if ( MCU_SERIES == 	R5F11BLE) || (MCU_SERIES == 	R5F11BBC)
	    /* R5F11BLE */
	    /* Set Tx0 (P51) , Rx0 (P50) pin config */
	    /* Set RxD0 pin */
	    PM5 |= 0x01U;
	    /* Set TxD0 pin */
	    P5 |= 0x02U;
	    PM5 &= 0xFDU;

#endif

	    break ;

	/* UART1 --> SAU0 --> Ch2  */
	case UART1 :

		/* enables input clock supply */
		SAU0EN = 1U;

		NOP(); 	NOP(); 	NOP(); 	NOP();

		/* Array Unit Clock Selection */
//		SPS0 = (uint16_t) 0x0005 ;			// 1 MHz
		SPS0 = (uint16_t) 0x0004 ;			// 2 MHz

		/* Make sure channel2 , channel3  disabled */
		ST0 |= (uint16_t) 0x04 |  (uint16_t) 0x08  ;

		/*  interrupt Priority */
		STPR01 = 0U ;
		STPR11 = 0U ;
		SRPR01 = 0U ;
		SRPR11 = 0U ;

	    /*  interrupt status */
	    STMK1 = ! ch_config -> TR_type ;
	    SRMK1 = ! ch_config -> TR_type ;
	    /* clear INTIIC11 interrupt flag */
	    STIF1 = 0U;
	    SRIF1 = 0U;

	    // Clear error flags
	    SIR02 =0x07  ;

	    /********************************************************/
	    /* UART  selection */ /* Falling edge as start Bit */
	    SMR02 = (uint16_t) 0x0020 |(uint16_t) 0x0002 /* | (uint16_t) 0x0100 */ ;

									/* Tx	 					, Parity , 											  Direction (LSB)  								,  Stop bit(1) 		, 8-Bit Data Length */
	    SCR02 =(uint16_t) 0x0004 | (uint16_t)0x8000 |(uint16_t) ((ch_config ->Parity_status &0b11) <<8) | (uint16_t)((ch_config->Data_order_status & 1) <<  7) | (uint16_t)0x0010 |  (uint16_t)0x0003 ;


		/* Set the clock division for baud rate */
		SDR02 = (uint16_t) BaudRates_Map [ch_config -> BaudRate] << 9 ;
		/********************************************************/
	    /* UART  selection */ /* Falling edge as start Bit */  /* Valid edge for RXD */
	    SMR03 = (uint16_t) 0x0020 |(uint16_t) 0x0002  | (uint16_t) 0x0100  ;

	    							/* Rx 					, Parity , 											  Direction (LSB)  								,  Stop bit(1) 		, 8-Bit Data Length */
	    SCR03 =(uint16_t) 0x0004 | (uint16_t)0x4000 |(uint16_t) ((ch_config ->Parity_status &0b11) <<8) | (uint16_t)((ch_config->Data_order_status & 1) <<  7) | (uint16_t)0x0010 |  (uint16_t)0x0003 ;


		/* Set the clock division for baud rate */
		SDR03 = (uint16_t) BaudRates_Map [ch_config -> BaudRate] << 9 ;
		/********************************************************/
		/* No data Invertion */
		SOL0 &= (uint16_t) ~0x0004 ;

	    /* Disable UART Output Communication */
	    SOE0 &= (uint16_t) ~0x0004 ;

	    /* Initial Output Level output Level : Tx , Rx */
	    SO0 |= (uint16_t) 0x0400 | (uint16_t) 0x0004;			// Pulled High


#if MCU_SERIES == 	R5F11BBC

	    /* Set Tx1 (P00) , Rx1 (P01) pin config */
	    /* Set RxD1 pin */
	    PMC0 &= 0xFDU;
	    PM0 |= 0x02U;
	    /* Set TxD1 pin */
	    PMC0 &= 0xFEU;
	    P0 |= 0x01U;
	    PM0 &= 0xFEU;


#elif MCU_SERIES == 	R5F11BLE

	  /* Set Tx1 (P02) , Rx1 (P03) pin config */
	    /* Set RxD1 pin */
	    PMC0 &= 0xF7U;
	    PM0 |= 0x08U;
	    /* Set TxD1 pin */
	    PMC0 &= 0xFBU;
	    P0 |= 0x04U;
	    PM0 &= 0xFBU;

#endif

	    break ;

	/* UART2 --> SAU1 --> Ch0  */
	case UART2 :

		/* enables input clock supply */
		SAU1EN = 1U;

		NOP(); 	NOP(); 	NOP(); 	NOP();

		/* Array Unit Clock Selection */
//		SPS0 = (uint16_t) 0x0005 ;			// 1 MHz
		SPS1 = (uint16_t) 0x0004 ;			// 2 MHz

		/* Make sure channel0 , channel1  disabled */
		ST1 |= (uint16_t) 0x01 |  (uint16_t) 0x02  ;

		/*  interrupt Priority */
		STPR02 = 0U ;
		STPR12 = 0U ;
		SRPR02 = 0U ;
		SRPR12 = 0U ;

	    /*  interrupt status */
	    STMK2 = ! ch_config -> TR_type ;
	    SRMK2 = ! ch_config -> TR_type ;
	    /* clear INTIIC11 interrupt flag */
	    STIF2 = 0U;
	    SRIF2 = 0U;

	    // Clear error flags
	    SIR10 =0x07  ;
	    /********************************************************/
	    /* UART  selection */ /* Falling edge as start Bit */
		SMR10 = (uint16_t) 0x0020 |(uint16_t) 0x0002 /* | (uint16_t) 0x0100 */ ;

									/* Tx	 					, Parity , 											  Direction (LSB)  								,  Stop bit(1) 		, 8-Bit Data Length */
		SCR10 =(uint16_t) 0x0004 | (uint16_t)0x8000 |(uint16_t) ((ch_config ->Parity_status &0b11) <<8) | (uint16_t)((ch_config->Data_order_status & 1) <<  7) | (uint16_t)0x0010 |  (uint16_t)0x0003 ;


		/* Set the clock division for baud rate */
		SDR10 = (uint16_t) BaudRates_Map [ch_config -> BaudRate] << 9 ;
		/********************************************************/
	    /* UART  selection */ /* Falling edge as start Bit */  /* Valid edge for RXD */
		SMR11 = (uint16_t) 0x0020 |(uint16_t) 0x0002  | (uint16_t) 0x0100  ;

	    							/* Rx 					, Parity , 											  Direction (LSB)  								,  Stop bit(1) 		, 8-Bit Data Length */
	    SCR11 =(uint16_t) 0x0004 | (uint16_t)0x4000 |(uint16_t) ((ch_config ->Parity_status &0b11) <<8) | (uint16_t)((ch_config->Data_order_status & 1) <<  7) | (uint16_t)0x0010 |  (uint16_t)0x0003 ;


		/* Set the clock division for baud rate */
	    SDR11 = (uint16_t) BaudRates_Map [ch_config -> BaudRate] << 9 ;
		/********************************************************/
		/* No data Invertion */
		SOL1 &= (uint16_t) ~0x0001 ;

	    /* Disable UART Output Communication */
	    SOE1 &= (uint16_t) ~0x0001 ;

	    /* Initial Output Level output Level : Tx , Rx */
	    SO1 |= (uint16_t) 0x0100 | (uint16_t) 0x0001;			// Pulled High


#if ( MCU_SERIES == 	R5F11BLE) || (MCU_SERIES == 	R5F11BBC)

	    /* Set Tx2 (P13) , Rx2 (P14) pin config */
	    /* Set RxD2 pin */
	    PMC1 &= 0xEFU;
	    PM1 |= 0x10U;
	    /* Set TxD2 pin */
	    PMC1 &= 0xF7U;
	    P1 |= 0x08U;
	    PM1 &= 0xF7U;

#endif

	   break ;
	}

}
/***********************************************************************************************************************/





/***********************************************************************************************************************/
void Uart_vDeInit( UART_Channel  channel_no)
{
	switch (channel_no)
	{
	case UART0 :

		 /* Disable UART Output Communication */
		 SOE0 &= (uint16_t) ~0x0001 ;

		 /* Clear all flags (OVF,PEF,FEF)*/
		 SIR00 |= (uint16_t) 0x07		;

		/* Disable input clock supply */
		SAU0EN = 0U;

	break ;

	case UART1 :

		 /* Disable UART Output Communication */
		 SOE0 &= (uint16_t) ~0x0004 ;

		 /* Clear all flags (OVF,PEF,FEF)*/
		 SIR02 |= (uint16_t) 0x07		;

		/* Disable input clock supply */
		SAU0EN = 0U;

	break ;

	case UART2 :

		 /* Disable UART Output Communication */
		 SOE1 &= (uint16_t) ~0x0001 ;

		 /* Clear all flags (OVF,PEF,FEF)*/
		 SIR10 |= (uint16_t) 0x07		;

		/* Disable input clock supply */
		SAU1EN = 0U;

	break ;
	}
}
/***********************************************************************************************************************/








/***********************************************************************************************************************/
void Uart_vStartBit( UART_Channel  channel_no)
{

	switch (channel_no)
	{
	case UART0 :

		/* OutPut High */
	    SO0 |= (uint16_t) 0x0100 | (uint16_t) 0x0001;			// Pulled High

	    /* Enable UART Output Communication */
	    SOE0 |= (uint16_t) 0x0001 ;

	    /* Start operation */
	    SS0 |=0x01 | 0x02 ;

		/* Wait until Start */
	//	while ( SE0L_bit.no0 == 0 ) ;

	break ;

	case UART1 :

		/* OutPut High */
	    SO0 |= (uint16_t) 0x0400 | (uint16_t) 0x0004;			// Pulled High

	    /* Enable UART Output Communication */
	    SOE0 |= (uint16_t) 0x0004 ;

	    /* Start operation */
	    SS0 |=0x04 | 0x08 ;

		/* Wait until Start */
	//	while ( SE0L_bit.no0 == 0 ) ;

	break ;

	case UART2 :

		/* OutPut High */
	    SO1 |= (uint16_t) 0x0100 | (uint16_t) 0x0001;			// Pulled High

	    /* Enable UART Output Communication */
	    SOE1 |= (uint16_t) 0x0001 ;

	    /* Start operation */
	    SS1 |=0x01 | 0x02 ;

		/* Wait until Start */
	//	while ( SE0L_bit.no0 == 0 ) ;

	break ;
	}


}
/***********************************************************************************************************************/





/***********************************************************************************************************************/
void Uart_vStopBit( UART_Channel  channel_no )
{

	switch (channel_no)
	{
	case UART0 :

		 /* Stop operation */
		 ST0 |= (uint16_t) 0x0001 | (uint16_t) 0x0002 ;

		 SO0 |= (uint16_t) 0x0001 ;						/* OutPut High */

		 /* Disable UART Output Communication */
		 SOE0 &= (uint16_t) ~0x0001 ;

		 /* Clear all flags (OVF,PEF,FEF)*/
		 SIR00 |= (uint16_t) 0x07		;

	break ;

	case UART1 :

		 /* Stop operation */
		 ST0 |= (uint16_t) 0x0004 | (uint16_t) 0x0008 ;

		 SO0 |= (uint16_t) 0x0400 | (uint16_t) 0x0004;			// Pulled High

		 /* Disable UART Output Communication */
		 SOE0 &= (uint16_t) ~0x0004 ;

		 /* Clear all flags (OVF,PEF,FEF)*/
		 SIR02 |= (uint16_t) 0x07		;

	break ;

	case UART2 :

		 /* Stop operation */
		 ST1 |= (uint16_t) 0x0001 | (uint16_t) 0x0002 ;

		 SO1 |= (uint16_t) 0x0001 ;						/* OutPut High */

		 /* Disable UART Output Communication */
		 SOE1 &= (uint16_t) ~0x0001 ;

		 /* Clear all flags (OVF,PEF,FEF)*/
		 SIR10 |= (uint16_t) 0x07		;

	break ;
	}
}
/***********************************************************************************************************************/







/* Using Polling */
/***********************************************************************************************************************/
void Uart_vSendData( UART_Channel  channel_no  , uint8_t DataSend , uint8_t DataLength )
{
	switch (channel_no)
	{
	case UART0 :

//		SIO00 = DataSend ;
	    TXD0 = DataSend ;

		/* wait until data transfer ends */
		 while (STIF0 == 0 )	{	asm("NOP")  ; }
//		 while (((SSR00>>5)&1) == 1 )	{	asm("NOP")  ; }

		 // Clear Flag
		 STIF0 = 0 ;

	break ;

	case UART1 :

//		SIO10 = DataSend ;
	    TXD1 = DataSend ;

		/* wait until data transfer ends */
		 while (STIF1 == 0 )	{	asm("NOP")  ; }
//		 while (((SSR02>>5)&1) == 1 )	{	asm("NOP")  ; }

		 // Clear Flag
		 STIF1 = 0 ;

	break ;

	case UART2 :

//		SIO10 = DataSend ;
	    TXD2 = DataSend ;

		/* wait until data transfer ends */
		 while (STIF2 == 0 )	{	asm("NOP")  ; }
//		 while (((SSR10>>5)&1) == 1 )	{	asm("NOP")  ; }

		 // Clear Flag
		 STIF2 = 0 ;

	break ;
	}

}
/***********************************************************************************************************************/




/* Using Polling */
/***********************************************************************************************************************/
void Uart_vReceiveData( UART_Channel  channel_no , uint8_t * DataReceived , uint8_t DataLength )
{
	* DataReceived  = 0xFF ;

	switch (channel_no)
	{
	case UART0 :

		/* wait until data transfer ends */
//		 while (STIF0 == 0 )	{	asm("NOP")  ; }
		 while (((SSR00>>5)&1) == 1 )	{	asm("NOP")  ; }

		* DataReceived = RXD0   ;

	break ;

	case UART1 :

		/* wait until data transfer ends */
//		 while (STIF1 == 0 )	{	asm("NOP")  ; }
		 while (((SSR00>>5)&1) == 1 )	{	asm("NOP")  ; }

		* DataReceived = RXD1   ;

	break ;

	case UART2 :

		/* wait until data transfer ends */
//		 while (STIF2 == 0 )	{	asm("NOP")  ; }
		 while (((SSR10>>5)&1) == 1 )	{	asm("NOP")  ; }

		* DataReceived = RXD2   ;

	break ;
	}
}
/***********************************************************************************************************************/


/* Using Interrupt */
/***********************************************************************************************************************/
void Uart_vSendEnd( UART_Channel  channel_no , void (* UART_SetCallBack_Send )(void) )
{
	FPTR_UART_SetCallBack_ST[channel_no] = UART_SetCallBack_Send  ;
}
/***********************************************************************************************************************/




/* Using Interrupt */
/***********************************************************************************************************************/
void Uart_vReceiveEnd( UART_Channel  channel_no , void (* UART_SetCallBack_Receive )(void) )
{
	FPTR_UART_SetCallBack_SR[channel_no] = UART_SetCallBack_Receive  ;
}
/***********************************************************************************************************************/











/***********************************************/
void INT_ST0(void)
{
	FPTR_UART_SetCallBack_ST[UART0]() ;
}
void INT_SR0(void)
{
	FPTR_UART_SetCallBack_SR[UART0]() ;

}
/***********************************************/



/***********************************************/
void INT_ST1(void)
{
	FPTR_UART_SetCallBack_ST[UART1]() ;
}
void INT_SR1(void)
{
	FPTR_UART_SetCallBack_SR[UART1]() ;
}
/***********************************************/



/***********************************************/
void INT_ST2(void)
{
	FPTR_UART_SetCallBack_ST[UART2]() ;
}
void INT_SR2(void)
{
	FPTR_UART_SetCallBack_SR[UART2]() ;
}
/***********************************************/
