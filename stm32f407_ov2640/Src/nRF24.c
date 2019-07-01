/*
 * nRF24.c
 *
 *  Created on: 25 àïð. 2019 ã.
 *      Author: NASA
 */



#include "nRF24.h"
#include "main.h"
#include "stm32f4xx_hal.h"


extern SPI_HandleTypeDef hspi2;

#define	RX_DR			0x40
#define	TX_DS			0x20
#define	MAX_RT			0x10

#define TX_PLOAD_WIDTH 32

unsigned char TX_ADDRESS[TX_ADR_WIDTH] = {0xb2,0xb2,0xb3,0xb4,0x01};
unsigned char RX_BUF[TX_PLOAD_WIDTH];
unsigned char TX_BUF[TX_PLOAD_WIDTH];


#define CE(x) HAL_GPIO_WritePin(nRF_CE_GPIO_Port, nRF_CE_Pin, x)
#define CSN(x) HAL_GPIO_WritePin(nRF_CSN_GPIO_Port, nRF_CSN_Pin, x)
#define IRQ HAL_GPIO_ReadPin(nRF_IRQ_GPIO_Port, nRF_IRQ_Pin)


unsigned char SPI_Receive_byte(unsigned char reg)
{
	HAL_SPI_TransmitReceive(&hspi2, &reg, &reg, 1, 100);
   return reg;
}

unsigned char SPI_Send_byte(unsigned char reg)
{
	HAL_SPI_Transmit(&hspi2, &reg, 1, 100);
   return reg;
}

unsigned char  SPI_Write_Buf(unsigned char  reg, unsigned char  *pBuf, unsigned char  bytes)
{
	unsigned char  status,byte_ctr;
	CSN(0);
	status= SPI_Receive_byte(reg);
	//HAL_Delay(10); //delayMicroseconds(10);//delay1us(1);
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		SPI_Send_byte(*pBuf++);
	CSN(1);
	return(status);
}

unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
	unsigned char status;//,byte_ctr;
	CSN(0);
	status=SPI_Receive_byte(reg);
	unsigned char * bufer[32];
	HAL_SPI_TransmitReceive(&hspi2, (unsigned char *)&bufer[0], pBuf, bytes, 100);
	CSN(1);
	return(status);
}

unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)
{
	unsigned char status;
	CSN(0);
	status=SPI_Receive_byte(reg);   //select register  and write value to it
	SPI_Send_byte(value);
	CSN(1);
	return(status);
}

unsigned char SPI_Read_Reg(unsigned char reg)
{
	unsigned char status;
	CSN(0);
	SPI_Send_byte(reg);
	status=SPI_Receive_byte(0);   //select register  and write value to it
	CSN(1);
	return(status);
}

unsigned char bbb = 0;
void TX_Mode(unsigned char * tx_buf)
{
	if (tx_buf[0] != bbb)Printf("error, ");
	bbb = tx_buf[0];
	bbb++;

	CE(0);
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // {0xb2,0xb2,0xb3,0xb4,0x01}
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // address 5 byte is {0xb2,0xb2,0xb3,0xb4,0x01} to pipe 0
  	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // wr payload
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x00);//0x00);//0x3f);       // Enable AA to 1-5 pipe
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_AW, 0x01);       // addres len is 3 bytes
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);   // Enable rx data to 1-5 pipe
  	//SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x0a);  // number retr to error
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x00);  // retr disable
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 125);         // work channel
  	//SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);//0x07);    // no carrier;  no encoder; 1Mbps; 0dbm
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x0F);    // no carrier;  no encoder; 2Mbps; 0dbm
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // num width payload
  	//SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e);      // Enable CRC; 2byte CRC; Power UP; PTX;
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0a);      // Enable CRC; 1byte CRC; Power UP; PTX;
	CE(1);
	//delay(1);//delay1us(10);
}


void RX_Mode(void)
{
	CE(0);
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // address 5 byte is {0xb2,0xb2,0xb3,0xb4,0x01} to pipe 0
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // ½ÓÊÕÍ¨µÀ0Ñ¡ÔñºÍ·¢ËÍÍ¨µÀÏàÍ¬ÓÐÐ§Êý¾Ý¿í¶È

  	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_AW, 0x01);       // addres len is 3 bytes	//FIXME
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x00);//0x00);//0x3f);               // enable ask
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);           // set address rx
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 125);                 // channel

  	//SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);//0x07);            // 0dbm 1MBps
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x0F);            // 0dbm 2MBps
  	//SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0f);               // Enable CRC; 2byte CRC; Power UP; PRX;;
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0a);      // Enable CRC; 1byte CRC; Power UP; PTX;
  	CE(1);
}


/*
#include "mesh.h"

extern unsigned char toSendCmdbuf[CMD_LIST_SIZE][CMD_BUF_SIZE];
extern int toSendCmd_RD;
extern int toSendCmd_WR;

extern unsigned char toRxCmdbuf[CMD_LIST_SIZE][CMD_BUF_SIZE];
extern int toRxCmd_RD;
extern int toRxCmd_WR;
*/

unsigned char buff_out[32];

int NRF24L01_Receive(void)
{
    unsigned char status=0x01;
    int rx = 0;

	CE(0);
	HAL_Delay(2);//delayMicroseconds(10);//delay1us(10);

	status=SPI_Read_Reg(STATUS);

	if(status & 0x40)
	{
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x40);

		unsigned char fifo_status = 0;
		do{
			SPI_Read_Buf(RD_RX_PLOAD,buff_out,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
			rx+=TX_PLOAD_WIDTH;
			Printf("RX --");
			//Printf("data zerro is: 0x%x\r\n",RX_BUF[0]);
			//int t=0; for (t=0; t<32; t++)printf("0x%x,",RX_BUF[t]); printf("\r\n");

			//memcpy(toRxCmdbuf[toRxCmd_WR], RX_BUF, 32);
			//DataArrayPP(toRxCmd_WR);

			fifo_status=SPI_Read_Reg(FIFO_STATUS);
		}while((fifo_status&0x01) != 0x01);
	}
	if(status&TX_DS)
	{
		//Printf("data sended INRX\r\n");
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x20);
	}
	if(status&MAX_RT)
	{
	//Printf("NO SEND..INRX\n\r");
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x10);
	}

	CE(1);
	return rx;
}

unsigned char GetStatus(void)
{
    unsigned char status=0x00;
	CE(0);
	//HAL_Delay(2);//delayMicroseconds(10);//delay1us(10);

	status=SPI_Read_Reg(STATUS);	// ¶ÁÈ¡×´Ì¬¼Ä´æÆäÀ´ÅÐ¶ÏÊý¾Ý½ÓÊÕ×´¿ö
	if(status&TX_DS)	//tx_ds == 0x20
	{
		//Printf("data sended f\r\n");
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x20);      // Çå³ýTX£¬ÈÃIRQÀ­µÍ£»

	}
	else if(status&MAX_RT)
	{
		//Printf("NO SEND.. f\n\r");
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x10);      // Çå³ýTX£¬ÈÃIRQÀ­µÍ£»
	}

	CE(1);
	return status;
}

void NRF24L01_Send(unsigned char * butToSend)
{
	TX_Mode(butToSend);

	while(IRQ == GPIO_PIN_SET);

	GetStatus();

}



void TX_Simple(unsigned char * tx_buf)
{
	if (tx_buf[0] != bbb)Printf("error, ");
	bbb = tx_buf[0];
	bbb++;
	CE(0);
  	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // wr payload
	CE(1);
	while(IRQ == GPIO_PIN_SET);
	GetStatus();
}




int NRF24L01_Init(void){

	CSN(GPIO_PIN_RESET);
	CE(GPIO_PIN_SET);


	HAL_Delay(200);
	RX_Mode();

	CE(0);
	HAL_Delay(2);//delayMicroseconds(10);//delay1us(10);
	unsigned char fifo_status=SPI_Read_Reg(FIFO_STATUS);
	Printf("Fifo status is: %d\n\r", fifo_status);

	SPI_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
	SPI_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
	SPI_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer


	fifo_status=SPI_Read_Reg(FIFO_STATUS);
	Printf("Fifo status is: %d\n\r", fifo_status);

	CE(1);

	return 0;
}



