#include "Int_SI24R1.h"

uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x0A,0x01,0x07,0x0E,0x01};  // 发送地址


// 单字节SPI读写函数
static uint8_t SPI_RW(uint8_t byte)
{
	uint8_t rx_data=0;
	HAL_SPI_TransmitReceive(&hspi1, &byte, &rx_data, 1,1000);
	return rx_data;                            
}

// 写寄存器函数
uint8_t Int_SI24R1_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

	CSN_LOW;                 
	status = SPI_RW(reg);				
	SPI_RW(value);
	CSN_HIGH;
	
	return(status);
}


// 写缓冲区函数
uint8_t Int_SI24R1_Write_Buf(uint8_t reg, const uint8_t *pBuf, uint8_t bytes)
{
	uint8_t status,byte_ctr;

  CSN_LOW;                                  			
  status = SPI_RW(reg);                          
  for(byte_ctr=0; byte_ctr<bytes; byte_ctr++)     
    SPI_RW(*pBuf++);                                   	
  CSN_HIGH;
  
  return(status);       
}							  					   


// 读寄存器函数
uint8_t Int_SI24R1_Read_Reg(uint8_t reg)
{
 	uint8_t value;

	CSN_LOW;    
	SPI_RW(reg);			
	value = SPI_RW(0);
	CSN_HIGH;              

	return(value);
}


// 读缓冲区函数
uint8_t Int_SI24R1_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
	uint8_t status,byte_ctr;

  CSN_LOW;                                        
  status = SPI_RW(reg);                           
  for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
	pBuf[byte_ctr] = SPI_RW(0);
                    
  CSN_HIGH;                                       

  return(status);    
}


// 进入接收模式
void Int_SI24R1_RX_Mode(void)
{
	CE_LOW;
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);	// 接收地址
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_AA, 0x01);   
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_RXADDR, 0x01);           					
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_CH, 40);                 						
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);  						
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_SETUP, 0x06);            						
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + CONFIG, 0x0f);              						
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + STATUS, 0xff);  												
	CE_HIGH;                                       									
}						


// 进入发送模式
void Int_SI24R1_TX_Mode(void)
{
	CE_LOW;
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  

	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_AA, 0x01);       											
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_RXADDR, 0x01);   											
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + SETUP_RETR, 0x0a);  											
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_CH, 40);         											
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_SETUP, 0x06);    											
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + CONFIG, 0x0e);      											
	CE_HIGH;
}


// 接收数据函数
uint8_t Int_SI24R1_RxPacket(uint8_t *rxbuf)
{
	uint8_t state;
	state = Int_SI24R1_Read_Reg(STATUS);  			                 
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + STATUS,state);            

	if(state & RX_DR)								                           
	{
		Int_SI24R1_Read_Buf(RD_RX_PLOAD,rxbuf,TX_PLOAD_WIDTH);     
		Int_SI24R1_Write_Reg(FLUSH_RX,0xff);					            
		return 0; 
	}	   
	return 1;                                                   
}

// 发送数据函数
// 0: 发送成功
// 1: 发送失败
uint8_t Int_SI24R1_TxPacket(uint8_t *txbuf)
{
	uint8_t state;
	CE_LOW;																										  
    Int_SI24R1_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH);	   
 	CE_HIGH;																										     
																				
	state = Int_SI24R1_Read_Reg(STATUS); 
	while(((state & TX_DS)==0)&&((state & MAX_RT)==0))
	{
		state = Int_SI24R1_Read_Reg(STATUS); 
	}
	
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + STATUS, state); 								
	if(state&MAX_RT)																			 
	{
		Int_SI24R1_Write_Reg(FLUSH_TX,0xff);										   
		return 1; 
	}
	if(state&TX_DS)																			     
	{
		return 0;
	}
	return 1;																						 
}


uint8_t si24r1_rx_buff[5] = {0};

// 检查SI24R1是否正常工作
// 0: 正常
// 1: 异常
uint8_t Int_SI24R1_Check(void)
{
	// 测试SPI通信能否正常读写寄存器
	// 先读取一次再写
	Int_SI24R1_Read_Buf(SI24R1_READ_REG + TX_ADDR, si24r1_rx_buff, TX_ADR_WIDTH);
	// 写寄存器
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
	// 读取寄存器
	Int_SI24R1_Read_Buf(SI24R1_READ_REG + TX_ADDR, si24r1_rx_buff, TX_ADR_WIDTH);
	// 检查读取到的寄存器值是否与写入的一致
	for(uint8_t i=0;i<TX_ADR_WIDTH;i++)
	{
		if(si24r1_rx_buff[i]!=TX_ADDRESS[i])
		{
			return 1;
		}
	}
	return 0;
}

void Int_SI24R1_Init(void)
{
	HAL_Delay(200);
	// 校验检测
	while(Int_SI24R1_Check())
	{
		HAL_Delay(10);
	}

	// 设置SI24R1为接收模式
	Int_SI24R1_RX_Mode();
	debug_printf("SI24R1 Init Success!\n");
}
