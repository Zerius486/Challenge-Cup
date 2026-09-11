#ifndef __STMFLASH_H__
#define __STMFLASH_H__

#include "sys.h"

//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址
#define FLASH_WAITETIME  50000          //FLASH等待超时时间

//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((unsigned int)0x08000000) 	//扇区0起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((unsigned int)0x08004000) 	//扇区1起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((unsigned int)0x08008000) 	//扇区2起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((unsigned int)0x0800C000) 	//扇区3起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((unsigned int)0x08010000) 	//扇区4起始地址, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((unsigned int)0x08020000) 	//扇区5起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((unsigned int)0x08040000) 	//扇区6起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((unsigned int)0x08060000) 	//扇区7起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((unsigned int)0x08080000) 	//扇区8起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((unsigned int)0x080A0000) 	//扇区9起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((unsigned int)0x080C0000) 	//扇区10起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((unsigned int)0x080E0000) 	//扇区11起始地址,128 Kbytes 

#define STMFlash_ReadChar(__faddr__)	(*(volatile unsigned char*)(__faddr__))
#define STMFlash_ReadInt(__faddr__)		(STMFlash_ReadChar(__faddr__)<<24)|(STMFlash_ReadChar(__faddr__+1)<<16)|(STMFlash_ReadChar(__faddr__+2)<<8)|STMFlash_ReadChar(__faddr__+3)
#define STMFlash_ReadFloat(__faddr__)	(*(volatile float*)__faddr__)						//读取一个float数字
	
void STMFLASH_Write(unsigned int WriteAddr,unsigned int *pBuffer,unsigned int NumToWrite);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(unsigned int ReadAddr,unsigned int *pBuffer,unsigned int NumToRead);   		//从指定地址开始读出指定长度的数据
//测试写入
void Test_Write(unsigned int WriteAddr,unsigned int WriteData);	
#endif
