/*--------------------------------------------------------------------------
// �ļ�����LcdLib.c
// ������  LCD�⺯��
// ����ߣ�EU���� 
// ����EU���ӳ�Ʒ-��Ȩ����-����ؾ�
// EU-�Ȱ�Ƕ��ʽ����
// http://euse.taobao.com
//-------------------------------------------------------------------------*/
/* ͷ�ļ����� INCLUDES */
#include "prohead.h"
#include "stm32f429_lcd.h"
#include "ILI9341.h"
#include "ASC8x16.h"
#include "GB2312.h"

#include "stm32f4xx.h"

/*-------------------------------------------------------------------------------------------------------
*  �ڲ�����								 
-------------------------------------------------------------------------------------------------------*/
//lcd drivers
#define	 LCD_CS_H  				Driver_LcdCS(1)
#define	 LCD_CS_L  				Driver_LcdCS(0)
#define	 LCD_RS_H  				Driver_LcdRS(1)
#define	 LCD_RS_L  				Driver_LcdRS(0)
#define	 LCD_RD_H  				Driver_LcdRD(1)
#define	 LCD_RD_L  				Driver_LcdRD(0)
#define	 LCD_WR_H  				Driver_LcdWR(1)
#define	 LCD_WR_L  				Driver_LcdWR(0)
#define  LCD_SEND(x)    	Driver_LcdSendData((x))

//lcd resolution 
#if LCD_DIRECT == DIRECTION_H
	#define  LCD_ROW_NUM    240                //??
	#define  LCD_COL_NUM    320                //??
#else
	#define  LCD_ROW_NUM    320                //??
	#define  LCD_COL_NUM    240                //??
#endif

/*-------------------------------------------------------------------------------------------------------
*  ��Դ����											 
-------------------------------------------------------------------------------------------------------*/



/*-------------------------------------------------------------------------------------------------------
*  ��������												 
-------------------------------------------------------------------------------------------------------*/
//lcd bottom funtions
void LCD_WriteReg(uint16_t Index);
void LCD_WriteData(uint16_t Data);
void LCD_TimerCountHandle(void);
void LCD_PortInit(void);
void LCD_Reset(void);
void LCD_Init(void);
//back light
//...
//lcd display
void LCD_OpenWin(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void LCD_ClrScr(uint16_t BackColor);
void LCD_DisAPoint(uint16_t x0, uint16_t y0, uint16_t Color);
void LCD_DisALine(uint16_t x0, uint16_t y0, u8 dir, uint16_t lenth, uint16_t color);
void LCD_DisABorder(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t Color);
void LCD_DisALoop(uint16_t x0, uint16_t y0, u8 r, uint16_t Color);
bool LCD_DisASquare(uint16_t x0, uint16_t y0, uint16_t wide, uint16_t Color);   
void LCD_DisARectangular(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t Color);
void LCD_DisASCString16x8(uint16_t x0, uint16_t y0, u8 *s, uint16_t fColor, uint16_t bColor);
void LCD_DisGB2312String16x16(uint16_t x0, uint16_t y0, u8 *s, uint16_t fColor, uint16_t bColor);
void LCD_DisGB2312String32x32(uint16_t x0, uint16_t y0, u8 *s, uint16_t fColor, uint16_t bColor);
void LCD_DisAPhoto(uint16_t x0, uint16_t y0, uint16_t high, uint16_t wide, void *pData);


/*-------------------------------------------------------------------------------------------------------
*  ִ�д���													 
-------------------------------------------------------------------------------------------------------*/
/********************************************************************************************************
*  Function: LCD_WriteReg				                                                           
*  Object: lcd write reg
*  Input: index
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void LCD_WriteReg(uint16_t Index)
{
		BSP_LCD_WriteReg(Index);
}

/********************************************************************************************************
*  Function: LCD_WriteData				                                                           
*  Object: lcd write data
*  Input: index and data
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void LCD_WriteData(uint16_t Data)
{
		BSP_LCD_WriteData(Data);
}

/********************************************************************************************************
*  Function: LCD_Reset				                                                           
*  Object: lcd reset control
*  Input: none
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void LCD_Reset(void)
{
		BSP_LcdReset(0);
		HAL_Delay(10);
		BSP_LcdReset(1);
		HAL_Delay(200); //min 120
}

/********************************************************************************************************
*  Function: Driver_LcdReset				                                                           
*  Object: lcd reset control
*  Input: sta
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void LCD_Init(void)
{ 
		//-- LCD PORT INIT --
		BSP_LCD_Init();
		//-- LCD RESET--
		LCD_Reset();
		//-------------- Initial Sequence ---------------
		//************* Start Initial Sequence **********//	
#if LCD_DIRECT == DIRECTION_V
		LCD_WriteReg(0xCF);  
		LCD_WriteData(0x00); 
		LCD_WriteData(0xC1); 
		LCD_WriteData(0X30); 
		LCD_WriteReg(0xED);  
		LCD_WriteData(0x64); 
		LCD_WriteData(0x03); 
		LCD_WriteData(0X12); 
		LCD_WriteData(0X81); 
		LCD_WriteReg(0xE8);  
		LCD_WriteData(0x85); 
		LCD_WriteData(0x10); 
		LCD_WriteData(0x7A); 
		LCD_WriteReg(0xCB);  
		LCD_WriteData(0x39); 
		LCD_WriteData(0x2C); 
		LCD_WriteData(0x00); 
		LCD_WriteData(0x34); 
		LCD_WriteData(0x02); 
		LCD_WriteReg(0xF7);  
		LCD_WriteData(0x20); 
		LCD_WriteReg(0xEA);  
		LCD_WriteData(0x00); 
		LCD_WriteData(0x00); 
		LCD_WriteReg(0xC0);    //Power control 
		LCD_WriteData(0x1B);   //VRH[5:0] 
		LCD_WriteReg(0xC1);    //Power control 
		LCD_WriteData(0x01);   //SAP[2:0];BT[3:0] 
		LCD_WriteReg(0xC5);    //VCM control 
		LCD_WriteData(0x30); 	 //3F
		LCD_WriteData(0x30); 	 //3C
		LCD_WriteReg(0xC7);    //VCM control2 
		LCD_WriteData(0XB7); 
		LCD_WriteReg(0x36);    // Memory Access Control 
		LCD_WriteData(0x48); 
		LCD_WriteReg(0x3A);   
		LCD_WriteData(0x55); 
		LCD_WriteReg(0xB1);   
		LCD_WriteData(0x00);   
		LCD_WriteData(0x1A); 
		LCD_WriteReg(0xB6);    // Display Function Control 
		LCD_WriteData(0x0A); 
		LCD_WriteData(0xA2); 
		LCD_WriteReg(0xF2);    // 3Gamma Function Disable 
		LCD_WriteData(0x00); 
		LCD_WriteReg(0x26);    //Gamma curve selected 
		LCD_WriteData(0x01); 
		LCD_WriteReg(0xE0);    //Set Gamma 
		LCD_WriteData(0x0F); 
		LCD_WriteData(0x2A); 
		LCD_WriteData(0x28); 
		LCD_WriteData(0x08); 
		LCD_WriteData(0x0E); 
		LCD_WriteData(0x08); 
		LCD_WriteData(0x54); 
		LCD_WriteData(0XA9); 
		LCD_WriteData(0x43); 
		LCD_WriteData(0x0A); 
		LCD_WriteData(0x0F); 
		LCD_WriteData(0x00); 
		LCD_WriteData(0x00); 
		LCD_WriteData(0x00); 
		LCD_WriteData(0x00); 		 
		LCD_WriteReg(0XE1);    //Set Gamma 
		LCD_WriteData(0x00); 
		LCD_WriteData(0x15); 
		LCD_WriteData(0x17); 
		LCD_WriteData(0x07); 
		LCD_WriteData(0x11); 
		LCD_WriteData(0x06); 
		LCD_WriteData(0x2B); 
		LCD_WriteData(0x56); 
		LCD_WriteData(0x3C); 
		LCD_WriteData(0x05); 
		LCD_WriteData(0x10); 
		LCD_WriteData(0x0F); 
		LCD_WriteData(0x3F); 
		LCD_WriteData(0x3F); 
		LCD_WriteData(0x0F); 
		LCD_WriteReg(0x2B); 
		LCD_WriteData(0x00);
		LCD_WriteData(0x00);
		LCD_WriteData(0x01);
		LCD_WriteData(0x3f);
		LCD_WriteReg(0x2A); 
		LCD_WriteData(0x00);
		LCD_WriteData(0x00);
		LCD_WriteData(0x00);
		LCD_WriteData(0xef);	 
		LCD_WriteReg(0x11); //Exit Sleep
		HAL_Delay(120);
		LCD_WriteReg(0x29); //display on
   	LCD_WriteReg(0x36);  
		LCD_WriteData(0xC9);	  
#else		
		LCD_WriteReg(0xCF);
		LCD_WriteData(0x00);
		LCD_WriteData(0x81);
		LCD_WriteData(0x30);
		LCD_WriteReg(0xED);
		LCD_WriteData(0x64);
		LCD_WriteData(0x03);
		LCD_WriteData(0x12);
		LCD_WriteData(0x81);
		LCD_WriteReg(0xE8);
		LCD_WriteData(0x85);
		LCD_WriteData(0x10);
		LCD_WriteData(0x78);
		LCD_WriteReg(0xCB);
		LCD_WriteData(0x39);
		LCD_WriteData(0x2C);
		LCD_WriteData(0x00);
		LCD_WriteData(0x34);
		LCD_WriteData(0x02);
		LCD_WriteReg(0xF7);
		LCD_WriteData(0x20);
		LCD_WriteReg(0xEA);
		LCD_WriteData(0x00);
		LCD_WriteData(0x00);
		LCD_WriteReg(0xB1);
		LCD_WriteData(0x00);
		LCD_WriteData(0x1B);
		LCD_WriteReg(0xB6);
		LCD_WriteData(0x0A);
		LCD_WriteData(0xA2);
		LCD_WriteReg(0xC0);
		LCD_WriteData(0x35);
		LCD_WriteReg(0xC1);
		LCD_WriteData(0x11);
		LCD_WriteReg(0xC5);
		LCD_WriteData(0x45);
		LCD_WriteData(0x45);
		LCD_WriteReg(0xC7);
		LCD_WriteData(0xA2);
		LCD_WriteReg(0xF2);
		LCD_WriteData(0x00);
		LCD_WriteReg(0x26);
		LCD_WriteData(0x01);
		LCD_WriteReg(0xE0);
		LCD_WriteData(0x0F);
		LCD_WriteData(0x26);
		LCD_WriteData(0x24);
		LCD_WriteData(0x0B);
		LCD_WriteData(0x0E);
		LCD_WriteData(0x09);
		LCD_WriteData(0x54);
		LCD_WriteData(0xA8);
		LCD_WriteData(0x46);
		LCD_WriteData(0x0C);
		LCD_WriteData(0x17);
		LCD_WriteData(0x09);
		LCD_WriteData(0x0F);
		LCD_WriteData(0x07);
		LCD_WriteData(0x00);
		LCD_WriteReg(0xE1);
		LCD_WriteData(0x00);
		LCD_WriteData(0x19);
		LCD_WriteData(0x1B);
		LCD_WriteData(0x04);
		LCD_WriteData(0x10);
		LCD_WriteData(0x07);
		LCD_WriteData(0x2A);
		LCD_WriteData(0x47);
		LCD_WriteData(0x39);
		LCD_WriteData(0x03);
		LCD_WriteData(0x06);
		LCD_WriteData(0x06);
		LCD_WriteData(0x30);
		LCD_WriteData(0x38);
		LCD_WriteData(0x0F);
		LCD_WriteReg(0x36);     //set the model of scanning
		LCD_WriteData((1<<5)|(0<<6)|(1<<7)|(1<<3)); //???
		//LCD_WriteData((1<<5)|(1<<6));????
		//LCD_WriteData(0x08); ???
		LCD_WriteReg(0x2B);     //set the page address ?????
		LCD_WriteData(0x00);
		LCD_WriteData(0x00);
		LCD_WriteData(0x00);
		LCD_WriteData(0xEF);
		LCD_WriteReg(0x2A);    //set the column address
		LCD_WriteData(0x00);
		LCD_WriteData(0x00);
		LCD_WriteData(0x01);
		LCD_WriteData(0x3F);
		//        LCD_WriteReg(0x2A);     //set the page address ?????
		//        LCD_WriteData(0x00);
		//        LCD_WriteData(0x00);
		//        LCD_WriteData(0x00);
		//        LCD_WriteData(0xEF);
		//        LCD_WriteReg(0x2B);    //set the column address
		//        LCD_WriteData(0x00);
		//        LCD_WriteData(0x00);
		//        LCD_WriteData(0x01);
		//        LCD_WriteData(0x3F);        
		LCD_WriteReg(0x3A);
		LCD_WriteData(0x55); //16pixel
		LCD_WriteReg(0x11);
		HAL_Delay(120);
		LCD_WriteReg(0x29);
		LCD_WriteReg(0x2C);
#endif
}

/********************************************************************************************************
*  Function: LCD_OpenWin				                                                           
*  Object: lcd open window for display
*  Input: x0,y0, x1, y1
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void LCD_OpenWin(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
		/*
		#if (LCD_DIRECT == 1)
			LCD_WriteReg(0x0044);
			LCD_WriteData((x1<<8)+x0);
			LCD_WriteReg(0x0045);
			LCD_WriteData((y1<<8)+y0);	
			LCD_WriteReg(0x0021);
			LCD_WriteData((y0<<8)+x0); 
		#else
			LCD_WriteReg(0x0045);
			LCD_WriteData((x1<<8)+x0);
			LCD_WriteReg(0x0044);
			LCD_WriteData((y1<<8)+y0);
			LCD_WriteReg(0x0021);
			LCD_WriteData((x0<<8)+y0); 
		#endif
			LCD_WriteReg(0x0022);
		//S_DogFeed();
		*/
		LCD_WriteReg(0x2A);	
		LCD_WriteData(y0>>8);
		LCD_WriteData(0x00FF&y0);		
		LCD_WriteData(y1>>8);
		LCD_WriteData(0x00FF&y1);
		LCD_WriteReg(0x2B);	
		LCD_WriteData(x0>>8);
		LCD_WriteData(0x00FF&x0);		
		LCD_WriteData(x1>>8);
		LCD_WriteData(0x00FF&x1);
		LCD_WriteReg(0x2C);
}

/********************************************************************************************************
*  Function: LCD_ClrScr				                                                           
*  Object: lcd clear screen
*  Input: backcolor
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void LCD_ClrScr(uint16_t BackColor)
{
		uint16_t i,j;
		LCD_OpenWin(0, 0, LCD_ROW_NUM-1, LCD_COL_NUM-1);
		for(i = 0; i < LCD_ROW_NUM; i++)
			 for(j =0; j < LCD_COL_NUM; j++)
					 LCD_WriteData(BackColor);
}

/********************************************************************************************************
*  Function: LCD_DisAPoint				                                                           
*  Object: Display a point at screen
*  Input: site and color
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void LCD_DisAPoint(uint16_t x0, uint16_t y0, uint16_t Color)
{
		LCD_DisASquare(x0, y0, 1, Color);
}

/********************************************************************************************************
*  Function: LCD_DisALine				                                                           
*  Object: Display a line
*  Input: site dir lenth wide color
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void LCD_DisALine(uint16_t x0, uint16_t y0, u8 dir, uint16_t lenth, uint16_t color)
{
		uint16_t x1,y1;
		x1 = x0;
		y1 = y0;
		if(dir == 1)
			y1 = y1 + lenth;
		else
			x1 = x1 + lenth;
		LCD_DisARectangular(x0, y0, x1, y1, color);
}

/********************************************************************************************************
*  Function: LCD_DisABorder				                                                           
*  Object: Display a border
*  Input: x0, y0, x1, y1, color
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void LCD_DisABorder(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t Color)
{
		LCD_DisALine(x0, y0, 1, y1-y0, Color);
		LCD_DisALine(x1, y0, 1, y1-y0, Color);
		LCD_DisALine(x0, y0, 2, x1-x0, Color);
		LCD_DisALine(x0, y1, 2, x1-x0, Color);
}

/********************************************************************************************************
*  Function: LCD_DisALoop				                                                           
*  Object: Display a loop
*  Input: site,radius and color
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void LCD_DisALoop(uint16_t x0, uint16_t y0, u8 r, uint16_t Color)
{
		s16 a,b,next;
		a	=	0;
		b = r;	  
		next = 3 - (r<<1);            
		while(a <= b)
		{
				LCD_DisAPoint(x0+a, y0-b, Color);             
				LCD_DisAPoint(x0+b, y0-a, Color);                      
				LCD_DisAPoint(x0+b, y0+a, Color);                          
				LCD_DisAPoint(x0+a, y0+b, Color);             
				LCD_DisAPoint(x0-a, y0+b, Color);                  
				LCD_DisAPoint(x0-b, y0+a, Color);             
				LCD_DisAPoint(x0-a, y0-b, Color);                          
				LCD_DisAPoint(x0-b, y0-a, Color);              	         
				a++;
				//use the bresenham    
				if(next<0)
					next += 4*a+6;	  
				else
				{
						next += 10+4*(a-b);   
						b--;
				} 						    
		}
} 

/********************************************************************************************************
*  Function: LCD_DisASquare				                                                           
*  Object: Display a square
*  Input: start point, wide, color
*  Output: none                                  
*  brief: none
********************************************************************************************************/
bool LCD_DisASquare(uint16_t x0, uint16_t y0, uint16_t wide, uint16_t Color)
{	
	if(LCD_DMA_busy){
		return false;
	}
	LCD_DMA_busy = true;
	uint16_t i,j;
	LCD_OpenWin(x0, y0, x0+wide-1, y0+wide-1);
	for(i = 0; i < wide; i++)
		for(j = 0; j < wide; j++)
				 LCD_WriteData(Color);
	LCD_DMA_busy = false;
	return false;
}

/********************************************************************************************************
*  Function: LCD_DisARectangular				                                                           
*  Object: Display a rectangular
*  Input: start point, end point, color
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void LCD_DisARectangular(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t Color)
{
		uint16_t i,j;
		LCD_OpenWin(x0, y0, x1, y1);
		for(i = 0; i <= x1-x0; i++)
			 for(j = 0; j <= y1-y0; j++)
					 LCD_WriteData(Color);
}

/********************************************************************************************************
*  Function: LCD_DisASC8x16				                                                           
*  Object: Display a ASC(16*8)
*  Input: site, char, fColor, bColor
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void LCD_DisASCString16x8(uint16_t x0, uint16_t y0, u8 *s, uint16_t fColor, uint16_t bColor)
{
		uint16_t i,j,l = 1;
		while(*s)
		{
				LCD_OpenWin(x0, y0+(8-1)*(l-1), x0+16-1, y0+(8-1)*l);
				for(i=0; i<16; i++) 
				{
						u8 m = InforCode_Font8x16[(*s)*16+i];
						for(j=0; j<8; j++) 
						{
								if(m&CHSBIT7)
									LCD_WriteData(fColor);
								else 
									LCD_WriteData(bColor);
								m <<= 1;
						}
				}
				s++;
				l++;
		}
}

/********************************************************************************************************
*  Function: LCD_DisGB2312String16x16				                                                           
*  Object: display a chinese string
*  Input: site, char, fColor, bColor
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void LCD_DisGB2312String16x16(uint16_t x0, uint16_t y0, u8 *s, uint16_t fColor, uint16_t bColor)
{
		uint16_t Num;
		u8 i,j,m,l = 1;
		while(*s)
		{
				LCD_OpenWin(x0, y0+(16-1)*(l-1), x0+16-1, y0+(16-1)*l);
				for(Num = 0; Num < sizeof(GB2312Code16x16)/35; Num++)
				{
						if((GB2312Code16x16[Num].Head[0] == *s) && (GB2312Code16x16[Num].Head[1] == *(s+1)))
						{ 
								for(i = 0; i < 32; i++) 
								{
										m = GB2312Code16x16[Num].Infor[i];
										for(j = 0; j<8; j++) 
										{
												if(m&CHSBIT7)
													LCD_WriteData(fColor);
												else 
													LCD_WriteData(bColor);
												m<<=1;
										}
								}
						}
				}
				s+=2;
				l++;
		}
}
/********************************************************************************************************
*  Function: LCD_DisAPhoto				                                                           
*  Object: display a photo
*  Input: Site(x0,y0), high and wide, pData
*  Output: none                                  
*  brief: none
********************************************************************************************************/

void LCD_DisAPhoto(uint16_t x0, uint16_t y0, uint16_t high, uint16_t wide, void *pData)
{
		uint32_t i,length_8b;
		length_8b = high * wide * 2;  		//RGB565 ÿһ���ص�ռ�������ֽ�
		LCD_OpenWin(x0, y0, x0+high-1, y0+wide-1);
		HAL_Delay(1);
		for(i = 0; i < length_8b; i+=2)
			LCD_WriteData((*((uint8_t *)pData+i))* 0x100 + (*((uint8_t *)pData+i+1)));
}
bool LCD_DisAPhoto_DMA(uint16_t x0, uint16_t y0, uint16_t high, uint16_t wide, void *pData)
{
	if(LCD_DMA_busy){
		return false;
	}
	uint32_t length;
	length = high * wide; //RGB565 ÿһ���ص�ռ�������ֽ�
	LCD_OpenWin(x0, y0, x0+high-1, y0+wide-1);
	//HAL_Delay(1);
  if(HAL_DMA_Start_IT(&hDmaLCD, (uint32_t)pData, (uint32_t)&(LCD->LCD_RAM), length) != HAL_OK)
  {
		return false;
  }
	LCD_DMA_busy = 1;
	return true;
}



