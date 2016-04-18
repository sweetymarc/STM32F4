#ifndef __Lcdlib__
#define __Lcdlib__
#include "main.h"
/*--------------------------------------------------------------------------
// �ļ�����Lcdlib.h
// ������  LCD��ͷ�ļ�
// ����ߣ�EU����
// �������ڣ�2013��10��16��
// ����EU���ӳ�Ʒ-��Ȩ����-����ؾ�
// EU-�Ȱ�Ƕ��ʽ����
// http://euse.taobao.com
//-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------
*  ��������					 
-------------------------------------------------------------------------------------------------------*/
//��ɫֵ
#define BLACK          0x0000   //��

#define RED            0xF800   //��  
#define ORANGE 		   	 0xFB00   //��
#define YELLOW         0xFFE0   //��
#define GREEN          0x07E0   //��
#define BLUE           0x001F   //��
#define PURPLE		   	 0x881F   //��
#define GRAY  		     0X8430   //��
#define WHITE          0xFFFF   //��

#define GOLDEN         0XBC40   //��
#define LIGHTBLUE      0x051F   //ǳ��
#define MAGENTA        0xF81F   //����
#define CYAN           0x7FFF   //��
//----- RGB565ԭ�� ------
//xxxxxxxx xxxxxxxx
//rrrrrggg gggbbbbb
//11111000 00000000 �� 0xF800
//00000111 11100000 �� 0x07E0
//00000000 00011111 �� 0x001F


//lcd direction
#define DIRECTION_H 1
#define DIRECTION_V 2
#define  LCD_DIRECT        DIRECTION_H           		   //1 ??  2 ??
//lcd bus chose
#define  LCD_BUSTYPE       1                 //1:16λ����  2:8λ����

/*-------------------------------------------------------------------------------------------------------
*  ����ӿ�								 
-------------------------------------------------------------------------------------------------------*/
//------------------------- APP USER LIB -----------------------------
//lcd��ʼ��
extern void LCD_Init(void);
//lcd����Ļ
extern void LCD_ClrScr(uint16_t BackColor);

//--------------------------------------------------------------------
//---------------- ��Ļ���Ͻ�Ϊԭ��,xָ��, yָ�� ---------------------
//--------------------------------------------------------------------
//------------------------- �� -----------------------------
//��x0,y0λ����ʾһ����ɫΪColor��
extern void LCD_DisAPoint(uint16_t x0, uint16_t y0, uint16_t Color);

//------------------------- �� -----------------------------
//��x0,y0λ����ʾһ��ֱ��(dir:����1/����2),����Ϊlenth,��ɫΪcolor
extern void LCD_DisALine(uint16_t x0, uint16_t y0, uint8_t dir, uint16_t lenth, uint16_t color);
//��x0,y0 ~ �Խ�x1,y1,��һ����ɫΪColor�ľ��α߿�
extern void LCD_DisABorder(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t Color);
//��x0,y0λ�û�һ���뾶Ϊr,��ɫΪColor��Բ��
extern void LCD_DisALoop(uint16_t x0, uint16_t y0, uint8_t r, uint16_t Color);

//------------------------- �� -----------------------------
//��x0,y0λ�ÿ�ʼ,��ʾһ�����Ϊwide,��ɫΪColor��������
extern bool LCD_DisASquare(uint16_t x0, uint16_t y0, uint16_t wide, uint16_t Color);   
//��x0,y0 ~ x1,y1λ�ô�,��ʾһ����ɫΪColor�ĳ�����
extern void LCD_DisARectangular(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t Color);

//--------------------- Ӣ���ַ� ---------------------------
//��x0,y0λ����ʾһ��ASC�ַ�,��ɫΪfColor,����ɫΪbColor
extern void LCD_DisASCString16x8(uint16_t x0, uint16_t y0, uint8_t *s, uint16_t fColor, uint16_t bColor);

//--------------------- �����ַ� ---------------------------
//��x0,y0λ����ʾһ�����ַ���,��ɫΪfColor,����ɫΪbColor
extern void LCD_DisGB2312String16x16(uint16_t x0, uint16_t y0, uint8_t *s, uint16_t fColor, uint16_t bColor);
//��x0,y0λ����ʾһ�����ַ���,��ɫΪfColor,����ɫΪbColor
extern void LCD_DisGB2312String32x32(uint16_t x0, uint16_t y0, uint8_t *s, uint16_t fColor, uint16_t bColor);

//------------------------ ͼƬ -----------------------------
//��x0,y0λ����ʾһ����Ϊhigh,��Ϊwide��ͼƬ, ͼƬ��Ϣ��RGB565��֯(��˸�ʽÿ�����ص�ռ��2���ֽ�)
//ͼƬ��Ϣͷ��ַָ��ΪpData
void LCD_OpenWin(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
extern void LCD_DisAPhoto(uint16_t x0, uint16_t y0, uint16_t high, uint16_t wide, void *pData);
extern bool LCD_DisAPhoto_DMA(uint16_t x0, uint16_t y0, uint16_t high, uint16_t wide, void *pData);


#endif



