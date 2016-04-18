#include <stdio.h>
#include   "stdlib.h"

#include "jinclude.h"
#include "jcapi.h"

extern unsigned char *image;
extern volatile unsigned char  *JPG_enc_buf;//jpeg 输出缓冲
extern unsigned int pt_buf;//缓冲区指针
extern  int width, height;
JSAMPLE *image;//图像源数据指针

jpeg_compress_info info1;

JQUANT_TBL  JQUANT_TBL_2[2];

JHUFF_TBL  JHUFF_TBL_4[4];

unsigned char dcttab[3][512];//

volatile unsigned char inbuf_buf[10240];
////输入区缓冲,这个是为宽度240的图片大小设置的，如果要更大的图片，就需要更大的缓冲11520 = 240x16x3

void  jpeg_encode(void) 
{
  int t;
  unsigned char string[20];
  jpeg_compress_info *cinfo;
  pt_buf = 0;
  cinfo = jpeg_create_compress();
  if (!cinfo) 
  {
    printf("error in create cinfo, malloc faild!\n");
  }
  cinfo->image_width = width;
  cinfo->image_height= height;
  cinfo->output = (char *)JPG_enc_buf;//fopen("test.jpg", "wb");
  cinfo->in_color_space = JCS_GRAY;
  jpeg_set_default(cinfo);  
  
  jpeg_start_compress(cinfo);
  while (cinfo->next_line < cinfo->image_height) 
  {
    jpeg_write_scanline(cinfo, &image[(cinfo->next_line*cinfo->image_width)]);
  }
  jpeg_finish_compress(cinfo);
  //fclose(cinfo->output);
  jpeg_destory_compress(cinfo);
}
