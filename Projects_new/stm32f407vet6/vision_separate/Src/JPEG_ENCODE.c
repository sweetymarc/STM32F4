#include <stdio.h>
#include   "stdlib.h"

#include "jinclude.h"
#include "jcapi.h"

extern unsigned char *image;
extern volatile unsigned char  *JPG_enc_buf;//jpeg �������
extern unsigned int pt_buf;//������ָ��
extern  int width, height;
JSAMPLE *image;//ͼ��Դ����ָ��

jpeg_compress_info info1;

JQUANT_TBL  JQUANT_TBL_2[2];

JHUFF_TBL  JHUFF_TBL_4[4];

unsigned char dcttab[3][512];//

volatile unsigned char inbuf_buf[10240];
////����������,�����Ϊ���240��ͼƬ��С���õģ����Ҫ�����ͼƬ������Ҫ����Ļ���11520 = 240x16x3

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
