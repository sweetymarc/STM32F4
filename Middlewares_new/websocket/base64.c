/**
 *  base64���롢����ʵ��
 *       C����Դ����
 *
 *   ע�⣺��ʹ��gcc����
 *
 *             Ҷ����
 *
 * 
 *
 *  ʹ��˵����
 *      �����в���˵�������С�-d����������Ϊbase64���룬����Ϊbase64���롣
 *                      ���С�-o������������ļ��������������׼����ļ���
 *      �������Ա�׼����stdin�����Ϊ��׼���stdout�����ض��������������
 *
 *        base64���룺�������������������ȡ���ļ�������Ϊֹ�����������������ļ���β��Ϊֹ����
 *                    ������ı���base64���롣
 *
 *        base64���룺���봿�ı���base64���룬��ȡ���ļ�������Ϊֹ�����������������ļ���β��Ϊֹ����
 *                    ���ԭ���Ķ���������
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <unistd.h>
//#include <io.h>
//#include <fcntl.h>
//#include <stdbool.h>

#ifndef MAX_PATH
#define MAX_PATH 256
#endif
typedef unsigned int bool;
#define true 1
#define false 0
const char * base64char = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

char * base64_encode( const unsigned char * bindata, char * base64, int binlength )
{
    int i, j;
    unsigned char current;

    for ( i = 0, j = 0 ; i < binlength ; i += 3 )
    {
        current = (bindata[i] >> 2) ;
        current &= (unsigned char)0x3F;
        base64[j++] = base64char[(int)current];

        current = ( (unsigned char)(bindata[i] << 4 ) ) & ( (unsigned char)0x30 ) ;
        if ( i + 1 >= binlength )
        {
            base64[j++] = base64char[(int)current];
            base64[j++] = '=';
            base64[j++] = '=';
            break;
        }
        current |= ( (unsigned char)(bindata[i+1] >> 4) ) & ( (unsigned char) 0x0F );
        base64[j++] = base64char[(int)current];

        current = ( (unsigned char)(bindata[i+1] << 2) ) & ( (unsigned char)0x3C ) ;
        if ( i + 2 >= binlength )
        {
            base64[j++] = base64char[(int)current];
            base64[j++] = '=';
            break;
        }
        current |= ( (unsigned char)(bindata[i+2] >> 6) ) & ( (unsigned char) 0x03 );
        base64[j++] = base64char[(int)current];

        current = ( (unsigned char)bindata[i+2] ) & ( (unsigned char)0x3F ) ;
        base64[j++] = base64char[(int)current];
    }
    base64[j] = '\0';
    return base64;
}