#include "main.h"

// fatfs
FATFS SDFatFs;  /* File system object for SD card logical drive */
char SDPath[4]; /* SD card logical drive path */
char file_name[16];
void uSD_init();

void uSD_init(){
  /*##-1- Link the micro SD disk I/O driver ##################################*/
  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0){
    /*##-2- Register the file system object to the FatFs module ##############*/
    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
			printf("mount error\r\n");
      Error_Handler();
    }else{
			
		}
  }
  /*##-11- Unlink the micro SD disk I/O driver ###############################*/
  //FATFS_UnLinkDriver(SDPath);
}

			/*
			name_length = sprintf(file_name, "%3d.jpg", frame_count);
			i_name = 0;
			while(' ' == file_name[i_name]){
				file_name[i_name++] = '0';
			}
			file_name[7] = '\0';
			DISABLE_IRQ
			printf("file name: %s\r\n", file_name);
			ENABLE_IRQ */
