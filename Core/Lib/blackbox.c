#include "fatfs.h"
#include "string.h"
#include "timer.h"
#include "utils.h"
#include "blackbox.h"
#include "math.h"
#include <stdlib.h>

#define MAX_BUFFER_SIZE 512

typedef struct{
   int indexx;
   FIL *file;
   uint16_t buffer_index;
   char buffer[MAX_BUFFER_SIZE];
}black_box_file_t;

black_box_file_t fs;

static void reverse( char *str, int len);
static int intToStr(int x,  char *str, int d);
static int n_tu(int number, int count);
static int Float_to_string(float f, uint8_t places, char strOut[]);

FRESULT mount_state;
FRESULT open_state;
int32_t puts_state;

/*
 * init black box
 */
void black_box_init(){
	fs.file = &SDFile;
    //SDFile.fs->id = 1;
    //SDFile.id = 1;
    mount_state = f_mount(&SDFatFS,"",1);
    open_state = f_open(&SDFile,"flight.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
    f_lseek (&SDFile,SDFile.fsize);
}

uint32_t black_box_get_file_size(){
    return SDFile.fsize;
}

void black_box_pack_float(float val,uint8_t digit_after_point ){
    // do not write anything if sd card not valid

    char str_[20];
    memset(str_,0,20);
    Float_to_string(val,digit_after_point,str_);
    uint8_t index = 0;
    while(str_[index]){
        if((fs.buffer_index + index) > MAX_BUFFER_SIZE){
                fs.buffer_index = MAX_BUFFER_SIZE;
                return;
        }
        fs.buffer[fs.buffer_index + index] = str_[index];
        index ++;
    }
    fs.buffer_index += index;
}



void black_box_pack_int(int val){

	fs.indexx=0;
	int len_str;
	int val_ = val;
	char str_[11];
	memset(str_,0,11);
	if(val != 0){
		val = abs(val);
		len_str = intToStr(val,str_,0);
		if(val_ < 0){
			for(int i = len_str; i > 0; i--){
				str_[i] = str_[i - 1];
			}
			len_str ++;
			str_[0] = '-';
		}
	}
	else{
	   fs.buffer[fs.buffer_index] ='0';
	   len_str = 1;
	   fs.buffer_index ++;
		return;
	}
	// copy str to buffer
	int str_idx = 0;
	int index_flag;
	int max_index = fs.buffer_index + len_str;
	if(max_index <=  MAX_BUFFER_SIZE){
		max_index = fs.buffer_index + len_str;
		index_flag = 1;
	}
	else{
		max_index = MAX_BUFFER_SIZE;
		index_flag = 0;
	}
	for(int j = fs.buffer_index ; j < max_index; j++ ){
			fs.buffer[j] = str_[str_idx ++];
	}
	
	if(index_flag){
		fs.buffer_index += len_str;
	}
	else{
		fs.buffer_index += MAX_BUFFER_SIZE;
	}
}


void black_box_pack_str(char *c){

    int i = 0;
    while (c[i]){
        if((fs.buffer_index + i) > MAX_BUFFER_SIZE){
                fs.buffer_index = MAX_BUFFER_SIZE;
                return;
        }
        fs.buffer[fs.buffer_index + i] = c[i];
        i ++;
    }
    fs.buffer_index += i;
}

void black_box_pack_char(char c){
    fs.buffer[fs.buffer_index ] = c;
    fs.buffer_index ++;
}



void black_box_load()
 {
	  puts_state = f_puts(fs.buffer,fs.file);
      f_sync(fs.file);
      memset(fs.buffer,0,MAX_BUFFER_SIZE);
      fs.buffer_index = 0;
 }


void black_box_sync()
 { 
	 f_sync(fs.file);
 }


void black_box_close()
 {    
      f_close(fs.file);
 }


 uint16_t black_box_get_buffer_lenght()
 {
     return fs.buffer_index;
 }
 




static void reverse( char *str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

 static int intToStr(int x,  char *str, int d)
{
    while (x) {
        str[fs.indexx++] = (x % 10) + '0';
        x = x / 10;
    }

    while (fs.indexx < d)
        str[fs.indexx++] = '0';
    reverse(str,fs.indexx);
    return fs.indexx;
}




static int n_tu(int number, int count)
{
    int result = 1;
    while(count-- > 0)
        result *= number;

    return result;
}



static int Float_to_string(float f, uint8_t places, char strOut[])
{
    long long int length, length2, i, number, position, sign;
    float number2;

    sign = -1;   // -1 == positive number
    if (f < 0)
    {
        sign = '-';
        f *= -1;
    }

    number2 = f;
    number = f;
    length = 0;  // Size of decimal part
    length2 = 0; // Size of tenth


    while( (number2 - (float)number) != 0.0 && !((number2 - (float)number) < 0.0)  && (length2 < places))
    {
         number2 = f * (n_tu(10.0, length2 + 1));
         number = number2;

         length2++;
    }


    for (length = (f > 1) ? 0 : 1; f > 1; length++)
        f /= 10;

    position = length;
    length = length + 1 + length2;
    number = number2;
    if (sign == '-')
    {
        length++;
        position++;
    }

    for (i = length; i >= 0 ; i--)
    {
        if (i == (length))
            strOut[i] = '\0';
        else if(i == (position))
            strOut[i] = '.';
        else if(sign == '-' && i == 0)
            strOut[i] = '-';
        else
        {
            strOut[i] = (number % 10) + '0';
            number /=10;
        }
    }
    if( f - (int)f == 0.0){
        strOut[length++] = '0';
    }
    return length;
}


