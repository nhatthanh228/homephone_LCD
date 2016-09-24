/*  
This is arduino graphic library Version 1.0, that is designed only for homephone LCD (Viettel phone 10 pins) 

This library provide a commond way to draw simple basic things (draw pixel, line, rectangle, circles, triangle...etc).
Some functions is refer from Adafruit Industries.

you can change different pin to hookup Arduino with the LCD as this library is only used software SPI connection.

============
Copyright (c) 2013 Adafruit Industries.  All rights reserved.
============
if you want to modify, make sure that include the information above to respect author. and please contribute it.

The detail documentation of using this library is also public on this address: pls feel free to download it.

WE DON'T HAVE ANY RESPONSIBILITY IN CASE THAT YOUR DEVICE RUN INTO SOME ISSUE AFTER USE THIS SOFTWARE. 
BUT NORMALLY WE TESTED AND IT'S COMPATIBLE WITH HOMEPHONE (10 PINS).

if there is any thing you want to ask please contact me:
Contact name: TRAN NHAT THANH
Email address: NHATTHANH228@GMAIL.COM
i'd really appreciate if you send us any feedback. 
*/

#include "Arduino.h"
#include "pins_arduino.h"
#include <stdlib.h>
#include "homephone.h"
#include "glcdfont.c"
#include <avr/pgmspace.h>

#define allowUpdatePartially
//#define debug


const uint8_t pagemap[]{0,1,2,3,4,5,6,7};
uint8_t lcdBuffer[1024]={
};


#ifdef allowUpdatePartially
static uint8_t xUpdateMin, yUpdateMin, xUpdateMax, yUpdateMax;
#endif

//this will help to update screen parrtially. much faster that update whole screen each time data changed.
static void updateBoundingBox(uint8_t xmin, uint8_t ymin, uint8_t xmax, uint8_t ymax){
#ifdef allowUpdatePartially
	if (xmin < xUpdateMin) xUpdateMin = xmin;
	if (xmax > xUpdateMax) xUpdateMax = xmax;
	if (ymin < yUpdateMin) yUpdateMin = ymin;
	if (ymax > yUpdateMax) yUpdateMax = ymax;
#endif
}

void homephone::lcdWrite(uint8_t command, byte c){
	
	digitalWrite(cs, LOW);
	if (command){
		digitalWrite (a, HIGH);
	
	}
	else {
		digitalWrite (a, LOW);
	}
	shiftOut (sdin, slk, MSBFIRST, c);	
}

void homephone::begin(void){
	pinMode(sdin, OUTPUT);
	pinMode(slk, OUTPUT);
	pinMode(a, OUTPUT);
	pinMode(rst, OUTPUT);
	pinMode(cs, OUTPUT);
	
	digitalWrite (rst,LOW);
	delay(515);
	digitalWrite(rst, HIGH);
	lcdWrite (0, SET_BIAS_9);
	lcdWrite (0, SOFTWARE_RESET);
	lcdWrite (0, DISPLAY_ON);
	lcdWrite (0, POWER_CONTROL );
	lcdWrite(0, 0x26); //regulation ratio
	lcdWrite(0, 0x2F);
	lcdWrite(0, 0xA4);
	
	//for debuging 
	#ifdef debug
		Serial.begin(9600);
	#endif
}

void homephone::setContrast (uint8_t c){
	lcdWrite(0, 0x81);
	lcdWrite(0, 0|(c & 0x3f));	
} 

void homephone::display(){
	uint8_t col, maxcol, i;
	//check if page is apart of update
	for (i  =0; i<8; i++){
		#ifdef allowUpdatePartially
			if(yUpdateMin >=((i+1)*8)){
				continue;  //if no then skip it!
			}
			if (yUpdateMax <i*8){
				break;
			}
		#endif
		
		//set page
		lcdWrite(0, SET_PAGE | pagemap[i]);
		
		#ifdef allowUpdatePartially
			col = xUpdateMin; 			//update from begining 
			maxcol = xUpdateMax;		//to the maximum col is going to used
		#else
			col =0 ;					//update from begining
			maxcol = LCDWIDTH -1; 		//to the end of col (0 - 127)
		#endif
		
		//set col 
		lcdWrite(0, COL_LSB | ((col) & 0xf));
		lcdWrite(0, COL_MSB | ((col)>>4)& 0xf);
		
		
		for (;col<=maxcol;col++){
			lcdWrite(1, lcdBuffer[(128*i)+col]); //transfer data from buffer to lcd
		}
		
	}
#ifdef allowUpdatePartially
	xUpdateMin = LCDWIDTH -1;
	xUpdateMax = 0;
	yUpdateMax = 0;
	yUpdateMin = LCDHEIGHT -1;
#endif
}

void homephone::clearDisplay(){
	updateBoundingBox (0,0,127,63);
	for (int i =0; i <1024; i++){
		lcdBuffer[i] = 0;
	}
	display();
}

void homephone::drawPixel(uint8_t x, uint8_t y, uint8_t color){
	if (color){
		lcdBuffer[x+(y/8)*128] |= 1 << (y%8);
	}
	else{
		lcdBuffer[x+(y/8)*128] &= ~(1<<(y%8));
	}
	updateBoundingBox(x,y,x,y);	
}

void homephone::drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color){
	
	//check slope line.
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	  }

	int16_t dx, dy;
	  dx = x1 - x0;
	  dy = abs(y1 - y0);

	  int16_t err = dx / 2;
	  int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	  } else {
		ystep = -1;
	  }

	for (; x0<=x1; x0++) {
		if (steep) {
		  drawPixel(y0, x0, color);
		} else {
		  drawPixel(x0, y0, color);
		}
		err -= dy;
		if (err < 0) {
		  y0 += ystep;
		  err += dx;
		}
	}
	updateBoundingBox (x0,y0,x1,y1);
}	

void homephone::drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color){
	drawLine(x,y,x+w,y,color) ;//drawLine1
	drawLine(x,y,x,y+h,color) ;//drawLine2
	drawLine(x,y+h,x+w,y+h,color); //drawLine 3
	drawLine(x+w,y,x+w,y+h,color); //drawLine 4
	updateBoundingBox(x,y,x+w, y+h); //update screen faster
}

void homephone::fillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color){
	for (int i =y; i<=y+h; i++){
		drawLine(x, i ,x+w,i,color);
	}
	updateBoundingBox (x,y,x+w, y+h);
}

void homephone::drawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  drawPixel(x0  , y0+r, color);
  drawPixel(x0  , y0-r, color);
  drawPixel(x0+r, y0  , color);
  drawPixel(x0-r, y0  , color);


  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    //drawPixel(x0 + x, y0 + y, color);
    //drawPixel(x0 - x, y0 + y, color);
    //drawPixel(x0 + x, y0 - y, color);
    //drawPixel(x0 - x, y0 - y, color);
    //drawPixel(x0 + y, y0 + x, color);
    //drawPixel(x0 - y, y0 + x, color);
    drawPixel(x0 + y, y0 - x, color);
    drawPixel(x0 - y, y0 - x, color);
  }
  updateBoundingBox (x0-r, y0-r, x0+r, y0+r);
}  

 void homephone::fillCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color){
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  //drawPixel(x0  , y0+r, color);
  //drawPixel(x0  , y0-r, color);
  //drawPixel(x0+r, y0  , color);
  //drawPixel(x0-r, y0  , color);
	drawLine (x0-r, y0, x0+r, y0,color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    drawLine(x0 + x, y0 + y, x0 - x, y0 + y, color);
	drawLine(x0 + x, y0 - y, x0 - x, y0 - y, color);
	drawLine(x0 + y, y0 + x, x0 - y, y0 + x, color);
	drawLine(x0 + y, y0 - x, x0 - y, y0 - x, color);
  }
  updateBoundingBox (x0-r, y0-r, x0+r, y0+r);
} 

void homephone::drawTriangle (uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color){
	drawLine(x0,y0,x1,y1,color);
	drawLine(x1,y1,x2,y2,color);
	drawLine(x2,y2,x0,y0,color);
	updateBoundingBox (x1, y0, x2, y2);
}

void homephone::fillTriangle (uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color){
	
	//these three "if" conditions below is sort the point from smallest to bigest number
	if(y0>y1){
		swap(x0,x1);
		swap(y0,y1);
	}
	
	if (y1>y2){
		swap(x1,x2);
		swap(y1,y2);
	}
	
	if (y0>y1){
		swap(x0,x1);
		swap(y0,y1);
	}
	
	//in case triangle points are located on the same line
	if (y0 == y2){
		drawLine(x0,y0,x2,y2,color);
	}
	
	//allocate the intersection point xy 
	
	float m = (y2-y0)/(x2 -x0);
	float b = (y0 - m*x0);
	int y = y1;
	int x = round ((y1/m)-(b/m));
	#ifdef debug 
		Serial.print("("); Serial.print(x); Serial.print(" ");
		Serial.print(y); Serial.println(")");
	#endif
	
}

void homephone::drawChar (uint8_t x, uint8_t y, unsigned char c, uint8_t color, uint8_t bg){
	if ((x >= 128 )|| (y >= 64) || (x <0) || (y<0)){
		return;
	}
	
	for (int8_t i =0; i<6; i++){
		uint8_t line;
		if (i ==5){
			line =0x0;
		}
		else {
			line = pgm_read_byte (font+(c*5)+i);
			#ifdef debug
				Serial.print(line, HEX);
			#endif
		}
		for (int8_t j=0; j<8; j++){
			if (line & 0x1){
				drawPixel (x+i, y+j, color);
			}
			else if (bg!=color){
				drawPixel (x+i, y+j, bg);
			}
			line >>=1;
		}
		
	}
}

/* void homephone::drawBitmap (uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t w, uint8_t h, uint8_t color, uint8_t bg){
	//byte reader;
	int k =0; 
	for (int i =0 ; i <w; i ++){
		
		for (int j =0 ; j <h; j++){
			if ((pgm_read_byte(bitmap+((h+7)/8*i+(j/8)))<<k)&0b10000000){
				drawPixel(x+i, y+j,color);
			}
			else {
				drawPixel(x+i, y+j,bg);
			}
			k++;
			if (k > 7){
				k=0;
			}
		}
	}
} */

void homephone::drawBitmap(uint8_t x, uint8_t y, const uint8_t * bitmap, uint8_t w, uint8_t h, uint8_t color, uint8_t bg){
	int16_t i, j, byteWidth = (w+7)/8;  //caculate the byte need to use for width. for example if the picture is 16x16 then the number of byte use to represent width will be (16+7)/8 = 2
	for (i =0; i <h; i ++){
		for (j =0; j <w; j++){
			if (pgm_read_byte(bitmap+(i* byteWidth+j/8))&(128>>(j&7))){ //this pgm_read_byte(bitmap+(i* byteWidth+j/8))&(128>>(j&7))) is used to read only one bit in the byte of Pixel which need to represent. if there is a pixel then function will return 1, if no then 0.
				drawPixel(x+j, y+i, color); //if the function above return 1 then put a black pixel to that current position.
			}
			else{
				drawPixel(x+j, y+i, bg);// if not then put a white pixel on that current position.
			}
		}
	}
}

void homephone::setCursor(uint8_t x, uint8_t y){
	cursor_x = x;
	cursor_y = y;
}

void homephone::setTextColor(uint8_t color, uint8_t bgColor){
	textColor = color;
	textBgColor = bgColor;
}

size_t homephone::write(uint8_t c){
	if ( c == '\n'){
		cursor_y += 8;
		cursor_x =0;
	}
	else if (c=='\r'){
		
	}
	else {
		drawChar (cursor_x, cursor_y, c, textColor, textBgColor);
		cursor_x +=6;
	}
}	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	