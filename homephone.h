
//#include "Arduino.h";
#include "pins_arduino.h"
#include "Print.h"

#define black 1
#define white 0
#define LCDWIDTH 128
#define LCDHEIGHT 64
#define COL_LSB 0x00
#define COL_MSB 0x10
#define SET_PAGE 0xB0
#define SET_BIAS_9 0xA2
#define INVERT 0xA7
#define UNINVERT 0xA6
#define DISPLAY_ON 0xAF
#define DISPLAY_OFF 0xAE
#define SOFTWARE_RESET 0x2E
#define POWER_CONTROL 0x2F

#define swap(a,b) {uint8_t t =a; a =b; b =t;}

class homephone: public Print{
	public:
	homephone(int8_t sdin, int8_t slk, int8_t a, int8_t rst, int8_t cs): sdin (sdin), slk(slk), a(a), rst(rst), cs(cs){}
	//homephone(uint8_t sdin, uint8_t slk, uint8_t a,uint8_t rst, uint8_t cs);
	void lcdWrite(uint8_t command, byte c);
	void begin(void);
	void setContrast(uint8_t c);
	void display();
	void clearDisplay();
	void drawPixel(uint8_t x, uint8_t y, uint8_t color);
	void drawLine (uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
	void drawRect (uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
	void fillRect (uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
	void drawCircle (uint8_t x, uint8_t y, uint8_t r, uint8_t color);
	void fillCircle (uint8_t x, uint8_t y, uint8_t r, uint8_t color);
	void drawTriangle (uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);
	void fillTriangle (uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);
	
	void setCursor(uint8_t x, uint8_t y);
	void setTextColor(uint8_t color, uint8_t bgColor);
	void drawChar(uint8_t x, uint8_t y, unsigned char c, uint8_t color, uint8_t bg);
	
	void drawBitmap(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t w, uint8_t h, uint8_t color, uint8_t bg);
	
	virtual size_t write(uint8_t);
	
	private:
	uint8_t sdin,slk,a,rst,cs;
	const char *str;
	
	protected:
	uint8_t cursor_x, cursor_y, textColor, textBgColor;
	
};
