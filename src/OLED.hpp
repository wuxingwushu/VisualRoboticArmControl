#ifndef _RKNN_YOLOV5_DEMO_OLED_H_
#define _RKNN_YOLOV5_DEMO_OLED_H_

#include "GPIO.hpp"
#include "SPI.hpp"
#include "FontFunction.h"

#define Color_WHITE		0xffff
#define Color_BLACK		0x0000

class OLED
{
private:
    GPIO* SCL = nullptr;
    GPIO* SDA = nullptr;
	SPI* mSPI = nullptr;
    GPIO* RST = nullptr;
    GPIO* DC = nullptr;
    GPIO* CS = nullptr;
    GPIO* BLK = nullptr;
	//PWM* BLK = nullptr;

	FILE* FI = nullptr;
	FILE* FD = nullptr;

	const bool SPIbool;

	unsigned int mWide;
	unsigned int mHigh;

	unsigned int mMaxX;
	unsigned int mMinX = 0;
	unsigned int mMaxY;
	unsigned int mMinY = 0;
	bool UpdateRequest = true;
	unsigned short* mData = nullptr;

	inline void SetPix(unsigned int x, unsigned int y, unsigned short color);

	// 开始
	inline void Son_begin(bool DataOrIndex);
	// 向液晶屏写一个8位数据
	inline void Son_WriteData(unsigned char Data);
	// 向液晶屏写一个16位数据
    inline void Son_WriteData_16(unsigned int Data);
	// 结束
	inline void Son_end();

    // 向SPI总线传输一个8位数据
    inline void SPI_WriteData(unsigned char Data);
	// 向液晶屏写一个8位指令
    inline void Lcd_WriteIndex(unsigned char Data);
    // 向液晶屏写一个8位数据
    inline void Lcd_WriteData(unsigned char Data);
    // 向液晶屏写一个16位数据
    inline void Lcd_WriteData_16(unsigned int Data);
    // LCD复位时序
    inline void Reset();

    inline void Delay_ms(unsigned int time_ms);

	unsigned char UTF8_bytes;
	unsigned int X_Shifting;
public:
    OLED(unsigned int wide, unsigned int high, const bool SPIb = true);
    ~OLED();

    //液晶屏初始化
    void OLED_init();

    /*************************************************
    函数名：LCD_Set_Region
    功能：设置lcd显示区域，在此区域写点数据自动换行
    入口参数：xy起点和终点
    返回值：无
    *************************************************/
    void Lcd_SetRegion(unsigned int x_start,unsigned int y_start,unsigned int x_end,unsigned int y_end);

    //全屏填充函数
    void LCD_Clear(unsigned int Color, bool mode = 1);

	bool ShowFrame();

	void string_UTF8(const char* str, unsigned int x, unsigned int y, bool mode = 1);

	void char_UTF8(unsigned short size, unsigned int x, unsigned int y, bool mode);

	void SetLED_Brightness(float Percentage);
};

OLED::OLED(unsigned int wide, unsigned int high, const bool SPIb)
:mWide(wide), mHigh(high), mMaxX(wide), mMaxY(high), SPIbool(SPIb)
{
	if(SPIbool){
		mSPI = new SPI();
	}else{
		SCL = new GPIO(49, 1);
    	SDA = new GPIO(50, 1);
	}
    RST = new GPIO(66, 1);
    DC = new GPIO(65, 1);
    CS = new GPIO(64, 1);
    BLK = new GPIO(72, 1);
	BLK->SetValue(1);

	//BLK = new PWM(5, 3000000);
	//SetLED_Brightness(0.5f);

	mData = new unsigned short[wide*high];

	OLED_init();
}

void OLED::SetLED_Brightness(float Percentage){
	if(Percentage > 1){
		Percentage = 1;
	}else if(Percentage < 0){
		Percentage = 0;
	}
	//BLK->SetDutyCycle(1000000 * Percentage);
}

OLED::~OLED()
{
	if(SPIbool){
		delete mSPI;
	}else{
		delete SCL;
		delete SDA;
	}

    delete RST;
    delete DC;
    delete CS;
    delete BLK;

	delete mData;
}

//向SPI总线传输一个8位数据
inline void OLED::SPI_WriteData(unsigned char Data){
	if(SPIbool){
		mSPI->SPIwrite(&Data, 1);
	}else{
		for(unsigned char i=8;i>0;i--)
		{
			if(Data&0x80)SDA->SetValue(1); //输出数据
			else SDA->SetValue(0);
			SCL->SetValue(0);
			SCL->SetValue(1);
			Data<<=1;
		}
	}
}

// 开始
inline void OLED::Son_begin(bool DataOrIndex) {
	CS->SetValue(0);
    DC->SetValue(DataOrIndex);
}

// 向液晶屏写一个8位数据
inline void OLED::Son_WriteData(unsigned char Data) {
    SPI_WriteData(Data);
}

// 向液晶屏写一个16位数据
inline void OLED::Son_WriteData_16(unsigned int Data) {
	SPI_WriteData(Data>>8);
    SPI_WriteData(Data);
}

inline void OLED::SetPix(unsigned int x, unsigned int y, unsigned short color){
	if(x >= mWide)return;
	if(y >= mHigh)return;
	unsigned int IDD = y * mWide + x;
	if(mData[IDD] != color){
		UpdateRequest = true;
		mData[IDD] = color;
		if(x > mMaxX)mMaxX = x;
		else if(x < mMinX)mMinX = x;
		if(y > mMaxY)mMaxY = y;
		else if(y < mMinY)mMinY = y;
	}
}

// 结束
inline void OLED::Son_end() {
	CS->SetValue(1);
}

inline void OLED::Delay_ms(unsigned int time_ms){
    for(unsigned int i=0; i < time_ms; i++){
        usleep(1000);
    }
}

//向液晶屏写一个8位指令
inline void OLED::Lcd_WriteIndex(unsigned char Data){
    CS->SetValue(0);
    DC->SetValue(0);
    SPI_WriteData(Data);
    CS->SetValue(1);
}

//向液晶屏写一个8位数据
inline void OLED::Lcd_WriteData(unsigned char Data){
    CS->SetValue(0);
    DC->SetValue(1);
    SPI_WriteData(Data);
    CS->SetValue(1);
}

//向液晶屏写一个16位数据
inline void OLED::Lcd_WriteData_16(unsigned int Data){
    CS->SetValue(0);
    DC->SetValue(1);
    SPI_WriteData(Data>>8);
    SPI_WriteData(Data);
    CS->SetValue(1);
}




//LCD复位时序
inline void OLED::Reset(){
    RST->SetValue(0);
    Delay_ms(100);
    RST->SetValue(1);
    Delay_ms(100);
}


//液晶屏初始化
void OLED::OLED_init(){
    Reset();//Reset before LCD Init.
		
	//LCD Init For 1.44Inch LCD Panel with ST7735R.
	Lcd_WriteIndex(0x11);//Sleep exit 
	Delay_ms (120);
		
	//ST7735R Frame Rate
	Lcd_WriteIndex(0xB1); 
	Lcd_WriteData(0x01); 
	Lcd_WriteData(0x2C); 
	Lcd_WriteData(0x2D); 

	Lcd_WriteIndex(0xB2); 
	Lcd_WriteData(0x01); 
	Lcd_WriteData(0x2C); 
	Lcd_WriteData(0x2D); 

	Lcd_WriteIndex(0xB3); 
	Lcd_WriteData(0x01); 
	Lcd_WriteData(0x2C); 
	Lcd_WriteData(0x2D); 
	Lcd_WriteData(0x01); 
	Lcd_WriteData(0x2C); 
	Lcd_WriteData(0x2D); 
	
	Lcd_WriteIndex(0xB4); //Column inversion 
	Lcd_WriteData(0x07); 
	
	//ST7735R Power Sequence
	Lcd_WriteIndex(0xC0); 
	Lcd_WriteData(0xA2); 
	Lcd_WriteData(0x02); 
	Lcd_WriteData(0x84); 
	Lcd_WriteIndex(0xC1); 
	Lcd_WriteData(0xC5); 

	Lcd_WriteIndex(0xC2); 
	Lcd_WriteData(0x0A); 
	Lcd_WriteData(0x00); 

	Lcd_WriteIndex(0xC3); 
	Lcd_WriteData(0x8A); 
	Lcd_WriteData(0x2A); 
	Lcd_WriteIndex(0xC4); 
	Lcd_WriteData(0x8A); 
	Lcd_WriteData(0xEE); 
	
	Lcd_WriteIndex(0xC5); //VCOM 
	Lcd_WriteData(0x0E); 
	
	Lcd_WriteIndex(0x36); //MX, MY, RGB mode 
	Lcd_WriteData(0xC0); 
	
	//ST7735R Gamma Sequence
	Lcd_WriteIndex(0xe0); 
	Lcd_WriteData(0x0f); 
	Lcd_WriteData(0x1a); 
	Lcd_WriteData(0x0f); 
	Lcd_WriteData(0x18); 
	Lcd_WriteData(0x2f); 
	Lcd_WriteData(0x28); 
	Lcd_WriteData(0x20); 
	Lcd_WriteData(0x22); 
	Lcd_WriteData(0x1f); 
	Lcd_WriteData(0x1b); 
	Lcd_WriteData(0x23); 
	Lcd_WriteData(0x37); 
	Lcd_WriteData(0x00); 	
	Lcd_WriteData(0x07); 
	Lcd_WriteData(0x02); 
	Lcd_WriteData(0x10); 

	Lcd_WriteIndex(0xe1); 
	Lcd_WriteData(0x0f); 
	Lcd_WriteData(0x1b); 
	Lcd_WriteData(0x0f); 
	Lcd_WriteData(0x17); 
	Lcd_WriteData(0x33); 
	Lcd_WriteData(0x2c); 
	Lcd_WriteData(0x29); 
	Lcd_WriteData(0x2e); 
	Lcd_WriteData(0x30); 
	Lcd_WriteData(0x30); 
	Lcd_WriteData(0x39); 
	Lcd_WriteData(0x3f); 
	Lcd_WriteData(0x00); 
	Lcd_WriteData(0x07); 
	Lcd_WriteData(0x03); 
	Lcd_WriteData(0x10);  
	
	Lcd_WriteIndex(0x2a);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00+2);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x80+2);

	Lcd_WriteIndex(0x2b);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00+3);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x80+3);
	
	Lcd_WriteIndex(0xF0); //Enable test command  
	Lcd_WriteData(0x01); 
	Lcd_WriteIndex(0xF6); //Disable ram power save mode 
	Lcd_WriteData(0x00); 
	
	Lcd_WriteIndex(0x3A); //65k mode 
	Lcd_WriteData(0x05); 
	
	
	Lcd_WriteIndex(0x29);//Display on
}

/*************************************************
函数名：LCD_Set_Region
功能：设置lcd显示区域，在此区域写点数据自动换行
入口参数：xy起点和终点
返回值：无
*************************************************/
void OLED::Lcd_SetRegion(unsigned int x_start,unsigned int y_start,unsigned int x_end,unsigned int y_end)
{
	Lcd_WriteIndex(0x2a);
	Son_begin(1);
	if(SPIbool){
		unsigned char XData[4] = {0x02, (unsigned char)x_start, 0x02, (unsigned char)x_end};
		mSPI->SPIwrite(XData, 4);
	}else{
		Son_WriteData(0x02);
		Son_WriteData(x_start);
		Son_WriteData(0x02);
		Son_WriteData(x_end);
	}
	Son_end();

	Lcd_WriteIndex(0x2b);
	Son_begin(1);
	if(SPIbool){
		unsigned char YData[4] = {0x01, (unsigned char)y_start, 0x01, (unsigned char)y_end};
		mSPI->SPIwrite(YData, 4);
	}else{
		Son_WriteData(0x01);
		Son_WriteData(y_start);
		Son_WriteData(0x01);
		Son_WriteData(y_end);
	}
	Son_end();
	
	Lcd_WriteIndex(0x2c);

}

//全屏填充函数
void OLED::LCD_Clear(unsigned int Color, bool mode)
{
	unsigned char i,j;
	if(!mode){
		unsigned int IDD;
		for (i=0;i<mHigh;++i)
		{
    		for (j=0;j<mWide;++j)
			{
				IDD = i * mWide + j;
        		if(mData[IDD] != Color){
					UpdateRequest = true;
					mData[IDD] = Color;
					if(j > mMaxX)mMaxX = j;
					else if(j < mMinX)mMinX = j;
					if(i > mMaxY)mMaxY = i;
					else if(i < mMinY)mMinY = i;
				}
			}
		}
		return;
	}
	Lcd_SetRegion(0,0,mWide-1,mHigh-1);
	Son_begin(1);
	for (i=0;i<mHigh;++i)
	{
    	for (j=0;j<mWide;++j)
		{
        	Son_WriteData_16(Color);
		}
	}
	Son_end();
}

bool OLED::ShowFrame(){
	if(!UpdateRequest)return 0;
	unsigned int i, j;
	Lcd_SetRegion(mMinX,mMinY,mMaxX-1,mMaxY-1);
	Son_begin(1);
	for (i=mMinY;i<mMaxY;++i)
	{
		if(SPIbool){
			mSPI->SPIwrite((unsigned char*)(&mData[i * mWide + mMinX]), (mMaxX - mMinX) * sizeof(unsigned short));
		}else{
			for (j=mMinX;j<mMaxX;++j)
			{
        		Son_WriteData_16(mData[i * mWide + j]);
			}
		}
	}
	Son_end();
	UpdateRequest = false;
	mMaxX = 0;
	mMaxY = 0;
	mMinX = 128;
	mMinY = 160;
	return 1;
}

void OLED::char_UTF8(unsigned short size, unsigned int x, unsigned int y, bool mode){
	FontInformation Info;
	fseek(FI, 0, SEEK_SET); // 将文件指针移动到文件开头
    fseek(FI, size * sizeof(FontInformation), SEEK_CUR);// 将文件指针向后移动 size * sizeof(FontInformation) 个字节
    fread(&Info, sizeof(FontInformation), 1, FI);// 读取文件中的内容

	X_Shifting = Info.x;
	//x = x + Info.Dx;
	y = y + Info.Dy;

	unsigned int BINsize = Info.x * Info.y;
	BINsize = (BINsize / 8) + ((BINsize % 8) == 0 ? 0 : 1);
	unsigned char* TTFbin = new unsigned char[BINsize];
	fseek(FD, 0, SEEK_SET); // 将文件指针移动到文件开头
	fseek(FD, Info.Deviation, SEEK_CUR);// 将文件指针向后移动 Info.Deviation 个字节
    fread(TTFbin, BINsize, 1, FD);// 读取文件中的内容

	unsigned char Z;
	unsigned int shu = 0;
	if(mode){
		Lcd_SetRegion(x, y, x + Info.x - 1, y + Info.y - 1);
		Son_begin(1);
		for (size_t i = 0; i < BINsize; ++i)
		{
			Z = TTFbin[i];
			for (size_t j = 0; j < 8; ++j)
			{
				if(Z&0x80){
					Son_WriteData_16(Color_WHITE);
				}else{
					Son_WriteData_16(Color_BLACK);
				}
				Z = Z << 1;
				++shu;
				if(shu >= (Info.x * Info.y)){
					j = 8;
					i = BINsize;
				}
			}
		}
		Son_end();
	}else{
		int Xmax = x + Info.x;
		for (size_t i = 0; i < BINsize; ++i)
		{
			Z = TTFbin[i];
			for (size_t j = 0; j < 8; ++j)
			{
				if(Z&0x80){
					SetPix(x, y, Color_WHITE);
				}else{
					SetPix(x, y, Color_BLACK);
				}
				++x;
				if(x >= Xmax){
					x -= Info.x;
					++y;
				}
				Z = Z << 1;
				++shu;
				if(shu >= (Info.x * Info.y)){
					j = 8;
					i = BINsize;
				}
			}
		}
	}
	

	delete TTFbin;
}

void OLED::string_UTF8(const char* str, unsigned int x, unsigned int y, bool mode){
	const char* str_P = str;
	unsigned short zi;

	FI = fopen("FontInfo.bin", "r");
	FD = fopen("FontData.bin", "r");

	while (*str_P != 0)
	{
		zi = from_bytes(str_P, &UTF8_bytes);
		char_UTF8(zi, x, y, mode);
		str_P += UTF8_bytes;

		x += X_Shifting + 2;
		if(x > 108){
			break;
			x = 4;
			y += 22;
			if(y >= 140){
				break;
			}
		}
	}
	
	fclose(FD);
	fclose(FI);
}


#endif
