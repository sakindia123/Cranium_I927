#include "cmc623_tune_value_LCD.h"

mDNIe_data_type cmc623_Bypass[]= 
{
	//start 
	{0x0000,0x0000},	//BANK 0
	{0x0001,0x0020},	//LABC
	{0x002c,0x0fff},	//DNR bypass {0x003C
	{0x002d,0x1900},	//DNR bypass {0x0a08
	{0x002e,0x0000},	//DNR bypass {0x1010
	{0x002f,0x00ff},	//DNR bypass {0x0400
	{0x003a,0x0000},	//HDTR off
	{0x00B4,0x4640},	//Count PWM
	{0x0000,0x0001},	//BANK 1
	{0x0020,0x0000},	//GAMMA bypass
	{0x0021,0x2000},
	{0x0022,0x2000},
	{0x0023,0x2000},
	{0x0024,0x2000},
	{0x0025,0x2000},
	{0x0026,0x2000},
	{0x0027,0x2000},
	{0x0028,0x2000},
	{0x0029,0x2000},
	{0x002A,0x2000},
	{0x002B,0x2000},
	{0x002C,0x2000},
	{0x002D,0x2000},
	{0x002E,0x2000},
	{0x002F,0x2000},
	{0x0030,0x2000},
	{0x0031,0x2000},
	{0x0032,0x2000},
	{0x0033,0x2000},
	{0x0034,0x2000},
	{0x0035,0x2000},
	{0x0036,0x2000},
	{0x0037,0x2000},
	{0x0038,0xFF00},
	{0x0020,0x0001},
	{0x0000,0x0000},	//BANK 0
	{0x0028,0x0000},
	//end


	{END_SEQ,0x0000},
};

mDNIe_data_type cmc623_Bypass_CABC[]= 
{
	//start 
	{0x0000,0x0000},	//BANK 0
	{0x0001,0x0030},	//LABC CABC
	{0x002c,0x0fff},	//DNR bypass 0x003C
	{0x002d,0x1900},	//DNR bypass 0x0a08
	{0x002e,0x0000},	//DNR bypass 0x1010
	{0x002f,0x00ff},	//DNR bypass 0x0400
	{0x003a,0x0000},	//HDTR off
	{0x006E,0x0000},	//CABC Fgain
	{0x006F,0x0000},
	{0x0070,0x0000},
	{0x0071,0x0000},
	{0x0072,0x2110},	//CABC Dgain
	{0x0073,0x2B14},
	{0x0074,0x1e2D},
	{0x0075,0x3F00},
	{0x0076,0x3c50},	//PowerLUT
	{0x0077,0x2d64},	//PowerLUT
	{0x0078,0x3c32},	//PowerLUT
	{0x0079,0x1e10},	//PowerLUT
	{0x007a,0x3200},	//PowerLUT
	{0x007C,0x0002},	//Dynamic LCD
	{0x00B4,0x5640},	//CABC PWM
	{0x0000,0x0001},	//BANK 1
	{0x0020,0x0000},	//GAMMA bypass
	{0x0021,0x2000},
	{0x0022,0x2000},
	{0x0023,0x2000},
	{0x0024,0x2000},
	{0x0025,0x2000},
	{0x0026,0x2000},
	{0x0027,0x2000},
	{0x0028,0x2000},
	{0x0029,0x2000},
	{0x002A,0x2000},
	{0x002B,0x2000},
	{0x002C,0x2000},
	{0x002D,0x2000},
	{0x002E,0x2000},
	{0x002F,0x2000},
	{0x0030,0x2000},
	{0x0031,0x2000},
	{0x0032,0x2000},
	{0x0033,0x2000},
	{0x0034,0x2000},
	{0x0035,0x2000},
	{0x0036,0x2000},
	{0x0037,0x2000},
	{0x0038,0xFF00},
	{0x0020,0x0001},
	{0x0000,0x0000},	//BANK 0
	{0x0028,0x0000},
	//end

	{END_SEQ,0x0000},
};

typedef enum
{
	CMC_Bypass,
	CMC_Bypass_CABC,
	CMC_Video,
	CMC_Video_CABC,
	CMC_Video_Warm,
	CMC_Video_Warm_CABC,
	CMC_Video_Cold,
	CMC_Video_Cold_CABC,
	CMC_Camera,
	CMC_Camera_CABC,
	CMC_UI,
	CMC_UI_CABC,
	CMC_VT,
	CMC_VT_CABC,
	CMC_DMB,
	CMC_DMB_CABC,
	CMC_GALLERY,
	CMC_GALLERY_CABC,
	CMC_USERSELECT_DYNAMIC,
	CMC_USERSELECT_STANDARD,
	CMC_USERSELECT_MOVIE,
}Cmc623_Value_Type;

u16* cmc623_values[]=
{
	(u16*)cmc623_Bypass,
	
	(u16*)cmc623_Bypass_CABC,
	
	cmc623_Video_LCD,
	
	cmc623_Video_CABC_LCD,
	
	cmc623_Video_Warm_LCD,
	
	cmc623_Video_Warm_CABC_LCD,

	cmc623_Video_Cold_LCD,
	
	cmc623_Video_Cold_CABC_LCD,
	
	cmc623_Camera_LCD,
	
	cmc623_Camera_CABC_LCD,
	
	cmc623_UI_LCD,
	
	cmc623_UI_CABC_LCD,
	
	cmc623_VT_LCD,
	
	cmc623_VT_CABC_LCD,
	
	cmc623_DMB_LCD,
	
	cmc623_DMB_CABC_LCD,
	
	cmc623_GALLERY_LCD,
	
	cmc623_GALLERY_CABC_LCD,

	cmc623_USERSELECT_DYNAMIC,

	cmc623_USERSELECT_STANDARD,

	cmc623_USERSELECT_MOVIE,
	
};

u16 OVE_values[] = 
{
	0x0000,
	0x0000,
	0x6050,//video
	0x6050,//video cabc
	0x6050,//camera
	0x6050,//camera cabc
	0x4040,//ui
	0x3040,//ui cabc
	0x0000,//vt
	0x0000,//vt cabc
	0x6050,//dmb
	0x6050,//dmb cabc
	0x0000,//gallery
	0x0000,//gallery cabc
};


u16* cmc623_black_values[]=
{
	cmc623_Black_Minus_4_LCD,	cmc623_Black_Minus_4_CABC_LCD,
	
	cmc623_Black_Minus_3_LCD,	cmc623_Black_Minus_3_CABC_LCD,
	
	cmc623_Black_Minus_2_LCD,	cmc623_Black_Minus_2_CABC_LCD,
	
	cmc623_Black_Minus_1_LCD,	cmc623_Black_Minus_1_CABC_LCD,
	
	cmc623_Black_0_LCD,		cmc623_Black_0_CABC_LCD,
	
	cmc623_Black_Plus_1_LCD,	cmc623_Black_Plus_1_CABC_LCD,
	
	cmc623_Black_Plus_2_LCD,	cmc623_Black_Plus_2_CABC_LCD,
	
	cmc623_Black_Plus_3_LCD,	cmc623_Black_Plus_3_CABC_LCD,
	
	cmc623_Black_Plus_4_LCD,	cmc623_Black_Plus_4_CABC_LCD,
};
