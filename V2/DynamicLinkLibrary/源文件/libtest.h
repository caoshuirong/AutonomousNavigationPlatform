#pragma once
#include<stdio.h>
typedef unsigned char BYTE;

class Mymath {
	int sum(int, int);
	int sub(int, int);
	void  TransmitImage(BYTE * pRGBImg, int width, int height,int depth);
};

