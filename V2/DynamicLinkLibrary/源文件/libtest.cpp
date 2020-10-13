
#define DLLEXPORT extern "C"  // __declspec(dllexport)
/*
注：#define DLLEXPORT extern “C” __declspec(dllexport)
1. windows下需要使用__declspec(dllexport)的声明来说明这个函数是动态库导出的,Linux下可注释掉。
2.extern “C”声明避免编译器对函数名称进行name mangling，这对于使用C++来编写DLL/SO是必须的。

*/
#include"libtest.h"

//两数相加
DLLEXPORT int  sum(int a, int b) {
	return a + b;
}
#define M 2048 * 2048 * 3
BYTE pResImg[M];

DLLEXPORT void  TransmitImage(BYTE * pRGBImg,int width,int height,int depth) 
{
	if (width * height * depth > M)
	{
		printf("The total number of pixels is too large and exceeds the size of the array !\n");
		return;
	}
	BYTE * pCur = pRGBImg, *pEnd = pRGBImg + width * height * depth,*pRes = pResImg;
	for (; pCur < pEnd;)
		*(pRes++) = *(pCur++);
	return;
}

//两数相减

DLLEXPORT int sub(int a, int b) {
	return a - b;
}



