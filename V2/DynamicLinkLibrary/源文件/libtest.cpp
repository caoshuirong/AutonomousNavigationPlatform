
#define DLLEXPORT extern "C"  // __declspec(dllexport)
/*
ע��#define DLLEXPORT extern ��C�� __declspec(dllexport)
1. windows����Ҫʹ��__declspec(dllexport)��������˵����������Ƕ�̬�⵼����,Linux�¿�ע�͵���
2.extern ��C����������������Ժ������ƽ���name mangling�������ʹ��C++����дDLL/SO�Ǳ���ġ�

*/
#include"libtest.h"

//�������
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

//�������

DLLEXPORT int sub(int a, int b) {
	return a - b;
}



