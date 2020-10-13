#pragma once
#ifdef WIN32 || WIN64

#define DLLEXPORT extern "C"  __declspec(dllexport)

#else

#define DLLEXPORT extern "C"

#endif
/*

ע��#define DLLEXPORT extern ��C�� __declspec(dllexport)
1. windows����Ҫʹ��__declspec(dllexport)��������˵����������Ƕ�̬�⵼����,Linux�¿�ע�͵���
2.extern ��C����������������Ժ������ƽ���name mangling�������ʹ��C++����дDLL/SO�Ǳ���ġ�

*/
struct DWAresult {
	double resV;
	double resW;
} DWAres;
DLLEXPORT DWAresult DWA_port(double curV, double curW, double curX, double curY, double curTheta, double tarX, double tarY, int rho[], double phi[], bool findTarget);
