#pragma once

#define DLLEXPORT extern "C"  //__declspec(dllexport)
/*
注：#define DLLEXPORT extern “C” __declspec(dllexport)
1. windows下需要使用__declspec(dllexport)的声明来说明这个函数是动态库导出的,Linux下可注释掉。
2.extern “C”声明避免编译器对函数名称进行name mangling，这对于使用C++来编写DLL/SO是必须的。

*/
struct DWAresult {
	double resV;
	double resW;
} DWAres;
DLLEXPORT DWAresult DWA_port(double curV, double curW, double curX, double curY, double curTheta, double tarX, double tarY, int obstacle[], bool findTarget);
