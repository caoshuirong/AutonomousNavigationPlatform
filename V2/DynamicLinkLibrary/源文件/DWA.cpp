#include <iostream>
#include <cmath>
#include<algorithm>
#include<cstdio>
#include"DWA.h"
using namespace std;

//之前定义的一些结构题，接口函数将不再用到，写在这里只做说明

struct SpeedPair {
	double v;		//线速度
	double w;		//角速度
};

struct DWindow {
	double maxV;	//最大线速度
	double maxW;	//最大角速度
	double minV;	//最小线速度
	double minW;	//最小角速度
};

//参数的数据类型可以根据你们的需求修改
//void DWA_port(double curV, double curW, double curX, double curY, double curTheta, double tarX, double tarY, int obstacle[], bool findTarget);
/*
* 参数表
* double curV：小车在这一时刻的线速度
* double curW：小车在这一时刻的角速度
* double curX：小车当前的X坐标，需要根据初始坐标一步一步递推出来
* double curY：小车当前的Y坐标，需要根据初始坐标一步一步递推出来
* double curTheta：小车当前的旋转角（需要定义起点和正方向）
* double tarX：目标的X坐标
* double tarY：目标的Y坐标
* int rho[]：线扫雷达传回来的“极径”，长度定义为常量numR=1800		*** 新增
* double phi[]：线扫雷达传回来的“极角”，长度定义为常量numR=1800    	*** 新增
* bool findTarget：找到目标的标志位（同模拟平台中的S2Cdata.detect_object）
* double* resV：下一时刻的线速度（指针）
* double* resW：下一时刻的角速度（指针）
*/

//递推地计算坐标和旋转角的公式，约定好一个时间片(tickGap)0.2s
//curX = curX + resV * 0.2 * cos(curTheta);
//curY = curY + resW * 0.2 * sin(curTheta);
//curTheta = curTheta + curW * 0.2;
//一定要先算curX和curY再算curTheta

int main() {
	return 0;
}

DWAresult DWA_port(double curV, double curW, double curX, double curY, double curTheta, double tarX, double tarY, int rho[], double phi[], bool findTarget) {
	const double PI = 3.1415926;
	//4个极限速度
	const double lim_minV = 0;
	const double lim_maxV = 50;
	const double lim_minW = -PI;
	const double lim_maxW = PI;

	//2个加速度（单位仍为像素，这里根据你们需要自己调）
	const double a = 20;
	const double alpha = PI;

	//时间参数
	const double tickGap = 0.2;	//时间片间隔0.2s
	int ticks = 10;		//前向预测的时间片数目

						//速度空间离散值总个数
	const int N1 = 50;	//总共N1个离散线速度，范围从minV到maxV
	const int N2 = 50;	//总共N2个离散角速度，范围从minW到maxW
	const int maxN = 3000;	//规定N1*N2<maxN

							//////********************
							//线扫雷达采样点总个数，新增常量
	const int numR = 1800;	//这里按照你们的要求设置为了1800个点
							//////********************

							//阈值参数（单位仍为像素，需要你们自己调整）
	double nearDist = 80;	//小车与目标小于nearDist时，将前向预测时间片个数减少
	double R = 29;			//假定小车所占像素为29

							//权重参数
	double alpha1 = .237;
	double beta1 = .5;
	double gamma1 = .3;

	double alpha2 = .48;
	double beta2 = .25;
	double gamma2 = .4;

	//////第一步：确定下一个时刻的搜索窗口
	//计算下面四个确定窗口大小的参数即可
	double minV = max(lim_minV, curV - a * tickGap);
	double maxV = min(lim_maxV, curV + a * tickGap);
	double minW = max(lim_minW, curW - alpha * tickGap);
	double maxW = min(lim_maxW, curW + alpha * tickGap);

	//////第二步：根据先扫雷达数据计算当前障碍物所在坐标（绝对坐标）
	//障碍物点在地图上的x、y绝对坐标
	double map_x[numR];
	double map_y[numR];
	for (int i = 0; i < numR; i++) {
		if (rho[i] == 0) rho[i] = 600;	//仍然先一律按照最远距离处理
		map_x[i] = curX + rho[i] * cos(curTheta + phi[i] * PI / 180);
		map_y[i] = curY + rho[i] * sin(curTheta + phi[i] * PI / 180);
	}

	//////第三步：更新时间片参数
	double remainDist = sqrt((curX - tarX) * (curX - tarX) + (curY - tarY) * (curY - tarY));
	if (remainDist < nearDist) ticks = 5;

	//第四步：预测轨迹，计算三项权重
	//这里有可能爆栈，非常建议你们把这三个在DWA函数外面new到堆里面！！！
	double heading[maxN];
	double dist[maxN];
	double velocity[maxN];

	//速度空间量程
	double gapV = (maxV - minV) / N1;
	double gapW = (maxW - minW) / N2;
	double realV, realW;
	double xT, yT, thetaT;	//ticks个时间片之后的小车位姿参数

	double distScore, headingScore, velocityScore;	//三项临时评分
	double maxHeading, minHeading;	//三行用来归一化的变量
	double maxDist, minDist;
	double maxVelocity, minVelocity;
	maxHeading = maxDist = maxVelocity = 0.;
	minHeading = minDist = minVelocity = 1000.;

	for (int v = 0; v < N1; v++) {
		realV = minV + v * gapV;
		for (int w = 0; w < N2; w++) {
			realW = minW + w * gapW;

			//预测ticks个时间片之后的位姿
			xT = curX;
			yT = curY;
			thetaT = curTheta;
			for (int k = 0; k < ticks; k++) {
				xT += realV * tickGap * cos(thetaT);
				yT += realV * tickGap * sin(thetaT);
				thetaT += realW * tickGap;
			}

			//计算距离（Dist）评分
			double minDist = 1000;
			double tmpDist;
			for (int i = 0; i < numR; i++) {
				tmpDist = sqrt((xT - map_x[i]) * (xT - map_x[i]) + (yT - map_y[i]) * (yT - map_y[i]));

				if (tmpDist < minDist)
					minDist = tmpDist;
			}
			dist[v * N1 + w] = distScore = minDist;
			//避障逻辑，小车远离目标且靠近障碍物时，直接避障
			if (remainDist > nearDist && distScore < R) {
				dist[v * N1 + w] = 0.;
				heading[v * N1 + w] = 0.;
				velocity[v * N1 + w] = 0.;
				continue;
			}

			//计算航向（Heading）评分
			double omega = curTheta;
			while (true) {		//将omega的取值限定为[-PI,PI]
				if (omega >= -PI && omega <= PI)
					break;
				else {
					if (omega > PI) omega -= 2 * PI;
					if (omega < -PI) omega += 2 * PI;
				}
			}

			int dx = curX - tarX;
			int dy = curY - tarY;

			double theta;
			if (dy == 0) {
				if (dx >= 0) theta = 0;
				if (dx < 0) theta = PI;
			}
			else theta = atan2(dy, dx);

			double beta = theta - omega;
			if (beta < -PI) beta += 2 * PI;
			if (beta > PI) beta = 2 * PI - beta;
			heading[v * N1 + w] = headingScore = PI - abs(beta);

			//计算速度评分
			velocity[v * N1 + w] = velocityScore = abs(realV);

			//更新最值
			if (headingScore > maxHeading) maxHeading = headingScore;
			if (headingScore < minHeading) minHeading = headingScore;
			if (distScore > maxDist) maxDist = distScore;
			if (distScore < minDist) minDist = distScore;
			if (velocityScore > maxVelocity) maxVelocity = velocityScore;
			if (velocityScore < minVelocity) minVelocity = velocityScore;
		}
	}

	//////第五步：将heading、dist和velocity都归一化
	double diff;
	diff = maxHeading - minHeading;
	if (diff == 0) {
		for (int i = 0; i < N1*N2; i++)
			heading[i] -= minHeading;
	}
	else {
		for (int i = 0; i < N1*N2; i++)
			heading[i] = (heading[i] - minHeading) / diff;
	}

	diff = maxDist - minDist;
	if (diff == 0) {
		for (int i = 0; i < N1 * N2; i++)
			dist[i] -= minDist;
	}
	else {
		for (int i = 0; i < N1 * N2; i++)
			dist[i] = (dist[i] - minDist) / diff;
	}

	diff = maxVelocity - minVelocity;
	if (diff == 0) {
		for (int i = 0; i < N1 * N2; i++)
			velocity[i] -= minVelocity;
	}
	else {
		for (int i = 0; i < N1 * N2; i++)
			velocity[i] = (velocity[i] - minVelocity) / diff;
	}

	////第六步：将搜索得到最佳速度并返回
	double score;
	double maxScore = 0.;
	for (int v = 0; v < N1; v++) {
		realV = minV + v * gapV;
		for (int w = 0; w < N2; w++) {
			realW = minW + w * gapW;
			if (findTarget) {
				score = alpha2 * heading[v * N1 + w] + beta2 * dist[v * N1 + w] + gamma2 * velocity[v * N1 + w];
			}
			else score = alpha1 * heading[v * N1 + w] + beta1 * dist[v * N1 + w] + gamma1 * velocity[v * N1 + w];
			if (score > maxScore) {
				maxScore = score;
				DWAres.resV = realV;
				DWAres.resW = realW;
			}
		}
	}

	return DWAres;
}
