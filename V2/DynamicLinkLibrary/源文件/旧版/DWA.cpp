#include <iostream>
#include <cmath>
#include<algorithm>
#include<cstdio>
#include"DWA.h"
using namespace std;

//֮ǰ�����һЩ�ṹ�⣬�ӿں����������õ���д������ֻ��˵��

struct SpeedPair {
	double v;		//���ٶ�
	double w;		//���ٶ�
};

struct DWindow {
	double maxV;	//������ٶ�
	double maxW;	//�����ٶ�
	double minV;	//��С���ٶ�
	double minW;	//��С���ٶ�
};

//�������������Ϳ��Ը������ǵ������޸�
//void DWA_port(double curV, double curW, double curX, double curY, double curTheta, double tarX, double tarY, int obstacle[], bool findTarget);
/*
* ������
* double curV��С������һʱ�̵����ٶ�
* double curW��С������һʱ�̵Ľ��ٶ�
* double curX��С����ǰ��X���꣬��Ҫ���ݳ�ʼ����һ��һ�����Ƴ���
* double curY��С����ǰ��Y���꣬��Ҫ���ݳ�ʼ����һ��һ�����Ƴ���
* int obstacle[]����ɨ�״����飬Ĭ�ϳ���360����Ҫɸ��Ϊ0�ĵ�
* double& nextV����һʱ�̵����ٶȣ����ã�
* double& nextW����һʱ�̵Ľ��ٶȣ����ã�
*/

//���Ƶؼ����������ת�ǵĹ�ʽ��Լ����һ��ʱ��Ƭ(tickGap)0.2s
//curX = curX + resV * 0.2 * cos(curTheta);
//curY = curY + resW * 0.2 * sin(curTheta);
//curTheta = curTheta + curW * 0.2;
//һ��Ҫ����curX��curY����curTheta

int main() {
	return 0;
}

DWAresult DWA_port(double curV, double curW, double curX, double curY, double curTheta, double tarX, double tarY, int obstacle[], bool findTarget) {
	const double PI = 3.1415926;
	//4�������ٶ�
	const double lim_minV = 0;
	const double lim_maxV = 50;
	const double lim_minW = -PI;
	const double lim_maxW = PI;

	//2�����ٶȣ���λ��Ϊ���أ��������������Ҫ�Լ�����
	const double a = 20;
	const double alpha = PI;

	//ʱ�����
	const double tickGap = 0.2;	//ʱ��Ƭ���0.2s
	int ticks = 10;		//ǰ��Ԥ���ʱ��Ƭ��Ŀ

						//�ٶȿռ���ɢֵ�ܸ���
	const int N1 = 50;	//�ܹ�N1����ɢ���ٶȣ���Χ��minV��maxV
	const int N2 = 50;	//�ܹ�N2����ɢ���ٶȣ���Χ��minW��maxW
	const int maxN = 3000;	//�涨N1*N2<maxN

							//��ֵ��������λ��Ϊ���أ���Ҫ�����Լ�������
	double nearDist = 80;	//С����Ŀ��С��nearDistʱ����ǰ��Ԥ��ʱ��Ƭ��������
	double R = 29;			//�ٶ�С����ռ����Ϊ29

							//Ȩ�ز���
	double alpha1 = .237;
	double beta1 = .5;
	double gamma1 = .3;

	double alpha2 = .48;
	double beta2 = .25;
	double gamma2 = .4;

	//////��һ����ȷ����һ��ʱ�̵���������
	//���������ĸ�ȷ�����ڴ�С�Ĳ�������
	double minV = max(lim_minV, curV - a * tickGap);
	double maxV = min(lim_maxV, curV + a * tickGap);
	double minW = max(lim_minW, curW - alpha * tickGap);
	double maxW = min(lim_maxW, curW + alpha * tickGap);

	//////�ڶ�����������ɨ�״����ݼ��㵱ǰ�ϰ����������꣨�������꣩
	//�ϰ�����ڵ�ͼ�ϵ�x��y��������
	double map_x[360];
	double map_y[360];
	for (int i = 0; i < 360; i++) {
		map_x[i] = curX + obstacle[i] * cos(curTheta + i * PI / 180);
		map_y[i] = curY + obstacle[i] * sin(curTheta + i * PI / 180);
	}

	//////������������ʱ��Ƭ����
	double remainDist = sqrt((curX - tarX) * (curX - tarX) + (curY - tarY) * (curY - tarY));
	if (remainDist < nearDist) ticks = 5;

	//���Ĳ���Ԥ��켣����������Ȩ��
	//�����п��ܱ�ջ���ǳ��������ǰ���������DWA��������new�������棡����
	double heading[maxN];
	double dist[maxN];
	double velocity[maxN];

	//�ٶȿռ�����
	double gapV = (maxV - minV) / N1;
	double gapW = (maxW - minW) / N2;
	double realV, realW;
	double xT, yT, thetaT;	//ticks��ʱ��Ƭ֮���С��λ�˲���

	double distScore, headingScore, velocityScore;	//������ʱ����
	double maxHeading, minHeading;	//����������һ���ı���
	double maxDist, minDist;
	double maxVelocity, minVelocity;
	maxHeading = maxDist = maxVelocity = 0.;
	minHeading = minDist = minVelocity = 1000.;

	for (int v = 0; v < N1; v++) {
		realV = minV + v * gapV;
		for (int w = 0; w < N2; w++) {
			realW = minW + w * gapW;

			//Ԥ��ticks��ʱ��Ƭ֮���λ��
			xT = curX;
			yT = curY;
			thetaT = curTheta;
			for (int k = 0; k < ticks; k++) {
				xT += realV * tickGap * cos(thetaT);
				yT += realV * tickGap * sin(thetaT);
				thetaT += realW * tickGap;
			}

			//������루Dist������
			double minDist = 1000;
			double tmpDist;
			for (int i = 0; i < 360; i++) {
				tmpDist = sqrt((xT - map_x[i]) * (xT - map_x[i]) + (yT - map_y[i]) * (yT - map_y[i]));

				if (tmpDist < minDist)
					minDist = tmpDist;
			}
			dist[v * N1 + w] = distScore = minDist;
			//�����߼���С��Զ��Ŀ���ҿ����ϰ���ʱ��ֱ�ӱ���
			if (remainDist > nearDist && distScore < R) {
				dist[v * N1 + w] = 0.;
				heading[v * N1 + w] = 0.;
				velocity[v * N1 + w] = 0.;
				continue;
			}

			//���㺽��Heading������
			double omega = curTheta;
			while (true) {		//��omega��ȡֵ�޶�Ϊ[-PI,PI]
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

			//�����ٶ�����
			velocity[v * N1 + w] = velocityScore = abs(realV);

			//������ֵ
			if (headingScore > maxHeading) maxHeading = headingScore;
			if (headingScore < minHeading) minHeading = headingScore;
			if (distScore > maxDist) maxDist = distScore;
			if (distScore < minDist) minDist = distScore;
			if (velocityScore > maxVelocity) maxVelocity = velocityScore;
			if (velocityScore < minVelocity) minVelocity = velocityScore;
		}
	}

	//////���岽����heading��dist��velocity����һ��
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

	////���������������õ�����ٶȲ�����
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
