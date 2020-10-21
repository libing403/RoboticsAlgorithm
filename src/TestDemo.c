/**
 * @brief			Description: Test Demos for robot algorithm modules.
 * @file:			TestDemo.c
 * @author:			Brian
 * @date:			2019/03/01 12:23
 * Copyright(c) 	2019 Brian. All rights reserved.
 *
 * Contact 			https://blog.csdn.net/Galaxy_Robot
 * @note:     
 * @warning: 		
*/

#include <stdio.h>
#include <math.h>
#include "RobotAlgorithmModule.h"

void test_GrublersFormula()
{
	int N = 8;
	int m = 3;
	int J = 9;
	int f[9] = { 1,1,1,1,1,1,1,1,1 };
	int dof = GrublersFormula(m, N, J, f);
	printf("dof=%d\n", dof);
}

void test_RotInv()
{
	double R[3][3] =
	{
		0,0,1,
		1,0,0,
		0,1,0
	};
	double InvR[3][3] = { {0} };
	RotInv(R, InvR);
	int i;
	printf("R:\n");
	for (i = 0; i < 3; i++)
	{
		printf("%lf %lf %lf\n", R[i][0], R[i][1], R[i][2]);
	}
	printf("InvR:\n");
	for (i=0;i<3;i++)
	{
		printf("%lf %lf %lf\n", InvR[i][0], InvR[i][1], InvR[i][2]);
	}
}

void test_VecToso3()
{
	double omg[3] = {1,2,3};
	double so3[3][3] = { {0} };
	int i;
	VecToso3(omg, so3);
	printf("omg:\n");

	printf("%lf %lf %lf\n", omg[0], omg[1], omg[2]);

	printf("so3:\n");
	for (i = 0; i < 3; i++)
	{
		printf("%lf %lf %lf\n", so3[i][0], so3[i][1], so3[i][2]);
	}
}

void test_so3ToVec()
{
	double so3[3][3] = 
	{
		0, -3, 2,
		3, 0, -1,
		-2, 1, 0
	};
	double omg[3] = { 0 }; 
	int i;
	so3ToVec(so3, omg);

	printf("so3:\n");
	for (i = 0; i < 3; i++)
	{
		printf("%lf %lf %lf\n", so3[i][0], so3[i][1], so3[i][2]);
	}
	printf("omg:\n");
	printf("%lf %lf %lf\n", omg[0], omg[1], omg[2]);

}

void test_AxisAng3()
{
	double expc3[3] = { 1,2,3 };
	double omghat[3] = { 0 };
	double theta = 0;
	AxisAng3(expc3, omghat, &theta);
	printf("expc3:\n");
	printf("%lf %lf %lf\n", expc3[0], expc3[1], expc3[2]);
	printf("omg:\n");
	printf("%lf %lf %lf\n", omghat[0], omghat[1], omghat[2]);
	printf("theta:\n");
	printf("%lf\n", theta);
	return;
}


void test_MatrixExp3()
{
	double so3mat[3][3] = 
	{
		0, -3, 2,
		3, 0, -1,
		-2, 1, 0 
	};
	double R[3][3] = {{ 0 }};
	MatrixExp3(so3mat, R);
	printf("R:\n");
	int i;
	for (i = 0; i < 3; i++)
	{
		printf("%lf %lf %lf\n", R[i][0], R[i][1], R[i][2]);
	}

	return;
}


void test_MatrixLog3()
{
	double theta = 30 * PI / 180;
	double R[3][3] =
	{
		cos(theta), -sin(theta), 0,
		sin(theta), cos(theta), 0,
		0, 0, 1
	};
	double so3Mat[3][3] = { 0 };
	MatrixLog3(R, so3Mat);
	printf("so3Mat:\n");
	int i;
	for (i = 0; i < 3; i++)
	{
		printf("%lf %lf %lf\n", so3Mat[i][0], so3Mat[i][1], so3Mat[i][2]);
	}
	return;
}

void test_RpToTrans()
{
	double R[3][3] =
	{
		1,0,0,
		0,0,-1,
		0,1,0
	};
	double p[3] = { 1,2,5 };
	double T[4][4] = { {0} };
	RpToTrans(R, p, T);
	printf("T:\n");
	int i;
	for (i = 0; i < 4; i++)
	{
		printf("%lf %lf %lf %lf\n", T[i][0], T[i][1], T[i][2], T[i][3]);
	}
	return;
}

void test_TransToRp()
{
	double T[4][4] = {
		1, 0, 0, 0,
		0, 0, -1, 0,
		0, 1, 0, 3,
		0, 0, 0, 1
	};
	double R[3][3] = { {0} };
	double p[3] = { 0 };
	TransToRp(T, R, p);
	printf("R:\n");
	int i;
	for (i = 0; i < 3; i++)
	{
		printf("%lf %lf %lf\n", R[i][0], R[i][1], R[i][2]);
	}
	printf("p:\n");
	printf("%lf %lf %lf\n", p[0],p[1],p[2]);
	return;

}

void test_TransInv()
{
	double T[4][4] = {
		1, 0, 0, 0,
		0, 0, -1, 0,
		0, 1, 0, 3,
		0, 0, 0, 1	
	};
	double InvT[4][4];
	TransInv(T, InvT);
	printf("InvT:\n");
	int i;
	for (i = 0; i < 4; i++)
	{
		printf("%lf %lf %lf %lf\n", InvT[i][0], InvT[i][1], InvT[i][2], InvT[i][3]);
	}
	return;
}

void test_VecTose3()
{
	double V[6] = { 1,2,3,4,5,6 };
	double se3Mat[4][4];
	VecTose3(V, se3Mat);
	printf("se3Mat:\n");
	int i;
	for (i = 0; i < 4; i++)
	{
		printf("%lf %lf %lf %lf\n", se3Mat[i][0], se3Mat[i][1], se3Mat[i][2], se3Mat[i][3]);
	}
	return;
}


void test_se3ToVec()
{
	double V[6] = { 0 };
	double se3Mat[4][4] = {
		0, -3, 2, 4,
		3, 0, -1, 5,
		-2, 1, 0, 6,
		0, 0, 0, 0
	};
	se3ToVec(se3Mat, V);
	int i;
	printf("V:\n");
	for (i=0;i<6;i++)
	{
		printf("%lf\n", V[i]);
	}
	return;
}


void test_Adjoint()
{
	double T[4][4] = {
		1, 0, 0, 0,
		0, 0, -1, 0,
		0, 1, 0, 3,
		0, 0, 0, 1
	};
	double AdT[6][6] = { {0} };
	Adjoint(T, AdT);
	int i;
	int j;
	printf("AdT:\n");
	for (i = 0; i < 6; i++)
	{
		for(j = 0; j < 6; j++)
		{
			printf("%lf  ", AdT[i][j]);
		}
		printf("\n");
	}

	return;
}

void test_AxisAng6()
{
	double expc6[6] = { 0, 0, 0, 0, 0, 0 };
	double S[6];
	double theta;
	AxisAng6(expc6, S, &theta);

	printf("S:\n");
	int i;
	for (i = 0; i < 6; i++)
	{
		printf("%lf\n", S[i]);
	}
	printf("theta:\n");
	printf("%lf\n", theta);
	return;
}


void test_MatrixExp6()
{
	double se3Mat[4][4] =
	{
		0, 0, 0, 0,
		0, 0, -1.570796, 2.3562,
		0, 1.570796, 0, 2.3562,
		0, 0, 0, 0
	};
	double T[4][4];
	MatrixExp6(se3Mat, T);
	int i;
	printf("T:\n");
	for (i = 0; i < 4; i++)
	{
		printf("%lf %lf %lf %lf\n", T[i][0], T[i][1], T[i][2], T[i][3]);
	}
	return;
}


void test_MatrixLog6()
{
	double T[4][4] =
	{
		1, 0, 0, 0,
		0, 0, -1, 0,
		0, 1, 0, 3,
		0, 0, 0, 1
	};
	double se3mat[4][4];
	MatrixLog6(T, se3mat);
	int i;
	printf("se3mat:\n");
	for (i = 0; i < 4; i++)
	{
		printf("%lf %lf %lf %lf\n", se3mat[i][0], se3mat[i][1], se3mat[i][2], se3mat[i][3]);
	}

	return;
}

void test_FKinSpace()
{
	int i;
	double M[4][4] = {
		1.0000 ,   0 ,       0 , 213.0000,
		0,     1.0000,       0,  267.8000,
		0,          0,  1.0000,  478.9500,
		0,         0 ,       0,   1.0000 };
	int JoinNum = 6;
	double Slist[6][6] = {
		0  ,       0,         0,         0,         0,         0,
		0,    1.0000,    1.0000,    1.0000,         0,    1.0000,
		1.0000,    0,         0,         0,    1.0000,         0,
		0, -151.9000, -395.5500, -395.5500,  110.4000, -478.9500,
		0,         0,         0,         0, -213.0000,         0,
		0,         0,         0,  213.0000,         0,  213.0000 };
	double thetalist[6] = {0.1,0.2,0.3,0.4,0.5,PI/4 };
	printf("thetalist:\n");
	for (i = 0; i < 6; i++)
	{
		printf("%lf ", thetalist[i]);
	}
	printf("\n");
	double T[4][4];
	FKinSpace(M, JoinNum, (double *)Slist, thetalist, T);

	printf("T:\n");
	for (i = 0; i < 4; i++)
	{
		printf("%lf %lf %lf %lf\n", T[i][0], T[i][1], T[i][2], T[i][3]);
	}
	return;
}

void test_FKinBody()
{
	double M[4][4] = {
		-1, 0, 0, 0,
		0, 1, 0, 6,
		0, 0, -1, 2,
		0, 0, 0, 1 };
	int JoinNum = 3;
	//double Blist[3][6] = {
	//	0,	 0,  -1, 2, 0, 0,
	//	0,   0,   0, 0, 1, 0,
	//	0,   0,   1, 0, 0, 0.1 };
	double Blist[6][3] = {
		0,0,0,
		0,0,0,
		-1,0,1,
		2,0,0,
		0,1,0,
		0,0,0.1
	};
	double thetalist[3] = { PI / 2,3, PI };
	double T[4][4];
	FKinBody(M, JoinNum, (double *)Blist, thetalist, T);

	int i;
	printf("T:\n");
	for (i = 0; i < 4; i++)
	{
		printf("%lf %lf %lf %lf\n", T[i][0], T[i][1], T[i][2], T[i][3]);
	}
	return;
}

void test_JacobianBody()
{
	int JointNum = 4;//JointNum must be the actual number of joints.
	int i; int j;
	double Blist[6][4];
	double Bi[6][4] = {
		0, 1, 0,   1,
		0, 0, 1,   0,
		1, 0, 0,   0,
		0 ,2, 0,  0.2,
		0.2,0, 2,  0.3,
		0.2,3, 1, 0.4
	};
	//initial Blist.
	for (i=0;i<6;i++)
	{
		for (j=0;j<JointNum;j++)
		{
			Blist[i][j] = Bi[i][j];
		}
	}
	//initial thetalist.
	double thetalist[4] = { 0.2, 1.1, 0.1, 1.2 };
	double Jb[6][4] = { {0} };
	JacobianBody(JointNum,(double*) Blist, thetalist,(double *)Jb);

	printf("Jb:\n");

	for (i=0;i<6;i++)
	{
		for (j=0;j<JointNum;j++)
		{
			printf("%lf  ", Jb[i][j]);
		}
		printf("\n");
	}
	return ;
}


void test_JacobianSpace()
{
	int JointNum = 4;//JointNum must be the actual number of joints.
	int i; int j;
	double Slist[6][4];
	double Si[6][4] = {
		0, 1, 0,   1,
		0, 0, 1,   0,
		1, 0, 0,   0,
		0 ,2, 0,  0.2,
		0.2,0, 2,  0.3,
		0.2,3, 1, 0.4
	};
	//initial Blist.
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < JointNum; j++)
		{
			Slist[i][j] = Si[i][j];
		}
	}
	//initial thetalist.
	double thetalist[4] = { 0.2, 1.1, 0.1, 1.2 };
	double Js[6][4] = { { 0 } };
	JacobianSpace(JointNum, (double *)Slist, thetalist, (double *)Js);

	printf("Js:\n");

	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < JointNum; j++)
		{
			printf("%lf  ", Js[i][j]);
		}
		printf("\n");
	}

	return;
}

void test_MatrixCopy()
{
	double a[3][4] = {
		1,2,3,4,
		5,6,7,8,
		9,10,11,12
	};
	double b[3][4];
	MatrixCopy((double *)a, 3, 4, (double *)b);
	int i, j;
	for (i=0;i<3;i++)
	{
		for (j=0;j<4;j++)
		{
			printf("%lf\t", b[i][j]);
		}
		printf("\n");
	}


	return;
}

void test_svdcmp()
{
	double a[4][4] = {
		1,2,3,4,
		5,6,7,8,
		9,10,11,12,
		13,14,15,16
	};
	double w[4], v[4][4];
	double tol = 2.2e-15;
	svdcmp((double *)a, 4, 4, tol,w, (double *)v);
	int i, j,k;
	printf("svdcmp,u:\n");
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			printf("%lf   ", a[i][j]);
		}
		printf("\n");
	}
	printf("w:\n");
	for (i = 0; i < 1; i++)
	{
		for (j = 0; j < 4; j++)
		{
			printf("%lf   ", w[j]);
		}
		printf("\n");
	}
	printf("v:\n");
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			printf("%lf   ", v[i][j]);
		}
		printf("\n");
	}
	double vT[4][4];
	//验证分解是否正确
	MatrixT((double *)v, 4, 4, (double *)vT);
	double tmp[4][4];
	for (i=0;i<4;i++)
	{
		for (j=0;j<4;j++)
		{
			tmp[i][j]=0;
			for (k=0;k<4;k++)
			{
				tmp[i][j] = tmp[i][j] +w[k] *a[i][k] * vT[k][j];
			}
		}
	}
	printf("uwvT:\n");
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			printf("%lf   ", tmp[i][j]);
		}
		printf("\n");
	}


	return;
}


void test_MatrixT()
{
	double a[4][3] = {
		1,2,3,
		4,5,6,
		7,8,9,
		10,11,12
	};
	double c[3][4];
	MatrixT((double *)a, 4, 3, (double *)c);
	int i, j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 4; j++)
		{
			printf("%lf  ", c[i][j]);
		}
		printf("\n");
	}

	return;
}
void test_MatrixMult()
{
	double a[4][3] = {
		1,2,3,
		4,5,6,
		7,8,9,
		10,11,12
	};
	double b[3][2] = {
		1,2,
		4,5,
		7,8
	};
	double c[4][2];
	MatrixMult((double *)a, 4, 3, (double *)b, 2, (double *)c);
	int i ,j;
	for (i=0;i<4;i++)
	{
		for (j=0;j<2;j++)
		{
			printf("%lf  ", c[i][j]);
		}
		printf("\n");
	}

	return;
}

void test_MatrixPinv()
{
	double a[4][3] = {
		1,2,3,
		5,6,7,
		9,10,11,
		13,14,15
	};
	double b[3][4];
	double tol = 2.22e-15;
	MatrixPinv((double *)a, 4, 3, tol,(double *)b);
	printf("MatrixPinv(a):\n");
	int i, j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 4; j++)
		{
			printf("%lf  ", b[i][j]);
		}
		printf("\n");
	}

	return;
}


void test_IKinBodyNR()
{

	int JointNum = 2;
	double Blist[6][2] = {
		0,0,
		0,0,
		1,1,
		0,0,
		2,1,
		0,0,
	};
	double M[4][4] =
	{
		1,0,0,2,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1
	};
	double T[4][4] = {
		-0.5,-0.866,0,0.366,
		0.866,-0.5,0,1.366,
		0,0,1,0,
		0,0,0,1
	};
	double thetalist0[2] = { 0,0 };
	double eomg = 0.001;
	double ev = 0.0001;
	double thetalist[2] = { 0 };
	int maxiter = 20;
	int ret=IKinBodyNR(JointNum, (double *)Blist, M, T, thetalist0, eomg, ev, maxiter, thetalist);
	if (ret)
	{
		printf("IKinBody error %d\n", ret);
		return;
	}
	printf("solution thetalist:\n");
	int i;
	for (i=0;i<JointNum;i++)
	{
		printf("%lf  ", thetalist[i]);
	}
	printf("\n");
	return;
}

void test_IKinSpaceNR()
{

	int JointNum = 3;
	double Slist[6][3] = {
		0,0,0,
		0,0,0,
		1,0,-1,
		4,0,-6,
		0,1,0,
		0,0,-0.1
	};
	double M[4][4] =
	{
		-1,0,0,0,
		0,1,0,6,
		0,0,-1,2,
		0,0,0,1
	};
	double T[4][4] = {
		0,1,0,-5,
		1,0,0,4,
		0,0,-1,1.6858,
		0,0,0,1
	};
	double thetalist0[3] = { 1.5,2.5,3 };
	double eomg = 0.01;
	double ev = 0.001;
	double thetalist[3] = { 0 };
	int maxiter = 20;
	int ret = IKinSpaceNR(JointNum, (double *)Slist, M, T, thetalist0, eomg, ev, maxiter, thetalist);
	if (ret)
	{
		printf("IKinSpace error %d\n", ret);
		return;
	}
	printf("solution thetalist:\n");
	int i;
	for (i = 0; i < JointNum; i++)
	{
		printf("%lf  ", thetalist[i]);
	}
	printf("\n");
	return;
}


void test_IKOnUR3()
{
	int JointNum = 6;
	double Slist[6][6] = {
		0 ,        0,         0,         0 ,        0,         0,
		0 ,   1.0000 ,   1.0000 ,   1.0000,         0,    1.0000,
		1.0000,         0 ,        0 ,        0 ,   1.0000,         0,
		0, - 151.9000, - 395.5500, - 395.5500 , 110.4000 ,- 478.9500,
		0 ,        0 ,        0  ,       0 ,- 213.0000,         0,
		0  ,       0 ,        0,  213.0000 ,        0,  213.0000
	};
	double M[4][4] =
	{
		1.0000 ,        0,         0,  213.0000,
		0 ,   1.0000 ,        0,  267.8000,
		0 ,        0 ,   1.0000,  478.9500,
		0 ,        0  ,       0,    1.0000,
	};
	double T[4][4] = {
		1.0000,        0 ,        0,   10.0000,
		0,   0,    1,  375.0000,
		0,  -1 ,   0 , 200.0000,
		0,         0 ,        0 ,   1.0000
	};
	double thetalist0[6] = { 0,0,0,0,0,0 };
	double eomg = 1.0E-4;
	double ev = 1.0E-3;
	double thetalist[6] = { 0 };
	int maxiter = 20;
	int ret = IKinSpaceNR(JointNum, (double *)Slist, M, T, thetalist0, eomg, ev, maxiter, thetalist);
	if (ret)
	{
		printf("IKinSpace error %d\n", ret);
		return;
	}
	printf("solution thetalist for C(单位：弧度):\n");
	int i;
	for (i = 0; i < JointNum; i++)
	{
		printf("%lf, ",thetalist[i]);
	}
	printf("\n\n");
	return;
}

void test_RotToAxisAng()
{
	//绕z轴旋转30度的旋转矩阵.
	double R[3][3] =
	{
		cos(PI / 6.0),-sin(PI / 6.0),0,
		sin(PI / 6.0),cos(PI / 6.0),0,
		0,0,1
	};
	double omg[3];
	double theta;
	RotToAxisAng(R, omg, &theta);
	printf("omg:\n%lf\n%lf\n%lf\n", omg[0], omg[1], omg[2]);
	printf("theta:\n%lf\n", theta);
	return;
}

void test_AxisAngToQuaternion()
{
	double R[3][3] = { 0 };
	//绕z轴旋转30度的欧拉轴和角度.
	double omg[3] = {0,0,1};
	double theta = PI / 6.0;
	double q[4];
	AxisAngToQuaternion(omg, theta, q);
	printf("q:\n%lf\n%lf\n%lf\n%lf\n", q[0], q[1], q[2], q[3]);
	return;
}

void test_QuaternionToRot()
{
	double theta = PI / 6;
	//绕z轴旋转30度的四元数.
	double q[4] = { cos(theta / 2),0 * sin(theta / 2),0 * sin(theta / 2),1.0*sin(theta / 2) };
	double R[3][3];
	QuaternionToRot(q, R);
	int i;
	printf("R:\n");
	for (i=0;i<3;i++)
	{
		printf("%lf %lf %lf\n", R[i][0], R[i][1], R[i][2]);
	}
	return;
}

void test_RotToQuaternion()
{
	//绕z轴旋转30度的旋转矩阵.
	double R[3][3] =
	{
		cos(PI / 6.0),-sin(PI / 6.0),0,
		sin(PI / 6.0),cos(PI / 6.0),0,
		0,0,1
	};
	double q[4];
	RotToQuaternion(R, q);
	int i;
	printf("q:\n");
	for (i=0;i<4;i++)
	{
		printf("%lf\n", q[i]);
	}
	return;
}


void test_InitialOrientInpParam()
{
	double Rs[3][3] =
	{
		cos(PI / 6.0),-sin(PI / 6.0),0,
		sin(PI / 6.0),cos(PI / 6.0),0,
		0,0,1
	};
	double Re[3][3] =
	{
		cos(PI / 3.0),-sin(PI / 3.0),0,
		sin(PI / 3.0),cos(PI / 3.0),0,
		0,0,1
	};
	OrientInpParam p;
	InitialOrientInpParam(Rs, Re, &p);
	int i;
	printf("OrientInpParam Rs:\n");
	for (i = 0; i < 3; i++)
	{
		printf("%lf %lf %lf\n", p.Rs[i][0], p.Rs[i][1], p.Rs[i][2]);
	}
	printf("OrientInpParam Re:\n");
	for (i = 0; i < 3; i++)
	{
		printf("%lf %lf %lf\n", p.Re[i][0], p.Re[i][1], p.Re[i][2]);
	}
	printf("OrientInpParam R:\n");
	for (i = 0; i < 3; i++)
	{
		printf("%lf %lf %lf\n", p.R[i][0], p.R[i][1], p.R[i][2]);
	}
	printf("OrientInpParam omg:\n");
	printf("%lf %lf %lf\n", p.omg[0], p.omg[1], p.omg[2]);

	printf("OrientInpParam theta:\n");
	printf("%lf\n", p.theta);
	printf("OrientInpParam thetai:\n");
	printf("%lf\n", p.thetai);

	printf("OrientInpParam Ri:\n");
	for (i = 0; i < 3; i++)
	{
		printf("%lf %lf %lf\n", p.Ri[i][0], p.Ri[i][1], p.Ri[i][2]);
	}

	return;

}

void test_QuaternionOrientInp()
{
	double Rs[3][3] =
	{
		cos(PI / 6.0),-sin(PI / 6.0),0,
		sin(PI / 6.0),cos(PI / 6.0),0,
		0,0,1
	};
	double Re[3][3] =
	{
		cos(PI / 3.0),-sin(PI / 3.0),0,
		sin(PI / 3.0),cos(PI / 3.0),0,
		0,0,1
	};
	OrientInpParam p;
	double Ri[3][3];
	MatrixCopy((double *)Rs, 3, 3, (double *)Ri);
	InitialOrientInpParam(Rs, Re, &p);
	double dtheta = 0.01;
	int k = 0;
	printf("Orientation Ri:k=%d\n",k);
	int i;
	for (i = 0; i < 3; i++)
	{
		printf("%lf %lf %lf\n", Ri[i][0], Ri[i][1], Ri[i][2]);
	}
	//Orientation interpolation
	while (p.InpFlag !=3 && p.InpFlag!=0)
	{
		k++;
		QuaternionOrientInp(&p, dtheta, Ri);
		printf("Orientation Ri:k=%d\n",k);
		for (i = 0; i < 3; i++)
		{
			printf("%lf %lf %lf\n", Ri[i][0], Ri[i][1], Ri[i][2]);
		}
	}
	return;
}

void test_InitialLinePathParam()
{
	double p1[3] = { 0,0,0 };
	double p2[3] = { 1,2,3 };
	double dL = 0.05;
	LineInpParam p;
	InitialLinePathParam( p1, p2, &p);
	printf("LinePathParam p1;\n");
	printf("%lf\n%lf\n%lf\n", p.p1[0], p.p1[1], p.p1[2]);
	printf("LinePathParam p2;\n");
	printf("%lf\n%lf\n%lf\n", p.p2[0], p.p2[1], p.p2[2]);
	printf("LinePathParam L;\n%lf\n",p.L);
	printf("LinePathParam Li;\n%lf\n", p.Li);
	printf("LinePathParam pi;\n");
	printf("%lf\n%lf\n%lf\n", p.pi[0], p.pi[1], p.pi[2]);
	printf("LinePathParam InpFlag;\n%d\n",p.InpFlag);

	return;
}


void test_LinePathInp()
{
	double p1[3] = { 0,0,0 };
	double p2[3] = { 1,2,3 };
	LineInpParam p;
	double pi[3] = {p1[0],p1[2],p1[2]};
	InitialLinePathParam( p1, p2, &p);
	double dL = 0.05;
	int k = 0;
	printf("LinePathInp pi: k=%d,%lf %lf %lf\n", k,pi[0], pi[1], pi[2]);
	while (p.InpFlag !=3 && p.InpFlag != 0)
	{
		k++;
		LinePathInp(&p, dL, pi);
		printf("LinePathInp pi: k=%d,%lf %lf %lf\n",k, pi[0], pi[1], pi[2]);
	}
	return;
}

void test_LinePOInp()
{
	//double p1[6] = { 213.0,267.8,478.95,0,0,0 };
	//double p2[6] = { 10,425,200, -PI / 2,0 ,0 };
	//double p1[6] = { 10,425,200, -PI / 2,0 ,0 };
	//double p2[6] = { -10,525,200,-PI / 4,0,-PI / 6 };
	double p1[6] = { 10,425,200, -PI / 2,0 ,0 };
	double p2[6] = { 10,425,200, -PI *3/ 4,0 ,PI / 2 };
	double Ti[4][4];
	double dL = 1;
	FILE *fp1;
	int ret = fopen_s(&fp1, "LineTrajactory.txt","w");
	if (ret)
	{
		printf("fopen_s error %d\n", ret);
	}
	LinePOParam pt;
	InitialLinePOInpParam(p1, p2, &pt);
	//double dtheta =pt.Orient.theta/(pt.Line.L / dL);
	double dtheta = PI / 100;
	int JointNum = 6;
	double Slist[6][6] = {
		0 ,        0,         0,         0 ,        0,         0,
		0 ,   1.0000 ,   1.0000 ,   1.0000,         0,    1.0000,
		1.0000,         0 ,        0 ,        0 ,   1.0000,         0,
		0, -151.9000, -395.5500, -395.5500 , 110.4000 ,-478.9500,
		0 ,        0 ,        0  ,       0 ,-213.0000,         0,
		0  ,       0 ,        0,  213.0000 ,        0,  213.0000
	};
	double M[4][4] =
	{
		1.0000 ,        0,         0,  213.0000,
		0 ,   1.0000 ,        0,  267.8000,
		0 ,        0 ,   1.0000,  478.9500,
		0 ,        0  ,       0,    1.0000,
	};
	//double thetalist0[6] = { 0 };
	double thetalist0[6] = { 1.284569 ,0.488521, - 0.443200, 1.525477, - 1.570797, - 0.286227 };

	double thetalist[6];
	double eomg = 0.001;
	double ev = 0.01;
	while (pt.InpFlag!=3)
	{
		LinePOInp(&pt, dL, dtheta, Ti);
		IKinSpaceNR(JointNum,(double *)Slist, M, Ti, thetalist0,eomg,ev,10,thetalist);
		//MatrixCopy((double *)Ti, 4, 4, (double *)M);
		MatrixCopy(thetalist, 6, 1, thetalist0);
		fprintf(fp1, "%lf %lf %lf %lf %lf %lf\n", thetalist[0], thetalist[1], thetalist[2], thetalist[3], thetalist[4], thetalist[5]);
	}
	fclose(fp1);
	return;
}