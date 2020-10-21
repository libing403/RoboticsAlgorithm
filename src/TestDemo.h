/**
 * @brief			Description: 
 * @file:			TestDemo.h
 * @author:			Brian
 * @date:			2019/03/01 12:23
 * Copyright(c) 	2019 Brian. All rights reserved.
 *
 * Contact 			https://blog.csdn.net/Galaxy_Robot
 * @note:     
 * @warning: 		
*/

#ifndef TESTDEMO_H_
#define TESTDEMO_H_
#ifdef __cplusplus
extern "C" {
#endif

	void test_GrublersFormula();
	void test_RotInv();
	void test_VecToso3();
	void test_so3ToVec();
	void test_AxisAng3();
	void test_MatrixExp3();
	void test_MatrixLog3();
	void test_RpToTrans();
	void test_TransToRp();
	void test_TransInv();
	void test_VecTose3();
	void test_se3ToVec();
	void test_Adjoint();
	void test_AxisAng6();
	void test_MatrixExp6();
	void test_MatrixLog6();
	void test_FKinSpace();
	void test_FKinBody();
	void test_JacobianBody();
	void test_JacobianSpace();
	void test_MatrixCopy();
	void test_svdcmp();
	void test_MatrixT();
	void test_MatrixMult();
	void test_MatrixPinv();
	void test_IKinBodyNR();
	void test_IKinSpaceNR();
	void test_IKOnUR3();
	void test_RotToAxisAng();
	void test_AxisAngToQuaternion();
	void test_QuaternionToRot();
	void test_RotToQuaternion();
	void test_InitialOrientInpParam();
	void test_QuaternionOrientInp();
	void test_InitialLinePathParam();
	void test_LinePathInp();
	void test_LinePOInp();



#ifdef __cplusplus
}
#endif

#endif//TESTDEMO_H_