/**
 * @brief			Description: Algorithm module of robotics, according to the
 *					book[modern robotics : mechanics, planning, and control].
 * @file:			RobotAlgorithmModule.h
 * @author:			LiBing
 * @date:			2019/03/01 12:50
 * Copyright(c) 	2019 LiBing. All rights reserved.
 *					https://blog.csdn.net/libing403 
 * @note:     
 * @warning: 		
*/
#ifndef RobotALGORITHMMODULE_H_
#define RobotALGORITHMMODULE_H_
#ifdef __cplusplus
extern "C" {
#endif
	#define			DEBUGMODE			1			//打印调试信息

	#define			PI					3.14159265358979323846
	//if the norm of vector is near zero(< 1.0E-6),regard as zero.
	#define			ZERO_VECTOR			1.0E-6	
	#define			ZERO_ELEMENT		1.0E-6		
	#define			ZERO_ANGLE			1.0E-6
	#define			ZERO_DISTANCE		1.0E-6
	#define			ZERO_VALUE			1.0E-6

	/**
	*@brief			Description:use GrublersFormula calculate The number of degrees of
	*				freedom of a mechanism with links and joints.
	*@param[in]		N		the number of links( including the ground ).
	*@param[in]		m		the number of degrees of freedom of a rigid body
	*						(m = 3 for planar mechanisms and m = 6 for spatial mechanisms).
	*@param[in]		J		the number of joints.
	*@param[in]		f		the number of freedoms provided by each joint..
	*@return				The number of degrees of freedom of a mechanism
	*@note:					This formula holds only if all joint constraints are independent.
	*						If they are not independent then the formula provides a lower
	*						bound on the number of degrees of freedom.
	*@waring:
	*/
	int GrublersFormula(int m, int N,  int J, int *f);


	/**
	*@brief			Computes the inverse of the rotation matrix R.
	*@param[in]		R		rotation matrix .
	*@param[out]	InvR	inverse matrix of R.
	*@note					Input R must be a 3x3 rotation matrix.
	*/
	void RotInv(double R[3][3], double InvR[3][3]);

	/**
	*@brief			Description:Returns the 3 x 3 skew-symmetric matrix corresponding to omg.
	*@param[in]		omg		a unit 3-vector.
	*@param[out]	so3Mat	the 3 x 3 skew-symmetric matrix corresponding to omg.
	*@note:
	*@waring:
	*/
	void VecToso3(double omg[3], double so3Mat[3][3]);

	/**
	*@brief			Description:Returns the 3-vector corresponding to the 3×3 skew-symmetric matrix so3mat.
	*@param[in]		so3Mat		the 3×3 skew-symmetric matrix so3mat. 
	*@param[out]	omg			the 3-vector.
	*@note:
	*@waring:
	*/
	void so3ToVec(double so3Mat[3][3], double omg[3]);

	/**
	*@brief			Description:Extracts  the unit rotation axis omghat and the corresponding rotation angle theta from
	*				exponential coordinates omghat*theta for rotation, expc3.
	*@param[in]		expc3
	*@param[out]	omghat		the unit vector of rotation axis .
	*@param[out]	theta		rotation angle.
	*@retval        0			success.
	*@retval		1			failure,normal of expc3 is zero or near zero(<1.0E-6).
	*@note:
	*@waring:
	*/
	void AxisAng3(double expc3[3], double omghat[3], double *theta);

	/**
	*@brief			Description:Computes the rotation matrix R in SO(3) corresponding to
	*				the matrix exponential of so3mat in so(3).
	*@param[in]		so3Mat		[omghat]*theta,matrix exponential of so3mat in so(3).
	*@param[out]	R			rotation matrix R in SO(3).
	*@note:
	*@waring:
	*/
	void MatrixExp3(double so3Mat[3][3], double R[3][3]);

	/**
	*@brief			Description:Computes the matrix logarithm so3mat in so(3) of the rotation matrix R in SO(3).
	*@param[in]		R		the rotation matrix
	*@param[out]	so3Mat	matrix logarithm
	*@note:
	*@waring:
	*/
	void MatrixLog3(double R[3][3], double so3Mat[3][3]);

	/**
	*@brief			Description:Builds the homogeneous transformation matrix T corresponding to a rotation
	*				matrix R in SO(3) and a position vector p in R3.
	*@param[in]		R		a rotation matrix R in SO(3).
	*@param[in]		p		a position vector p in R3.
	*@param[out]	T		the homogeneous transformation matrix T.
	*@note:
	*@waring:
	*/
	void RpToTrans(double R[3][3], double p[3], double T[4][4]);

	/**
	*@brief			Description: Extracts the rotation matrix R and position vector p
	*				from a homogeneous transformation matrix T.
	*@param[in]		T		a homogeneous transformation matrix.
	*@param[out]	R		the rotation matrix.
	*@param[out]	p		position vector.
	*@note:
	*@waring:
	*/
	void TransToRp(double T[4][4], double R[3][3], double p[3]);

	/**
	*@brief			Description:Computes the inverse of a homogeneous transformation matrix T.
	*@param[in]		T		a homogeneous transformation matrix.
	*@param[out]	InvT	the inverse of T.
	*@note:
	*@waring:
	*/
	void TransInv(double T[4][4], double InvT[4][4]);

	/**
	*@brief			Description:Computes the se(3) matrix corresponding to a 6 - vector twist V.
	*@param[in]		V		a 6 - vector twist V.
	*@param[out]	se3Mat	the se(3) matrix.
	*@note:
	*@waring:
	*/
	void VecTose3(double V[6], double se3Mat[4][4]);

	/**
	*@brief			Description:Computes the 6-vector twist corresponding to an se(3) matrix se3mat.
	*@param[in]		se3Mat			an se(3) matrix.
	*@param[out]	V				he 6-vector twist.
	*@note:
	*@waring:
	*/
	void se3ToVec(double se3Mat[4][4], double V[6]);

	/**
	*@brief			Description:Computes the 6 × 6 adjoint representation [AdT ] of the homogeneous transformation matrix T.
	*@param[in]		T		a homogeneous transformation matrix.
	*@param[out]	AdT		the 6 × 6 adjoint representation [AdT ].
	*@note:
	*@waring:
	*/
	void Adjoint(double T[4][4], double AdT[6][6]);

	/**
	*@brief			Description: Extracts the normalized screw axis S and the distance traveled along the screw
	*				theta from the 6-vector of exponential coordinates S*theta.
	*@param[in]		expc6		the 6-vector of exponential coordinates.
	*@param[out]	S			the normalized screw axis.
	*@param[out]	theta		the distance traveled along the screw.
	*@retval		0			success;
	*@retval		1			failure,because the input expc6 is a zero vector
	*@note:
	*@waring:
	*/
	void AxisAng6(double expc6[6], double S[6], double *theta);

	/**
	*@brief			Description: Computes the homogeneous transformation matrix T in SE(3) corresponding to
	*				the matrix exponential of se3mat in se(3).
	*@param[in]		se3mat			the matrix exponential of se3mat in se(3).
	*@param[out]	T				the homogeneous transformation matrix T in SE(3).
	*@note:
	*@waring:
	*/
	void MatrixExp6(double se3Mat[4][4], double T[4][4]);

	/**
	*@brief			Description: Computes the matrix logarithm se3mat in se(3) of
	*				the homogeneous transformation matrix T in SE(3)
	*@param[in]		T			the homogeneous transformation matrix.
	*@param[out]	se3Mat		the matrix logarithm of T.
	*@note:
	*@waring:
	*/
	void MatrixLog6(double T[4][4], double se3Mat[4][4]);

	/**
	*@brief			Description: Computes the end-effector frame given the zero position of the end-effector M,
	*				the list of joint screws Slist expressed in the fixed-space frame, and the list of joint values thetalist.
	*@param[in]		M			the zero position of the end-effector expressed in the fixed-space frame.
	*@param[in]		Joint		Num	the number of joints.
	*@param[in]		Slist		the list of joint screws Slist expressed in the fixed-space frame.
	*							in the format of a matrix with the screw axes as the column.
	*@param[in]		thetalist   the list of joint values.
	*@param[out]	T			the end-effector frame expressed in the fixed-space frame.
	*@return		No return value.
	*@note:			when Slist is a matrix ,make sure that columns number of Slist is equal to JointNum,
	*				rows number of Slist 6 .The function call should be written as
	*				FKinSpace(...,(double *)Slist,...).
	*@waring:
	*/
	void FKinSpace(double M[4][4], int  JointNum, double *Slist, double *thetalist, double T[4][4]);

	/**
	*@brief			Description:Computes the end-effector frame given the zero position of the end-effector M,
	*				the list of joint screws Blist expressed in the end-effector frame, and the list of joint values thetalist.
	*@param[in]		M			the zero position of the end-effector expressed in the end-effector frame.
	*@param[in]		JointNum	the number of joints.
	*@param[in]		Blist		the list of joint screws Slist expressed in the end-effector frame.
	*							in the format of a matrix with the screw axes as the column.
	*@param[in]		thetalist   the list of joint values.
	*@param[out]	T			the end-effector frame expressed in the end-effector frame.
	*@return		No return value.
	*@note:			when Blist is a matrix ,make sure that columns number of Slist is equal to JointNum,
	*				rows number of Slist 6 .The function call should be written as
	*				FKinBody(...,(double *)Blist,...).
	*@waring:
	*/
	void FKinBody(double M[4][4], int  JointNum, double *Blist, double thetalist[], double T[4][4]);

	/**
	*@brief Description: Computes the body Jacobian Jb(theta) in 6×n given a list of joint screws Bi
	expressed in the body frame and a list of joint angles.
	*@param[in]		Blist		The joint screw axes in the end - effector frame when the manipulator is
	*							at the home position, in the format of a matrix with the screw axes as the row.
	*@param[in]		thetalist	A list of joint coordinates.
	*@param[out]	Jb			Body Jacobian matrix.
	*@return        No return value.
	*@note:			 when Blist and Jb are matrixes ,make sure that columns number of Slist or Jb is equal to JointNum,
	*				 rows number of Slist or Jb is 6 .The function call should be written as
	*				 JacobianSpace(JointNum,(double *)Slist,thetalist,(double *)Jb).
	*@waring:
	*/
	void JacobianBody(int JointNum, double *Blist, double *thetalist, double *Jb);


	/**
	*@brief			Description:Computes the space Jacobian Js(theta) in R6 x n given a list of joint screws Si
	*				expressed in the fixed space frame and a list of joint angles.
	*@param[in]		Slist		The joint screw axes expressed in the fixed space frame when the manipulator is
	*							at the home position, in the format of a matrix with the screw axes as the column.
	*@param[in]		thetalist	A list of joint coordinates.
	*@param[out]	Js			Space Jacobian matrix.
	*@return		 No return value.
	*@note:			 when Slist and Js are matrixes ,make sure that columns number of Slist or Js is equal to JointNum,
	*				 rows number of Slist or Js is 6 .The function call should be written as
	*				 JacobianSpace(JointNum,(double *)Slist,thetalist,(double *)Js).
	*@waring:
	*/
	void JacobianSpace(int JointNum, double *Slist, double *thetalist, double *Js);

	/**
	*@brief			Given the matrix a,this routine computes its singular value decomposition.
	*@param[in/out]	a		input matrix a or output matrix u.
	*@param[in]		m		number of rows of matrix a.
	*@param[in]		n		number of columns of matrix a.
	*@param[in]		tol		any	singular values less than a tolerance are treated as zero.
	*@param[out]	w		output vector w.
	*@param[out]	v		output matrix v.
	*@return        No return value.
	*@note:			input a must be a  m rows and n columns matrix,w is n-vector,v is a n rows and n columns matrix. 
	*@waring:
	*/
	int svdcmp(double *a, int m, int n,double tol, double *w, double *v);

	/**
	*@brief			Copy  matrix from a to  b .
	*@param[in]		a		First matrix.
	*@param[in]		m		rows of matrix a.
	*@param[in]		n		columns of matrix a.
	*@param[in]		b		Second matrix.
	*@note:
	*@waring:
	*/
	void MatrixCopy(const double *a, int m, int n, double *b);

	/**
	*@brief			Computes matrix multiply.
	*@param[in]		a		First matrix.
	*@param[in]		m		rows of matrix a.
	*@param[in]		n		columns of matrix b.
	*@param[in]		b		Second matrix.
	*@param[in]		l		columns of matrix b.
	*@param[out]	c		result of a*b.
	*@note:
	*@waring:
	*/
	void MatrixMult(const double *a, int m, int n, const double *b, int l, double *c);

	/**
	*@brief			Computes Matrix transpose.
	*@param[in]		a			a matrix.
	*@param[in]		m			rows of matrix a.
	*@param[in]		n			columns of matrix a.
	*@param[out]	b			transpose of matrix a.
	*@note:
	*@waring:
	*/
	void MatrixT(double *a, int m, int n, double *b);

	/**
	*@brief			Computes Moore-Penrose pseudoinverse by  Singular Value Decomposition (SVD) algorithm.
	*@param[in]		a			an arbitrary order matrix.
	*@param[in]		m			rows.
	*@param[in]		n			columns.
	*@param[in]		tol			any	singular values less than a tolerance are treated as zero.
	*@param[out]	b			Moore-Penrose pseudoinverse of matrix a.
	*@return        0:success,Nonzero:failure.
	*@retval		0			success.
	*@retval		1			failure when use malloc to allocate memory.
	*@note:
	*@waring:
	*/
	int MatrixPinv(const double *a, int m, int n, double tol, double *b);


	/**
	*@brief 		Description:use an iterative Newton-Raphson root-finding method.
	*@param[in]		JointNum		Number of joints.
	*@param[in]		Blist			The joint screw axes in the Body frame when the manipulator
	*								is at the home position, in the format of a matrix with the
	*								screw axes as the columns.
	*@param[in]		M				The home configuration of the end-effector.
	*@param[in]		T				The desired end-effector configuration Tsd.
	*@param[in]		thetalist0		An initial guess of joint angles that are close to satisfying Tsd.
	*@param[in]		eomg			A small positive tolerance on the end-effector orientation error.
	*								The returned joint angles must give an end-effector orientation error less than eomg.
	*@param[in]		ev				A small positive tolerance on the end-effector linear position error. The returned
	*								joint angles must give an end-effector position error less than ev.
	*@param[in]		maxiter			The maximum number of iterations before the algorithm is terminated.
	*@param[out]	thetalist		Joint angles that achieve T within the specified tolerances.
	*@return		0:success,Nonzero:failure
	*@retval		0				success.
	*@retval		1				The maximum number of iterations reach before the algorithm is terminated.
	*@retval		2				failure to allocate memory for Jacobian matrix.
	*@note:
	*@waring:
	*/
	int IKinBodyNR(int JointNum, double *Blist, double M[4][4], double  T[4][4], double *thetalist0, double eomg, double ev, int maxiter, double *thetalist);


	/**
	*@brief 		Description:use an iterative Newton-Raphson root-finding method.
	*@param[in]		JointNum		Number of joints.
	*@param[in]		Slist			The joint screw axes in the space frame when the manipulator
	*								is at the home position, in the format of a matrix with the
	*								screw axes as the columns.
	*@param[in]		M				The home configuration of the end-effector.
	*@param[in]		T				The desired end-effector configuration Tsd.
	*@param[in]		thetalist0		An initial guess of joint angles that are close to satisfying Tsd.
	*@param[in]		eomg			A small positive tolerance on the end-effector orientation error.
	*								The returned joint angles must give an end-effector orientation error less than eomg.
	*@param[in]		ev				A small positive tolerance on the end-effector linear position error. The returned
	*								joint angles must give an end-effector position error less than ev.
	*@param[in]		maxiter			The maximum number of iterations before the algorithm is terminated.
	*@param[out]	thetalist		Joint angles that achieve T within the specified tolerances.
	*@return		0:success,Nonzero:failure
	*@retval		0
	*@note:
	*@waring:
	*/
	int IKinSpaceNR(int JointNum, double *Slist, double M[4][4], double T[4][4], double *thetalist0, double eomg, double ev, int maxiter, double *thetalist);


	/**
	* @brief 			Description: Algorithm for Computing the roll-pitch-yaw angles(rotate around fix reference X,Y,Z axis).
	* @param[in]		R				Rotation matrix.
	* @param[out]		roll			Angles for rotate around fix reference X axis.
	* @param[out]		pitch			Angles for rotate around fix reference Y axis.
	* @param[out]		yaw				Angles for rotate around fix reference Z axis.
	* @return			No return value.
	* @note:
	*@warning:
	*/
	void RotToRPY(double R[3][3], double *roll, double *pitch, double *yaw);


	/**
	* @brief 			Description: Algorithm for Computing the rotation matrix of the roll-pitch-yaw angles.
	* @param[in]		roll			Angles for rotate around fix reference X axis.
	* @param[in]		pitch			Angles for rotate around fix reference Y axis.
	* @param[in]		yaw				Angles for rotate around fix reference Z axis.
	* @param[out]		R				Rotation matrix.
	* @return			No return value.
	* @note:
	*@warning:
	*/
	void RPYToRot(double roll, double pitch, double yaw, double R[3][3]);


	/**
	* @brief 			Description: Algorithm for Computing the ZYX Euler Angles according to rotation matrix.
	* @param[in]		R				Rotation matrix.
	* @param[out]		alpha			Angles for rotation around Z axis.
	* @param[out]		beta			Angles for rotation around Y axis.
	* @param[out]		gamma			Angles for rotation around X axis.
	* @return			No return value.
	* @note:
	* @waring:
	*/
	void RotToZYXEulerAngle(double R[3][3], double *alpha, double *beta, double *gamma);


	/**
	* @brief 			Description: Algorithm for Computing the rotation matrix of ZYX Euler Angles.
	* @param[in]		alpha			Angles for rotation around Z axis.
	* @param[in]		beta			Angles for rotation around Y axis.
	* @param[in]		gamma			Angles for rotation around X axis.
	* @param[out]		R				Rotation matrix.
	* @return			No return value.
	* @note:
	* @waring:
	*/
	void ZYXEulerAngleToRot(double alpha, double beta, double gamma, double R[3][3]);


	/**
	* @brief 			Description: Computes the unit vector of Euler axis and rotation angle corresponding to rotation matrix.
	* @param[in]		R				A rotation matrix.
	* @param[out]		omghat			the unit vector of Euler axis .
	* @param[out]		theta			the rotation angle.
	* @return			No return value.
	* @retval			0
	* @note:			if  theta is zero ,the unit axis is undefined and set it as a zero vector [0;0;0].
	* @waring:
	*/
	void RotToAxisAng(double R[3][3], double omghat[3], double *theta);


	/**
	* @brief 			Description: Computes the unit quaternion corresponding to the Euler axis and rotation angle.
	* @param[in]		omg				Unit vector of Euler axis.
	* @param[in]		theta			Rotation angle.
	* @param[in]		q				The unit quaternion
	* @return			No return value.
	* @note:
	* @warning:
	*/
	void AxisAngToQuaternion(double omg[3], double theta, double q[4]);


	/**
	* @brief 			Description:Computes the unit quaternion corresponding to a rotation matrix.
	* @param[in]		q				Unit quaternion.
	* @param[out]		R				Rotation matrix.
	* @return			No return value.
	* @note:
	* @warning:
	*/
	void QuaternionToRot(double q[4], double R[3][3]);


	/**
	* @brief 			Description: Computes the unit quaternion corresponding to the rotation matrix.
	* @param[in]		R				The rotation matrix.
	* @param[out]		q				The unit quaternion.
	* @return			No return value.
	* @note:
	* @warning:
	*/
	void RotToQuaternion(double R[3][3], double q[4]);


	/**
	* @brief 			Description: structure of LinePath interpolation parameters.
	*/
	typedef struct
	{
		double p1[3];
		double p2[3];
		double L;
		double t[3];
		double pi[3];
		double Li;
		int InpFlag;
	}LineInpParam;


	/**
	* @brief 			Description: structure of arc interpolation parameters.
	*/
	typedef struct  
	{
		double p1[3];
		double p2[3];
		double p3[3];
		double N[3];
		double C[3];
		double theta;
		double R;
	}ArcInpParam;


	/**
	* @brief 			Description: structure of orientation interpolation parameters.
	*/
	typedef struct
	{
		double Rs[3][3];//Start orientation Rotation matrix. 
		double Re[3][3];//End orientation Rotation matrix.
		double R[3][3];	//Matrix rotation from Start orientation to End orientation.
		double omg[3];  //unit vector of Euler axis.
		double theta;	//Total interpolation angle.
		double Ri[3][3];//Current orientation rotation Matrix.
		double thetai;  //Current angle.
		int InpFlag;	//1:finish initial ,2;interpolating ,3;finish interpolation
	}OrientInpParam;


	/**
	* @brief 			Description: structure of line Path and Orientation (PO) interpolation parameters.
	*/
	typedef struct  
	{
		LineInpParam Line;
		OrientInpParam Orient;
		double Ts[4][4];
		double Te[4][4];
		double Ti[4][4];
		int InpFlag;
	}LinePOParam;


	/**
	* @brief 			Description:Computes the parameters of  interpolation between two orientations.
	* @param[in]		Rs				Start posture Rotation matrix.
	* @param[in]		Re				End posture Rotation matrix.
	* @param[in]		Param			structure of orientation interpolation parameters..
	* @return			No return value.
	* @note:
	* @warning:
	*/
	void InitialOrientInpParam(double Rs[3][3], double Re[3][3], OrientInpParam *Param);


	/**
	* @brief 			Description: Computes orientations interpolation.
	* @param[in]		Param			Interpolation parameter structure.
	* @param[out]		dtheta			angle  need to rotate from previous orientation to next orientation in next time step.
	* @return			Ri1				orientations in next time step.
	* @retval			0
	* @note:
	* @warning:
	*/
	void QuaternionOrientInp(OrientInpParam *Param, double dtheta, double Ri1[3][3]);


	/**
	* @brief 			Description:Computes the parameters of line path for interpolation.
	* @param[in]		p1				Coordinates of start point.
	* @param[in]		p2				Coordinates of end point.
	* @param[out]		p				Line path parameters structure.
	* @return			No return value.
	* @note:
	* @warning:
	*/
	void InitialLinePathParam(double p1[3], double p2[3], LineInpParam *p);


	/**
	* @brief 			Description:Computes the line path interpolation coordinates in each interpolation cycle.
	* @param[in]		p				Line path parameters structure.
	* @param[in]		dL				step length in next interpolation cycle.
	* @param[in]		pi1				coordinates in next interpolation cycle.
	* @return			No return value.
	* @note:
	* @warning:
	*/
	void LinePathInp(LineInpParam *p, double dL, double pi1[3]);

	/**
	* @brief 			Description: Computes the parameters of both line path and orientation for interpolation.
	* @param[in]		p1				Start coordinates,including x,y,z coordinates and orientation angles roll-pitch-yaw angles.
	* @param[in]		p2				End coordinates,including x,y,z coordinates and orientation angles roll-pitch-yaw angles.
	* @param[out]		LPO				Parameters of both line path and orientation for interpolation.
	* @return			No return value.
	* @note:
	* @warning:
	*/
	void InitialLinePOInpParam(double p1[6], double p2[6], LinePOParam *LPO);


	/**
	* @brief 			Description:Computes the line path interpolation coordinates and orientation in each interpolation cycle.
	* @param[out]		LPO				Line path and orientation parameters structure.
	* @param[in]		dL				Line path interpolation step length.
	* @param[out]		dtheta			angle interpolation step length for Orientation interpolation.
	* @return			No return value.
	* @note:
	* @warning:
	*/
	void LinePOInp(LinePOParam *LPO, double dL, double dtheta, double Ti[4][4]);






#ifdef __cplusplus
}
#endif

#endif//RobotALGORITHMMODULE_H_