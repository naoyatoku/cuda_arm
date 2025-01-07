#ifndef	__JACOBI__
#define	__JACOBI__
#include "units.h"

void set_jacobi_parameter(double l1,double l2 , double l3);

//ヤコビ行列
Matrix3d jacobian(double theta1, double theta2, double theta3);
bool inv_jacobi(double th1,double th2 ,double th3 , Matrix3d &J_inv);


#endif