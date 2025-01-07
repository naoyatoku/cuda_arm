#include <iostream>
#include <cmath>
#include "jacobi.h"


static	double	_l1=290.0;
static	double	_l2=290.0; 
static	double	_l3=135.0;

//このモジュールで使う間接のアーム長さを登録出来るようにしておきます。
void set_jacobi_parameter(double l1,double l2 , double l3)
{	
	_l1=l1;
	_l2=l2;
	_l3=l3;
}
//
Matrix3d jacobian(double th1, double th2, double th3)
{
	Eigen::Matrix3d J;

	J <<	-_l1*sin(th1)	-	_l2 * sin(th1 + th2) - _l3 * sin(th1 + th2 + th3)	, 	-_l2*sin(th1 + th2) - _l3 * sin(th1 + th2 + th3)	, -_l3 * sin(th1 + th2 + th3),
			_l1*cos(th1)	+	_l2 * cos(th1 + th2) + _l3 * cos(th1 + th2 + th3)	,	_l2*cos(th1 + th2) + _l3 * cos(th1 + th2 + th3)	,	_l3*cos(th1 + th2 + th3),
			1, 1, 1;
	return J;
}

//
bool inv_jacobi(double th1,double th2 ,double th3 , Matrix3d &J_inv) 
{
	// Calculate the Jacobian
	Matrix3d J = jacobian(th1, th2, th3);

	// Check if the determinant is non-zero
	{
		double det = J.determinant();
		if (std::abs(det) < 1e-6) {
			std::cerr << "The determinant is zero. The inverse Jacobian does not exist." << std::endl;
			return false;
		}
	}

	// Calculate the inverse Jacobian
	J_inv = J.inverse();

	// Print the Jacobian and its inverse
	std::cout << "Jacobian:\n" << J << std::endl;
	std::cout << "Inverse Jacobian:\n" << J_inv << std::endl;

	return true;
}

