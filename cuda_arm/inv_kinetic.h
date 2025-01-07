#ifndef	__INV_KINETIC__
#define	__INV_KINETIC__
#include "units.h" 
#include "CommonModule.h"
#include <math.h>
#include <cmath>	//isnan
//--------------------------------------------------------------------------------
//	
//--------------------------------------------------------------------------------
//__device__ __host__ vector<Vector3d> calc_inv_kinetic(_cood p);
//__device__ __host__ _cood	calc_fwd_kinetic(Vector3d th /* rdouble rad[3]*/);

//
/*	DFD6755 	DFD6561 	DFD6342
�A�[��1 	290 	220 	140
�A�[��2 	290 	220 	140
�A�[��3 	135 	140 	150
*/


//�e���v���[�g�łŐV�����v�Z���@�ō��܂��B
template<typename T>
struct _3link_calc_T
{
	T _l1, _l2, _l3;

	//A:L2��[���猴�_��ʂ钼�O�����O�p�`�̉��̒����ł��B�iL3�̍��{�̈ʒu���v�Z�j�����v�Z�Ƌt�Ȃ̂Œ���
	__device__ __host__ T	_A(T x, T rad)
	{
		T	A = x - (_l3 * cos(rad));
		return A;
	}
	//B:A�Ɠ��l�̈ʒu�� �c�̒���
	__device__ __host__ T	_B(T y, T rad)
	{
		T B = y - (_l3 * sin(rad));
		return B;
	}
public:
	__host__ _3link_calc_T() : _l1(1.0),_l2(1.0),_l3(3.0){}
	__host__ _3link_calc_T(T l1 ,T l2 ,T l3) : _l1(l1) , _l2(l2) , _l3(l3){}
		//���̃��W���[���Ŏg���Ԑڂ̃A�[��������o�^�o����悤�ɂ��Ă����܂��B
	__device__ __host__ void set_kinetic_parameter(T l1, T l2, T l3)
	{
		_l1 = l1;
		_l2 = l2;
		_l3 = l3;
	}
	__device__ __host__ mlti< _Vector3d<T>, 2>  calc_inv_kinetic(T x,T y,T rad)
	{
		mlti< _Vector3d<T> , 2> _r;	//���ł�
		
		T A		= _A(x, rad);	//x:cos
		T B		= _B(y, rad);	//y:sin
		T gamma = atan2(B, A);	//atan(float y,float x)
//		printf("a:%lf , b:%lf ,gamma:%lf ,\r\n", A, B, gamma);
		//---------------------------
		//	��1
		//---------------------------
		{
			T	v		= ((A*A)+(B*B)+(_l1*_l1)-(_l2*_l2))/(2*_l1*sqrt((A*A)+(B*B)));
			T	_cos	=	acos(v);
			_r[0](0) = gamma + _cos;
			_r[1](0) = gamma - _cos;
		}
		//---------------------------
		//	��2
		//---------------------------
		{
			for(int i=0; i<2 ; ++i) { 
				T y = B - (_l1 * sin(_r[i](0)));
				T x = A - (_l1 * cos(_r[i](0)));
				_r[i](1) = (-1 * _r[i](0)) + atan2(y,x);
			}
		}
		//---------------------------
		//	��3
		//---------------------------
		{
			_r[0](2) = rad - (_r[0][0] + _r[0][1]);
			_r[1](2) = rad - (_r[1][0] + _r[1][1]);
		}

		//�Ƃ肠�����A2�΂̒��ɐ�����[�߂܂��B
		for (int k = 0; k < 2; ++k) {
			for (int i = 0; i < 3; ++i) {
				_r[k](i) = fmodf(_r[k](i), (2.0 * PI));		//�܂�2�΂̒��ɔ[�߂܂��B
//				_gpuAssert( ! std::isnan(_r[k](i)) , "calc_inv_kinetick is nan" );
			}
		}

		//printf("q3 = %lf \r\n" , r.q3[0] );cd
		//{for(int i=0;i<2;++i){	printf("q3[%d]  = %lf \r\n"  , i , r.q3[i] );}}
		//	_dump_theta(r);
		return _r;
	}

	__device__ __host__  mlti< _Vector3d<T>, 2> calc_inv_kinetic(_cood p) {
		// calc_inv_kinetic(_cood p){
		return calc_inv_kinetic(p.x, p.y, p.rad);
	}
	//------------------------------------------------------------------------------------------------------------
	//	3 link ���ʃA�[����  ���^���w
	//-----------------------------------------------------------------------------------------------------------
	__device__ __host__ _cood calc_fwd_kinetic(float rad[3])
	{
		//3�̃A�[���̃x�N�g�������Z�ł��B
		_cood p;
		p.x = (_l1 * cos(rad[0])) + (_l2 * cos(rad[0]+rad[1])) + (_l3 * cos(rad[0]+rad[1]+rad[2]));
		p.y = (_l1 * sin(rad[0])) + (_l2 * sin(rad[0]+rad[1])) + (_l3 * sin(rad[0]+rad[1]+rad[2]));
		p.rad = rad[0] + rad[1] + rad[2];
		return p;	
	}
	//���R�r��������
	//toku �Ƃ肠����cuda�ł�Eigen���C�u�������g���Ȃ��̂ŁA���R�r�֘A�͕ۗ��A
	__host__  Matrix3d jacobian(T th1,T th2,T th3)
	{
		Eigen::Matrix3d J;

		J << -_l1 * sin(th1) - _l2 * sin(th1 + th2) - _l3 * sin(th1 + th2 + th3), -_l2 * sin(th1 + th2) - _l3 * sin(th1 + th2 + th3), -_l3 * sin(th1 + th2 + th3),
			_l1* cos(th1) + _l2 * cos(th1 + th2) + _l3 * cos(th1 + th2 + th3), _l2* cos(th1 + th2) + _l3 * cos(th1 + th2 + th3), _l3* cos(th1 + th2 + th3),
			1, 1, 1;
		return J;
	}

	__host__ bool inv_jacobi(T th1,T th2,T th3, Matrix3d& J_inv)
	{
		// Calculate the Jacobian
		Matrix3d J = jacobian(th1, th2, th3);

		// Check if the determinant is non-zero
		{
			T det = J.determinant();
//			if (_gpuAssert(std::abs(det) > 1e-6, "The determinant is zero. The inverse Jacobian does not exist.") != true) {				return false;			}
			if (std::abs(det) < 1e-6) {
				printf("The determinant is zero. The inverse Jacobian does not exist." );
				return false;
			}
		}

		// Calculate the inverse Jacobian
		J_inv = J.inverse();
		return true;
	}
};		//class 


#endif	//__INV_KINETIC__
