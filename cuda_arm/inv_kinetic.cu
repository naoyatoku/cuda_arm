#include	"inv_kinetic.h"
#include	<math.h>
#include	<stdio.h>
//------------------------------------------------------------------------------------------------------------
//		defineq
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//		global
//------------------------------------------------------------------------------------------------------------
//計算の都合上、doubleでアーム長さを持ちます。
//以下はデフォルトです。要らないかも

//__host__ __device__ void set_kinetic_parameter(double l1,double l2 , double l3);


//旧としてここにおいておきます。

/*
struct _3link_calc
{
	float _l1, _l2, _l3;

	__device__ __host__ float	_a(float y, float rad)
	{
		float a = y - (_l3 * sin(rad));
		return a;
	}
	__device__ __host__ float	_b(float x, float rad)
	{
		float	b = x - (_l3 * cos(rad));
		return b;
	}
	__device__ __host__ float	_c(float x, float y, float rad)
	{
		float a = _a(y, rad);
		float b = _b(x, rad);
		float c = ((a * a) + (b * b) + (_l1 * _l1) - (_l2 * _l2)) / (2 * _l1);
		return c;
	}
	__device__ __host__ float		_d(float x, float y, float rad)
	{
		float a = _a(y, rad);
		float b = _b(x, rad);
		float d = ((a * a) + (b * b) - (_l1 * _l1) + (_l2 * _l2)) / (2 * _l1);
		return d;
	}
public:
	//このモジュールで使う間接のアーム長さを登録出来るようにしておきます。
	__device__ __host__ void set_kinetic_parameter(float l1, float l2, float l3)
	{
		_l1 = l1;
		_l2 = l2;
		_l3 = l3;
	}


		//3要素の配列： 

	//	__device__ __host__ vector<Vector3d> calc_inv_kinetic(double x, double y, double rad)
	__device__ __host__ mlti< _Vector3d<float>, 2>  calc_inv_kinetic(float x, float y, float rad)
	{
		//	vector<Vector3d> r(2);
		mlti< _Vector3d<float>, 2> _r;

		//	Vector3d	r[2];			//解が二通り、関節が3つです。
		float a = _a(y, rad);
		float b = _b(x, rad);
		float c = _c(x, y, rad);
		float d = _d(x, y, rad);
		float phy = atan2(a, b);
		//		printf("a:%lf , b:%lf , c:%lf , d:%lf , phy:%lf ,\r\n", a, b, c, d, phy);
#if 1
		//---------------------------
		//	θ1
		//---------------------------
		float	_sub_1 = atan2(sqrt((a * a) + (b * b) - (c * c)), c);
		//		r[0](0) = phy - _sub_1;
		//		r[1](0) = phy + _sub_1;		//これは必要なければ消す。
				//{for(int i=0;i<2;++i){	printf("q1[%d]  = %lf \r\n"  , i , r.q1[i] );}}
		_r[0](0) = phy - _sub_1;
		_r[1](0) = phy + _sub_1;
		//---------------------------
		//	θ2
		//---------------------------
		float _sub_2 = atan2(sqrt((a * a) + (b * b) - (d * d)), d);
		//		r[0](1) = phy - r[0][0] + _sub_2;
		//		r[1](1) = phy - r[1][0] - _sub_2;
		_r[0](1) = phy - _r[0][0] + _sub_2;
		_r[1](1) = phy - _r[1][0] - _sub_2;

		//{for(int i=0;i<2;++i){	printf("q2[%d]  = %lf \r\n"  , i , r.q2[i] );}}
			//---------------------------
			//	θ3
			//---------------------------
//		r[0](2) = rad - (r[0][0] + r[0][1]);
//		r[1](2) = rad - (r[1][0] + r[1][1]);
		_r[0](2) = rad - (_r[0][0] + _r[0][1]);
		_r[1](2) = rad - (_r[1][0] + _r[1][1]);

		//とりあえず、2πの中に数字を納めます。
		for (int k = 0; k < 2; ++k) {
			for (int i = 0; i < 3; ++i) {
				_r[k](i) = fmodf(_r[k](i), (2.0 * PI));		//まず2πの中に納めます。
				//				_gpuAssert( ! std::isnan(_r[k](i)) , "calc_inv_kinetick is nan" );
			}
		}

		//printf("q3 = %lf \r\n" , r.q3[0] );cd
		//{for(int i=0;i<2;++i){	printf("q3[%d]  = %lf \r\n"  , i , r.q3[i] );}}
		//	_dump_theta(r);
#endif
		return _r;
	}

	__device__ __host__  mlti< _Vector3d<float>, 2> calc_inv_kinetic(_cood p) {
		// calc_inv_kinetic(_cood p){
		return calc_inv_kinetic(p.x, p.y, p.rad);
	}
	//------------------------------------------------------------------------------------------------------------
	//	3 link 平面アームの  順運同学
	//-----------------------------------------------------------------------------------------------------------
	__device__ __host__ _cood calc_fwd_kinetic(float rad[3])
	{
		//3つのアームのベクトル足し算です。
		_cood p;
		p.x = (_l1 * cos(rad[0])) + (_l2 * cos(rad[0] + rad[1])) + (_l3 * cos(rad[0] + rad[1] + rad[2]));
		p.y = (_l1 * sin(rad[0])) + (_l2 * sin(rad[0] + rad[1])) + (_l3 * sin(rad[0] + rad[1] + rad[2]));
		p.rad = rad[0] + rad[1] + rad[2];
		return p;
	}

	//ヤコビもここで
	//toku とりあえずcudaではEigenライブラリが使えないので、ヤコビ関連は保留、
	__host__  Matrix3d jacobian(float th1, float th2, float th3)
	{
		Eigen::Matrix3d J;

		J << -_l1 * sin(th1) - _l2 * sin(th1 + th2) - _l3 * sin(th1 + th2 + th3), -_l2 * sin(th1 + th2) - _l3 * sin(th1 + th2 + th3), -_l3 * sin(th1 + th2 + th3),
			_l1* cos(th1) + _l2 * cos(th1 + th2) + _l3 * cos(th1 + th2 + th3), _l2* cos(th1 + th2) + _l3 * cos(th1 + th2 + th3), _l3* cos(th1 + th2 + th3),
			1, 1, 1;
		return J;
	}

	__host__ bool inv_jacobi(float th1, float th2, float th3, Matrix3d& J_inv)
	{
		// Calculate the Jacobian
		Matrix3d J = jacobian(th1, th2, th3);

		// Check if the determinant is non-zero
		{
			float det = J.determinant();
			//			if (_gpuAssert(std::abs(det) > 1e-6, "The determinant is zero. The inverse Jacobian does not exist.") != true) {				return false;			}
			if (std::abs(det) < 1e-6) {
				printf("The determinant is zero. The inverse Jacobian does not exist.");
				return false;
			}
		}

		// Calculate the inverse Jacobian
		J_inv = J.inverse();
		return true;
	}
};		//class 
*/