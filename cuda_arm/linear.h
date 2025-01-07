#pragma once
#include <math.h>
#include "units.h"
#include <algorithm>
#include "CommonModule.h"	//_round()



//device内では派生が使えないのでfunc_basicは派生しない。
//floatのほうが効率がいいのでfloatにする。
class linear
{
	float		_a;		//傾き
	float		_b;		//切片

public:
	linear():_a(1.0),_b(0.0)	{;}
	linear(float a,float b) :_a(a),_b(b)	{;}

	__device__ __host__ vec2 f(float x) const {
		return vec2(x , _a * x +_b);
	}
	//微分、これなんでいるんだっけ
	__device__ __host__	float dy_dx(float x)const {
		return _a;
	}
	__device__ __host__ float dist(const vec2 &start, vec2 &end) const
	{	//一応点のチェックする
		_Assert( _equal<float>(start.y,f(start.x).y) ,"linear::dist() not on line\r\n" );
		_Assert( _equal<float>(end.y,f(end.x).y) ,"linear::dist() not on line\r\n" );
		//これは単なる二点間の距離です。
        vec2 v(end-start);
        return sqrt((v.x*v.x) + (v.y*v.y));	
	}

	//直線の場合は、ｘ軸が進む方向を+,
	enum{
    	DIR_P=1, 
    	DIR_M=-1,
	};
	__device__ __host__ vec2 progress(const vec2 &start, double dist ,int dir) const
    {
       double dx = dir * dist * sqrt(1/(1+_a));
        double dy = _a * dx;
        return start+vec2(dx,dy);
    }
	__device__ __host__
	float a()const{return _a;}
	float b()const{return _b;}

	__device__ __host__ 
	linear & operator=(const linear &o){
		_a = o._a;
		_b = o._b;
		return *this;
	}
};


