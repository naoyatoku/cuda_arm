
#pragma once
#include "units.h"


static int __109debug;
#if 0
class func_basic	//なんかabstruct的な感じの
{
protected:

public:
	func_basic()					{ ; }
	//直交座標上での計算結果を返す
	__device__ __host__ vector<vec2>  f(double x)	const {		_Assert(0, "f() is not avaliable(in this object)");		return vector<vec2>{vec2()};	}

	//極座標上での計算結果を返す。これは、円の場合、暗黙的に
//	virtual vector<rvec2> rf(double rad)	const {		_Assert(0, "rf() is not avaliable(in this object)");		return vector<rvec2>{rvec2()};	}
	//指示された点が
	__device__ __host__ bool in(const vec2 &a)	const;
	//指示された始点から終点までに進むのに、どちらの方向にいけばいいかを考える。
/*	__device__ __host__ int	dir(const vec2& start, vec2& end) {
		if (in(start) != true)		{			return 0;		}
		if (in(end) != true)		{			return 0;		}
		return _dir(start, end);
	}
	__device__ __host__ int _dir(const vec2 &start, vec2 &end)const {return };	//これは派生先で実装する。
*/
	//微分の値を返す
//	__device__ __host__ double dy_dx(const vec2 a)const=0;
	//距離を返す
	__device__ __host__ double dist(const vec2 &start, vec2 &end)const {
		if (in(start) != true) { return 0; }
		if (in(end) != true) { return 0; }
		return _dist(start, end);
	}
	__device__ __host__ virtual double _dist(const vec2 &start, vec2 &end) const= 0;
	//ある方向に指示された距離だけ動いた位置を返します。
	//ここでは、もし
	__device__ __host__ vec2 progress(const vec2 &start, double dist) const {
#if 0
//		_Assert( in(start) , "func_basic::progress : start isn't on the function" );
#else
		if (in(start) != true) {
			in(start);
			__109debug= 1;
			return vec2();	//デフォルトで作ったものを返してみます。
		}
#endif
		return _progress(start, dist);
	}
	__device__ __host__ vec2 _progress(const vec2& start, double dist)const { return vec2(); }	
};
#endif