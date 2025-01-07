
#pragma once
//#include <windows.h>
#include <math.h>
#include "units.h"
#include <algorithm>
#include "func.h"
#include "CommonModule.h"	//_round()

//	円の場合、ｘに対して二つ解がある。
//			→

//	アームを、円のある位置から、ある位置まで動作させたい。
//	いまは、角度 t -> t2 まで動作させる。
enum {
//	CIRCLE_DIR_LEAST = 0,
	CIRCLE_DIR_CW=1,		//これ逆かもしれない(終点のほうが角度が大きい場合こちらです)(左回り)
	CIRCLE_DIR_CCW=-1		//
};

//dist()と、、progress()がin()チェックを行う。
//このモジュールは極座標で動作します。
class circle
{
	double		_r;		//半径です。
	vec2		_org;	//原点です。
	//ある軌跡をたどる場合、基本的にどちらに進んでもいいので、進行方向を指定できるようにします。
	int			_dir;	//方向です。
public:
	circle():_r(100.0),_org(),_dir(CIRCLE_DIR_CW) {;}
	circle(double r,double x,double y):_r(r),_org(x,y),_dir(CIRCLE_DIR_CW)	{;}

	//円が座標を表すのは、やっぱりx,y座標。

	//これは、 θを指示して、vec2を返すもの。
	__device__ __host__ inline  bool in(const vec2& a)	const;
	__device__ __host__ mlti<vec2, 2> f(double x) const {
		double _x = x - _org.x;				//
		double _y = sqrt((_r * _r) - (_x * _x));	//これは±です。
		mlti<vec2, 2>m;
		m[0] = vec2(_x, _y) + _org;
		m[1] = vec2(_x, -_y) + _org;
		return m;
	}
	//この関数は、円特有の指示となります。（規定クラスからははずした)
	__device__ __host__ vec2 rf(double rad)	const {
		return vec2(rvec2(_r,rad)) + _org;
	}

	//setter/getter
	__device__ __host__ void		set_dir(int dir) { _dir=dir; }		//
	__device__ __host__ int			get_dir()const { return _dir; }	//__dirの取得
	__device__ __host__ int			dir(const vec2 &start, vec2 &end)const;
	//微分、これなんでいるんだっけ
	__device__ __host__	mlti<double,2> dy_dx(double x)const {
		mlti<double, 2>r;
		r[0] = -x / sqrt(_r * _r - x * x);
		r[1] = -r[0];	//もう一つの解は符号を変えたものになる。
		return  r;
	}
	//距離を指定すると相当する角度を返す
	//距離を計算する
	__device__ __host__ inline double dist(const vec2 &start, vec2 &end) const;		//
	__device__ __host__ inline double dist(double rad, bool sign = false)const;		//角度を指示するバージョン
	//ある距離進む。
	__device__ __host__ inline vec2 progress(const vec2 &start, double dist , int dir = 0 ) const;
	__device__ __host__ inline vec2 progress_rad(const vec2 &start, double rad , int dir = 0 ) const;
	//位置や距離から角度を求める
	__device__ __host__ inline double angle(double dist)const;
	__device__ __host__ inline double angle(const vec2& p)const;
};


__device__ __host__ inline bool circle::in(const vec2& a)	const {
	bool r = false;
	mlti<vec2, 2> p = f(a.x);	//x座標から、
	//解の中で、指示された位置が存在するかどうか
	for (int i = 0; i < 2; ++i) {
		if (a == p[i]) { return true; }
	}
	return false;	//一致がなかった。
}
//指示された始点から終点までに進むのに、どちらの方向にいけばいいかを考える。
__device__ __host__ inline int circle::dir(const vec2& start, vec2& end) const
{
	//	if (_dir == CIRCLE_DIR_LEAST) {
			//これは、一度角度に直して、
	rvec2 s(start - _org);	//始点を、円の中心からの極座標にて、
	rvec2 e(end - _org);	//終点を、円の中心からの極座標にて

	//これで、円の場合、終点と始点の角度の差から、どっち周りかを判別します。
	int d = ((e.r - s.r) > 0) ? 1 : -1;
	//方向が決まると、時計回りか、どうか
	return d;
	//	}
	//	return _dir;	//そのほかの場合は指定された方向を返します。
}
//距離を

__device__ __host__ inline double circle::dist(const vec2& start, vec2& end) const
{
	///ここで、円の場合は__dirに応じた距離を算出するようにします。
	//やっぱり角度に直して、その円周を計算します。
	rvec2 s(start - _org);		//始点を、円の中心からの極座標にて、
	rvec2 e(end - _org);		//終点を、円の中心からの極座標にて
	double d = dist(e.r - s.r);	//これが円周です。
	//もし進行方向と逆方向の距離を計算した場合には、反対側の距離を返すようにします。
	if (((dir(start, end) > 0) && ((e.r - s.r) < 0))	//CW方向に向かうのに、終点が始点よりも小さい場合
		|| ((dir(start, end) < 0) && ((e.r - s.r) > 0))	//CCW方向に向かうのに、終点が始点よりも大きい場合。
		) {
		d = dist(2.0 * PI) - d;
	}
	return d;
	//	return (dist(e.r-s.r));
}

//角度を指示するバージョンのやつ。その場合はdirは関係ない？
__device__ __host__ inline double circle::dist(double rad, bool sign/*=false*/)const
{
	double t = sign ? rad : fabs(rad);	//これが角度です。
	double d = _r * t;					//これが円弧です。
	return d;
}

__device__ __host__ inline double circle::angle(double dist)const
{
	return dist / _r;
}
__device__ __host__ inline double circle::angle(const vec2& p)const
{
	_gpuAssert(in(p), "circle::angle() p is not on circle");
	vec2 _p ( p - _org);
	return atan2(_p.y, _p.x);
}
//進める

//__device__ __host__ vec2 progress(const vec2& start, double dist, int dir = 0) const
__device__ __host__ inline vec2 circle::progress(const vec2& start, double dist, int dir) const
{
	return progress(start, angle(dist), dir);
}
__device__ __host__ inline vec2 circle::progress_rad(const vec2& start, double rad, int dir) const
{
	if (_gpuAssert(in(start), "circle::_progress(): start is not on clircle ") != true)return vec2();
	if (dir == 0) { dir = _dir; }
	//ここで誤差をいろいろ見てみます。
	vec2 src(start - _org);
	double _dt = atan2(src.y, src.x) + rad;
	vec2 v(_r * cos(_dt), _r * sin(_dt));
	v += _org;
	if (in(v) != true) {
		//ここでxのときの解yを求めてみる。
//		vector<vec2>arr = f(_x);
		if (_gpuAssert(in(v), "circle::_progress() not in circle") != true ) return vec2();
	}
	return v;
}


//これのGPUバージョンを作らないといけない。継承が使えない。
//汎用性は考えずに円のある位置から、ある位置まで動作させた場合の
//座標が返ればよい。
/*
struct gpu_circle {
	double	_r;			//半径
	vec2	_org;		//中心
	
	__device__ __host__ mlt_v<vec2,2> f(double x){
		double _x = x - _org.x;				//
		double _y = sqrt((_r*_r) - (_x*_x));	//これは±です。
		mlt_v<vec2,2>m;
		m.v[0] = vec2(_x, _y) +_org;
		m.v[0] = vec2(_x, - _y) +_org;
		return m;
	}
	//スタート地点から、ある位置まで進んだ時の位置を返す。
	virtual vec2 _progress(const vec2& start, double dist) const;
	__device__ __host__  vec2 _progress(const vec2& start, double dist) const
	{

	}
};
*/

