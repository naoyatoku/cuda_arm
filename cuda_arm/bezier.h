#pragma once

#include <math.h>
//#include <cmath>
#include "units.h"
#include <algorithm>
#include "CommonModule.h"	//_round()


//ベジェ曲線の位置に関する情報データ構造です。
//lenが大きくなると有効数字7桁のfloatでは、微小な移動距離のときに丸まってしまい、計算ができない部分ができてしまったためdoubleに変更。
//データパディングがはいるためサイズが変わらないのでtもdoubleにしました。
struct bezier_pos
{
	double t;			//0-1の間の割合。
	double len;			//始点からの距離
	__device__ __host__
	bezier_pos(double _t,double _len) :t(_t), len(_len) { ; }
	__device__ __host__
	bezier_pos() :t(0), len(0) { ; }
};

struct arc_len_tbl {
	cpu_gpu_mem<double> t;   		// tの値を格納 (0.0, 0.01, 0.02, ...)
	cpu_gpu_mem<double> len; 		// その区間までの累積弧長
	//
	cpu_gpu_mem<vec2> pt; 		//tによるそのままの座標です。
	cpu_gpu_mem<vec2> dpt;		//導関数です。



	//
	int			steps;		//ステップ数
	arc_len_tbl() :t(0), len(0), pt(0), dpt(0), steps(0) { ; }
	arc_len_tbl(int steps) { alloc(steps); }
	~arc_len_tbl() {
		free();
	}
	void alloc(int _steps) {
		steps = _steps;
		t.alloc(steps+1);
		len.alloc(steps+1);
		pt.alloc(steps+1);
		dpt.alloc(steps+1);
	}
	//gpuへ転送する。
	void transfer_to_gpu() {
		t.Transfer_to_GPU();
		len.Transfer_to_GPU();
		pt.Transfer_to_GPU();
		dpt.Transfer_to_GPU();
	}
	void free() {
		t.free();   		// tの値を格納 (0.0, 0.01, 0.02, ...)
		len.free(); 		// その区間までの累積弧長
		pt.free(); 		//tによるそのままの座標です。
		dpt.free();		//導関数です。
	}
};
class bezier
{
	vec2			_p[4];
	arc_len_tbl		_tbl;

public:
	__host__	bezier() {;}
	__host__	bezier(const vec2 &p0, const vec2 &p1, const vec2 &p2 , const vec2 &p3 , int steps)
	{	//ここでテーブルを作成します。
		set_params(p0, p1, p2, p3, steps);
	}
	__host__	~bezier() {
		_tbl.free();
	}
	//ベジェ曲線の制御点とステップ数をセットし、レングステーブル作成します。
	__host__ void set_params(const vec2&p0,const vec2 &p1,const vec2 &p2,const vec2 &p3,int steps){
		_p[0] = p0;
		_p[1] = p1;
		_p[2] = p2;
		_p[3] = p3;
		_tbl.alloc(steps);
		//テーブルの作成
		_tbl.t(CPU,0)	= 0.f;
    	_tbl.len(CPU,0)	= 0.f;
	    double dt = 1.f / _tbl.steps;
    	double currentLength = 0.f;
	    // 累積長さを計算
    	for (int i = 1; i < _tbl.steps+1; ++i) {
        	double tPrev = _tbl.t(CPU,i - 1);
        	double tNow =	tPrev + dt;
			if(tNow > 1.0)	tNow = 1.0;	//精度の問題で上限を超える場合があるので
			// t の格納
        	_tbl.t(CPU,i) = tNow;
        	// 区間 [tPrev, tNow] の移動距離を台形近似で計算
        	double speedPrev	=	dy_dt(tPrev).norm();		//ベジェ曲線のtでの微分はベクトルです。（速度ベクトル）ノルムがΔt進んだ場合の距離.
        	double speedNow		=	dy_dt(tNow).norm();		//ベジェ曲線のtでの微分はベクトルです。（速度ベクトル）ノルムがΔt進んだ場合の距離.
	        // 台形則: (v1 + v2)/2 * Δt
       		double segmentLength = (speedPrev + speedNow) * 0.5f * dt;
			currentLength += segmentLength;
   			_tbl.len(CPU,i) = currentLength;		//各ステップにおける始点からの距離です。
		}
		//完成したらGPUへ転送します。
		_tbl.transfer_to_gpu();
	}

	//関数値:t : 0.0 - 1.0まで
	__device__ __host__ 
	vec2 f(double t) const {
		_Assert(t >= 0.0 && t <= 1.0, "bezier::f() t is illegal");
	    double u = 1.f - t;
   		return  _p[0].operator*(u * u * u)
        	+	_p[1].operator*( 3.f * u * u * t)
        	+	_p[2].operator*( 3.f * u * t * t)
        	+	_p[3].operator*(t*t*t);
	}
	//微分(ベクトル)
	__device__ __host__ 
	vec2 dy_dt(double t) const {
		_Assert(t >= 0.0 && t <= 1.0, "bezier::dy_dt() t is illegal");	
		//ここでfloatに丸めてしまう。※tはdoubleだが、0-1のstep=1000程度の値なので、floatで十分。
		float u = 1.f - (float)t;
		vec2 dP0 = _p[1] - _p[0];
		vec2 dP1 = _p[2] - _p[1];
		vec2 dP2 = _p[3] - _p[2];
		vec2 dP = dP0 * (3 * u * u) + dP1 * (6 * u * t) + dP2 * (3 * t * t);
		return dP;
	}

	//ベジェ曲線の場合は、片方の方向前提にします。逆方向に進みたかったら始点を終点を交換して
	//t_startは、測定開始地点の0-1の間の値です。
	__device__ __host__
	vec2 progress( int location , bezier_pos & now , float dist ,int dir, bezier_pos &aft ) const
    {
		_Assert( now.t >= 0.0 && now.t <= 1.0, "bezier::progress() t_start is illegal");
		//※ここでlenの精度がfloatで足りなくなる場合がある。
		double len = now.len + (dist * dir);
		double t = find_t_from_len(location , len);
		//ベジェ状態を記録します。
		{
			aft.t = t;
			aft.len = len;
		}
		//tから座標を
		return f(t);
    }


	//lengthを指示すると、tの位置を返す。
	__host__ __device__
	double find_t_from_len(int location , double length)const
	{
    	// 1. length が table.arcs の最小値未満 or 最大値より大きい場合の処理
    	if (length <= 0.f) return 0.f;
    	if (length >= _tbl.len.back(location)) return 1.f; // すでに末端まで

	    // 2. バイナリサーチ log2(step)回→1000ステップで10回で見つかる
    	int left = 0;
    	int right = _tbl.len.size - 1;
	    while (left < right) {
    	    int mid = (left + right) / 2;
       		if (_tbl.len(location,mid) == length) {
            	return _tbl.t(location,mid);		//※	ぴったりだった場合はそのものを返す。
        	}
        	else if (_tbl.len(location,mid) < length) {
	        	left = mid + 1;
        	}else {
				right = mid;
        	}
    	}
		//lengthは、各インデックスの間にある場合があり、このときにleftは、lengthを追い越した初めての位置をさしている。
		//(idx-1 ～ idxまでの間にlengthは存在している。
    	int idx = left;

	    // 安全策: 配列外アクセス防止
    	if (idx == 0) return _tbl.t(location,0);
    	if (idx >= (int)_tbl.len.size) return _tbl.t.back(location);

	    // arcs[idx-1], arcs[idx] の区間で線形補間
    	double l0 = _tbl.len(location,idx - 1);
    	double l1 = _tbl.len(location,idx);
    	double t0 = _tbl.t(location,idx - 1);
    	double t1 = _tbl.t(location,idx);
	    double ratio = (float)((length - l0) / (l1 - l0));		//idx-1 ～　idx の間のlengthの割合
    	double tInterp = t0 + (t1 - t0) * ratio;						//t[idx-1] +  t[idx-1]-t[idx]の間の割合 × (t[idx-1] ～ t[idx] の長さ(step長))
    	return tInterp;
	}
	//tからレングスを返す
	__device__ __host__
	double find_len_from_t(int location,double t)const{
		// 1. t が table.arcs の最小値未満 or 最大値より大きい場合の処理
		if (t <= 0.f) return 0.f;
		if (t >= 1.f) return _tbl.len.back(location); // すでに末端まで

		//tの場合は、step毎にindexになっています。
		int idx= (int)(t * _tbl.steps);

	    // 安全策: 配列外アクセス防止
		if (idx == 0) return 0.f;
		if (idx >= (int)_tbl.len.size) return _tbl.len.back(location);

	    // arcs[idx-1], arcs[idx] の区間で線形補間
		double l0 = _tbl.t(location,idx - 1);
		double l1 = _tbl.len(location,idx);
		double t0 = _tbl.t(location,idx - 1);
		double t1 = _tbl.t(location,idx);
	    double ratio = (t - t0) / _tbl.steps;		//idx-1 ～　idx の間のtの割合
		//
		//lenが大きくなるのでfloatの場合に有効数字桁数が足りなくなるのでdoubleに拡張して計算する。
		double len = l0 + (l1 - l0) * ratio;		//t[idx-1] +
		return len;
	}
	//tから、bezier_posを返す
	__device__ __host__
	bezier_pos	pos(double t,int location=CPU)const{
		return bezier_pos(t,find_len_from_t(location,t));
	}


};


