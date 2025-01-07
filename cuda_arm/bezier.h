#pragma once

#include <math.h>
//#include <cmath>
#include "units.h"
#include <algorithm>
#include "CommonModule.h"	//_round()


//ベジェ曲線の位置に関する情報データ構造です。
struct bezier_pos
{
	float t;			//0-1の間の割合。
	float len;			//始点からの距離
};

struct arc_len_tbl {
	cpu_gpu_mem<float> t;   		// tの値を格納 (0.0, 0.01, 0.02, ...)
	cpu_gpu_mem<float> len; 		// その区間までの累積弧長
	//
	cpu_gpu_mem<vec2> pt; 		//tによるそのままの座標です。
	cpu_gpu_mem<vec2> dpt;		//導関数です。
	//
	int			steps;		//ステップ数
	arc_len_tbl() :t(0), len(0), pt(0), dpt(0), steps(0) { ; }
	arc_len_tbl(int steps) { alloc(steps); }
	void alloc(int _steps) {
		steps = _steps;
		t.alloc(steps);
		len.alloc(steps);
		pt.alloc(steps);
		dpt.alloc(steps);
	}
	//gpuへ転送する。
	void transfer_to_gpu() {
		t.Transfer_to_GPU();
//		len.transfer_to_gpu();
//		pt.transfer_to_gpu();
//		dpt.transfer_to_gpu();
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
	    float dt = 1.f / _tbl.steps;
    	float currentLength = 0.f;
	    // 累積長さを計算
    	for (int i = 1; i < _tbl.steps; ++i) {
        	float tPrev = _tbl.t(CPU,i - 1);
        	float tNow = tPrev + dt;
        	// t の格納
        	_tbl.t(CPU,i) = tNow;
        	// 区間 [tPrev, tNow] の移動距離を台形近似で計算
        	float speedPrev	=	dy_dt(tPrev).norm();		//ベジェ曲線のtでの微分はベクトルです。（速度ベクトル）ノルムがΔt進んだ場合の距離.
        	float speedNow	=	dy_dt(tNow).norm();		//ベジェ曲線のtでの微分はベクトルです。（速度ベクトル）ノルムがΔt進んだ場合の距離.
	        // 台形則: (v1 + v2)/2 * Δt
       		float segmentLength = (speedPrev + speedNow) * 0.5f * dt;
			currentLength += segmentLength;
   			_tbl.len(CPU,i) = currentLength;		//各ステップにおける始点からの距離です。
		}
		//完成したらGPUへ転送します。
		_tbl.transfer_to_gpu();
	}

	//関数値:t : 0.0 - 1.0まで
	__device__ __host__ 
	vec2 f(float t) const {
		_Assert(t >= 0.0 && t <= 1.0, "bezier::f() t is illegal");
	    float u = 1.f - t;
   		return  _p[0].operator*(u * u * u)
        	+	_p[1].operator*( 3.f * u * u * t)
        	+	_p[2].operator*( 3.f * u * t * t)
        	+	_p[3].operator*(t*t*t);
	}
	//微分(ベクトル)
	__device__ __host__ 
	vec2 dy_dt(float t) const {
		_Assert(t >= 0.0 && t <= 1.0, "bezier::dy_dt() t is illegal");
		float u = 1.f - t;
		vec2 dP0 = _p[1] - _p[0];
		vec2 dP1 = _p[2] - _p[1];
		vec2 dP2 = _p[3] - _p[2];
		vec2 dP = dP0 * (3 * u * u) + dP1 * (6 * u * t) + dP2 * (3 * t * t);
		return dP;
	}

	//ベジェ曲線の場合は、片方の方向前提にします。逆方向に進みたかったら始点を終点を交換して
	//t_startは、測定開始地点の0-1の間の値です。
	__device__ __host__
	vec2 progress( int location , bezier_pos & now , float dist , bezier_pos &aft ) const
    {
		_Assert( now.len >= 0.0 && now.len <= 1.0, "bezier::progress() t_start is illegal");
		//
		float t = find_t_from_len(location , now.len + dist );
		//ベジェ状態を記録します。
		{
			aft.t = t;
			aft.len = now.len + dist;
		}
		//tから座標を
		return f(t);
    }


	//lengthを指示すると、tの位置を返す。
	__host__ __device__
	float find_t_from_len(int location , float length)const
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
    	float length0 = _tbl.len(location,idx - 1);
    	float t0 = _tbl.t(location,idx - 1);
    	float length1 = _tbl.len(location,idx);
    	float t1 = _tbl.t(location,idx);

	    float ratio = (length - length0) / (length1 - length0);		//idx-1 ～　idx の間のlengthの割合
    	float tInterp = t0 + (t1 - t0) * ratio;						//t[idx-1] +  t[idx-1]-t[idx]の間の割合 × (t[idx-1] ～ t[idx] の長さ(step長))
    	return tInterp;
	}
};


