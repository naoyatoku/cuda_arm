#include "link.h"
#include "units.h"
#include <math.h>

#if 0
__device__ __host__ double link::nearest(const double new_rad){
	double min =2*PI;int min_n=0;
	for(int n = -1 ; n < 2 ; ++n ){
		double d;
		if( (d = fabs(r - (new_rad + (2*PI*n)) )) < min ){	//最小値を見つけていきます。
			min = d;
			min_n=n;
		}
	}
	//見つけた最小のものにnewposを変更します。
	double _new_rad = new_rad + (2.0*PI*min_n);
	return _new_rad;
}

//位置の指示ができるかを判断します。
/*bool link::judge_set(double newpos) {
	//まず、位置の差分が自分の加減速速度より小さくなければなりません。
	_cood _newpos( vec2(_l*cos(newpos),_l*sin(newpos)) , newpos);	//これが新しい式です。	
	_cood me = *this;	//自分のコピーです。
	me = _newpos;		//セットするとで
  
	//加速も減速もせずに到達する位置
	double next_pos = _pos + (_spd / 1000);  //現在の位置＋現在速度です。(1msの場合、1000で割る必要がある)
	//要求された位置との差が、差第加速度を超えなければ動ける
	//角度が近い場所を探します。どうやってさがすんか
	double near_newpos = nearest_pos(next_pos, newpos);
	if (fabs(next_pos - nearest_pos(next_pos,newpos)) <= _max_acc) {       //大きくなるにせよ小さくなるにせよ、最大加速を超えなければＯＫ
		return true;
	}
	return false;
}
*/
__device__ __host__ int	link::move_able(const double new_rad, double ms)
{
//	double	_new_rad = nearest(new_rad);
//	_cood _newpos( vec2(_l*cos(_new_rad),_l*sin(_new_rad)) , _new_rad);	//これが新しい角度の_coodです。
	//自分のコピーを動かしてみて、その加速度や速度を調べます。
	link me(*this);	
	me.move(new_rad, false);	//自分のコピーを動かしてみます。
	//
	//これ、加速度がある程度誤差をもってもいいようにするか？
	//加速度が小さい場合、

	//ここでは、加速がマイナス側に大きすぎたのか、プラス側に大きすぎたかだけを返すようにします。
	if( fabs(me.d.acc) - _max_acc > (_max_acc/10) ){		//加速（減速)が大きすぎた
									//^^^^^^^^^^これなんだっけ? (_max_acc +　10％　は許容するようにしている？)

		//多き過ぎか小さすぎかを返します
		//現在の速度と、結果加速度が同じ方向かどうかの判定です。

		//アーム動作の補正の方向を見つけます。
		//これは次回どちらの方向に向かえばいいかの方向をあわらします。
		//アームの動きに対して、各リンクは非線形に動作する。
		//この瞬間の勾配の値をもとにしないといけない。

		if(me.d.acc < 0.0 ){	return MOV_DEC_OVER;	}	//加速度がマイナス側に大きすぎた
		return MOV_ACC_OVER;								//加速度がプラス側に大きすぎた
/*
		if( (this->d.spd) * me.d.acc  >= 0.0 ) {		//加速度と現在の速度の符号が同じならば、加速がおおきすぎると答えます。
			return MOV_ACC_OVER;		
		}
		return MOV_DEC_OVER;		//反対方向へ動作させようとした場合には減速しきれないとします。
*/
	}
	if( fabs(me.d.spd) > _max_spd ){	//速度が大きすぎた場合
		return MOV_SPD_OVER;			//速度が大きすぎる。※いまのところ符号の判定が必要ない
	}
	return MOV_OK;
}
__device__ __host__ link &	link::move(const double new_rad, bool log,double ms) 
{
	double	_new_rad = nearest(new_rad);	//一番近い角度に変換sします。

	//debug いまの位置を計算します。
//	_cood _nowpos(vec2(_l*cos(rad), _l*sin(rad)), rad);
	rvec2 _newpos( l , new_rad );	//これが新しい式です。

	rvec2::move(_newpos,ms);
	//動いたときにログをとってみます。
	if (log) {
		//
		vec2 v(to_vec2());
		_log.writef("%lf,%lf,%lf,%lf,%lf\n", v.x, v.y, r, d.spd, d.acc);
	}
	return *this;
}

//基準軸からの角度を返します。
/*double link::total_t (void)const {
	double t = _pos;		//自分自身の位置
	if(_parent){
		t+=_parent->total_t();
	}
	return t;
}*/
//この関節の座標を返します（自分が接続しているリンクの座標を考慮した、自分自身の位置を返します。
//自分自身は、自分の軸が原点にあるときの
//
//double link::total_angle(void)const;
__device__ __host__ double	link::total_angle(void)const
{
	double th=0;
	if (_parent)	//親がいる場合には親までの全関節の角度です。
	{
		th = _parent->total_angle();
	}
	return th + r;		//自分自身の
}

__device__ __host__ vec2 link::linked_vect(void) const {
	vec2 v(rvec2(l, total_angle()));	//これが自分自身のベクトル。で(ややこしいですが極形式で登録して直交座標に変換してます。)
	//親がいる場合には親の先端座標を足します。
	if (_parent)
	{
		v += _parent->linked_vect();	//親までの先端座標を足します。（ここが、自分自身の根元の座標になる)
	}
	return v;		//座標は
}
//


#endif