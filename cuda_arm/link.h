#pragma once
#include "units.h"
#include "csv.h"

//move_able に関する戻り値です。
//0より上がうまくいったとするか
enum {
	//
	MOV_SPD_OVER	=3,		//最大速度を超えて動かそうとした
	MOV_ACC_OVER	=2,		//加速が大きすぎてNG
	MOV_DEC_OVER	=1,		//減速が大きすぎてNG
	MOV_OK=0,				//成功
};


//各関節です。
class link	:	public rvec2		//
{
	const link *	_parent;		//接続されている親です。
	//max/limits
	double	_max_acc;		//(最大加減速)
	double	_max_spd;		//最大速度
	//csvが、__device__に対応していないため、除外しました。
//	csv		_log;			//自分自身のログを記録するためにCSVを実装しておくか。
	//最低限の分解能があるはず
	double	_min_spd;		//最低限の送り出し量
public:
	__device__ __host__ link() :_max_acc(0.0), _max_spd(0.0),_min_spd(0.0)/*,_log("link_?", false)*/ { ; }
	__device__ __host__ link(double max_s, double acc_t_const ,double l,link *p,const char*name=0) : _max_spd(max_s), rvec2(l,0.0) , _parent(p)  /*, _log(name, name ? true : false)*/ {
		_max_acc	=	max_s * 1000 / acc_t_const;		//時定数から加速率を決めます。
		_min_spd = _max_acc;		//加速度と同じにします。
	}
	__device__ __host__ link(const link&a){
		operator=(a);	
	}
 
	//回転角度で現在の位置に一番近いものを探す（+-のどちらか)
	__device__ __host__ inline double	nearest(const double rad );
	__device__ __host__ inline int		move_able(const double new_rad, double ms=1.0);
	__device__ __host__ inline link &	move(const double new_rad , bool log=true, double ms=1.0) ;

//	double	total_t (void)const;
	__device__ __host__ inline double	total_angle(void)const;		//
	//自分の位置を返します。_coodでまとめて返したほうが
	__device__ __host__ inline vec2	linked_vect(void) const;	//角度に応じたxy座標を返します。
	//
/*	double	spd(void)	const	{	return		_spd;	}		//角速度です。
	double	pos(void)	const	{	return		_pos;	}		//自分自身の（軸としての）位置（＝実質には角度）です。
	*/
	__device__ __host__ inline const link*	parent(void)	const		{		return _parent;		}
	__device__ __host__ inline double		max_acc(void)	const		{		return _max_acc;	}
	__device__ __host__ inline double		max_spd(void)	const		{		return _max_spd;	}
//	double		l(void)			const	{	return		l;		}		//
	__device__ __host__ link& operator=(const link &a) {
		rvec2::operator=(a);
//		_parent		=	a.parent();	//
		_max_acc	=	a.max_acc();
		_max_spd	=	a.max_spd();

		//親リンクはあくまでも変更しない・・
//		_parent = a.parent();

		//注意：logについては、コピーされないようにします。...これ大丈夫か
		//		コピーしてしまうと、同じハンドルでコピーが作られ、その変数の生存期間が終わったときにデストラクタが呼ばれた際、
		//		CSVの持っているハンドルを解放してしまう。・・その後、コピー元のCSVが使えなくなる問題がある。
		//_log.
		//
		return *this;
	}
};



__device__ __host__ inline double link::nearest(const double new_rad) {
	//もし十分近い位置にあるなら評価しない
	if (fabs(r - new_rad) < (PI / 2)) {
		return new_rad;
	}
	//180度以上変わっているならば数値の近い
	double min = 2 * PI; int min_n = 0;
	for (int n = -1; n < 2; ++n) {
		double d;
		if ((d = fabs(r - (new_rad + (2 * PI * n)))) < min) {	//最小値を見つけていきます。
			min = d;
			min_n = n;
		}
	}
	//見つけた最小のものにnewposを変更します。
	double _new_rad = new_rad + (2.0 * PI * min_n);
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
__device__ __host__ inline int	link::move_able(const double new_rad, double ms)
{
//	double	_new_rad = nearest(new_rad);	//一周まわっていることがあるので、自分自身に一番近い角度に修正します。
	//	_cood _newpos( vec2(_l*cos(_new_rad),_l*sin(_new_rad)) , _new_rad);	//これが新しい角度の_coodです。
		//自分のコピーを動かしてみて、その加速度や速度を調べます。
	link me(*this);
	me.move(new_rad, false);	//自分のコピーを動かしてみます。
	//
	//これ、加速度がある程度誤差をもってもいいようにするか？
	//加速度が小さい場合、

	//ここでは、加速がマイナス側に大きすぎたのか、プラス側に大きすぎたかだけを返すようにします。
	if (fabs(me.d.acc) - _max_acc > (_max_acc / 10)) {		//加速（減速)が大きすぎた
		//^^^^^^^^^^これなんだっけ? (_max_acc +　10％　は許容するようにしている？)

//多き過ぎか小さすぎかを返します
//現在の速度と、結果加速度が同じ方向かどうかの判定です。

//アーム動作の補正の方向を見つけます。
//これは次回どちらの方向に向かえばいいかの方向をあわらします。
//アームの動きに対して、各リンクは非線形に動作する。
//この瞬間の勾配の値をもとにしないといけない。

		if (me.d.acc < 0.0) { return MOV_DEC_OVER; }	//加速度がマイナス側に大きすぎた
		return MOV_ACC_OVER;								//加速度がプラス側に大きすぎた
		/*
				if( (this->d.spd) * me.d.acc  >= 0.0 ) {		//加速度と現在の速度の符号が同じならば、加速がおおきすぎると答えます。
					return MOV_ACC_OVER;
				}
				return MOV_DEC_OVER;		//反対方向へ動作させようとした場合には減速しきれないとします。
		*/
	}
	if (fabs(me.d.spd) > _max_spd) {	//速度が大きすぎた場合
		return MOV_SPD_OVER;			//速度が大きすぎる。※いまのところ符号の判定が必要ない
	}
	return MOV_OK;
}
__device__ __host__ inline link& link::move(const double new_rad, bool log, double ms)
{
	double	_new_rad = nearest(new_rad);	//一番近い角度に変換sします。

	//debug いまの位置を計算します。
//	_cood _nowpos(vec2(_l*cos(rad), _l*sin(rad)), rad);
	rvec2 _newpos(l, _new_rad);	//これが新しい式です。

	rvec2::move(_newpos, ms);
	//動いたときにログをとってみます。
/*	if (log) {
		//
		vec2 v(to_vec2());
		_log.writef("%lf,%lf,%lf,%lf,%lf\n", v.x, v.y, r, d.spd, d.acc);
	}
*/
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
__device__ __host__ inline double	link::total_angle(void)const
{
	double th = 0;
	if (_parent)	//親がいる場合には親までの全関節の角度です。
	{
		th = _parent->total_angle();
	}
	return th + r;		//自分自身の
}

__device__ __host__ inline vec2 link::linked_vect(void) const {
	vec2 v(rvec2(l, total_angle()));	//これが自分自身のベクトル。で(ややこしいですが極形式で登録して直交座標に変換してます。)
	//親がいる場合には親の先端座標を足します。
	if (_parent)
	{
		v += _parent->linked_vect();	//親までの先端座標を足します。（ここが、自分自身の根元の座標になる)
	}
	return v;		//座標は
}
//


