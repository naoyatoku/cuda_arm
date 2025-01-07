#pragma once

#include	"units.h"
#include	"link.h"
#include	"csv.h"
#include	"inv_kinetic.h"
#include	"CommonModule.h"
#include	"jacobi.h"
#include	"func.h"

//三関節を持つアームです。

//movetoに指示するコマンド定義です。
//ビット割り付けで複数指示できるようにしておきます。


enum {
	_MVTO_SPD	= 0x1,		//指示された速度に到達するまで
	_MVTO_POS = 0x2,		//指示された位置に到達するまで
	_MVTO_POS_MIN_PEAK = 0x4,	//位置の極小の地点です。
	_MVTO_POS_MAX_PEAK = 0x8,	//位置の極大の地点です。
};

//各リンクの状況を返す必要がある場合にまとめたデータ構造です。（戻り値に使う)
struct link_stat {
	int stat;		//ステータス
	int no;			//上記ステータスをもつリンクの番号
	__host__ __device__ link_stat() :stat(MOV_OK), no(0) { ; }
	__host__ __device__ link_stat(int s, int n = -1) :stat(s), no(n) { ; }
};

//実際に即した形で
//Θ1 : 
#define		MAX_SPD_0			(PI/2)					//90 °/sec ( = 1.57 rad)
#define		ACC_T_CONST_0		400						//加速時定数
//Θ2：
#define		MAX_SPD_1			(PI*2/3)				//120 °/sec ( = 2.093 rad)
#define		ACC_T_CONST_1		400						//加速時定数
//Θ3：
#define		MAX_SPD_2			(PI)					//180 °/sec ( = 3.14 rad)
#define		ACC_T_CONST_2		400						//加速時定数
//
#define		MAX_ACC(MAXSPD,ACC_T)		(((MAXSPD)/(ACC_T))*1000)	//加速時定数 400ms
#define		MIN_SPD(MAXSPD,ACC_T)		 ((MAXSPD)/(ACC_T))			//最低速度は加速度の一番最初の値としておく。

class	arm : public _cood
{
	link	_link[3];		//三関節です。
	//
	int		_n_mv;			//moveした回数。わかりやすいように
	//3関節の計算ルーチンを持っておきます。
	// toku ちょっと
	_3link_calc_T<float> _calc;		//
//	_3link_calc_T<double> _calc;

	//リンク状態を二つの解のうち、どちらを選択するか
	const int		_pat;

public:
	//デフォルトコンストラクタ
//	arm() : _link{ link(PI/2 ,  PI/2/500 , PI/2/1000, 290.0 , 0 ,"csv\\l1.csv") , link(PI , PI/600 , PI/1000 , 290.0 , &_link[0],"csv\\l2.csv") , link(PI,PI/500 , PI/1000 , 135.0 , &_link[1] ,"csv\\l3.csv")} ,_log("csv\\arm.csv",true){

	//デバッグ環境のアームに合わせます。(L1:230 . L2:210  L3:144 ) ※本番はL1 290 ,L2  290 ,L3 135です。
	__host__
	arm() : _link{ link(MAX_SPD_0 ,  ACC_T_CONST_0 ,230.0 , 0) , link(MAX_SPD_1 , ACC_T_CONST_1, 210.0 , &_link[0]) , link(MAX_SPD_2 , ACC_T_CONST_2 ,144.0 , &_link[1]) } 
	,_pat(1)
	{
		_calc.set_kinetic_parameter(_link[0].l ,_link[1].l , _link[2].l);
//		set_jacobi_parameter(_link[0].l ,_link[1].l , _link[2].l);
	}
	//log をとるかどうかのスイッチをつけますが実際にはfalseで使う前提です。
	//cudeでlogをとるのが難しいのでやめる。
/*	arm(bool log) : _link{link(MAX_SPD	,	MAX_ACC	, MIN_SPD	, 290.0	, 0 		,	log ? "csv\\l1.csv" : 0)
								,	link(MAX_SPD	,	MAX_ACC	, MIN_SPD	, 290.0 , &_link[0]	,	log?"csv\\l2.csv":0) 
								,	link(MAX_SPD	,	MAX_ACC	, MIN_SPD	, 135.0	, &_link[1]	,	log?"csv\\l3.csv":0)} 
								,_log("csv\\arm.csv",log) {
		_calc.set_kinetic_parameter(_link[0].l ,_link[1].l , _link[2].l);
		
//		set_jacobi_parameter(_link[0].l ,_link[1].l , _link[2].l);
	}*/

	//アームを動かします。
	__device__ __host__ link_stat	move(_cood pos, bool judge = false, double ms = 1.0) {
		//動かせるかの判定をします。
		if (judge) {
			link_stat res;
			if ((res = move_able(pos, ms)).stat != MOV_OK) {
				return res;	//エラー状況を返します。44
			}
		}
		//動かせそうなのでセットします。
		mlti<_Vector3d<float> ,2>t = _calc.calc_inv_kinetic(pos); 		//指示された位置への各リンクのΘを求めます。
		for (int i = 0; i < 3; ++i) {
			_link[i].move(t.v[_pat](i));	//強制的に位置をセットです。	※ここで、各間接の位置と、各々の速度、加速度が計算されます。
		}
		_cood::move(pos, ms);		//これで位置、姿勢がセット＆各々の速度、加速度が計算される。
		return link_stat(MOV_OK);
	}

	__device__ __host__ link_stat	move_able(_cood pos, double ms = 1.0 , link_stat* ps = 0 ) {
		link_stat res;
		mlti<_Vector3d<float>, 2>t = _calc.calc_inv_kinetic(pos); 		//指示された位置への各リンクのΘを求めます。
//		mlti<_Vector3d, 2>t;
		for (int i = 0; i < 3; ++i) {
			res.no = i;						//今回のリンク番号です。
			if ((res.stat = _link[i].move_able(t[_pat](i))) != MOV_OK) {
#if 1	//
				if (ps) {
					for (int j = 0; j < 3; ++j) {
						if (i == j) {
							ps[j] = res;
							continue;
						}
						ps[j].no = j;
						ps[j].stat = _link[j].move_able(t[_pat](j));
					}
				}
#endif

				break;//return false;		//指示できなかった。
			}
		}

		return res;
	}
	//全部のリンク状態を格納するバージョンです。(調査用)
//	__device__ __host__ link_stat move_able(_cood pos , link_stat )


	//とりあえず、この部分はGPU処理のため、外部で行うことにする。
//	__device__ __host__ bool		moveto(func_basic &f, _cood tgt,unsigned int cmd=(_MVTO_SPD | _MVTO_POS));		//_coodが、ターゲットの状態。（位置vec2 + rad(姿勢）と、速度、加速度も内包します。)

//	virtual bool rel_move(_cood delta , bool judge=false , double ms=1.0 ){	return move( operator+(delta) , judge, ms );}

	//強制的に位置を変更します（初期位置をセットするなどの理由で）
	__host__ __device__ void set(_cood pos){ move(pos,false); stop();}//

//	virtual _cood &	move(const _cood &a, double ms) {
	//
	__device__ __host__
	const link& lnk(int idx)const{
		_gpuAssert(idx < 3, "arm::lnk() idx illegal(%d)", idx);
		return _link[idx];
	}
	//
	__device__ __host__ void stop(void){
		_cood::stop();			//自分自身のストップ
		for(int i=0;i<3;++i){	//内包する各関節も止めます。
			_link[i].stop();
		}
	}
	//トリップメータのようなもの.move()でカウントアップするようにします。
	//set等でリセットするようにします。
	__device__ __host__ int	n()const	{return _n_mv;}		//	


	//(debug)反転を作ってみます。速度の状態を反対にするだけ。
	__device__ __host__ void reverse() {
		wd.reverse();
		d.reverse();
		for (int i = 0; i < 3; ++i) {
			_link[i].d.reverse();
		}
	}
	//状態保存します(軸指示内容を、_logを使って保存します。
	__host__ bool save() {
		csv		_log[3] = { csv("csv\\l1.bin", true) , csv("csv\\l2.bin", true) , csv("csv\\l3.bin", true) };
		//3個の関節の位置データを保存する。
//		double link_data;
		for (int i = 0; i < 3; ++i) {
			_log[i].bin_write	((void*)&(_link[i].r)	, sizeof(double));
		}
		return true;
	}
	//
	__host__ __device__
	arm& operator=(const arm &a)
	{
		_cood::operator=(a);
		for(int i=0 ; i < 3 ; ++i){
			_link[i]=a.lnk(i);
		}
		_n_mv = a.n();
		return *this;
	}
};

//三関節アーム自身をエクスポートします。
extern arm _arm;

