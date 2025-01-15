
#ifndef	__UNITS__
#define	__UNITS__
#include <math.h>
#include "CommonModule.h"
#include <vector>
#include "Eigen/Dense" 

//とりあえず今は
using namespace Eigen;
using namespace std;

#define PI 3.141592653589793		//π
#define	DELTA_T	(1/1000.0)			//RTEXの周期。速度の時間分解能です。

//速度と加速度のペアです。
//本来はこれらもベクトルにしないと・・・
struct spd_acc
{
	float	spd;
	float	acc;
	__device__ __host__ spd_acc():spd(0.0),acc(0.0){;}
	__device__ __host__ spd_acc(float _s,float _a) : spd(_s),acc(_a){;}
	__device__ __host__ void stop(void){spd=0.0;acc=0.0;}		//速度、加速度を０に（静止状態を作る	
	__device__ __host__ bool operator==(const spd_acc &a)const {		return _equal<>(spd, a.spd) && _equal<>(acc, a.acc);	}

	//反転させてみる。(debug用)
	__device__ __host__ void reverse() {
		spd *= -1.0;	acc *= -1.0;
	}
};

//変換を作ります。
struct rvec2;;struct vec2;
/*
vec2	to_vec2(const rvec2&a);		//
rvec2	to_rvec2(const vec2&a);
*/


//これはいるかどうかわからん。
//２次元ベクトルです。
//二つのベクトルの差分から速度
struct vec2 {
	float  x;
	float  y;
	spd_acc	d;
	__device__ __host__ vec2():x(0.0),y(0.0){;}
	__device__ __host__ vec2(float _x , float _y  ,spd_acc _d=spd_acc()  ):x(_x),y(_y), d(_d) {;}
	__device__ __host__ vec2(const rvec2& a);
//	__device__ __host__ inline vec2(const rvec2& a);
	//	{		operator=(a.to_vec2());	}						//極座標を変換したもの


	__device__ __host__ float scalar(void)const {return sqrtf((x*x)+(y*y));		}		//これがすからです。
	//各種演算です。
	__device__ __host__ rvec2	inline to_rvec2(void)const;
	__device__ __host__ vec2	operator-(const vec2 &a)	const	{
		vec2 r(x-a.x,y-a.y);
		return r;
	}
	__device__ __host__ vec2	&operator=(const vec2 &a)			{
		x = a.x;
		y=a.y;
		d = a.d;
		return *this;
	}
	__device__ __host__ vec2	operator+(const vec2 &a)	const	{	vec2 r(x+a.x,y+a.y);	return r;					}
	__device__ __host__ vec2	operator+=(const vec2& a) { return operator=(*this + a); }

	__device__ __host__ bool	operator==(const vec2 &a)	const	{
		return (_equal<float>(a.x, x) && _equal<float>(a.y, y))
			;
	}
	__device__ __host__ vec2 operator*(const float a) const {
		vec2 r(x * a, y * a);
		return r;
	}
    __device__ __host__ vec2 operator/(const float a)const{
        _Assert(a!=0.0f,"dev by 0");
        return vec2( x/a , y/a );
    }	

	//二点間の距離です。
	__device__ __host__ float distance(const vec2& a )const {	
		return sqrt( ((a.x-x)*(a.x-x)) + ((a.y-y)*(a.y-y)) );
	}

	//ベクトルをセットする。現在の速度を考慮して
	__device__ __host__ vec2 move(const vec2 &a , float ms=1.0)	{		//ある時間を掛けてその位置まで行く
		//現在の速度で指示された時間で進む距離をまず進める。
		if (_gpuAssert(ms > 0, "vec2::move() : ms is 0") != true) {
			return vec2();
		}
		vec2 old_this(*this);	//自分のバックアップをとっておきます。
		operator=(a);	//まず位置を書き換えてしまいます。
		//速度は、等速で移動した距離＋足りない距離分加速します。
//		s.acc	=	(vec2(a -*this).scalar() - (d.spd*ms/1000.0))/ms*1000.0 ;	//この距離をmsかけて進む。

		d.spd = ( (*this - old_this).scalar()) / (ms / 1000);		//Δr / Δt     =>    ((r-old_this.r)*1000) / (ms*1000) mm/sec
		//加速度です。
		d.acc = (d.spd - old_this.d.spd) / (ms / 1000);			//
/*
		d.acc	=	((vec2(*this-old_this).scalar())/ms*1000.0) - old_this.d.spd;	//現在の速度に足す速度が加速度です。
		d.spd	=	old_this.d.spd + d.acc;		//現在の速度に加速度を足す。
*/
		vec2 test = vec2(*this-old_this);
		return *this;
	}
	//止める。
	__device__ __host__ void stop(void){
		d.stop();
	}
	//ノルム
	__device__ __host__ float norm()const{
		 return sqrt( (x*x) + (y*y));
	}
};





//極座標上のベクトルです。vec2と、rvec2は変換可能だと思う。
struct rvec2
{
	float l;
	float r;
	spd_acc	d;
	__device__ __host__ rvec2() :l(0.0), r(0.0) { ; }
	__device__ __host__ rvec2(float _l, float _r  ,spd_acc _d=spd_acc() ) :l(_l), r(_r) , d(_d) {
		//ここで、rは0-2πの間に収めるようにします(マイナスはないように)
		;
	}
//	__device__ __host__ rvec2(const vec2 &a);
	__host__ __device__ rvec2::rvec2(const vec2& a) {	operator = (a.to_rvec2());}

	__device__ __host__ float scalar(void)const { return l; }		//極座標のスカラは、lでいいよね？
	//各種演算です。
	__device__ __host__ vec2 inline to_vec2(void)const;
	__device__ __host__ rvec2 &operator=(const rvec2 &a) { l = a.l;	r = a.r;	d = a.d;	return *this; }

	__device__ __host__ bool	operator==(const rvec2 &a) {//一致 
		return _equal<>(a.l, l) && _equal<>(a.r, r);
	}
	__device__ __host__ rvec2 operator-(const rvec2 &a)	const {
		//極座標同士の引き算は、一度直交座標に直す必要がある。
		vec2	v1(*this);		//
		vec2	v2(a);			//
		vec2	v = v2 - v1;	//
		rvec2  ans = v.to_rvec2();
		//一行にまとめると
		{
			rvec2 debug_ans(vec2(a) - vec2(*this));
			_Assert(debug_ans == ans, "rvec2::operator+() is failed");
		}

		return ans;
	}
	__device__ __host__ rvec2 operator+(const rvec2 &a)	const {
		vec2	v1(*this);		//
		vec2	v2(a);			//
		vec2	v = v2 + v1;	//
		rvec2  ans = v.to_rvec2();
		//①行にまとめると
		{
			rvec2 debug_ans(vec2(*this) + vec2(a));
			_Assert(debug_ans == ans, "rvec2::operator+() is failed");
		}
		return ans;
//		rvec2 v(l + a.l, r + a.r);	return v; 
	}
	__device__ __host__ rvec2 operator+=(const rvec2& a) {		return operator=( *this + a);	}
	__device__ __host__ bool	operator==(const rvec2 &a)	const { return (_equal<float>(a.l, l) && _equal(a.r, r)); }


	//ベクトルをセットする。現在の速度を考慮して
	__device__ __host__ rvec2 move(const rvec2 &a, float ms = 1.0) {		//ある時間を掛けてその位置まで行く
		//現在の速度で指示された時間で進む距離をまず進める。
		if (_gpuAssert(ms > 0, "vec2::move() : ms is 0") != true) { return *this; }	//アサートの場合なにもしないで返します。
		rvec2 old_this(*this);	//自分のバックアップをとっておきます。
		operator=(a);	//まず位置を書き換えてしまいます。
		//速度は、等速で移動した距離＋足りない距離分加速します。

		//加速度は、とりあえず角度の速度だけを考慮します。
		//今回の速度です。
		// 
		//r - this_this-r =  Δrad は、ms分の進み量。速度 [rad/sec]に直すには、 Δrad / (ms/1000) ※ ms=1のとき、Δrad * 1000 となる

		d.spd = (r - old_this.r) / (ms/1000);		//Δr / Δt     =>    ((r-old_this.r)*1000) / (ms*1000) mm/sec
		//↑この時点で、rad/secとなっている。
		// 
		//加速度は、上記で計算した	d.spd [rad/sec]  - old_this.rad[rad/sec] = Δspd　[rad/sec] ※msあたり　のため、
		//	rad/sec・sec(1secでの加速度)にするには、 Δspd / (ms / 1000)　※
		d.acc = (d.spd - old_this.d.spd) / (ms / 1000);			

//		d.acc = (r - old_this.r) / ms * 1000.0 - old_this.d.spd;		//現在の速度に足す速度が加速度です。(rad/s^2)
//		d.spd = old_this.d.spd + d.acc;		//現在の速度に加速度を足す。
//		rvec2 test =r vec2(*this - old_this);
		return *this;
	}
	//止める。
	__device__ __host__ void stop(void) {
		d.stop();
	}
};

//vec2+角度を加えた3次元。		アームの場合は、座標＋先端の姿勢を表す。（座標が同じで、先端の姿勢だけ違うことがあり得る）
//								間接の場合は、座標とX軸となす角を表す（座標と角度は一意に決まるので注意）
#define ADD_INF_SZ	32	//付加情報のサイズです。
//回転軸の座標の定義です。
struct _cood : public vec2
{
	float	rad;
	spd_acc	wd;				//角度に関する速度と各速度に一応しておきます。

	//付加情報です。処理中に都合のよい値を保存しておけるようにしておきます。
	//int型のほうがデバッグ時にわかりやすいのでこうしておきます。doubleが指示されても
	 alignas(8)	int	additioal_inf[ADD_INF_SZ/sizeof(int)];

    __device__ __host__ _cood():vec2(),rad(0.0){;}
	__device__ __host__ _cood(vec2 v,float _rad,spd_acc _wd=spd_acc() ):vec2(v),rad(_rad), wd(_wd) {;}
#if 1		//debug デストラクタよびだし
	__device__ __host__ ~_cood() {
		wd.acc = 1;	//ダミー処理です。デバッグのため
	}
#endif

	//operator
	__device__ __host__ _cood operator-(const _cood a)	const	{
		_cood r(vec2::operator-(a),rad-a.rad);
		return r;
	}
	__device__ __host__ _cood operator+(const _cood a)const{
		_cood r(vec2::operator+(a),rad+a.rad);		//角度は足します。
		return r;
	}
	__device__ __host__ _cood & operator=(const _cood&a){
		vec2::operator=(a);
		rad	=	a.rad;
		wd	=	a.wd;
		memcpy((void*)additioal_inf, (void*)a.additioal_inf, ADD_INF_SZ);
		return *this;
	}
	__device__ __host__ _cood operator+=(const _cood &a){
		return operator=(operator+(a));
	}
	//operator==では、速度の評価をしないようにします。
	__device__ __host__ bool operator==(const _cood &a) const{
		if (vec2::operator==(a)) {
			return (rad == a.rad);		//位置と、速度、加速度
		}
		//
	}
	__device__ __host__ bool operator!=(const _cood &a)const{
		return !operator==(a);
	}

	//スカラと速度
	//その位置まで動作させるという意味の命令を作ったほうがいいと思いました。
	__device__ __host__  _cood &	move(const _cood &a , float ms) {
		//ベクトル部分の計算です。	
		_cood old_this = *this;	//自分を保存しておきます。
		vec2::move(a);		//x,y部分を動かす。

		//==================================
		//	姿勢部分の角速度の計算です。
		//==================================
		//角度、角速度、各加速度
		//これも変位から考えていきます。
//		d.spd = (r - old_this.r) / (ms / 1000);		//Δr / Δt     =>    ((r-old_this.r)*1000) / (ms*1000) mm/sec
		//加速度です。
//		d.acc = (d.spd - old_this.d.spd) / (ms / 1000);			//
		rad = a.rad;			//姿勢を代入します。
		wd.spd = ( rad - old_this.rad)			/	(ms / 1000.0);		//変位です。
		wd.acc = ( wd.spd - old_this.wd.spd)	/	(ms / 1000.0);		//速度の差です。
				//^^^^^^^^^^^^^^^^^^^^^^
				//		sec あたりのrad増加量です 1msあたりに直した後1000倍すると1secあたり進む量になる
				//		現在の速度との差分が加速度です。
//		wd.spd	+= wd.acc;		//加速度です。
//		_cood _b(a - *this);
		//付加情報はそのままコピーします。
		memcpy((void*)additioal_inf, (void*)a.additioal_inf, ADD_INF_SZ);
		return *this;
	}
	__device__ __host__ void stop(void){
		vec2::stop();
		wd.stop();
	}
	//付加情報に簡単にアクセスできるような関数を作ります。
	template<class T>
	__device__ __host__
	void write_add_info(const T &a){
		_gpuAssert(sizeof(T) < ADD_INF_SZ, "write_add_info() : size is too large");
		memcpy((void*)&additioal_inf[0], &a, sizeof(T));
	}
	template<class T>
	__device__ __host__
	T &read_add_info(void)const{
		_gpuAssert(sizeof(T) < ADD_INF_SZ, "read_add_info() : size is too large");
		return *(T*)&additioal_inf[0];
	}
};
//rad <--> deg
float	_deg(float rad);
float	_rad(float deg);


//===================================================================================================
// 複数個の格納データです。	cuda対応
//===================================================================================================
template<class T , std::size_t N>
struct mlti {
	T v[N];
	__device__ __host__ T& operator[](int idx) {
		if (!(idx < N)) { printf("mlt_v::operator()illegal idx");		return v[0]; }	//とりあえず0番目を返してしまう。
		return v[idx];
	}
};
//3要素のベクトルをeigenのように表してみる。
template<typename T=float>
struct _Vector3d : mlti<T , 3>
{
	_Vector3d() {	for (int i = 0; i < 3; ++i) { v[i] = 0.0; } }	//とりあえず初期化できるか
	__host__ __device__ T& operator()(int idx) {
		if (!(idx < 3)) {			printf("vector3d::operator()illegal idx");		return v[0];		}	//とりあえず0番目を返してしまう。
		return v[idx];	//これで参照が返るか
	}
	__host__ __device__ T& operator[](int idx) {		return operator()(idx);	}
	//dump
//	__host__ __device__ 
};



//============================================================================
//	vec2 imprement
//============================================================================
//__device__ __host__ inline  vec2::vec2(const rvec2& a) {		operator=(a.to_vec2());	}						//極座標を変換したもの
__device__ __host__  inline vec2::vec2(const rvec2& a) {		operator=(a.to_vec2());	}						//極座標を変換したもの

//直交座標への変換
__host__ __device__ vec2 inline rvec2::to_vec2(void) const
{
	vec2	v(l * cos(r), l * sin(r));
	//速度加速度の要素dも変換する。こと。
	return v;

}

//============================================================================
//	rvec2 imprement
//============================================================================
//__host__ __device__ rvec2::rvec2(const vec2& a) {	operator = (a.to_rvec2());}
__host__ __device__ rvec2	vec2::to_rvec2(void)const
{
	rvec2	v(scalar(), atan2(y, x));
	return v;
}

__host__ __device__ inline  float	_deg(float rad) { return	rad * 180.0 / PI; }
__host__ __device__ inline  float	_rad(float deg){	return	deg * PI / 180.0;}

#endif	//__UNITS__
