//---------------------------------------------------------------------------
//	CommonModule.h
//	汎用的な関数群
//---------------------------------------------------------------------------
#ifndef _COMMONMODULE_H_
#define _COMMONMODULE_H_
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <type_traits>
#include <stdio.h>
#include <math.h>

#include <new>  //
#include <stdarg.h>
#include <algorithm>
using namespace std;
//===============================================================================================
//	defines
//===============================================================================================

//===============================================================================================
//	apis
//===============================================================================================
//===============================================================================================
//	ダンプ
//===============================================================================================
//============================================================================================
//		文字列操作関連
//============================================================================================
//-------------------------------------------------------------
//	指定された文字が、指定された文字列の中に含まれる
//	文字と一致するかどうか
//-------------------------------------------------------------
bool matchStr(char c , const char *str);

//-------------------------------------------------------------
//	デリミタを見つける
//	引数
//		const char *buf		:	テキストバッファ（NULL終端必要）
//		const char *delm	:	デリミタ文字列
//	戻り値
//		指定デリミタ(文字列のうちどれか一文字)が見つかった時点のポインタを返します。
//		もしデリミタが見つからかった場合はNULLポインタを返します。
//-------------------------------------------------------------
char *findDelm(const char *buf , const char *delm);

//-------------------------------------------------------------
//	逆からデリミタを探す
//-------------------------------------------------------------
char *findDelm_rev(const char *buf , const char *delm);

//-------------------------------------------------------------
//	デリミタをとおりすぎる
//	引数
//		const char *buf		:	テキストバッファ（NULL終端必要）
//		const char *delm	:	デリミタ文字列
//	戻り値
//		指定デリミタでない文字列を見つけたポインタを返します
//-------------------------------------------------------------
char *overDelm(const char *buf , const char*delm);

//デリミタをとおりすぎる(文字列の最後から逆に探すバージョンです)
char *overDelm_rev(const char *buf , const char*delm);


//-------------------------------------------------------------
//	デリミタを見つけてさらに通り過ぎた場所のポインタを返す
//-------------------------------------------------------------
char *findoverDelm(const char *buf , const char *delm);

//-------------------------------------------------------------
//	アスキー文字以外の文字列を飛ばします。
//-------------------------------------------------------------
char *overNoAscii( char *buf );

//-------------------------------------------------------------
//	コメントアウト処理
//-------------------------------------------------------------
char *CommentOut(char *buf , const char *s , const char *e);

//-------------------------------------------------------------
//	クオーテーションをはずして文字列を作り直します
//-------------------------------------------------------------
void removeQuotation( char *str , char quotation );


//-------------------------------------------------------------
//	バッファから、デリミタまで読み込み
//	とにかくデリミタまで読みます。
//
//	戻り値：コピー後のコピーもとバッファの文字列の位置（ポインタ）
//-------------------------------------------------------------
char *getElem(const char*src , char*dst , const char *delm,int dstmax=0);

//-----------------------------------------------------------------------
//	ダブルクォーテーションの中身ではデリミタのチェックをしないgetElem
//-----------------------------------------------------------------------
char *getElem_withoutQuotation(const char*src , char*dst ,const char *delm ,char *quote , int dstmax=0 );

//-------------------------------------------------------------
//	文字列の置換
//	src文字列の中身でorg_strと同じものを探してto_strに置き換えてdstにしまいます。
//-------------------------------------------------------------
void Replace(char *src ,int src_size , char *org_str ,char *to_str);




//-------------------------------------------------------------------------------
//	printfです。
//	debugviewに出力します。
//	Releaseビルドでしか出ないみたいです。
//-------------------------------------------------------------------------------
#include <math.h>

#define		M_PI			(3.1415)
//ちょっと変です。activate.hで参照してしまっている。
#define     _MAX_UNITS      1024        //128   units each layer max
static double rand_uniform(double a, double b) {
	double x = (rand() + 1.0) / ((double)RAND_MAX + 2.0);
	return (b - a) * x + a;
}
template<class T>
static double rand_normal(T mu, T sigma) {
	T z = (T)sqrt(-2.0 * log(rand_uniform(0.0, 1.0))) * sin(2.0 * M_PI * rand_uniform(0.0, 1.0));
	return mu + sigma * z;
}

//-------------------------------------------------------------------------------------
//	アサート
//-------------------------------------------------------------------------------------
typedef	void(*_LOG_CALLBACK)(void);
void log_init(_LOG_CALLBACK pcallback=0);
const char*get_log(int idx);
void write_log(const char*fmt, ...);


void _Assert_log(bool a, const char* fmt, ...);	//csvに記録するタイプ。

__device__ __host__
void _Assert(bool a, const char*fmt ,...);      //※実際
__device__ __host__ inline 
bool _gpuAssert(bool a, const char* fmt, ...)
{
//	va_list ap;	va_start(ap, fmt);		//これはWINDOWSだけだが。。。
	if (!a) {		//cudaで可変個引数がわからない
        printf("==== assert =====[%s] \r\n" , fmt);       

//cudaの__device__コードでは以下が使えない。・・・違うやり方をしないといけない
//        va_list ap; va_start(ap, fmt);
//		vprintf(fmt,ap);
//      va_end(ap);
        for (;;) {  //とめちゃいます。
            ;
        }
	}
//	va_end(ap);
	return a;
}

//char* LastErrorMsg(void);

//======================================================================================
//  浮動小数点関連
//======================================================================================
template<class T = double>
__device__ __host__ inline bool _equal(T a, T b,double accuracy=0.00001)
{
	bool constexpr s = std::is_same<double, T>::value;
	if (std::is_same<double, T>::value || std::is_same<float, T>::value)
    {		//変数がdoubleならば
		//const T d = 0.0000001;		//差分が1/1000000のときに同じとする。
		return fabs(a - b) < accuracy;
	}
	return a == b;
}


//値をある程度に丸めるような関数です。(μ単位で)
#include <cmath>
__device__ __host__ inline double _round(double a,double accuracy=5){
//	a = 123.56789123;
	a = std::floor(a * std::pow(10.0, accuracy)) / std::pow(10.0, accuracy);
	return a;
}


//(device 性能表示
int device_query();


//CPU,GPUの種類
//======================================================================================
//	メモリ管理
//======================================================================================
enum {
    CPU,GPU
};


//CPUとGPU両方に対応したアドレスを表す型を作っておきます。
template<typename T>
class cpu_gpu_mem
{
public:
    T* cpu;
    T* gpu;
    int size;
    cpu_gpu_mem() : cpu(nullptr),gpu(nullptr){  //デフォルトで一つできるようにします。
        alloc(1);
    }
    cpu_gpu_mem(int sz) :cpu_gpu_mem() {
        alloc(sz);
    }
    ~cpu_gpu_mem() {
        free();
    }
//    cpu_gpu_mem(T* _cpu, T* _gpu) : cpu(_cpu), gpu(_gpu) { ; }
    cpu_gpu_mem(const cpu_gpu_mem<T>&a) = default;

    T* _cpu()const { return cpu; }
    T* _gpu()const { return gpu; }
    int _size()const { return size; }

    template<typename S> operator cpu_gpu_mem<S> () const {
        return cpu_gpu_mem<S>((S*)cpu, (S*)gpu);
    }
//    cpu_gpu_mem<T>& operator=(const cpu_gpu_mem<T>& a) = default;
    //
    __host__ cpu_gpu_mem& alloc(int sz)
    {
        //再度アロケートしようとした場合、一度解放します。
        free();
//      _Assert((cpu == nullptr) && (gpu == nullptr), "already memory allocated");
        //CPU
        try {
            cpu = new T[sz];       //※デフォルトコンストラクタで作ります。
        }
        catch (const bad_alloc& e) {
            printf("new[] failed(%s)\n", e.what());
            _Assert(false, "cpu_gpu_mem : alloc failed "); 
        }
        //GPU
        cudaError_t e;
        if ((e = cudaMalloc((void**)&gpu, sz * sizeof(T))) != cudaSuccess) { _Assert(false, "cpugpu_alloc : cudaMalloc failed"); }
        size = sz;
        return *this;
    }
    //cpu→gpuへ転送する。
    __host__ T* Transfer_to_GPU() {
        //ここでヌルポインタをチェックします。
        if (!cpu && !gpu) {
            return gpu;//
        }
        _Assert(cpu && gpu, "Tranfser_to_GPU nullpointer");
        cudaError_t cudaStatus = cudaMemcpy((void*)gpu, (void*)cpu, size * sizeof(T), cudaMemcpyHostToDevice); //GPUへ転送する。
        _Assert(cudaStatus == cudaSuccess, "Transfer_to_GPU() error");
        return gpu;
    }
    //続けて命令を出せるように、自分の参照を返す
    __host__ T* Transfer_to_CPU() {
        if (!cpu && !gpu) {
            return cpu;
        }
        _Assert(cpu && gpu, "Tranfser_to_CPU nullpointer");
        cudaError_t cudaStatus = cudaMemcpy((void*)cpu, (void*)gpu, size * sizeof(T), cudaMemcpyDeviceToHost); //GPUへ転送する。
        _Assert(cudaStatus == cudaSuccess, "Transfer_to_CPU() error");
        return cpu;
    }
    __host__ void free()
    {
        if (cpu != nullptr) {
            delete[] cpu;
            cpu = nullptr;
        }
        if (gpu != nullptr) {
            cudaFree((void*)gpu);
            gpu = nullptr;
        }
        size=0;
    }
    __host__ __device__ void dump()const{printf("CPU[%p]GPU[%p]size[%d]\n",cpu, gpu , size); }

    //()でアドレスを取得できると読みやすくなるか
    __host__ __device__ T& operator()(int location,int  idx)const{
        _Assert(idx < size , "cpu_gpu_mem::() size illegal");
        if(location == GPU){
            return gpu[idx];
        }
        return cpu[idx];
    }
    __host__ __device__ T& operator()(int location=CPU) {  //なにも添え字を指示しない場合は0番目を
        return operator()(location, 0);
    }
    //もし自分がアロケートしてしまっていたらそれは削除します。
    __host__
    cpu_gpu_mem<T>& operator=(const cpu_gpu_mem<T>& a){
        if (this == &a) return *this; // 自己代入チェック
        free(); // 既存のメモリを解放
        size = a.size;
        if (size > 0) {
            // 新しいメモリを確保してコピー
            cpu = new T[size];
            std::copy(a.cpu, a.cpu + size, cpu);
            cudaMalloc((void**)&gpu, size * sizeof(T));
            cudaMemcpy(gpu, a.gpu, size * sizeof(T), cudaMemcpyDeviceToDevice);
        } else {
            cpu = nullptr;
            gpu = nullptr;
        }
        return *this;
    }
    //vector::back()と同じ関数
    __device__ __host__
    T& back(int location=CPU) const {
         return operator()(location, size - 1);
    }
};

//======================================================================================
//		エスケープシーケンス
//======================================================================================
#define print_pos(x,y)  printf("\x1B[%d;%dH", y, x)
#define esc_clr()       printf("\x1B[2J")


//======================================================================================
//  rad->degree
//======================================================================================
#define RAD2DEG(rad)        (rad*180/M_PI)


#endif	//_COMMONMODULE_H_

