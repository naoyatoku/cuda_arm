//---------------------------------------------------------------------------
//	CommonModule.h
//	�ėp�I�Ȋ֐��Q
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
//	�_���v
//===============================================================================================
//============================================================================================
//		�����񑀍�֘A
//============================================================================================
//-------------------------------------------------------------
//	�w�肳�ꂽ�������A�w�肳�ꂽ������̒��Ɋ܂܂��
//	�����ƈ�v���邩�ǂ���
//-------------------------------------------------------------
bool matchStr(char c , const char *str);

//-------------------------------------------------------------
//	�f���~�^��������
//	����
//		const char *buf		:	�e�L�X�g�o�b�t�@�iNULL�I�[�K�v�j
//		const char *delm	:	�f���~�^������
//	�߂�l
//		�w��f���~�^(������̂����ǂꂩ�ꕶ��)�������������_�̃|�C���^��Ԃ��܂��B
//		�����f���~�^�������炩�����ꍇ��NULL�|�C���^��Ԃ��܂��B
//-------------------------------------------------------------
char *findDelm(const char *buf , const char *delm);

//-------------------------------------------------------------
//	�t����f���~�^��T��
//-------------------------------------------------------------
char *findDelm_rev(const char *buf , const char *delm);

//-------------------------------------------------------------
//	�f���~�^���Ƃ��肷����
//	����
//		const char *buf		:	�e�L�X�g�o�b�t�@�iNULL�I�[�K�v�j
//		const char *delm	:	�f���~�^������
//	�߂�l
//		�w��f���~�^�łȂ���������������|�C���^��Ԃ��܂�
//-------------------------------------------------------------
char *overDelm(const char *buf , const char*delm);

//�f���~�^���Ƃ��肷����(������̍Ōォ��t�ɒT���o�[�W�����ł�)
char *overDelm_rev(const char *buf , const char*delm);


//-------------------------------------------------------------
//	�f���~�^�������Ă���ɒʂ�߂����ꏊ�̃|�C���^��Ԃ�
//-------------------------------------------------------------
char *findoverDelm(const char *buf , const char *delm);

//-------------------------------------------------------------
//	�A�X�L�[�����ȊO�̕�������΂��܂��B
//-------------------------------------------------------------
char *overNoAscii( char *buf );

//-------------------------------------------------------------
//	�R�����g�A�E�g����
//-------------------------------------------------------------
char *CommentOut(char *buf , const char *s , const char *e);

//-------------------------------------------------------------
//	�N�I�[�e�[�V�������͂����ĕ��������蒼���܂�
//-------------------------------------------------------------
void removeQuotation( char *str , char quotation );


//-------------------------------------------------------------
//	�o�b�t�@����A�f���~�^�܂œǂݍ���
//	�Ƃɂ����f���~�^�܂œǂ݂܂��B
//
//	�߂�l�F�R�s�[��̃R�s�[���ƃo�b�t�@�̕�����̈ʒu�i�|�C���^�j
//-------------------------------------------------------------
char *getElem(const char*src , char*dst , const char *delm,int dstmax=0);

//-----------------------------------------------------------------------
//	�_�u���N�H�[�e�[�V�����̒��g�ł̓f���~�^�̃`�F�b�N�����Ȃ�getElem
//-----------------------------------------------------------------------
char *getElem_withoutQuotation(const char*src , char*dst ,const char *delm ,char *quote , int dstmax=0 );

//-------------------------------------------------------------
//	������̒u��
//	src������̒��g��org_str�Ɠ������̂�T����to_str�ɒu��������dst�ɂ��܂��܂��B
//-------------------------------------------------------------
void Replace(char *src ,int src_size , char *org_str ,char *to_str);




//-------------------------------------------------------------------------------
//	printf�ł��B
//	debugview�ɏo�͂��܂��B
//	Release�r���h�ł����o�Ȃ��݂����ł��B
//-------------------------------------------------------------------------------
#include <math.h>

#define		M_PI			(3.1415)
//������ƕςł��Bactivate.h�ŎQ�Ƃ��Ă��܂��Ă���B
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
//	�A�T�[�g
//-------------------------------------------------------------------------------------
typedef	void(*_LOG_CALLBACK)(void);
void log_init(_LOG_CALLBACK pcallback=0);
const char*get_log(int idx);
void write_log(const char*fmt, ...);


void _Assert_log(bool a, const char* fmt, ...);	//csv�ɋL�^����^�C�v�B

__device__ __host__
void _Assert(bool a, const char*fmt ,...);      //������
__device__ __host__ inline 
bool _gpuAssert(bool a, const char* fmt, ...)
{
//	va_list ap;	va_start(ap, fmt);		//�����WINDOWS���������B�B�B
	if (!a) {		//cuda�ŉό������킩��Ȃ�
        printf("==== assert =====[%s] \r\n" , fmt);       

//cuda��__device__�R�[�h�ł͈ȉ����g���Ȃ��B�E�E�E�Ⴄ���������Ȃ��Ƃ����Ȃ�
//        va_list ap; va_start(ap, fmt);
//		vprintf(fmt,ap);
//      va_end(ap);
        for (;;) {  //�Ƃ߂��Ⴂ�܂��B
            ;
        }
	}
//	va_end(ap);
	return a;
}

//char* LastErrorMsg(void);

//======================================================================================
//  ���������_�֘A
//======================================================================================
template<class T = double>
__device__ __host__ inline bool _equal(T a, T b,double accuracy=0.00001)
{
	bool constexpr s = std::is_same<double, T>::value;
	if (std::is_same<double, T>::value || std::is_same<float, T>::value)
    {		//�ϐ���double�Ȃ��
		//const T d = 0.0000001;		//������1/1000000�̂Ƃ��ɓ����Ƃ���B
		return fabs(a - b) < accuracy;
	}
	return a == b;
}


//�l��������x�Ɋۂ߂�悤�Ȋ֐��ł��B(�ʒP�ʂ�)
#include <cmath>
__device__ __host__ inline double _round(double a,double accuracy=5){
//	a = 123.56789123;
	a = std::floor(a * std::pow(10.0, accuracy)) / std::pow(10.0, accuracy);
	return a;
}


//(device ���\�\��
int device_query();


//CPU,GPU�̎��
//======================================================================================
//	�������Ǘ�
//======================================================================================
enum {
    CPU,GPU
};


//CPU��GPU�����ɑΉ������A�h���X��\���^������Ă����܂��B
template<typename T>
class cpu_gpu_mem
{
public:
    T* cpu;
    T* gpu;
    int size;
    cpu_gpu_mem() : cpu(nullptr),gpu(nullptr){  //�f�t�H���g�ň�ł���悤�ɂ��܂��B
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
        //�ēx�A���P�[�g���悤�Ƃ����ꍇ�A��x������܂��B
        free();
//      _Assert((cpu == nullptr) && (gpu == nullptr), "already memory allocated");
        //CPU
        try {
            cpu = new T[sz];       //���f�t�H���g�R���X�g���N�^�ō��܂��B
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
    //cpu��gpu�֓]������B
    __host__ T* Transfer_to_GPU() {
        //�����Ńk���|�C���^���`�F�b�N���܂��B
        if (!cpu && !gpu) {
            return gpu;//
        }
        _Assert(cpu && gpu, "Tranfser_to_GPU nullpointer");
        cudaError_t cudaStatus = cudaMemcpy((void*)gpu, (void*)cpu, size * sizeof(T), cudaMemcpyHostToDevice); //GPU�֓]������B
        _Assert(cudaStatus == cudaSuccess, "Transfer_to_GPU() error");
        return gpu;
    }
    //�����Ė��߂��o����悤�ɁA�����̎Q�Ƃ�Ԃ�
    __host__ T* Transfer_to_CPU() {
        if (!cpu && !gpu) {
            return cpu;
        }
        _Assert(cpu && gpu, "Tranfser_to_CPU nullpointer");
        cudaError_t cudaStatus = cudaMemcpy((void*)cpu, (void*)gpu, size * sizeof(T), cudaMemcpyDeviceToHost); //GPU�֓]������B
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

    //()�ŃA�h���X���擾�ł���Ɠǂ݂₷���Ȃ邩
    __host__ __device__ T& operator()(int location,int  idx)const{
        _Assert(idx < size , "cpu_gpu_mem::() size illegal");
        if(location == GPU){
            return gpu[idx];
        }
        return cpu[idx];
    }
    __host__ __device__ T& operator()(int location=CPU) {  //�Ȃɂ��Y�������w�����Ȃ��ꍇ��0�Ԗڂ�
        return operator()(location, 0);
    }
    //�����������A���P�[�g���Ă��܂��Ă����炻��͍폜���܂��B
    __host__
    cpu_gpu_mem<T>& operator=(const cpu_gpu_mem<T>& a){
        if (this == &a) return *this; // ���ȑ���`�F�b�N
        free(); // �����̃����������
        size = a.size;
        if (size > 0) {
            // �V�������������m�ۂ��ăR�s�[
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
    //vector::back()�Ɠ����֐�
    __device__ __host__
    T& back(int location=CPU) const {
         return operator()(location, size - 1);
    }
};

//======================================================================================
//		�G�X�P�[�v�V�[�P���X
//======================================================================================
#define print_pos(x,y)  printf("\x1B[%d;%dH", y, x)
#define esc_clr()       printf("\x1B[2J")


//======================================================================================
//  rad->degree
//======================================================================================
#define RAD2DEG(rad)        (rad*180/M_PI)


#endif	//_COMMONMODULE_H_

