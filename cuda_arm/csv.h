//------------------------------------------------------------------------------------------------------------
//	csv�t�@�C���Ɋւ��鑀��ł��B
//------------------------------------------------------------------------------------------------------------
#ifndef		_CSVFILE_H_
#define		_CSVFILE_H_

#include <Windows.h>
#include "commonmodule.h"

//toku �Ƃ肠�����Ȃ��֐��� stub
//void _Assert(bool a, const char*fmt, ...) { ; }

//���z�X�g��p�R�[�h�ł�

//�����t�@�C���ɑΉ��������Ȃ����̂ŃN���X�ɂ��܂��B
class csv
{
protected:
	HANDLE	_hcsv;
	void	_open(void);
	void	_close(void);
	bool	_one_sentence(char  *buf , int bufmax );
	bool	_getstr(int n , const char*src ,char *dst , int dstmax);
	//
	const char*	_path;
	const char*	_err;

public:
	//�R���X�g���N�^/�f�X�g���N�^
	csv();	csv(const char*path,bool auto_open=false);
	~csv();
	bool		avail(void);	//���W���[�����L�����ǂ���
	const char*		err(void);		//���W���[���������ȏꍇ�ɗ��R(LastError)��Ԃ��B

	//�f�[�^����
	void	write(const char*data , const char*title=0);
	int		search(int colmn, const char*key);	//�w��J�����Ŏw��L�[�Ɉ�v������̂�T���āA������΂��̍s�ԍ���Ԃ�
	bool	get(int row,int col , char *buf , int buf_max);
	int		get_n(int row,int col , int exception_default=0);
	//�t�H�[�}�b�g�t���ł���
	void	writef(const char*fmt , ...);
	//operator=�Ӗ��Ȃ���������Ȃ����ǁA�E�E�E
	const char*path(void)const { return _path; }
	const char*err(void)const { return _err; }
	//�o�C�i���ŏ�����悤�ɂ��܂��B
	void	bin_write(void *data, int size);
	template<typename T>
	void	bin_write(const T&v){
		bin_write((void*)&v , sizeof(T));
	}

	//�����X���b�h�E�R���e�L�X�g�ōs���悤�ɂ���K�v������i�n���h�������ʂŎg����j
	csv &operator=(const csv&a) {
		_hcsv = a._hcsv;
		_path = a.path();
		_err = a.err();
		return *this;
	}


};


#endif		//_CSVFILE_H_
