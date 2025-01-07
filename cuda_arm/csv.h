//------------------------------------------------------------------------------------------------------------
//	csvファイルに関する操作です。
//------------------------------------------------------------------------------------------------------------
#ifndef		_CSVFILE_H_
#define		_CSVFILE_H_

#include <Windows.h>
#include "commonmodule.h"

//toku とりあえずない関数を stub
//void _Assert(bool a, const char*fmt, ...) { ; }

//※ホスト専用コードです

//複数ファイルに対応したくなったのでクラスにします。
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
	//コンストラクタ/デストラクタ
	csv();	csv(const char*path,bool auto_open=false);
	~csv();
	bool		avail(void);	//モジュールが有効かどうか
	const char*		err(void);		//モジュールが無効な場合に理由(LastError)を返す。

	//データ操作
	void	write(const char*data , const char*title=0);
	int		search(int colmn, const char*key);	//指定カラムで指定キーに一致するものを探して、見つかればその行番号を返す
	bool	get(int row,int col , char *buf , int buf_max);
	int		get_n(int row,int col , int exception_default=0);
	//フォーマット付きでかく
	void	writef(const char*fmt , ...);
	//operator=意味ないかもしれないけど、・・・
	const char*path(void)const { return _path; }
	const char*err(void)const { return _err; }
	//バイナリで書けるようにします。
	void	bin_write(void *data, int size);
	template<typename T>
	void	bin_write(const T&v){
		bin_write((void*)&v , sizeof(T));
	}

	//同じスレッド・コンテキストで行うようにする必要がある（ハンドルが共通で使える）
	csv &operator=(const csv&a) {
		_hcsv = a._hcsv;
		_path = a.path();
		_err = a.err();
		return *this;
	}


};


#endif		//_CSVFILE_H_
