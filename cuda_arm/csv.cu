//---------------------------------------------------------------------------------------------------------------------------
//	CSVファイルに関する操作です
//----------------------------------------------------------------------------
#include <windows.h>
#include <stdio.h>
#include "csv.h"



//----------------------------------------------------------------------------
//	local defines
//----------------------------------------------------------------------------
#define	DEFAULT_PATH	".\\CSV\\DEFAUTL.CSV"
#define CR	0x0d
#define LF	0x0a

//----------------------------------------------------------------------------
//	globals
//----------------------------------------------------------------------------
static const char* LastErrorMsg(void) { return ""; }


//----------------------------------------------------------------------------
//	constructor/destructor
//----------------------------------------------------------------------------
csv::csv() : csv(DEFAULT_PATH, false) { ; }
csv::csv(const char* path, bool auto_open)
{
	_hcsv = INVALID_HANDLE_VALUE;
	_path = path;
	_err = "";
	if (auto_open)	_open();	//コンストラクタにて自動的にオープンしてしまう指示の場合はここでオープンします。
}
csv::~csv() {
	//_printf("csv;path[%s] DESTRUCTOR ====> close() call" , _path);
	_close();
}


//----------------------------------------------------------------------------
//	imprement
//----------------------------------------------------------------------------
//CSVへ追記する
void	csv::bin_write(void*data, int size)
{
	DWORD _written;
	_Assert( avail() ,"bin_write():file not opened");		//アサートするように
	//追記するようにします。
	if (::SetFilePointer(_hcsv, 0, NULL, FILE_END) == 0xFFFFFFFF) { _Assert(0, "[seek file(end)] [h=%d ] error:%s", _hcsv, LastErrorMsg()); }
	if (::WriteFile(_hcsv, data,size, &_written, 0) != TRUE) 	_Assert(0, "[writefile] [h=%d ] error:%s", _hcsv, LastErrorMsg());
}
void	csv::write(const char* data, const char* title)
{
	//ここは無視するようにしておきます。
	if (!avail()) {
		return;		//なにもしないで返す
	}
	_Assert(avail(), "csv(%s)::write file not open", _path);		//オープンしてないときにアサートしてしまいます

	DWORD _written;

	//もしファイルサイズが0でタイトル指定がある場合はそれをまず書き込みマス。
	if (title) {
		if (GetFileSize(_hcsv, NULL) == 0) {
			if (::WriteFile(_hcsv, title, strlen(title), &_written, 0) != TRUE) 	_Assert(0, "[addCSv:write title] [h=%d ] error:%s", _hcsv, LastErrorMsg());
		}
	}
	bin_write((void*)data,strlen(data));
/*	//一番最後へファイルポインタを移動します。
	if (::SetFilePointer(_hcsv, 0, NULL, FILE_END) == 0xFFFFFFFF) { _Assert(0, "[seek file(end)] [h=%d ] error:%s", _hcsv, LastErrorMsg()); }
	//書き込みします。
	if (::WriteFile(_hcsv, data, strlen(data), &_written, 0) != TRUE) 	_Assert(0, "[writefile] [h=%d ] error:%s", _hcsv, LastErrorMsg());
*/
}
//CSVから探す
//	引数
//		int colmn			カラム番号	0～
//		const char*	key		サーチキー
//	戻り値
//		一致した場合の行番号
//		一致しない場合は-1を返します。
int	csv::search(int colmn, const char* key)
{
	//_printf("csv: search in ! key=%s" , key);
	if (!avail())	return -1;	//モジュールが有効でない

	if (::SetFilePointer(_hcsv, 0, NULL, FILE_BEGIN) == 0xFFFFFFFF) { _Assert(0, "[seek file(begin)] [h=%d ] error:%s", _hcsv, LastErrorMsg()); }

	//1行がとても大きくなってしまうのでバッファを大きく取ります。(staticメモリにします)
	static	char sentence[4096];
	{
		int idx;
		for (idx = 0; (_one_sentence(sentence, sizeof(sentence)) == true); idx++) {	//1行読み出せている限り
			char s[256];	_getstr(colmn, sentence, s, sizeof(s));
			if (strcmp(key, s) == 0) {
				//_printf("csv: search found ! key=%s" , key);
				return idx;
			}
		}
	}
	//_printf("csv: search not found ! key=%s" , key);
	return	-1;
}

//指示された行・列の要素を取り出します。
bool	csv::get(int row, int col, char* buf, int buf_max)
{
	if (!avail())	return false;	//モジュールが有効でない

	if (::SetFilePointer(_hcsv, 0, NULL, FILE_BEGIN) == 0xFFFFFFFF) { _Assert(0, "[seek file(begin)] [path=%s] [h=%d ] error:%s", _path, _hcsv, LastErrorMsg()); }

	static	char sentence[4096];
	{
		//該当行の取得です。
		int r;	for (r = 0; r <= row; r++) {
			if (_one_sentence(sentence, sizeof(sentence)) != true) {
				return	false;
			}
		}
	}
	//該当カラムのデータを取り出します。このバッファを返しますので、非同期では使えません。
	return	_getstr(col, sentence, buf, buf_max);
}
//整数として該当行列・列の要素を返します。要素が無い場合は指示されたデフォルト値を返します。
int	csv::get_n(int row, int col, int exception_default/*=0*/)
{
	char	str_val[128];	//128
	return	(get(row, col, str_val, sizeof(str_val)) == true) ? atoi(str_val) : exception_default;
}
//----------------------------------------------------------------------------
//	(protected member)
//----------------------------------------------------------------------------
void	csv::_open(void)
{
	if (_hcsv != INVALID_HANDLE_VALUE) {
		//		_printf("csv::open() file((%s) is alwais opened" , _path);
		return;	//既にオープンされている
	}

	this->_hcsv = ::CreateFileA(
		_path,	//ファイル名
		GENERIC_WRITE | GENERIC_READ,	//読み込みオープン
		FILE_SHARE_READ,	//共有モード：読み込みは許可
		NULL,	//子プロセスにハンドルの継承を許すか（NULL:許さない）
		OPEN_ALWAYS,	//新しいファイルを作成します。指定ファイルがすでに存在している場合、ファイルを作ります
		FILE_ATTRIBUTE_NORMAL,	//ファイル属性：属性なし
		NULL);
	//オープン失敗
	//オープン失敗で、エラーにするのをやめます。
	if (_hcsv == INVALID_HANDLE_VALUE) {
		DWORD e = ::GetLastError();
		printf("");
	}
	_Assert(_hcsv != INVALID_HANDLE_VALUE, "[_opencsv] FileOpen error %s [%s]\n", _path, LastErrorMsg());
	if (_hcsv == INVALID_HANDLE_VALUE) {
		_err = LastErrorMsg();	//オープン時のエラーを保存しておきます。
		//		_printf("csv: open error [%s] --[%s] " , _path , _err );	
	}
	else {
		//		_printf("csv: open Succeed [%s]" , _path);
	}
}
void	csv::_close(void)
{
	if (_hcsv != INVALID_HANDLE_VALUE) {
		::CloseHandle(_hcsv);	_hcsv = INVALID_HANDLE_VALUE;
	}
}
//Csvファイルを走査して、該当カラムの値を取ってきます。
//-------------------------------------
//	一行読み出し。
//-------------------------------------
bool	csv::_one_sentence(char* buf, int bufmax)
{
	if (!avail())	return false;	//モジュールが有効でない

	int i = 0;
	for (i = 0; ; i++) {
		DWORD	r;
		_Assert(i < bufmax, "[one_sentence]buffer overflow");
		if (::ReadFile(_hcsv, (void*)&buf[i], 1, &r, NULL) == 0) goto error;	//0で失敗
		if (r != 1)	goto end_of_file;										//成功だけど読み込みが0バイトはファイルの終わりです。

		if (buf[i] == CR || buf[i] == LF) {
			::ReadFile(_hcsv, (void*)&buf[i], 1, &r, NULL);						//改行を飛ばす処理です。
			buf[i] = '\0';													//改行をヌルに変更して終了
			//_printf("[OneSentence]buf=[%s]\r\n" , buf);
			return true;
		}
	}
error:
	return false;
end_of_file:
	buf[i] = '\0';
	return	(strlen(buf) > 0);
}
//1行分の文字列から、カンマ区切りされている文字列を抽出して返します
bool	csv::_getstr(int n, const char* src, char* dst, int dstmax)
{
	const char* p;
	{	int i;	for (i = 0, p = src; i < n; i++)	if ((p = findoverDelm(p, ",")) == NULL)	goto error;	}//文字列のはじめです。
	//文字列をコピーしていきます
	{
		int i, j;
		for (i = j = 0; ((p[i] != ',') && (p[i] != '\0')); i++) {
			if (p[i] == ' ' || p[i] == '\t') continue;	//ホワイトスペースはとりのぞいて・・
			_Assert(i < dstmax, "[_getstr]dst buf overflow");	//toku 使いにくかったらboolで返すバージョンも作る
			dst[j++] = p[i];
		}
		dst[j] = '\0';
	}
	return true;

error:
	dst[0] = '\0';	return false;	//カラ文字列です
}
bool	csv::avail(void) { return	(_hcsv != INVALID_HANDLE_VALUE); }
const char* csv::err(void) { return	_err; }

void	csv::writef(const char* fmt, ...)
{
	char _buf[32 * 1024];						//これ用のバッファです。非同期があるとやばいです。
	va_list ap;	va_start(ap, fmt);		//これはWINDOWSだけだが。。。
	vsprintf_s(_buf, sizeof(_buf), fmt, ap);
	va_end(ap);
	write(_buf);
}
