//---------------------------------------------------------------------------------------------------------------------------
//	CSV�t�@�C���Ɋւ��鑀��ł�
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
	if (auto_open)	_open();	//�R���X�g���N�^�ɂĎ����I�ɃI�[�v�����Ă��܂��w���̏ꍇ�͂����ŃI�[�v�����܂��B
}
csv::~csv() {
	//_printf("csv;path[%s] DESTRUCTOR ====> close() call" , _path);
	_close();
}


//----------------------------------------------------------------------------
//	imprement
//----------------------------------------------------------------------------
//CSV�֒ǋL����
void	csv::bin_write(void*data, int size)
{
	DWORD _written;
	_Assert( avail() ,"bin_write():file not opened");		//�A�T�[�g����悤��
	//�ǋL����悤�ɂ��܂��B
	if (::SetFilePointer(_hcsv, 0, NULL, FILE_END) == 0xFFFFFFFF) { _Assert(0, "[seek file(end)] [h=%d ] error:%s", _hcsv, LastErrorMsg()); }
	if (::WriteFile(_hcsv, data,size, &_written, 0) != TRUE) 	_Assert(0, "[writefile] [h=%d ] error:%s", _hcsv, LastErrorMsg());
}
void	csv::write(const char* data, const char* title)
{
	//�����͖�������悤�ɂ��Ă����܂��B
	if (!avail()) {
		return;		//�Ȃɂ����Ȃ��ŕԂ�
	}
	_Assert(avail(), "csv(%s)::write file not open", _path);		//�I�[�v�����ĂȂ��Ƃ��ɃA�T�[�g���Ă��܂��܂�

	DWORD _written;

	//�����t�@�C���T�C�Y��0�Ń^�C�g���w�肪����ꍇ�͂�����܂��������݃}�X�B
	if (title) {
		if (GetFileSize(_hcsv, NULL) == 0) {
			if (::WriteFile(_hcsv, title, strlen(title), &_written, 0) != TRUE) 	_Assert(0, "[addCSv:write title] [h=%d ] error:%s", _hcsv, LastErrorMsg());
		}
	}
	bin_write((void*)data,strlen(data));
/*	//��ԍŌ�փt�@�C���|�C���^���ړ����܂��B
	if (::SetFilePointer(_hcsv, 0, NULL, FILE_END) == 0xFFFFFFFF) { _Assert(0, "[seek file(end)] [h=%d ] error:%s", _hcsv, LastErrorMsg()); }
	//�������݂��܂��B
	if (::WriteFile(_hcsv, data, strlen(data), &_written, 0) != TRUE) 	_Assert(0, "[writefile] [h=%d ] error:%s", _hcsv, LastErrorMsg());
*/
}
//CSV����T��
//	����
//		int colmn			�J�����ԍ�	0�`
//		const char*	key		�T�[�`�L�[
//	�߂�l
//		��v�����ꍇ�̍s�ԍ�
//		��v���Ȃ��ꍇ��-1��Ԃ��܂��B
int	csv::search(int colmn, const char* key)
{
	//_printf("csv: search in ! key=%s" , key);
	if (!avail())	return -1;	//���W���[�����L���łȂ�

	if (::SetFilePointer(_hcsv, 0, NULL, FILE_BEGIN) == 0xFFFFFFFF) { _Assert(0, "[seek file(begin)] [h=%d ] error:%s", _hcsv, LastErrorMsg()); }

	//1�s���ƂĂ��傫���Ȃ��Ă��܂��̂Ńo�b�t�@��傫�����܂��B(static�������ɂ��܂�)
	static	char sentence[4096];
	{
		int idx;
		for (idx = 0; (_one_sentence(sentence, sizeof(sentence)) == true); idx++) {	//1�s�ǂݏo���Ă������
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

//�w�����ꂽ�s�E��̗v�f�����o���܂��B
bool	csv::get(int row, int col, char* buf, int buf_max)
{
	if (!avail())	return false;	//���W���[�����L���łȂ�

	if (::SetFilePointer(_hcsv, 0, NULL, FILE_BEGIN) == 0xFFFFFFFF) { _Assert(0, "[seek file(begin)] [path=%s] [h=%d ] error:%s", _path, _hcsv, LastErrorMsg()); }

	static	char sentence[4096];
	{
		//�Y���s�̎擾�ł��B
		int r;	for (r = 0; r <= row; r++) {
			if (_one_sentence(sentence, sizeof(sentence)) != true) {
				return	false;
			}
		}
	}
	//�Y���J�����̃f�[�^�����o���܂��B���̃o�b�t�@��Ԃ��܂��̂ŁA�񓯊��ł͎g���܂���B
	return	_getstr(col, sentence, buf, buf_max);
}
//�����Ƃ��ĊY���s��E��̗v�f��Ԃ��܂��B�v�f�������ꍇ�͎w�����ꂽ�f�t�H���g�l��Ԃ��܂��B
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
		return;	//���ɃI�[�v������Ă���
	}

	this->_hcsv = ::CreateFileA(
		_path,	//�t�@�C����
		GENERIC_WRITE | GENERIC_READ,	//�ǂݍ��݃I�[�v��
		FILE_SHARE_READ,	//���L���[�h�F�ǂݍ��݂͋���
		NULL,	//�q�v���Z�X�Ƀn���h���̌p�����������iNULL:�����Ȃ��j
		OPEN_ALWAYS,	//�V�����t�@�C�����쐬���܂��B�w��t�@�C�������łɑ��݂��Ă���ꍇ�A�t�@�C�������܂�
		FILE_ATTRIBUTE_NORMAL,	//�t�@�C�������F�����Ȃ�
		NULL);
	//�I�[�v�����s
	//�I�[�v�����s�ŁA�G���[�ɂ���̂���߂܂��B
	if (_hcsv == INVALID_HANDLE_VALUE) {
		DWORD e = ::GetLastError();
		printf("");
	}
	_Assert(_hcsv != INVALID_HANDLE_VALUE, "[_opencsv] FileOpen error %s [%s]\n", _path, LastErrorMsg());
	if (_hcsv == INVALID_HANDLE_VALUE) {
		_err = LastErrorMsg();	//�I�[�v�����̃G���[��ۑ����Ă����܂��B
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
//Csv�t�@�C���𑖍����āA�Y���J�����̒l������Ă��܂��B
//-------------------------------------
//	��s�ǂݏo���B
//-------------------------------------
bool	csv::_one_sentence(char* buf, int bufmax)
{
	if (!avail())	return false;	//���W���[�����L���łȂ�

	int i = 0;
	for (i = 0; ; i++) {
		DWORD	r;
		_Assert(i < bufmax, "[one_sentence]buffer overflow");
		if (::ReadFile(_hcsv, (void*)&buf[i], 1, &r, NULL) == 0) goto error;	//0�Ŏ��s
		if (r != 1)	goto end_of_file;										//���������Ǔǂݍ��݂�0�o�C�g�̓t�@�C���̏I���ł��B

		if (buf[i] == CR || buf[i] == LF) {
			::ReadFile(_hcsv, (void*)&buf[i], 1, &r, NULL);						//���s���΂������ł��B
			buf[i] = '\0';													//���s���k���ɕύX���ďI��
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
//1�s���̕����񂩂�A�J���}��؂肳��Ă��镶����𒊏o���ĕԂ��܂�
bool	csv::_getstr(int n, const char* src, char* dst, int dstmax)
{
	const char* p;
	{	int i;	for (i = 0, p = src; i < n; i++)	if ((p = findoverDelm(p, ",")) == NULL)	goto error;	}//������̂͂��߂ł��B
	//��������R�s�[���Ă����܂�
	{
		int i, j;
		for (i = j = 0; ((p[i] != ',') && (p[i] != '\0')); i++) {
			if (p[i] == ' ' || p[i] == '\t') continue;	//�z���C�g�X�y�[�X�͂Ƃ�̂����āE�E
			_Assert(i < dstmax, "[_getstr]dst buf overflow");	//toku �g���ɂ���������bool�ŕԂ��o�[�W���������
			dst[j++] = p[i];
		}
		dst[j] = '\0';
	}
	return true;

error:
	dst[0] = '\0';	return false;	//�J��������ł�
}
bool	csv::avail(void) { return	(_hcsv != INVALID_HANDLE_VALUE); }
const char* csv::err(void) { return	_err; }

void	csv::writef(const char* fmt, ...)
{
	char _buf[32 * 1024];						//����p�̃o�b�t�@�ł��B�񓯊�������Ƃ�΂��ł��B
	va_list ap;	va_start(ap, fmt);		//�����WINDOWS���������B�B�B
	vsprintf_s(_buf, sizeof(_buf), fmt, ap);
	va_end(ap);
	write(_buf);
}
