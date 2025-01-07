//---------------------------------------------------------------------------
//	�ėp�I�Ȋ֐��Q�ł��B
//---------------------------------------------------------------------------
#include	<windows.h>
#include	<stdio.h>
#include	<varargs.h>	
#include "CommonModule.h"

#include "string.h"

//toku �ȉ� ���ʎqLs���݂���܂���̑Ώ�
#include <locale.h>
//#include<stdio.h>
//#include<wchar.h>
//#include<tchar.h>


static bool debug=false;
//------------------------------------------------------------------------------------------------------
//	�N�I�[�e�[�V�����𐔂��郂�W���[��
//	�������]�����āA�w��̃N�I�[�e�[�V���������łƂ܂��Ă��邩�ǂ���
//	�𔻒肵�܂��B
//	�N�I�[�e�[�V�����̏�ԃo�b�t�@���w�肷��΁A�N�I�[�e�[�V�����󋵂�
//	�񍐂��܂��B�O��̏�Ԃ������Ɏw�肷��ƁA���̑����Ƃ��Đ����܂��B
//
//	����
//	const char	*str		�E�E�E		�]�����镶����
//	const char	*quotes		�E�E�E		�N�I�[�e�[�V�����̑g�ݍ��킹�ł��B�񕶎���g�ł��B
//	int			*quotestat	�E�E�E		�N�I�[�e�[�V�����̏�Ԃł��Bint �~ QUOTE_MAX�̃o�b�t�@��p�ӂ��Ă��������B
//										NULL�ł������ł��B
//										����Ƀo�b�t�@���w�肷��΁A�O�񂩂�̑����Ƃ��ăN�I�[�e�[�V�����𐔂��A�ŏI�I��
//										��Ԃ������߂��܂��B
//	int ofs					�E�E�E		���������������Ă����NULL�I�[�łȂ��A�擪�`ofs�܂ł̕������]�����܂�
//	�߂�l
//		BOOL	�����񂪃N�I�[�e�[�V�����̑����łƂ܂��Ă��邩�ǂ���
//---------------------------------------------------------------------------------------------------------------------
#define QUOTE_MAX	12		//�N�I�[�e�[�V�����̍ő�l�ł��B
bool isinQuote(const char *str ,  const char* quote , int ofs=0)
{
	//�N�I�[�e�[�V������Ԃł��B
	int quotestat[QUOTE_MAX];	memset(quotestat , 0 , sizeof(quotestat));

	//�y�A�ɂȂ��Ă��Ȃ��ꍇ�̓G���[�ł��B
	_Assert( (strlen(quote) % 2) == 0	,	"isinQuote::not avail quote : %s\r\n"			,	quote);

	//�N�I�[�e�[�V�����𐔂��܂��B
	//
	for(int p=0 ; *(str+p) != '\0' ; p++ ) {
		int i;	
		for( i = 0 ; quote[i]!='\0' ; i+=2 ) {
			if( quote[i]==quote[i+1] ) {	//�N�H�[�e�[�V�����������ꍇ�̓l�X�g�Ή����Ȃ�(���ڂɏo�Ă�����A����́u���v)
				if(*(str+p) == quote[i])	quotestat[i] = quotestat[i] ? 0 : 1;	//�P���ȃg�O���ł��B
			} else {
				if		(	*(str+p)==quote[i]		)	quotestat[i]++;
				else if	(	*(str+p)==quote[i+1]	)	quotestat[i]--;
				//�����ԈႢ��0�ȉ��ɂȂ��Ă��܂����ꍇ�͂���͐����Ȃ����Ƃɂ��܂��B(�����Ȃ�I�[�������ꍇ�Ȃ�)
				if(quotestat[i] < 0) quotestat[i]=0;
			}
		}
		//����ofs�w�肪����΂����̕����܂łł��B
		if(ofs) {	if( p >= ofs ) break;		}
	}
	//���݃N�I�[�e�[�V�����̒��ɂ�����̂��ǂ����𔻒肵�܂��B
	{
		int i;
		for(i=0 ; quote[i]!='\0' ; i++) if(quotestat[i]) return true;
	}
	return false;
}

//-------------------------------------------------------------
//	�w�肳�ꂽ�������A�w�肳�ꂽ������̒��Ɋ܂܂��
//	�����ƈ�v���邩�ǂ���
//-------------------------------------------------------------
bool matchStr(char c , const char *str)
{
	int i;
	for(i= 0 ; *(str+i) != '\0' ; i++ ) {
		if( c == *(str+i) )  {
			return true;
		}
	}
	return false;
}



//-------------------------------------------------------------
//	�f���~�^��������(�����p�^�[���w���)
//	����
//		char *			buf				�e�L�X�g�o�b�t�@�iNULL�I�[�K�v�j
//		const char*		find_delm		�f���~�^������
//		const char*		str_pattern		�����ΏۂƂȂ镶����
//
//	�߂�l
//		�����������_�ł̃|�C���^��Ԃ��܂��B
//
//	����
//		str_pattern�Ɋ܂܂�镶���������ԁA�������܂��B
//		���̊ԁAfind_delm������ɓ����Ă��镶�������������_��
//		�����I�����āA���̃|�C���^��Ԃ��܂��B
//
//		����str_pattern�Ɋ܂܂�Ȃ������񂪏o�Ă�����Anull���o�Ă����肷���
//		�����Ō����I���ƂȂ�ANULL��Ԃ��܂��B
//		str_pattern��NULL�̏ꍇ�́A�����p�^�[���̕]���͂����ANULL�I�[�݂̂̕]���ɂȂ�܂��B
//-------------------------------------------------------------
char *_findDelm(const char *buf , const char *delm ,const char *str_pattern )
{
	int i=0;
	//�܂��f���~�^�������܂ŃT�[�`���܂��B
	for( ; ; i++ )	{
		if( matchStr( *(buf+i) , delm ) )	break;			//delm����v���܂����B
		if(	*(buf+i) == '\0' )				return (char*)NULL;	//�����炸�ɁA�����ɂȂ�܂����B
		//�w�蕶���p�^�[��������΁A���̕����łȂ��Ȃ������_�ŏI���ł��B
		if( str_pattern ) {
			if( ! matchStr( *(buf+i) , str_pattern) ) return (char*)NULL;
		}
	}
	return (char*)(buf+i);
}

//-------------------------------------------------------------
//	�f���~�^��������
//	����
//		const char *buf		:	�e�L�X�g�o�b�t�@�iNULL�I�[�K�v�j
//		const char *delm	:	�f���~�^������
//	�߂�l
//		�w��f���~�^(������̂����ǂꂩ�ꕶ��)�������������_�̃|�C���^��Ԃ��܂��B
//		�����f���~�^�������炩�����ꍇ��NULL�|�C���^��Ԃ��܂��B
//-------------------------------------------------------------
char *findDelm(const char *buf , const char *delm)
{
	return ( _findDelm( buf , delm , (const char*)NULL ));
}


//-------------------------------------------------------------
//	�t����Delm��������p�^�[���ł�
//-------------------------------------------------------------
char *findDelm_rev(const char *buf , const char *delm)
{
	int i;
	for( i = strlen(buf) - 1 ; i >= 0 ; i--) {
		if(	matchStr( buf[i] , delm) ) return (char*)&(buf[i]);
	}
	return (char*)NULL;
}

//-------------------------------------------------------------
//	�f���~�^���Ƃ��肷����
//	����
//		const char *buf		:	�e�L�X�g�o�b�t�@�iNULL�I�[�K�v�j
//		const char *delm	:	�f���~�^������
//	�߂�l
//		�w��f���~�^�łȂ���������������|�C���^��Ԃ��܂�
//-------------------------------------------------------------
char *overDelm(const char *buf , const char*delm)
{
	int i=0;
	//�܂��f���~�^�������܂ŃT�[�`���܂��B
	for( ; ; i++ )	{
		if( !matchStr( *(buf+i) , delm ) )	break;			//delm�łȂ������������܂���
		if(	*(buf+i) == '\0' )				return (char*)NULL;	//�����炸�ɁA�����ɂȂ�܂����B
	}
	return (char*)(buf+i);
}
//�f���~�^���Ƃ��肷����(������̍Ōォ��t�ɒT���o�[�W�����ł�)
char *overDelm_rev(const char *buf , const char*delm)
{
	int i;
	for( i = strlen(buf) - 1 ; i >= 0 ; i -- ){
		if( !matchStr( *(buf+i) , delm ) )	break;			//delm�łȂ������������܂���
	}
	if(i<0) return (char*)NULL;	//������Ȃ�����

	return (char*)(buf+i);
}

//-------------------------------------------------------------
//	�f���~�^�������Ă���ɒʂ�߂����ꏊ�̃|�C���^��Ԃ�
//-------------------------------------------------------------
char *findoverDelm(const char *buf , const char *delm)
{
	char *p;
	if( (p = findDelm( buf , delm))!=NULL ) return( overDelm(p , delm) );
	return (char*)NULL;
}

//-------------------------------------------------------------
//	�A�X�L�[�����ȊO�̕�������΂��܂��B
//-------------------------------------------------------------
char *overNoAscii( char *buf )
{
	for( ; *buf != 0  ; buf++ ) {
		if( *(unsigned char*)buf <= 0x7f ) return buf;
	}
	return 0;
}

//-------------------------------------------------------------
//	�o�b�t�@����A�f���~�^�܂œǂݍ���
//	�Ƃɂ����f���~�^�܂œǂ݂܂��B
//
//	�߂�l�F�f���~�^�i�܂��͕�����I�[�j���o�Ă����Ƃ���̃|�C���^��Ԃ��܂��B
//			���������ꕶ�����ǂ߂Ȃ������ꍇ�́A�k���|�C���^��Ԃ��܂��B
//-------------------------------------------------------------
char *getElem(const char*src , char*dst ,const char *delm , int dstmax )
{
	//�܂��f���~�^�͔�΂��܂��B
	if( ! (src = overDelm( src , delm)) ) goto Error;

	//�f���~�^���o�Ă���܂ŕ����R�s�[
	int si, di;
	si = di = 0;
	for(	;	; si++ , di++ ) {
		if(dstmax)	_Assert( di < dstmax, "getElem : buf overflow src=%s , max=%d" , src , dstmax);	//�o�b�t�@�I�[�o�[�t���[�`�F�b�N

		if(  *(src+si) == '\0' ){ if(!si) { goto Error;}  else{	break;}	}//�����������Ȃ��Ă��܂��܂����B
		if( matchStr ( *(src+si) , delm ) ) break;
		*(dst+di) = *(src+si);
	}

	*(dst + di) = '\0';	//�Ō�ɏI�[�����܂��B

	return (char*)(src + si);
Error:
	return (char*)0;
}
//---------------------------------------------------------------------------------------
//	�N�H�[�e�[�V�����̒��̃f���~�^�`�F�b�N�����Ȃ�getElem
//	�N�H�[�e�[�V�����́A������ł��B
//	"\"\"()"�ȂǓ񕶎��łЂƂ̃y�A�Ƃ��Ă��������B�i�K���y�A�Ŏw�肵�Ă��������B���Ȃ���Η�O�Ŏ~�߂܂��B�j
//---------------------------------------------------------------------------------------
char *getElem_withoutQuotation(const char*src , char*dst ,const char *delm , char *quote , int dstmax )
{
	//�����͓����|�C���^�ł�
	char *pdst;
	const char*psrc;
	for( pdst=dst , psrc=src ; ; ) {
		//�l�������܂��B
		{
			char *p;
			if( (p = getElem( psrc , pdst , delm ,dstmax)) == 0 ) {	//������Ȃ������B
				if(psrc!=src)	break;				//��x�ł�getElem�ł��Ă���ꍇ�́A�O��̒l��Ԃ��܂��B
				else			return 0;			//�����Ȃ茩����Ȃ��ꍇ��NULL�ł��B
			}
			psrc = p;	//������΁A�ʒu���X�V���܂��B
		}
//_printf("[getalem_quo: getelem[%s]\r\n" , dst);
		//����src�̓f���~�^�̂Ƃ���łƂ܂��Ă��āA�Ȃ�����dst�ɂ���̓R�s�[����Ă܂���B�i�P��������Ȃ�)
		if( isinQuote( dst ,  quote ) ){		//�擾����������́A�N�I�[�e�[�V�����̓r���Ŏ~�܂��Ă��܂��B������ǂނƂɂ��܂�
			pdst		+=	strlen(pdst);		//�������݃|�C���^��i�߂܂��B
			*pdst++		=	*psrc++;			//�������f���~�^�܂ŃR�s�[���܂��B
			*pdst = '\0';						//NULL�I�[�����̏����ŏ����Ă��܂��̂ŕt�������܂��B
			continue;
		}
		break;
	}

	return (char*)psrc;
}

//-------------------------------------------------------------
//	�R�����g�A�E�g����
//-------------------------------------------------------------
char *CommentOut(char *buf , const char *s , const char *e)
{
	for( ; ; ) {
		//�z���C�g�X�y�[�X�������܂��B
		if(	( buf = overDelm( buf ,	" \t\r\n"))	==	0 ) goto Error;

		//�R�����g�s���ǂ����𔻒f���܂��B�R�����g�s�łȂ��Ȃ�΁A�I���ł��B
		if( strncmp( buf , s  , strlen(s) ) != 0 )		break;

		//�R�����g�X�^�[�g�ł��ΏI�[�������T���܂��B
		for( ; ; buf++ ) {
			if( ! *buf )								goto Error;	//�I�[�ɂȂ��Ă��܂��܂���
			if( strncmp(buf , e , strlen(e)) == 0 )		break;		//�R�����g�̍Ōオ������܂���
		}
		//����buf�̓R�����g�I���̕�����e�̐擪�ɂ��܂��B������΂����Ƃ���̃|�C���^��Ԃ��܂��B
		buf = overDelm(buf , e);
	}
	return buf;

Error:
	return 0;
}

//-------------------------------------------------------------
//	�N�I�[�e�[�V�������͂����ĕ��������蒼���܂�
//-------------------------------------------------------------
void removeQuotation( char *str , char quotation )
{
	if( *str != quotation ) return ;	//�N�H�[�e�[�V�������Ȃ��ꍇ�͕Ԃ��܂�
	for(  ; *(str+1) != quotation	; str++ ) 	*str = *(str+1);
	*str = '\0';
}



//===============
//	log �֘A
//===============
#include "csv.h"
#define		DISP_ROWS	30		//50�s���炢������
#define		BUFFSIZE	256		//�P�s������ł��B

static	char			__str[DISP_ROWS][BUFFSIZE];		//�\�����e��\��
static	int				__i_str;
static	bool			__looped;
static _LOG_CALLBACK	__log_callback;			//���O�������݂��s�����Ƃ��ɃR�[���o�b�N���܂��B

static	csv _log("log.txt",true);	//


void log_init(_LOG_CALLBACK callback)
{
	for (int i = 0; i < DISP_ROWS; ++i) {
		sprintf_s( &__str[i][0] , BUFFSIZE , "[%d]" ,i);
	}
	__i_str = 0;	//
	__looped = false;
	if(callback){
		__log_callback = callback;
	}
}
//�����ό����ɂ��邩
void write_log(const char*fmt,...)
{
//	static char _buf[2*1024];				//����p�̃o�b�t�@�ł��B�񓯊�������Ƃ�΂��ł��B
	va_list ap;	va_start( ap , fmt );		//�����WINDOWS���������B�B�B
	//�Ȃ��������ŗ�O���łĂ��܂��B
	vsprintf_s( &__str[__i_str][0]  ,BUFFSIZE , fmt , ap);
//	sprintf(&__str[__i_str][0], fmt, ap);
	//�����Ńt�@�C���ɂ��������ނ悤�ɂ��܂��B�t�@�C���������݂̂Ƃ��ɂ͊J�Ƃ����܂��B
	{	//�t�@�C���ɂ́A�������t�^���܂��B
		_SYSTEMTIME t;::GetLocalTime(&t);
		_log.writef("[%d.%d.%d %02d:%02d]\t%s\n", t.wYear,t.wMonth,t.wDay,t.wHour,t.wMinute  , &__str[__i_str][0]);
	}
	va_end(ap);
//	strcpy_s(&__str[__i_str][0], BUFFSIZE, str);
	//�\���ʒu���C���N�������g���܂��B
	if (++__i_str > (DISP_ROWS-1)) {
		__looped = true;
		__i_str = 0;
	}
//	log()
//	::SendMessage(__hWnd, WM_USER + 1, 0, 0);
	if(__log_callback){
		__log_callback();
	}
	//�����ɁA���O�t�@�C���ɂ������Ă����܂��B
	//�������݈ʒu�ɏ������ށB
}
//���t�����Ȃ��o�[�W����
void write_log_rare(const char* fmt, ...) {
	va_list ap;	va_start(ap, fmt);		//�����WINDOWS���������B�B�B1222222222222^
	vsprintf_s(&__str[__i_str][0], BUFFSIZE, fmt, ap);
	//�����Ńt�@�C���ɂ��������ނ悤�ɂ��܂��B�t�@�C���������݂̂Ƃ��ɂ͊J�Ƃ����܂��B
/* {	//�t�@�C���ɂ́A�������t�^���܂��B
		_SYSTEMTIME t; ::GetLocalTime(&t);
		_log.writef("[%d.%d.%d %02d:%02d]\t%s\n", t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, &__str[__i_str][0]);
	}*/
	_log.write("\s", &__str[__i_str][0] );
	va_end(ap);
	//	strcpy_s(&__str[__i_str][0], BUFFSIZE, str);
		//�\���ʒu���C���N�������g���܂��B
	if (++__i_str > (DISP_ROWS - 1)) {
		__looped = true;
		__i_str = 0;
	}
	//	log()
	//	::SendMessage(__hWnd, WM_USER + 1, 0, 0);
	if (__log_callback) {
		__log_callback();
	}
	//�����ɁA���O�t�@�C���ɂ������Ă����܂��B
	//�������݈ʒu�ɏ������ށB
}

const char*get_log(int idx)
{
	//�Ƃ肠����
	int i = __looped ? __i_str  : 0;	//��x���[�v�������Ƃ́A�\���̃C���f�b�N�X�̍ŏ��͏������݃|�C���^��
	return &__str[  (i + idx)%DISP_ROWS  ][0];
			//
//		if (!__looped && (row == __i_str)) {	//�����o�b�t�@���ꏄ���ĂȂ���΁A�������݈ʒu�ɓ��B�������_�ŏI���ł��B
//			break;
//		}	

}
//==================================================================================================================
//	last_error�Ȃ�
//==================================================================================================================
const char* _error_msg(DWORD msg_id, char *msgcpy,int msgcpy_size)
{
	//	printf("1(id=%d)\n", msg_id);
	LPVOID buffer;DWORD n;
	if((n=FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL,
		GetLastError(),
		MAKELANGID(LANG_ENGLISH, SUBLANG_DEFAULT),
		(LPSTR)&buffer, 0, NULL))==0){
//			::OutputDebugStringA("format message fin\n");
		};
	//	wprintf(L"[%s]", (LPCTSTR)buffer);
	//�ԍ����o��
/*	wchar_t buf[64];
	wsprintf(buf, L"lasterr %d [", ::GetLastError());
	::OutputDebugString(buf);	::OutputDebugString((LPWSTR)buffer);	::OutputDebugStringA("]\n");*/
	//
	if(msgcpy){
		strcpy_s(msgcpy,msgcpy_size,(const char*)buffer);
	}
	LocalFree(buffer);
	return msgcpy;
}
/*
const char*LastErrorMsg(void) {
	static char str[256];
	_error_msg(::GetLastError(),str,sizeof(str));
	//���s����������΂���͂̂����܂��B
	for (int i = 0; str[i] != '\0';++i) {
		if ((str[i] == '\r') || (str[i] == '\n')) {
			str[i] = '\0';break;
		}
	}
	return str;
}
*/
//static 
//int Error(const char*str)
int		Error(const char*fmt, ...)
{
	//	printf("\n ==== Eror(%s) last error =====\n" ,str);
	//	LastErrorMsg();
//	tokutoku
	va_list ap;	va_start( ap , fmt );		//�����WINDOWS���������B�B�B
		static char	_buf[256];	//write_log�̃o�b�t�@�ɍ��킹�Ă����܂��B
		_buf[0] = 'E'; _buf[1] = '>';		//�ŏ���E:�Ƃ��Ă݂�B
		vsprintf_s(&_buf[2], sizeof(_buf), fmt, ap);
		write_log(_buf);
	va_end(ap);
	return -1;
}

//assert���܂�
void _Assert_log( bool a, const char*fmt,...){
	if(!a){
		Error(fmt);	
//		draw("ASSERT: [%s]", str);
		//
//		dump(str);
//		for (;;);
		exit(0);
	}
}
void _Assert(bool a, const char*fmt,...)
{
	va_list ap;	va_start(ap, fmt);		//�����WINDOWS���������B�B�B
	if (!a) {
//		Error(false, fmt)
		printf(fmt,ap);
		exit(0);
	}
	va_end(ap);

}



#include <iostream>
#include <cuda_runtime.h>
// Converts the version number of the device into the number of cores
// Note: This is a simple version, and newer architectures may not be covered.
int _ConvertSMVer2Cores(int major, int minor) {
    typedef struct {
        int SM;
        int Cores;
    } sSMtoCores;

    sSMtoCores nGpuArchCoresPerSM[] = {
        {0x30, 192}, // Kepler Generation (SM 3.0) GK10x class
        {0x32, 192}, // Kepler Generation (SM 3.2) GK10x class
        {0x35, 192}, // Kepler Generation (SM 3.5) GK11x class
        {0x37, 192}, // Kepler Generation (SM 3.7) GK21x class
        {0x50, 128}, // Maxwell Generation (SM 5.0) GM10x class
        {0x52, 128}, // Maxwell Generation (SM 5.2) GM20x class
        {0x53, 128}, // Maxwell Generation (SM 5.3) GM20x class
        {0x60,  64}, // Pascal Generation (SM 6.0) GP100 class
        {0x61, 128}, // Pascal Generation (SM 6.1) GP10x class
        {0x62, 128}, // Pascal Generation (SM 6.2) GP10x class
        {0x70,  64}, // Volta and Turing Generation (SM 7.0) GV10x class
        {0x72,  64}, // Turing Generation (SM 7.2) TU10x class
        {0x75,  64}, // Turing Generation (SM 7.5) TU10x class
        {0x80, 128}, // Ampere Generation (SM 8.0) GA10x class
        {0x86, 128}, // Ampere Generation (SM 8.6) GA10x class
        {-1, -1}
    };

    int index = 0;
    while (nGpuArchCoresPerSM[index].SM != -1) {
        if (nGpuArchCoresPerSM[index].SM == ((major << 4) + minor)) {
            return nGpuArchCoresPerSM[index].Cores;
        }
        index++;
    }
    // If we don't find the values, we default to return -1
    return -1;
}

int device_query()
{
    int deviceCount;
    cudaGetDeviceCount(&deviceCount); // ���p�\�ȃf�o�C�X�̐����擾

    for (int device = 0; device < deviceCount; ++device) {
        cudaDeviceProp deviceProp;
        cudaGetDeviceProperties(&deviceProp, device); // device�ԍ��̃f�o�C�X�̃v���p�e�B���擾

        printf("\nDevice %d: \"%s\"\n", device, deviceProp.name);

        // Compute capabilities
        printf("  CUDA Capability Major/Minor version number:    %d.%d\n", deviceProp.major, deviceProp.minor);

        // Amount of global memory
        printf("  Total amount of global memory:                 %.2f MBytes\n", (float)deviceProp.totalGlobalMem / 1048576.0f);

        // Number of multiprocessors
        printf("  Number of multiprocessors:                     %d\n", deviceProp.multiProcessorCount);
        printf("  max threads per multiprocessors:               %d\n", deviceProp.maxThreadsPerMultiProcessor);
//        printf("  ==>cuda core total                             %d\n", deviceProp.multiProcessorCount * deviceProp..maxThreadsPerMultiProcessor);

        // Maximum number of threads per block
        printf("  Max number of threads per block:               %d\n", deviceProp.maxThreadsPerBlock);

        // Maximum sizes of each dimension of a block
        printf("  Max sizes of each dimension of a block:        %d x %d x %d\n", deviceProp.maxThreadsDim[0], deviceProp.maxThreadsDim[1], deviceProp.maxThreadsDim[2]);

        // Maximum sizes of each dimension of a grid
        printf("  Max sizes of each dimension of a grid:         %d x %d x %d\n", deviceProp.maxGridSize[0], deviceProp.maxGridSize[1], deviceProp.maxGridSize[2]);
        //
        printf("warp size                                        %d\n", deviceProp.warpSize);
        printf("regs per multiprocessor                          %d\n", deviceProp.regsPerMultiprocessor);
        printf("regs per block                                   %d\n", deviceProp.regsPerBlock);
        printf("clock                                            %d\n", deviceProp.clockRate);
        printf("memory clock                                     %d\n", deviceProp.memoryClockRate);
        printf("shared mem per block                             %d\n", deviceProp.sharedMemPerBlock);
        printf("unifiedAddressing                                %d\n", deviceProp.unifiedAddressing);
        printf("unifiedFuncPointers                              %d\n", deviceProp.unifiedFunctionPointers);

        std::cout << "Device " << device << ": " << deviceProp.name << std::endl;
        std::cout << "  Number of multiprocessors: " << deviceProp.multiProcessorCount << std::endl;
        std::cout << "  CUDA Cores per multiprocessor: " << _ConvertSMVer2Cores(deviceProp.major, deviceProp.minor) * deviceProp.multiProcessorCount << std::endl;
        std::cout << "  Max threads per multiprocessor: " << deviceProp.maxThreadsPerMultiProcessor << std::endl;
        std::cout << "  Max threads per block: " << deviceProp.maxThreadsPerBlock << std::endl;
    }

    return 0;
}
