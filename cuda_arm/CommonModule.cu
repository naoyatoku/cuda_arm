//---------------------------------------------------------------------------
//	汎用的な関数群です。
//---------------------------------------------------------------------------
#include	<windows.h>
#include	<stdio.h>
#include	<varargs.h>	
#include "CommonModule.h"

#include "string.h"

//toku 以下 識別子Lsがみつかりませんの対処
#include <locale.h>
//#include<stdio.h>
//#include<wchar.h>
//#include<tchar.h>


static bool debug=false;
//------------------------------------------------------------------------------------------------------
//	クオーテーションを数えるモジュール
//	文字列を評価して、指定のクオーテーション内部でとまっているかどうか
//	を判定します。
//	クオーテーションの状態バッファを指定すれば、クオーテーション状況を
//	報告します。前回の状態をそこに指定すると、その続きとして数えます。
//
//	引数
//	const char	*str		・・・		評価する文字列
//	const char	*quotes		・・・		クオーテーションの組み合わせです。二文字一組です。
//	int			*quotestat	・・・		クオーテーションの状態です。int × QUOTE_MAX個のバッファを用意してください。
//										NULLでもいいです。
//										これにバッファを指定すれば、前回からの続きとしてクオーテーションを数え、最終的な
//										状態を書き戻します。
//	int ofs					・・・		もし数字が入っていればNULL終端でなく、先頭～ofsまでの文字列を評価します
//	戻り値
//		BOOL	文字列がクオーテーションの続きでとまっているかどうか
//---------------------------------------------------------------------------------------------------------------------
#define QUOTE_MAX	12		//クオーテーションの最大値です。
bool isinQuote(const char *str ,  const char* quote , int ofs=0)
{
	//クオーテーション状態です。
	int quotestat[QUOTE_MAX];	memset(quotestat , 0 , sizeof(quotestat));

	//ペアになっていない場合はエラーです。
	_Assert( (strlen(quote) % 2) == 0	,	"isinQuote::not avail quote : %s\r\n"			,	quote);

	//クオーテーションを数えます。
	//
	for(int p=0 ; *(str+p) != '\0' ; p++ ) {
		int i;	
		for( i = 0 ; quote[i]!='\0' ; i+=2 ) {
			if( quote[i]==quote[i+1] ) {	//クォーテーションが同じ場合はネスト対応しない(二回目に出てきたら、それは「閉じ」)
				if(*(str+p) == quote[i])	quotestat[i] = quotestat[i] ? 0 : 1;	//単純なトグルです。
			} else {
				if		(	*(str+p)==quote[i]		)	quotestat[i]++;
				else if	(	*(str+p)==quote[i+1]	)	quotestat[i]--;
				//もし間違いで0以下になってしまった場合はそれは数えないことにします。(いきなり終端がきた場合など)
				if(quotestat[i] < 0) quotestat[i]=0;
			}
		}
		//もしofs指定があればそこの文字までです。
		if(ofs) {	if( p >= ofs ) break;		}
	}
	//現在クオーテーションの中にあるものかどうかを判定します。
	{
		int i;
		for(i=0 ; quote[i]!='\0' ; i++) if(quotestat[i]) return true;
	}
	return false;
}

//-------------------------------------------------------------
//	指定された文字が、指定された文字列の中に含まれる
//	文字と一致するかどうか
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
//	デリミタを見つける(文字パターン指定つき)
//	引数
//		char *			buf				テキストバッファ（NULL終端必要）
//		const char*		find_delm		デリミタ文字列
//		const char*		str_pattern		検索対象となる文字列
//
//	戻り値
//		見つかった時点でのポインタを返します。
//
//	説明
//		str_patternに含まれる文字が続く間、検索します。
//		その間、find_delm文字列に入っている文字を見つけた時点で
//		検索終了して、そのポインタを返します。
//
//		もしstr_patternに含まれない文字列が出てきたり、nullが出てきたりすれば
//		そこで検索終了となり、NULLを返します。
//		str_patternがNULLの場合は、文字パターンの評価はせず、NULL終端のみの評価になります。
//-------------------------------------------------------------
char *_findDelm(const char *buf , const char *delm ,const char *str_pattern )
{
	int i=0;
	//まずデリミタが現れるまでサーチします。
	for( ; ; i++ )	{
		if( matchStr( *(buf+i) , delm ) )	break;			//delmが一致しました。
		if(	*(buf+i) == '\0' )				return (char*)NULL;	//見つからずに、文末になりました。
		//指定文字パターンがあれば、その文字でなくなった時点で終了です。
		if( str_pattern ) {
			if( ! matchStr( *(buf+i) , str_pattern) ) return (char*)NULL;
		}
	}
	return (char*)(buf+i);
}

//-------------------------------------------------------------
//	デリミタを見つける
//	引数
//		const char *buf		:	テキストバッファ（NULL終端必要）
//		const char *delm	:	デリミタ文字列
//	戻り値
//		指定デリミタ(文字列のうちどれか一文字)が見つかった時点のポインタを返します。
//		もしデリミタが見つからかった場合はNULLポインタを返します。
//-------------------------------------------------------------
char *findDelm(const char *buf , const char *delm)
{
	return ( _findDelm( buf , delm , (const char*)NULL ));
}


//-------------------------------------------------------------
//	逆からDelmを見つけるパターンです
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
//	デリミタをとおりすぎる
//	引数
//		const char *buf		:	テキストバッファ（NULL終端必要）
//		const char *delm	:	デリミタ文字列
//	戻り値
//		指定デリミタでない文字列を見つけたポインタを返します
//-------------------------------------------------------------
char *overDelm(const char *buf , const char*delm)
{
	int i=0;
	//まずデリミタが現れるまでサーチします。
	for( ; ; i++ )	{
		if( !matchStr( *(buf+i) , delm ) )	break;			//delmでない文字を見つけました
		if(	*(buf+i) == '\0' )				return (char*)NULL;	//見つからずに、文末になりました。
	}
	return (char*)(buf+i);
}
//デリミタをとおりすぎる(文字列の最後から逆に探すバージョンです)
char *overDelm_rev(const char *buf , const char*delm)
{
	int i;
	for( i = strlen(buf) - 1 ; i >= 0 ; i -- ){
		if( !matchStr( *(buf+i) , delm ) )	break;			//delmでない文字を見つけました
	}
	if(i<0) return (char*)NULL;	//見つからなかった

	return (char*)(buf+i);
}

//-------------------------------------------------------------
//	デリミタを見つけてさらに通り過ぎた場所のポインタを返す
//-------------------------------------------------------------
char *findoverDelm(const char *buf , const char *delm)
{
	char *p;
	if( (p = findDelm( buf , delm))!=NULL ) return( overDelm(p , delm) );
	return (char*)NULL;
}

//-------------------------------------------------------------
//	アスキー文字以外の文字列を飛ばします。
//-------------------------------------------------------------
char *overNoAscii( char *buf )
{
	for( ; *buf != 0  ; buf++ ) {
		if( *(unsigned char*)buf <= 0x7f ) return buf;
	}
	return 0;
}

//-------------------------------------------------------------
//	バッファから、デリミタまで読み込み
//	とにかくデリミタまで読みます。
//
//	戻り値：デリミタ（または文字列終端）が出てきたところのポインタを返します。
//			もしもし一文字も読めなかった場合は、ヌルポインタを返します。
//-------------------------------------------------------------
char *getElem(const char*src , char*dst ,const char *delm , int dstmax )
{
	//まずデリミタは飛ばします。
	if( ! (src = overDelm( src , delm)) ) goto Error;

	//デリミタが出てくるまで文字コピー
	int si, di;
	si = di = 0;
	for(	;	; si++ , di++ ) {
		if(dstmax)	_Assert( di < dstmax, "getElem : buf overflow src=%s , max=%d" , src , dstmax);	//バッファオーバーフローチェック

		if(  *(src+si) == '\0' ){ if(!si) { goto Error;}  else{	break;}	}//文字が無くなってしまいました。
		if( matchStr ( *(src+si) , delm ) ) break;
		*(dst+di) = *(src+si);
	}

	*(dst + di) = '\0';	//最後に終端を入れます。

	return (char*)(src + si);
Error:
	return (char*)0;
}
//---------------------------------------------------------------------------------------
//	クォーテーションの中のデリミタチェックをしないgetElem
//	クォーテーションは、文字列です。
//	"\"\"()"など二文字でひとつのペアとしてください。（必ずペアで指定してください。しなければ例外で止めます。）
//---------------------------------------------------------------------------------------
char *getElem_withoutQuotation(const char*src , char*dst ,const char *delm , char *quote , int dstmax )
{
	//これらは動くポインタです
	char *pdst;
	const char*psrc;
	for( pdst=dst , psrc=src ; ; ) {
		//値を見つけます。
		{
			char *p;
			if( (p = getElem( psrc , pdst , delm ,dstmax)) == 0 ) {	//見つからなかった。
				if(psrc!=src)	break;				//一度でもgetElemできている場合は、前回の値を返します。
				else			return 0;			//いきなり見つからない場合はNULLです。
			}
			psrc = p;	//見つかれば、位置を更新します。
		}
//_printf("[getalem_quo: getelem[%s]\r\n" , dst);
		//いまsrcはデリミタのところでとまっていて、なおかつdstにそれはコピーされてません。（１文字たりない)
		if( isinQuote( dst ,  quote ) ){		//取得した文字列は、クオーテーションの途中で止まっています。続きを読むとにします
			pdst		+=	strlen(pdst);		//書き込みポインタを進めます。
			*pdst++		=	*psrc++;			//見つけたデリミタまでコピーします。
			*pdst = '\0';						//NULL終端がこの処理で消えてしまうので付け直します。
			continue;
		}
		break;
	}

	return (char*)psrc;
}

//-------------------------------------------------------------
//	コメントアウト処理
//-------------------------------------------------------------
char *CommentOut(char *buf , const char *s , const char *e)
{
	for( ; ; ) {
		//ホワイトスペースを除きます。
		if(	( buf = overDelm( buf ,	" \t\r\n"))	==	0 ) goto Error;

		//コメント行かどうかを判断します。コメント行でなくなれば、終了です。
		if( strncmp( buf , s  , strlen(s) ) != 0 )		break;

		//コメントスタートであば終端文字列を探します。
		for( ; ; buf++ ) {
			if( ! *buf )								goto Error;	//終端になってしまいました
			if( strncmp(buf , e , strlen(e)) == 0 )		break;		//コメントの最後が見つかりました
		}
		//いまbufはコメント終了の文字列eの先頭にいます。これを飛ばしたところのポインタを返します。
		buf = overDelm(buf , e);
	}
	return buf;

Error:
	return 0;
}

//-------------------------------------------------------------
//	クオーテーションをはずして文字列を作り直します
//-------------------------------------------------------------
void removeQuotation( char *str , char quotation )
{
	if( *str != quotation ) return ;	//クォーテーションがない場合は返します
	for(  ; *(str+1) != quotation	; str++ ) 	*str = *(str+1);
	*str = '\0';
}



//===============
//	log 関連
//===============
#include "csv.h"
#define		DISP_ROWS	30		//50行くらいかくか
#define		BUFFSIZE	256		//１行あたりです。

static	char			__str[DISP_ROWS][BUFFSIZE];		//表示内容を表す
static	int				__i_str;
static	bool			__looped;
static _LOG_CALLBACK	__log_callback;			//ログ書き込みを行ったときにコールバックします。

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
//ここ可変個引数にするか
void write_log(const char*fmt,...)
{
//	static char _buf[2*1024];				//これ用のバッファです。非同期があるとやばいです。
	va_list ap;	va_start( ap , fmt );		//これはWINDOWSだけだが。。。
	//なぜかここで例外がでてしまう。
	vsprintf_s( &__str[__i_str][0]  ,BUFFSIZE , fmt , ap);
//	sprintf(&__str[__i_str][0], fmt, ap);
	//ここでファイルにも書き込むようにします。ファイル書き込みのときには開業を入れます。
	{	//ファイルには、時刻も付与します。
		_SYSTEMTIME t;::GetLocalTime(&t);
		_log.writef("[%d.%d.%d %02d:%02d]\t%s\n", t.wYear,t.wMonth,t.wDay,t.wHour,t.wMinute  , &__str[__i_str][0]);
	}
	va_end(ap);
//	strcpy_s(&__str[__i_str][0], BUFFSIZE, str);
	//表示位置をインクリメントします。
	if (++__i_str > (DISP_ROWS-1)) {
		__looped = true;
		__i_str = 0;
	}
//	log()
//	::SendMessage(__hWnd, WM_USER + 1, 0, 0);
	if(__log_callback){
		__log_callback();
	}
	//同時に、ログファイルにも書いていきます。
	//書き込み位置に書き込む。
}
//日付をつけないバージョン
void write_log_rare(const char* fmt, ...) {
	va_list ap;	va_start(ap, fmt);		//これはWINDOWSだけだが。。。1222222222222^
	vsprintf_s(&__str[__i_str][0], BUFFSIZE, fmt, ap);
	//ここでファイルにも書き込むようにします。ファイル書き込みのときには開業を入れます。
/* {	//ファイルには、時刻も付与します。
		_SYSTEMTIME t; ::GetLocalTime(&t);
		_log.writef("[%d.%d.%d %02d:%02d]\t%s\n", t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, &__str[__i_str][0]);
	}*/
	_log.write("\s", &__str[__i_str][0] );
	va_end(ap);
	//	strcpy_s(&__str[__i_str][0], BUFFSIZE, str);
		//表示位置をインクリメントします。
	if (++__i_str > (DISP_ROWS - 1)) {
		__looped = true;
		__i_str = 0;
	}
	//	log()
	//	::SendMessage(__hWnd, WM_USER + 1, 0, 0);
	if (__log_callback) {
		__log_callback();
	}
	//同時に、ログファイルにも書いていきます。
	//書き込み位置に書き込む。
}

const char*get_log(int idx)
{
	//とりあえず
	int i = __looped ? __i_str  : 0;	//一度ループしたあとは、表示のインデックスの最初は書き込みポインタの
	return &__str[  (i + idx)%DISP_ROWS  ][0];
			//
//		if (!__looped && (row == __i_str)) {	//もしバッファが一巡してなければ、書き込み位置に到達した時点で終わりです。
//			break;
//		}	

}
//==================================================================================================================
//	last_errorなど
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
	//番号も出す
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
	//改行文字があればそれはのぞきます。
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
	va_list ap;	va_start( ap , fmt );		//これはWINDOWSだけだが。。。
		static char	_buf[256];	//write_logのバッファに合わせておきます。
		_buf[0] = 'E'; _buf[1] = '>';		//最初にE:とつけてみる。
		vsprintf_s(&_buf[2], sizeof(_buf), fmt, ap);
		write_log(_buf);
	va_end(ap);
	return -1;
}

//assert作ります
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
	va_list ap;	va_start(ap, fmt);		//これはWINDOWSだけだが。。。
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
    cudaGetDeviceCount(&deviceCount); // 利用可能なデバイスの数を取得

    for (int device = 0; device < deviceCount; ++device) {
        cudaDeviceProp deviceProp;
        cudaGetDeviceProperties(&deviceProp, device); // device番号のデバイスのプロパティを取得

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
