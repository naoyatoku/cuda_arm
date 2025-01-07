#include <math.h>

#include "inv_kinetic.h"
#include "circle.h"


//分割コンパイルができなかった・・
#if 0
__device__ __host__ inline vec2 circle::progress_rad(const vec2& start, double rad, int dir) const
{
	_Assert(in(start), "circle::_progress(): start is not on clircle ");
	if (dir == 0) { dir = _dir; }
	//ここで誤差をいろいろ見てみます。
	vec2 src(start - _org);
	double _dt = atan2(src.y, src.x) + rad;
	vec2 v(_r * cos(_dt), _r * sin(_dt));
	v += _org;
	if (in(v) != true) {
		//ここでxのときの解yを求めてみる。
//		vector<vec2>arr = f(_x);
		_Assert(in(v), "circle::_progress() not in circle");
	}
	return v;
}
#endif
