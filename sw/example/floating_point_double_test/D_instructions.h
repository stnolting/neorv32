#ifndef D_INSTRUCTIONS_H_
#define D_INSTRUCTIONS_H_

#define qNaN  0x7FF8000000000000
#define sNaN  0x7FF4000000000000
#define pINF  0x7FF0000000000000
#define nINF  0xFFF0000000000000
#define pSUB  0x000FFFFFFFFFFFFF
#define nSUB  0x800fffffffffffff
#define nZRO  0x8000000000000000

#define qNaNs 0x7FC00000
#define sNaNs 0x7FA00000
#define pINFs 0x7F800000
#define nINFs 0xFF800000
#define pSUBs 0x007FFFFF
#define nSUBs 0x807FFFFF
#define nZROs 0x80000000

void pass_process(int _num);
void fail_process(int _num);

bool areEqual(double _d0, double _d1);
bool areEqual_s(float _f0, float _f1);

void TEST_FCLASS_D(double _d0, int _result);
int TEST_FP_OP_D_INTERNAL_fclass(double _d0);

void TEST_FSGNJD(double _d0, double _d1, double _result, char _inst);
double TEST_FP_OP_D_INTERNAL_fsgnj(double _d0, double _d1);
double TEST_FP_OP_D_INTERNAL_fsgnjn(double _d0, double _d1);
double TEST_FP_OP_D_INTERNAL_fsgnjx(double _d0, double _d1);

void TEST_FP_CMP_OP_D(double _d0, double _d1, int _result, int _fflags, char _inst);
int TEST_FP_OP_D_INTERNAL_feq(double _d0, double _d1);
int TEST_FP_OP_D_INTERNAL_fle(double _d0, double _d1);
int TEST_FP_OP_D_INTERNAL_flt(double _d0, double _d1);

void TEST_FP_INT_OP_D(double _d0, int _result, int _fflags, char _inst);
int TEST_FP_OP_D_INTERNAL_fcvt_w_d(double _d0);
int TEST_FP_OP_D_INTERNAL_fcvt_wu_d(double _d0);

void TEST_INT_FP_OP_D(int _d0, double _result, char _inst);
double TEST_FP_OP_D_INTERNAL_fcvt_d_w(int _d0);
double TEST_FP_OP_D_INTERNAL_fcvt_d_wu(int _d0);

void TEST_FP_OP3_D(double _d0, double _d1, double _d2, double _result, int _fflags, char _inst);
double TEST_FP_OP_D_INTERNAL_fmadd(double _d0, double _d1, double _d2);
double TEST_FP_OP_D_INTERNAL_fmsub(double _d0, double _d1, double _d2);
double TEST_FP_OP_D_INTERNAL_fnmadd(double _d0, double _d1, double _d2);
double TEST_FP_OP_D_INTERNAL_fnmsub(double _d0, double _d1, double _d2);

void TEST_FP_OP2_D(double _d0, double _d1, double _result, int _fflags, char _inst);
double TEST_FP_OP_D_INTERNAL_fadd(double _d0, double _d1);
double TEST_FP_OP_D_INTERNAL_fsub(double _d0, double _d1);
double TEST_FP_OP_D_INTERNAL_fmul(double _d0, double _d1);
double TEST_FP_OP_D_INTERNAL_fdiv(double _d0, double _d1);
double TEST_FP_OP_D_INTERNAL_fmax(double _d0, double _d1);
double TEST_FP_OP_D_INTERNAL_fmin(double _d0, double _d1);

void TEST_FP_OP1_D(double _d0, double _result, int _fflags);
double TEST_FP_OP_D_INTERNAL_fsqrt(double _d0);



void TEST_FCLASS_S(float _f0, int _result);
int TEST_FP_OP_S_INTERNAL_fclass(float _f0);

void TEST_FSGNJS(float _f0, float _f1, float _result, char _inst);
float TEST_FP_OP_S_INTERNAL_fsgnj(float _f0, float _f1);
float TEST_FP_OP_S_INTERNAL_fsgnjn(float _f0, float _f1);
float TEST_FP_OP_S_INTERNAL_fsgnjx(float _f0, float _f1);

void TEST_FP_CMP_OP_S(float _f0, float _f1, int _result, int _fflags, char _inst);
int TEST_FP_OP_S_INTERNAL_feq(float _f0, float _f1);
int TEST_FP_OP_S_INTERNAL_fle(float _f0, float _f1);
int TEST_FP_OP_S_INTERNAL_flt(float _f0, float _f1);

void TEST_FP_INT_OP_S(float _f0, int _result, int _fflags, char _inst);
int TEST_FP_OP_S_INTERNAL_fcvt_w_s(float _f0);
int TEST_FP_OP_S_INTERNAL_fcvt_wu_s(float _f0);

void TEST_INT_FP_OP_S(int _d0, float _result, char _inst);
float TEST_FP_OP_S_INTERNAL_fcvt_s_w(int _d0);
float TEST_FP_OP_S_INTERNAL_fcvt_s_wu(int _d0);

void TEST_FP_OP2_S(float _f0, float _f1, float _result, int _fflags, char _inst);
float TEST_FP_OP_S_INTERNAL_fadd(float _f0, float _f1);
float TEST_FP_OP_S_INTERNAL_fsub(float _f0, float _f1);
float TEST_FP_OP_S_INTERNAL_fmul(float _f0, float _f1);
float TEST_FP_OP_S_INTERNAL_fdiv(float _f0, float _f1);
float TEST_FP_OP_S_INTERNAL_fmax(float _f0, float _f1);
float TEST_FP_OP_S_INTERNAL_fmin(float _f0, float _f1);

void TEST_FP_OP1_S(float _f0, float _result, int _fflags);
float TEST_FP_OP_S_INTERNAL_fsqrt(float _f0);

void TEST_FP_OP3_S(float _f0, float _f1, float _f2, float _result, int _fflags, char _inst);
float TEST_FP_OP_S_INTERNAL_fmadd(float _f0, float _f1, float _f2);
float TEST_FP_OP_S_INTERNAL_fmsub(float _f0, float _f1, float _f2);
float TEST_FP_OP_S_INTERNAL_fnmadd(float _f0, float _f1, float _f2);
float TEST_FP_OP_S_INTERNAL_fnmsub(float _f0, float _f1, float _f2);

#endif
