.file "D_instructions.S"
.text
.global TEST_FP_OP_S_INTERNAL_fclass
.global TEST_FP_OP_S_INTERNAL_fsgnj
.global TEST_FP_OP_S_INTERNAL_fsgnjn
.global TEST_FP_OP_S_INTERNAL_fsgnjx
.global TEST_FP_OP_S_INTERNAL_feq
.global TEST_FP_OP_S_INTERNAL_fle
.global TEST_FP_OP_S_INTERNAL_flt
.global TEST_FP_OP_S_INTERNAL_fcvt_w_s
.global TEST_FP_OP_S_INTERNAL_fcvt_wu_s
.global TEST_FP_OP_S_INTERNAL_fcvt_s_w
.global TEST_FP_OP_S_INTERNAL_fcvt_s_wu
.global TEST_FP_OP_S_INTERNAL_fadd
.global TEST_FP_OP_S_INTERNAL_fsub
.global TEST_FP_OP_S_INTERNAL_fmul
.global TEST_FP_OP_S_INTERNAL_fdiv
.global TEST_FP_OP_S_INTERNAL_fmax
.global TEST_FP_OP_S_INTERNAL_fmin
.global TEST_FP_OP_S_INTERNAL_fsqrt
.global TEST_FP_OP_S_INTERNAL_fmadd
.global TEST_FP_OP_S_INTERNAL_fmsub
.global TEST_FP_OP_S_INTERNAL_fnmadd
.global TEST_FP_OP_S_INTERNAL_fnmsub

.global TEST_FP_OP_D_INTERNAL_feq
.global TEST_FP_OP_D_INTERNAL_fle
.global TEST_FP_OP_D_INTERNAL_flt
.global TEST_FP_OP_D_INTERNAL_fmadd
.global TEST_FP_OP_D_INTERNAL_fmsub
.global TEST_FP_OP_D_INTERNAL_fnmadd
.global TEST_FP_OP_D_INTERNAL_fnmsub
.global TEST_FP_OP_D_INTERNAL_fadd
.global TEST_FP_OP_D_INTERNAL_fsub
.global TEST_FP_OP_D_INTERNAL_fmul
.global TEST_FP_OP_D_INTERNAL_fdiv
.global TEST_FP_OP_D_INTERNAL_fmax
.global TEST_FP_OP_D_INTERNAL_fmin
.global TEST_FP_OP_D_INTERNAL_fsqrt
.global TEST_FP_OP_D_INTERNAL_fcvt_w_d
.global TEST_FP_OP_D_INTERNAL_fcvt_wu_d
.global TEST_FP_OP_D_INTERNAL_fcvt_d_w
.global TEST_FP_OP_D_INTERNAL_fcvt_d_wu
.global TEST_FP_OP_D_INTERNAL_fclass
.global TEST_FP_OP_D_INTERNAL_fsgnj
.global TEST_FP_OP_D_INTERNAL_fsgnjn
.global TEST_FP_OP_D_INTERNAL_fsgnjx

TEST_FP_OP_S_INTERNAL_fclass:
  fclass.s a0, fa0
  ret

TEST_FP_OP_S_INTERNAL_fsgnj:
  fsgnj.s fa0, fa0, fa1
  ret

TEST_FP_OP_S_INTERNAL_fsgnjn:
  fsgnjn.s fa0, fa0, fa1
  ret

TEST_FP_OP_S_INTERNAL_fsgnjx:
  fsgnjx.s fa0, fa0, fa1
  ret

TEST_FP_OP_S_INTERNAL_feq:
  feq.s a0, fa0, fa1
  ret

TEST_FP_OP_S_INTERNAL_fle:
  fle.s a0, fa0, fa1
  ret

TEST_FP_OP_S_INTERNAL_flt:
  flt.s a0, fa0, fa1
  ret

TEST_FP_OP_S_INTERNAL_fcvt_w_s:
  fcvt.w.s a0, fa0
  ret

TEST_FP_OP_S_INTERNAL_fcvt_wu_s:
  fcvt.wu.s a0, fa0
  ret

TEST_FP_OP_S_INTERNAL_fcvt_s_w:
  fcvt.s.w fa0, a0
  ret

TEST_FP_OP_S_INTERNAL_fcvt_s_wu:
  fcvt.s.wu fa0, a0
  ret

TEST_FP_OP_S_INTERNAL_fadd:
  fadd.s fa0, fa0, fa1
  ret

TEST_FP_OP_S_INTERNAL_fsub:
  fsub.s fa0, fa0, fa1
  ret

TEST_FP_OP_S_INTERNAL_fmul:
  fmul.s fa0, fa0, fa1
  ret

TEST_FP_OP_S_INTERNAL_fdiv:
  fdiv.s fa0, fa0, fa1
  ret

TEST_FP_OP_S_INTERNAL_fmax:
  fmax.s fa0, fa0, fa1
  ret

TEST_FP_OP_S_INTERNAL_fmin:
  fmin.s fa0, fa0, fa1
  ret

TEST_FP_OP_S_INTERNAL_fsqrt:
  fsqrt.s fa0, fa0
  ret

TEST_FP_OP_S_INTERNAL_fmadd:
  fmadd.s fa0, fa0, fa1, fa2
  ret

TEST_FP_OP_S_INTERNAL_fmsub:
  fmsub.s fa0, fa0, fa1, fa2
  ret

TEST_FP_OP_S_INTERNAL_fnmadd:
  fnmadd.s fa0, fa0, fa1, fa2
  ret

TEST_FP_OP_S_INTERNAL_fnmsub:
  fnmsub.s fa0, fa0, fa1, fa2
  ret


TEST_FP_OP_D_INTERNAL_feq:
  feq.d a0, fa0, fa1
  ret

TEST_FP_OP_D_INTERNAL_fle:
  fle.d a0, fa0, fa1
  ret

TEST_FP_OP_D_INTERNAL_flt:
  flt.d a0, fa0, fa1
  ret

TEST_FP_OP_D_INTERNAL_fmadd:
  fmadd.d fa0, fa0, fa1, fa2
  ret

TEST_FP_OP_D_INTERNAL_fmsub:
  fmsub.d fa0, fa0, fa1, fa2
  ret

TEST_FP_OP_D_INTERNAL_fnmadd:
  fnmadd.d fa0, fa0, fa1, fa2
  ret

TEST_FP_OP_D_INTERNAL_fnmsub:
  fnmsub.d fa0, fa0, fa1, fa2
  ret

TEST_FP_OP_D_INTERNAL_fadd:
  fadd.d fa0, fa0, fa1
  ret

TEST_FP_OP_D_INTERNAL_fsub:
  fsub.d fa0, fa0, fa1
  ret

TEST_FP_OP_D_INTERNAL_fmul:
  fmul.d fa0, fa0, fa1
  ret

TEST_FP_OP_D_INTERNAL_fdiv:
  fdiv.d fa0, fa0, fa1
  ret

TEST_FP_OP_D_INTERNAL_fmax:
  fmax.d fa0, fa0, fa1
  ret

TEST_FP_OP_D_INTERNAL_fmin:
  fmin.d fa0, fa0, fa1
  ret

TEST_FP_OP_D_INTERNAL_fsqrt:
  fsqrt.d fa0, fa0
  ret

TEST_FP_OP_D_INTERNAL_fcvt_w_d:
  fcvt.w.d a0, fa0
  ret

TEST_FP_OP_D_INTERNAL_fcvt_wu_d:
  fcvt.wu.d a0, fa0
  ret

TEST_FP_OP_D_INTERNAL_fcvt_d_w:
  fcvt.d.w fa0, a0
  ret

TEST_FP_OP_D_INTERNAL_fcvt_d_wu:
  fcvt.d.wu fa0, a0
  ret

TEST_FP_OP_D_INTERNAL_fclass:
  fclass.d a0, fa0
  ret

TEST_FP_OP_D_INTERNAL_fsgnj:
  fsgnj.d fa0, fa0, fa1
  ret

TEST_FP_OP_D_INTERNAL_fsgnjn:
  fsgnjn.d fa0, fa0, fa1
  ret

TEST_FP_OP_D_INTERNAL_fsgnjx:
  fsgnjx.d fa0, fa0, fa1
  ret
