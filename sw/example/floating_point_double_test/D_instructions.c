#include <neorv32.h>
#include <D_instructions.h>

typedef union converter{
        double value;
        long long int buff;
  } converter;

typedef union converter_s{
        float value;
        int buff;
  } converter_s;

void pass_process(int _num)
{
  switch (_num)
  {
    case 1:
      neorv32_uart0_printf("Correct instruction...\n");
      break;
    case 2:
      neorv32_uart0_printf("Correct COMPARE...\n");
      break;
    case 3:
      neorv32_uart0_printf("Correct ARITHMETIC...\n");
      break;
    case 4:
      neorv32_uart0_printf("Correct DIVISION...\n");
      break;
    case 5:
      neorv32_uart0_printf("Correct SQUARE ROOT...\n");
      break;
    case 6:
      neorv32_uart0_printf("Correct DOUBLE to INT32...\n");
      break;
    case 7:
      neorv32_uart0_printf("Correct INT32 to DOUBLE...\n");
      break;
    case 8:
      neorv32_uart0_printf("Correct MAX/MIN...\n");
      break;
    case 9:
      neorv32_uart0_printf("Correct FUSED...\n");
      break;
    case 10:
      neorv32_uart0_printf("Correct FLCASS...\n");
      break;
    case 11:
      neorv32_uart0_printf("Correct SIGN INJECT...\n");
      break;
    case 12:
      neorv32_uart0_printf("Correct FLOAT to INT32...\n");
      break;
    case 13:
      neorv32_uart0_printf("Correct INT32 to FLOAT...\n");
      break;
    default:
      neorv32_uart0_printf("Undefined correct instruction...\n");
      break;
  };
}
void fail_process(int _num)
{
  switch (_num)
  {
    case 2:
      neorv32_uart0_printf("== Error with COMPARE!\n");
      break;
    case 3:
      neorv32_uart0_printf("== Error with ARITHMETIC!\n");
      break;
    case 4:
      neorv32_uart0_printf("== Error with DIVISION!\n");
      break;
    case 5:
      neorv32_uart0_printf("== Error with SQUARE ROOT!\n");
      break;
    case 6:
      neorv32_uart0_printf("== Error with DOUBLE to INT32!\n");
      break;
    case 7:
      neorv32_uart0_printf("== Error with INT32 to DOUBLE!\n");
      break;
    case 8:
      neorv32_uart0_printf("== Error with MAX/MIN!\n");
      break;
    case 9:
      neorv32_uart0_printf("== Error with FUSED!\n");
      break;
    case 10:
      neorv32_uart0_printf("== Error with FLCASS!\n");
      break;
    case 11:
      neorv32_uart0_printf("== Error with SIGN INJECT!\n");
      break;
    case 12:
      neorv32_uart0_printf("== Error with FLOAT to INT32!\n");
      break;
    case 13:
      neorv32_uart0_printf("== Error with INT32 to FLOAT!\n");
      break;
    default:
      neorv32_uart0_printf("== Undefined error!\n");
      break;
  };
}

bool areEqual(double _d0, double _d1)
{
  converter buff0;
  converter buff1;
  buff0.value = _d0;
  buff1.value = _d1;
  return buff0.buff == buff1.buff;
}

bool areEqual_s(float _f0, float _f1)
{
  converter_s buff0;
  converter_s buff1;
  buff0.value = _f0;
  buff1.value = _f1;
  return buff0.buff == buff1.buff;
}

void TEST_FP_CMP_OP_D(double _d0, double _d1, int _result, int _fflags, char _inst)
{
  int res;
  int res_fflags;
  switch (_inst)
  {
    case '=': // equal
      res = TEST_FP_OP_D_INTERNAL_feq(_d0, _d1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(res != _result || res_fflags != _fflags)
        fail_process(2);
      else
        pass_process(2);
      break;

    case '~': // less equal
      res = TEST_FP_OP_D_INTERNAL_fle(_d0, _d1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(res != _result || res_fflags != _fflags)
        fail_process(2);
      else
        pass_process(2);
      break;

    case '<': // less than
      res = TEST_FP_OP_D_INTERNAL_flt(_d0, _d1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(res != _result || res_fflags != _fflags)
        fail_process(2);
      else
        pass_process(2);
      break;

    default:
      break;
  }
  neorv32_cpu_csr_write(CSR_FFLAGS, 0);
}

void TEST_FP_OP3_D(double _d0, double _d1, double _d2, double _result, int _fflags, char _inst)
{
  double res;
  int res_fflags;
  switch (_inst)
  {
    case '+':
      res = TEST_FP_OP_D_INTERNAL_fmadd(_d0, _d1, _d2);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual(_result, res) || res_fflags != _fflags)
        fail_process(9);
      else
        pass_process(9);
      break;

    case '-':
      res = TEST_FP_OP_D_INTERNAL_fmsub(_d0, _d1, _d2);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual(_result, res) || res_fflags != _fflags)
        fail_process(9);
      else
        pass_process(9);
      break;

    case '>':
      res = TEST_FP_OP_D_INTERNAL_fnmadd(_d0, _d1, _d2);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual(_result, res) || res_fflags != _fflags)
        fail_process(9);
      else
        pass_process(9);
      break;

    case '<':
      res = TEST_FP_OP_D_INTERNAL_fnmsub(_d0, _d1, _d2);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual(_result, res) || res_fflags != _fflags)
        fail_process(9);
      else
        pass_process(9);
      break;

    default:
      break;
  }
  neorv32_cpu_csr_write(CSR_FFLAGS, 0);
}

void TEST_FP_OP2_D(double _d0, double _d1, double _result, int _fflags, char _inst)
{
  double res;
  int res_fflags;
  switch (_inst)
  {
    case '+': //addition
      res = TEST_FP_OP_D_INTERNAL_fadd(_d0, _d1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual(_result, res) || res_fflags != _fflags)
        fail_process(3);
      else
        pass_process(3);
      break;

    case '-': // substract
      res = TEST_FP_OP_D_INTERNAL_fsub(_d0, _d1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual(_result, res) || res_fflags != _fflags)
        fail_process(3);
      else
        pass_process(3);
      break;

    case '*': // multiply
      res = TEST_FP_OP_D_INTERNAL_fmul(_d0, _d1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual(_result, res) || res_fflags != _fflags)
        fail_process(3);
      else
        pass_process(3);
      break;

    case '/': // div
      res = TEST_FP_OP_D_INTERNAL_fdiv(_d0, _d1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual(_result, res) || res_fflags != _fflags)
        fail_process(4);
      else
        pass_process(4);
      break;

    case '>': // max
      res = TEST_FP_OP_D_INTERNAL_fmax(_d0, _d1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual(_result, res) || res_fflags != _fflags)
        fail_process(8);
      else
        pass_process(8);
      break;

    case '<': // min
      res = TEST_FP_OP_D_INTERNAL_fmin(_d0, _d1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual(_result, res) || res_fflags != _fflags)
        fail_process(8);
      else
        pass_process(8);
      break;

    default:
      break;
  }
  neorv32_cpu_csr_write(CSR_FFLAGS, 0);
}

void TEST_FP_OP1_D(double _d0, double _result, int _fflags)
{
  double res;
  int res_fflags;

  res = TEST_FP_OP_D_INTERNAL_fsqrt(_d0);
  res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);

  if(!areEqual(_result, res) || res_fflags != _fflags)
    fail_process(5);
  else
    pass_process(5);

  neorv32_cpu_csr_write(CSR_FFLAGS, 0);
}

void TEST_FP_INT_OP_D(double _d0, int _result, int _fflags, char _inst)
{
  int res;
  int res_fflags;

  switch (_inst)
  {
    case 's': // signed
      res = TEST_FP_OP_D_INTERNAL_fcvt_w_d(_d0);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if (res != _result || res_fflags != _fflags)
        fail_process(6);
      else
        pass_process(6);
      break;

    case 'u': // unsigned
      res = TEST_FP_OP_D_INTERNAL_fcvt_wu_d(_d0);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if (res != _result || res_fflags != _fflags)
        fail_process(6);
      else
        pass_process(6);
      break;

    default:
      break;
  }
  neorv32_cpu_csr_write(CSR_FFLAGS, 0);
}

void TEST_INT_FP_OP_D(int _d0, double _result, char _inst)
{
  double res;

  switch (_inst)
  {
    case 's': // signed
      res = TEST_FP_OP_D_INTERNAL_fcvt_d_w(_d0);
      if (!areEqual(_result, res))
        fail_process(7);
      else
        pass_process(7);
      break;

    case 'u': // unsigned
      res = TEST_FP_OP_D_INTERNAL_fcvt_d_wu(_d0);
      if (!areEqual(_result, res))
        fail_process(7);
      else
        pass_process(7);
      break;

    default:
      break;
  }
  neorv32_cpu_csr_write(CSR_FFLAGS, 0);
}

void TEST_FCLASS_D(double _d0, int _result)
{
  int res;

  res = TEST_FP_OP_D_INTERNAL_fclass(_d0);
  if (res != _result)
    fail_process(10);
  else
    pass_process(10);

  neorv32_cpu_csr_write(CSR_FFLAGS, 0);
}

void TEST_FSGNJD(double _d0, double _d1, double _result, char _inst)
{
  double res;

  switch (_inst)
  {
    case '+':
      res = TEST_FP_OP_D_INTERNAL_fsgnj(_d0, _d1);
      if (!areEqual(_result, res))
        fail_process(11);
      else
        pass_process(11);
      break;

    case '-':
      res = TEST_FP_OP_D_INTERNAL_fsgnjn(_d0, _d1);
      if (!areEqual(_result, res))
        fail_process(11);
      else
        pass_process(11);
      break;

    case 'x':
      res = TEST_FP_OP_D_INTERNAL_fsgnjx(_d0, _d1);
      if (!areEqual(_result, res))
        fail_process(11);
      else
        pass_process(11);
      break;

    default:
      break;
  }
  neorv32_cpu_csr_write(CSR_FFLAGS, 0);
}

void TEST_FCLASS_S(float _f0, int _result)
{
  int res;

  res = TEST_FP_OP_S_INTERNAL_fclass(_f0);
  if (res != _result)
    fail_process(10);
  else
    pass_process(10);

  neorv32_cpu_csr_write(CSR_FFLAGS, 0);
}

void TEST_FSGNJS(float _f0, float _f1, float _result, char _inst)
{
  float res;

  switch (_inst)
  {
    case '+':
      res = TEST_FP_OP_S_INTERNAL_fsgnj(_f0, _f1);
      if (!areEqual_s(_result, res))
        fail_process(11);
      else
        pass_process(11);
      break;

    case '-':
      res = TEST_FP_OP_S_INTERNAL_fsgnjn(_f0, _f1);
      if (!areEqual_s(_result, res))
        fail_process(11);
      else
        pass_process(11);
      break;

    case 'x':
      res = TEST_FP_OP_S_INTERNAL_fsgnjx(_f0, _f1);
      if (!areEqual_s(_result, res))
        fail_process(11);
      else
        pass_process(11);
      break;

    default:
      break;
  }
  neorv32_cpu_csr_write(CSR_FFLAGS, 0);
}

void TEST_FP_CMP_OP_S(float _f0, float _f1, int _result, int _fflags, char _inst)
{
  int res;
  int res_fflags;
  switch (_inst)
  {
    case '=': // equal
      res = TEST_FP_OP_S_INTERNAL_feq(_f0, _f1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(res != _result || res_fflags != _fflags)
        fail_process(2);
      else
        pass_process(2);
      break;

    case '~': // less equal
      res = TEST_FP_OP_S_INTERNAL_fle(_f0, _f1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(res != _result || res_fflags != _fflags)
        fail_process(2);
      else
        pass_process(2);
      break;

    case '<': // less than
      res = TEST_FP_OP_S_INTERNAL_flt(_f0, _f1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(res != _result || res_fflags != _fflags)
        fail_process(2);
      else
        pass_process(2);
      break;

    default:
      break;
  }
  neorv32_cpu_csr_write(CSR_FFLAGS, 0);
}

void TEST_FP_INT_OP_S(float _f0, int _result, int _fflags, char _inst)
{
  int res;
  int res_fflags;

  switch (_inst)
  {
    case 's': // signed
      res = TEST_FP_OP_S_INTERNAL_fcvt_w_s(_f0);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if (res != _result || res_fflags != _fflags)
        fail_process(12);
      else
        pass_process(12);
      break;

    case 'u': // unsigned
      res = TEST_FP_OP_S_INTERNAL_fcvt_wu_s(_f0);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if (res != _result || res_fflags != _fflags)
        fail_process(12);
      else
        pass_process(12);
      break;

    default:
      break;
  }
  neorv32_cpu_csr_write(CSR_FFLAGS, 0);
}

void TEST_INT_FP_OP_S(int _d0, float _result, char _inst)
{
  float res;
  
  switch (_inst)
  {
    case 's': // signed
      res = TEST_FP_OP_S_INTERNAL_fcvt_s_w(_d0);
      if (!areEqual_s(_result, res))
        fail_process(13);
      else
        pass_process(13);
      break;

    case 'u': // unsigned
      res = TEST_FP_OP_S_INTERNAL_fcvt_s_wu(_d0);
      if (!areEqual_s(_result, res))
        fail_process(13);
      else
        pass_process(13);
      break;

    default:
      break;
  }
  neorv32_cpu_csr_write(CSR_FFLAGS, 0);
}

void TEST_FP_OP2_S(float _f0, float _f1, float _result, int _fflags, char _inst)
{
  float res;
  int res_fflags;
  switch (_inst)
  {
    case '+': //addition
      res = TEST_FP_OP_S_INTERNAL_fadd(_f0, _f1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual_s(_result, res) || res_fflags != _fflags)
        fail_process(3);
      else
        pass_process(3);
      break;

    case '-': // substract
      res = TEST_FP_OP_S_INTERNAL_fsub(_f0, _f1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual_s(_result, res) || res_fflags != _fflags)
        fail_process(3);
      else
        pass_process(3);
      break;

    case '*': // multiply
      res = TEST_FP_OP_S_INTERNAL_fmul(_f0, _f1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual_s(_result, res) || res_fflags != _fflags)
        fail_process(3);
      else
        pass_process(3);
      break;

    case '/': // div
      res = TEST_FP_OP_S_INTERNAL_fdiv(_f0, _f1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual_s(_result, res) || res_fflags != _fflags)
        fail_process(4);
      else
        pass_process(4);
      break;

    case '>': // max
      res = TEST_FP_OP_S_INTERNAL_fmax(_f0, _f1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual_s(_result, res) || res_fflags != _fflags)
        fail_process(8);
      else
        pass_process(8);
      break;

    case '<': // min
      res = TEST_FP_OP_S_INTERNAL_fmin(_f0, _f1);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual_s(_result, res) || res_fflags != _fflags)
        fail_process(8);
      else
        pass_process(8);
      break;

    default:
      break;
  }
  neorv32_cpu_csr_write(CSR_FFLAGS, 0);
}

void TEST_FP_OP1_S(float _f0, float _result, int _fflags)
{
  float res;
  int res_fflags;

  res = TEST_FP_OP_S_INTERNAL_fsqrt(_f0);
  res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);

  if(!areEqual_s(_result, res) || res_fflags != _fflags)
    fail_process(5);
  else
    pass_process(5);

  neorv32_cpu_csr_write(CSR_FFLAGS, 0);
}

void TEST_FP_OP3_S(float _f0, float _f1, float _f2, float _result, int _fflags, char _inst)
{
  float res;
  int res_fflags;
  switch (_inst)
  {
    case '+':
      res = TEST_FP_OP_S_INTERNAL_fmadd(_f0, _f1, _f2);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual_s(_result, res) || res_fflags != _fflags)
        fail_process(9);
      else
        pass_process(9);
      break;

    case '-':
      res = TEST_FP_OP_S_INTERNAL_fmsub(_f0, _f1, _f2);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual_s(_result, res) || res_fflags != _fflags)
        fail_process(9);
      else
        pass_process(9);
      break;

    case '>':
      res = TEST_FP_OP_S_INTERNAL_fnmadd(_f0, _f1, _f2);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual_s(_result, res) || res_fflags != _fflags)
        fail_process(9);
      else
        pass_process(9);
      break;

    case '<':
      res = TEST_FP_OP_S_INTERNAL_fnmsub(_f0, _f1, _f2);
      res_fflags = neorv32_cpu_csr_read(CSR_FFLAGS);
      if(!areEqual_s(_result, res) || res_fflags != _fflags)
        fail_process(9);
      else
        pass_process(9);
      break;

    default:
      break;
  }
  neorv32_cpu_csr_write(CSR_FFLAGS, 0);
}
