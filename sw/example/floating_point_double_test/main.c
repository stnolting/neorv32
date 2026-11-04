//based on the FD ISA tests: https://github.com/riscv-software-src/riscv-tests
#include <D_instructions.h>
#include <neorv32.h>

typedef union converter{      // have to use a union,
        double value;         // otherwise NANs and INF would
        long long int buff;   // compile as normal doubles
  } converter;

typedef union converter_s{      
        float value;         
        int buff;   
  } converter_s;

int main()
{
  neorv32_uart_setup(NEORV32_UART0, 19200, 0);
  
  converter NAN;    // quiet NAN
  converter s_NAN;  // signal NAN
  NAN.buff = qNaN;
  s_NAN.buff = sNaN;
  
  converter p_INF;  // positive infinity
  converter n_INF;  // negative infinity
  p_INF.buff = pINF;
  n_INF.buff = nINF;
  
  converter p_SUB;  // positive subnormal
  converter n_SUB;  // negative subnormal
  p_SUB.buff = pSUB;
  n_SUB.buff = nSUB;
  
  converter n_ZERO; // negative zero
  n_ZERO.buff = nZRO;
  
  converter_s NANs;
  converter_s s_NANs;
  NANs.buff = qNaNs;
  s_NANs.buff = sNaNs;
  
  converter_s p_INFs;
  converter_s n_INFs;
  p_INFs.buff = pINFs;
  n_INFs.buff = nINFs;
  
  converter_s p_SUBs;
  converter_s n_SUBs;
  p_SUBs.buff = pSUBs;
  n_SUBs.buff = nSUBs;
  
  converter_s n_ZEROs;
  n_ZEROs.buff = nZROs;
  
  neorv32_uart0_printf("=== ISA TESTING ===========\n");
  neorv32_uart0_printf("===========================\n");
  
  neorv32_cpu_csr_write(CSR_FRM, 0);  // Round to nearest
  
  neorv32_uart0_printf("\n=== FLOAT =================\n");
  neorv32_uart0_printf("===========================\n");
  
  neorv32_uart0_printf("\n=== FLCASS ================\n\n");
  
  TEST_FCLASS_S(n_INFs.value, 1<<0);
  TEST_FCLASS_S(-1.0, 1<<1);
  TEST_FCLASS_S(n_SUBs.value, 1<<2);
  TEST_FCLASS_S(n_ZEROs.value, 1<<3);
  TEST_FCLASS_S(0.0, 1<<4);
  TEST_FCLASS_S(p_SUBs.value, 1<<5);
  TEST_FCLASS_S(1.0, 1<<6);
  TEST_FCLASS_S(p_INFs.value, 1<<7);
  TEST_FCLASS_S(s_NANs.value, 1<<8);
  TEST_FCLASS_S(NANs.value, 1<<9);
  
  neorv32_uart0_printf("\n=== SIGN INJECT ===========\n\n");
  
  TEST_FSGNJS(0, 0, 0, '+');
  TEST_FSGNJS(0, -1, n_ZEROs.value, '+');
  TEST_FSGNJS(-1, 0, 1, '+');
  TEST_FSGNJS(-1, -1, -1, '+');
  
  TEST_FSGNJS(0, 0, n_ZEROs.value, '-');
  TEST_FSGNJS(0, -1, 0, '-');
  TEST_FSGNJS(-1, 0, -1, '-');
  TEST_FSGNJS(-1, -1, 1, '-');
  
  TEST_FSGNJS(0, 0, 0, 'x');
  TEST_FSGNJS(0, -1, n_ZEROs.value, 'x');
  TEST_FSGNJS(-1, 0, -1, 'x');
  TEST_FSGNJS(-1, -1, 1, 'x');
  
  neorv32_uart0_printf("\n=== COMPARE ===============\n\n");
  
  TEST_FP_CMP_OP_S(-1.36, -1.36, 1, 0x00, '=');
  TEST_FP_CMP_OP_S(-1.36, -1.36, 1, 0x00, '~');
  TEST_FP_CMP_OP_S(-1.36, -1.36, 0, 0x00, '<');
  
  TEST_FP_CMP_OP_S(-1.37, -1.36, 0, 0x00, '=');
  TEST_FP_CMP_OP_S(-1.37, -1.36, 1, 0x00, '~');
  TEST_FP_CMP_OP_S(-1.37, -1.36, 1, 0x00, '<');
  
  TEST_FP_CMP_OP_S(NANs.value, 0, 0, 0x00, '=');
  TEST_FP_CMP_OP_S(NANs.value, NANs.value, 0, 0x00, '=');
  TEST_FP_CMP_OP_S(s_NANs.value, 0, 0, 0x10, '=');
  
  TEST_FP_CMP_OP_S(NANs.value, 0, 0, 0x10, '<');
  TEST_FP_CMP_OP_S(NANs.value, NANs.value, 0, 0x10, '<');
  TEST_FP_CMP_OP_S(s_NANs.value, 0, 0, 0x10, '<');
  
  TEST_FP_CMP_OP_S(NANs.value, 0, 0, 0x10, '~');
  TEST_FP_CMP_OP_S(NANs.value, NANs.value, 0, 0x10, '~');
  TEST_FP_CMP_OP_S(s_NANs.value, 0, 0, 0x10, '~');
  
  neorv32_uart0_printf("\n=== CONVERT TO INT ========\n\n");
  
  neorv32_cpu_csr_write(CSR_FRM, 1);  // Round towards zero
  
  TEST_FP_INT_OP_S(-1.1, -1, 0x01, 's');
  TEST_FP_INT_OP_S(-1.0, -1, 0x00, 's');
  TEST_FP_INT_OP_S(-0.9, 0, 0x01, 's');
  TEST_FP_INT_OP_S(0.9, 0, 0x01, 's');
  TEST_FP_INT_OP_S(1.0, 1, 0x00, 's');
  TEST_FP_INT_OP_S(1.1, 1, 0x01, 's');
  TEST_FP_INT_OP_S(-3e9, -1<<31, 0x10, 's');
  TEST_FP_INT_OP_S(3e9, (1<<31)-1, 0x10, 's');
  
  TEST_FP_INT_OP_S(-1.1, 0, 0x10, 'u');
  TEST_FP_INT_OP_S(-1.0, 0, 0x10, 'u');
  TEST_FP_INT_OP_S(-0.9, 0, 0x01, 'u');
  TEST_FP_INT_OP_S(0.9, 0, 0x01, 'u');
  TEST_FP_INT_OP_S(1.0, 1, 0x00, 'u');
  TEST_FP_INT_OP_S(1.1, 1, 0x01, 'u');
  TEST_FP_INT_OP_S(-3e9, 0, 0x10, 'u');
  TEST_FP_INT_OP_S(3e9, 3000000000, 0x00, 'u');
  
  TEST_FP_INT_OP_S(NANs.value, 0x7fffffff, 0x10, 's');
  TEST_FP_INT_OP_S(n_INFs.value, 0x80000000, 0x10, 's');
  TEST_FP_INT_OP_S(p_INFs.value, 0x7fffffff, 0x10, 's');
  TEST_FP_INT_OP_S(NANs.value, 0xffffffff, 0x10, 'u');
  TEST_FP_INT_OP_S(n_INFs.value, 0, 0x10, 'u');
  TEST_FP_INT_OP_S(p_INFs.value, 0xffffffff, 0x10, 'u');
  
  neorv32_cpu_csr_write(CSR_FRM, 0);  // Round to nearest
  
  neorv32_uart0_printf("\n=== CONVERT TO FLOAT ======\n\n");
  
  TEST_INT_FP_OP_S(2, 2.0, 's');
  TEST_INT_FP_OP_S(-2, -2.0, 's');
  
  TEST_INT_FP_OP_S(2, 2.0, 'u');
  TEST_INT_FP_OP_S(-2, 4.2949673e9, 'u');
  
  neorv32_uart0_printf("\n=== MAX/MIN ===============\n\n");
  
  TEST_FP_OP2_S(2.5, 1.0, 1.0, 0x00, '<');
  TEST_FP_OP2_S(-1235.1, 1.1, -1235.1, 0x00, '<');
  TEST_FP_OP2_S(1.1, -1235.1, -1235.1, 0x00, '<');
  TEST_FP_OP2_S(NANs.value, -1235.1, -1235.1, 0x00, '<');
  TEST_FP_OP2_S(3.14159265, 0.00000001, 0.00000001, 0x00, '<');
  TEST_FP_OP2_S(-1.0, -2.0, -2.0, 0x00, '<');
  
  TEST_FP_OP2_S(2.5, 1.0, 2.5, 0x00, '>');
  TEST_FP_OP2_S(-1235.1, 1.1, 1.1, 0x00, '>');
  TEST_FP_OP2_S(1.1, -1235.1, 1.1, 0x00, '>');
  TEST_FP_OP2_S(NANs.value, -1235.1, -1235.1, 0x00, '>');
  TEST_FP_OP2_S(3.14159265, 0.00000001, 3.14159265, 0x00, '>');
  TEST_FP_OP2_S(-1.0, -2.0, -1.0, 0x00, '>');
  
  TEST_FP_OP2_S(s_NANs.value, 1.0, 1.0, 0x10, '>');
  TEST_FP_OP2_S(NANs.value, NANs.value, NANs.value, 0x00, '>');
  
  TEST_FP_OP2_S(n_ZEROs.value, 0.0, n_ZEROs.value, 0x00, '<');
  TEST_FP_OP2_S(0.0, n_ZEROs.value, n_ZEROs.value, 0x00, '<');
  TEST_FP_OP2_S(n_ZEROs.value, 0.0, 0.0, 0x00, '>');
  TEST_FP_OP2_S(0.0, n_ZEROs.value, 0.0, 0x00, '>');
  
  neorv32_uart0_printf("\n=== SIMPLE ARITHMETIC =====\n\n");
  
  TEST_FP_OP2_S(2.5, 1.0, 3.5, 0x00, '+');
  TEST_FP_OP2_S(-1235.1, 1.1, -1234 , 0x01, '+');
  TEST_FP_OP2_S(3.14159265, 0.00000001, 3.14159266, 0x01, '+');
  
  TEST_FP_OP2_S(2.5, 1.0, 1.5, 0x00, '-');
  TEST_FP_OP2_S(-1235.1, -1.1, -1234, 0x01, '-');
  TEST_FP_OP2_S(3.14159265, 0.00000001, 3.1415926400000001, 0x01, '-');
  
  TEST_FP_OP2_S(2.5, 1.0, 2.5, 0x00, '*');
  TEST_FP_OP2_S(-1235.1, -1.1, 1358.61, 0x01, '*');
  TEST_FP_OP2_S(3.14159265, 0.00000001, 3.14159265e-8, 0x01, '*');
  
  TEST_FP_OP2_S(p_INFs.value, p_INFs.value, NANs.value, 0x10, '-');
  
  neorv32_uart0_printf("\n=== DIV AND SQRT ==========\n\n");
  
  TEST_FP_OP2_S(3.14159265, 2.71828182, 1.1557273520668288, 0x01, '/');
  TEST_FP_OP2_S(-1234, 1235.1, -0.9991093838555584, 0x01, '/');
  TEST_FP_OP2_S(3.14159265, 1.0, 3.14159265, 0x00, '/');
  
  TEST_FP_OP1_S(3.14159265, 1.7724538498928541, 0x01);
  TEST_FP_OP1_S(10000.0, 100.0, 0x00);
  TEST_FP_OP1_S(171.0, 13.076696830622021, 0x01);
  TEST_FP_OP1_S(1.60795e-7, 0.00040099251863345283320230749702, 0x01);
  
  neorv32_uart0_printf("\n=== FUSED MULTIPLY ========\n\n");
  
  TEST_FP_OP3_S(1.0, 2.5, 1.0, 3.5, 0x00, '+');
  TEST_FP_OP3_S(-1.0, -1235.1, 1.1, 1236.2, 0x01, '+');   // doesn't rise the flag
  TEST_FP_OP3_S(2.0, -5.0, -2.0, -12.0, 0x00, '+');
  
  TEST_FP_OP3_S(1.0, 2.5, 1.0, -3.5, 0x00, '>');
  TEST_FP_OP3_S(-1.0, -1235.1, 1.1, -1236.2, 0x01, '>');  // doesn't rise the flag
  TEST_FP_OP3_S(2.0, -5.0, -2.0, 12.0, 0x00, '>');
  
  TEST_FP_OP3_S(1.0, 2.5, 1.0, 1.5, 0x00, '-');
  TEST_FP_OP3_S(-1.0, -1235.1, 1.1, 1234.0, 0x01, '-');               // doesn't rise the flag
  TEST_FP_OP3_S(2.0, -5.0, -2.0, -8.0, 0x00, '-');
  
  TEST_FP_OP3_S(1.0, 2.5, 1.0, -1.5, 0x00, '<');
  TEST_FP_OP3_S(-1.0, -1235.1, 1.1, -1234.0, 0x01, '<');              // doesn't rise the flag
  TEST_FP_OP3_S(2.0, -5.0, -2.0, 8.0, 0x00, '<');
  
  neorv32_uart0_printf("\n=== DONE ==================\n\n");
  
  neorv32_uart0_printf("\n=== DOUBLE ================\n");
  neorv32_uart0_printf("===========================\n");
  
  neorv32_uart0_printf("\n=== FLCASS ================\n\n");
  
  TEST_FCLASS_D(n_INF.value, 1<<0);
  TEST_FCLASS_D(-1.0, 1<<1);
  TEST_FCLASS_D(n_SUB.value, 1<<2);
  TEST_FCLASS_D(n_ZERO.value, 1<<3);
  TEST_FCLASS_D(0.0, 1<<4);
  TEST_FCLASS_D(p_SUB.value, 1<<5);
  TEST_FCLASS_D(1.0, 1<<6);
  TEST_FCLASS_D(p_INF.value, 1<<7);
  TEST_FCLASS_D(s_NAN.value, 1<<8);
  TEST_FCLASS_D(NAN.value, 1<<9);
  
  neorv32_uart0_printf("\n=== SIGN INJECT ===========\n\n");
  
  TEST_FSGNJD(0, 0, 0, '+');
  TEST_FSGNJD(0, -1, n_ZERO.value, '+');
  TEST_FSGNJD(-1, 0, 1, '+');
  TEST_FSGNJD(-1, -1, -1, '+');
  
  TEST_FSGNJD(0, 0, n_ZERO.value, '-');
  TEST_FSGNJD(0, -1, 0, '-');
  TEST_FSGNJD(-1, 0, -1, '-');
  TEST_FSGNJD(-1, -1, 1, '-');
  
  TEST_FSGNJD(0, 0, 0, 'x');
  TEST_FSGNJD(0, -1, n_ZERO.value, 'x');
  TEST_FSGNJD(-1, 0, -1, 'x');
  TEST_FSGNJD(-1, -1, 1, 'x');
  
  neorv32_uart0_printf("\n=== COMPARE ===============\n\n");
  
  TEST_FP_CMP_OP_D(-1.36, -1.36, 1, 0x00, '=');
  TEST_FP_CMP_OP_D(-1.36, -1.36, 1, 0x00, '~');
  TEST_FP_CMP_OP_D(-1.36, -1.36, 0, 0x00, '<');
  
  TEST_FP_CMP_OP_D(-1.37, -1.36, 0, 0x00, '=');
  TEST_FP_CMP_OP_D(-1.37, -1.36, 1, 0x00, '~');
  TEST_FP_CMP_OP_D(-1.37, -1.36, 1, 0x00, '<');
  
  TEST_FP_CMP_OP_D(NAN.value, 0, 0, 0x00, '=');
  TEST_FP_CMP_OP_D(NAN.value, NAN.value, 0, 0x00, '=');
  TEST_FP_CMP_OP_D(s_NAN.value, 0, 0, 0x10, '=');
  
  TEST_FP_CMP_OP_D(NAN.value, 0, 0, 0x10, '<');
  TEST_FP_CMP_OP_D(NAN.value, NAN.value, 0, 0x10, '<');
  TEST_FP_CMP_OP_D(s_NAN.value, 0, 0, 0x10, '<');
  
  TEST_FP_CMP_OP_D(NAN.value, 0, 0, 0x10, '~');
  TEST_FP_CMP_OP_D(NAN.value, NAN.value, 0, 0x10, '~');
  TEST_FP_CMP_OP_D(s_NAN.value, 0, 0, 0x10, '~');
  
  neorv32_uart0_printf("\n=== CONVERT TO INT ========\n\n");
  
  neorv32_cpu_csr_write(CSR_FRM, 1);  // Round towards zero
  
  TEST_FP_INT_OP_D(-1.1, -1, 0x01, 's');
  TEST_FP_INT_OP_D(-1.0, -1, 0x00, 's');
  TEST_FP_INT_OP_D(-0.9, 0, 0x01, 's');
  TEST_FP_INT_OP_D(0.9, 0, 0x01, 's');
  TEST_FP_INT_OP_D(1.0, 1, 0x00, 's');
  TEST_FP_INT_OP_D(1.1, 1, 0x01, 's');
  TEST_FP_INT_OP_D(-3e9, -1<<31, 0x10, 's');
  TEST_FP_INT_OP_D(3e9, (1<<31)-1, 0x10, 's');
  
  TEST_FP_INT_OP_D(-1.1, 0, 0x10, 'u');
  TEST_FP_INT_OP_D(-1.0, 0, 0x10, 'u');
  TEST_FP_INT_OP_D(-0.9, 0, 0x01, 'u');
  TEST_FP_INT_OP_D(0.9, 0, 0x01, 'u');
  TEST_FP_INT_OP_D(1.0, 1, 0x00, 'u');
  TEST_FP_INT_OP_D(1.1, 1, 0x01, 'u');
  TEST_FP_INT_OP_D(-3e9, 0, 0x10, 'u');
  TEST_FP_INT_OP_D(3e9, 0xffffffffb2d05e00, 0x00, 'u');
  
  TEST_FP_INT_OP_D(NAN.value, 0x7fffffff, 0x10, 's');
  TEST_FP_INT_OP_D(n_INF.value, 0x80000000, 0x10, 's');
  TEST_FP_INT_OP_D(p_INF.value, 0x7fffffff, 0x10, 's');
  TEST_FP_INT_OP_D(NAN.value, 0xffffffff, 0x10, 'u');
  TEST_FP_INT_OP_D(n_INF.value, 0, 0x10, 'u');
  TEST_FP_INT_OP_D(p_INF.value, 0xffffffff, 0x10, 'u');
  
  neorv32_cpu_csr_write(CSR_FRM, 0);  // Round to nearest
  
  neorv32_uart0_printf("\n=== CONVERT TO DOUBLE =====\n\n");
  
  TEST_INT_FP_OP_D(2, 2.0, 's');
  TEST_INT_FP_OP_D(-2, -2.0, 's');
  
  TEST_INT_FP_OP_D(2, 2.0, 'u');
  TEST_INT_FP_OP_D(-2, 4294967294, 'u');
  
  neorv32_uart0_printf("\n=== MAX/MIN ===============\n\n");
  
  TEST_FP_OP2_D(2.5, 1.0, 1.0, 0x00, '<');
  TEST_FP_OP2_D(-1235.1, 1.1, -1235.1, 0x00, '<');
  TEST_FP_OP2_D(1.1, -1235.1, -1235.1, 0x00, '<');
  TEST_FP_OP2_D(NAN.value, -1235.1, -1235.1, 0x00, '<');
  TEST_FP_OP2_D(3.14159265, 0.00000001, 0.00000001, 0x00, '<');
  TEST_FP_OP2_D(-1.0, -2.0, -2.0, 0x00, '<');
  
  TEST_FP_OP2_D(2.5, 1.0, 2.5, 0x00, '>');
  TEST_FP_OP2_D(-1235.1, 1.1, 1.1, 0x00, '>');
  TEST_FP_OP2_D(1.1, -1235.1, 1.1, 0x00, '>');
  TEST_FP_OP2_D(NAN.value, -1235.1, -1235.1, 0x00, '>');
  TEST_FP_OP2_D(3.14159265, 0.00000001, 3.14159265, 0x00, '>');
  TEST_FP_OP2_D(-1.0, -2.0, -1.0, 0x00, '>');
  
  TEST_FP_OP2_D(s_NAN.value, 1.0, 1.0, 0x10, '>');
  TEST_FP_OP2_D(NAN.value, NAN.value, NAN.value, 0x00, '>');
  
  TEST_FP_OP2_D(n_ZERO.value, 0.0, n_ZERO.value, 0x00, '<');
  TEST_FP_OP2_D(0.0, n_ZERO.value, n_ZERO.value, 0x00, '<');
  TEST_FP_OP2_D(n_ZERO.value, 0.0, 0.0, 0x00, '>');
  TEST_FP_OP2_D(0.0, n_ZERO.value, 0.0, 0x00, '>');
    
  neorv32_uart0_printf("\n=== SIMPLE ARITHMETIC =====\n\n");
  
  TEST_FP_OP2_D(2.5, 1.0, 3.5, 0x00, '+');
  TEST_FP_OP2_D(-1235.1, 1.1, -1234 , 0x01, '+');
  TEST_FP_OP2_D(3.14159265, 0.00000001, 3.14159266, 0x01, '+');
  
  TEST_FP_OP2_D(2.5, 1.0, 1.5, 0x00, '-');
  TEST_FP_OP2_D(-1235.1, -1.1, -1234, 0x01, '-');
  TEST_FP_OP2_D(3.14159265, 0.00000001, 3.1415926400000001, 0x01, '-');
  
  TEST_FP_OP2_D(2.5, 1.0, 2.5, 0x00, '*');
  TEST_FP_OP2_D(-1235.1, -1.1, 1358.61, 0x01, '*');
  TEST_FP_OP2_D(3.14159265, 0.00000001, 3.14159265e-8, 0x01, '*');
  
  TEST_FP_OP2_D(p_INF.value, p_INF.value, NAN.value, 0x10, '-');

  neorv32_uart0_printf("\n=== DIV AND SQRT ==========\n\n");
  
  TEST_FP_OP2_D(3.14159265, 2.71828182, 1.1557273520668288, 0x01, '/');
  TEST_FP_OP2_D(-1234, 1235.1, -0.9991093838555584, 0x01, '/');
  TEST_FP_OP2_D(3.14159265, 1.0, 3.14159265, 0x00, '/');
  
  TEST_FP_OP1_D(3.14159265, 1.7724538498928541, 0x01);
  TEST_FP_OP1_D(10000.0, 100.0, 0x00);
  TEST_FP_OP1_D(171.0, 13.076696830622021, 0x01);
  TEST_FP_OP1_D(1.60795e-7, 0.00040099251863345283320230749702, 0x01);
  
  neorv32_uart0_printf("\n=== FUSED MULTIPLY ========\n\n");
  
  TEST_FP_OP3_D(1.0, 2.5, 1.0, 3.5, 0x00, '+');
  TEST_FP_OP3_D(-1.0, -1235.1, 1.1, 1236.1999999999999, 0x01, '+');   // doesn't rise the flag
  TEST_FP_OP3_D(2.0, -5.0, -2.0, -12.0, 0x00, '+');
  
  TEST_FP_OP3_D(1.0, 2.5, 1.0, -3.5, 0x00, '>');
  TEST_FP_OP3_D(-1.0, -1235.1, 1.1, -1236.1999999999999, 0x01, '>');  // doesn't rise the flag
  TEST_FP_OP3_D(2.0, -5.0, -2.0, 12.0, 0x00, '>');
  
  TEST_FP_OP3_D(1.0, 2.5, 1.0, 1.5, 0x00, '-');
  TEST_FP_OP3_D(-1.0, -1235.1, 1.1, 1234.0, 0x01, '-');               // doesn't rise the flag
  TEST_FP_OP3_D(2.0, -5.0, -2.0, -8.0, 0x00, '-');
  
  TEST_FP_OP3_D(1.0, 2.5, 1.0, -1.5, 0x00, '<');
  TEST_FP_OP3_D(-1.0, -1235.1, 1.1, -1234.0, 0x01, '<');              // doesn't rise the flag
  TEST_FP_OP3_D(2.0, -5.0, -2.0, 8.0, 0x00, '<');
  
  neorv32_uart0_printf("\n=== DONE ==================\n\n");
  
  return 0;
}

