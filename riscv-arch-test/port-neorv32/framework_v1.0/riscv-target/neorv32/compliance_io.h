// RISC-V Compliance IO Test Header File

/*
 * Copyright (c) 2005-2018 Imperas Software Ltd., www.imperas.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied.
 *
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

// Modified by Stephan Nolting for the NEORV32 Processor

#ifndef _COMPLIANCE_IO_H
#define _COMPLIANCE_IO_H

//-----------------------------------------------------------------------
// RV IO Macros
//-----------------------------------------------------------------------

// enable UART (ctrl(28)) and enable UART_SIM_MODE (ctrl(12))
#define RVTEST_IO_INIT                                                        \
        uart_init:                                                            \
        li a0, 0xFFFFFFA0;                                                    \
        sw zero, 0(a0);                                                       \
        li a1, 1 << 28;                                                       \
        li a2, 1 << 12;                                                       \
        or a1, a1, a2;                                                        \
        sw a1, 0(a0);                                                         \

#define RVTEST_IO_WRITE_STR(_R, _STR)
#define RVTEST_IO_CHECK()
#define RVTEST_IO_ASSERT_GPR_EQ(_G, _R, _I)
#define RVTEST_IO_ASSERT_SFPR_EQ(_F, _R, _I)
#define RVTEST_IO_ASSERT_DFPR_EQ(_D, _R, _I)

#endif // _COMPLIANCE_IO_H
