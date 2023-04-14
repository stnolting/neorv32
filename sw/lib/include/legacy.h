// #################################################################################################
// # << NEORV32: legacy.h - Backwards compatibility wrappers and functions >>                      #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file legacy.h
 * @brief Wrappers and functions for backwards compatibility.
 * @warning Do not use these functions for new designs as they are no longer
 * supported and might get removed in the future.
 **************************************************************************/

#ifndef neorv32_legacy_h
#define neorv32_legacy_h


// ================================================================================================
// UART0 & UART1
// ================================================================================================

/**********************************************************************//**
 * @name UART0: Backward compatibility Wrapper, #neorv32_uart_h
 **************************************************************************/
/**@{*/
#define neorv32_uart0_available()                  neorv32_uart_available(NEORV32_UART0)
#define neorv32_uart0_get_rx_fifo_depth()          neorv32_uart_get_rx_fifo_depth(NEORV32_UART0)
#define neorv32_uart0_get_tx_fifo_depth()          neorv32_uart_get_tx_fifo_depth(NEORV32_UART0)
#define neorv32_uart0_setup(baudrate, irq_mask)    neorv32_uart_setup(NEORV32_UART0, baudrate, irq_mask)
#define neorv32_uart0_disable()                    neorv32_uart_disable(NEORV32_UART0)
#define neorv32_uart0_enable()                     neorv32_uart_enable(NEORV32_UART0)
#define neorv32_uart0_rtscts_disable()             neorv32_uart_rtscts_disable(NEORV32_UART0)
#define neorv32_uart0_rtscts_enable()              neorv32_uart_rtscts_enable(NEORV32_UART0)
#define neorv32_uart0_putc(c)                      neorv32_uart_putc(NEORV32_UART0, c)
#define neorv32_uart0_tx_busy()                    neorv32_uart_tx_busy(NEORV32_UART0)
#define neorv32_uart0_getc()                       neorv32_uart_getc(NEORV32_UART0)
#define neorv32_uart0_char_received()              neorv32_uart_char_received(NEORV32_UART0)
#define neorv32_uart0_char_received_get()          neorv32_uart_char_received_get(NEORV32_UART0)
#define neorv32_uart0_puts(s)                      neorv32_uart_puts(NEORV32_UART0, s)
#define neorv32_uart0_printf(...)                  neorv32_uart_printf(NEORV32_UART0, __VA_ARGS__)
#define neorv32_uart0_scan(buffer, max_size, echo) neorv32_uart_scan(NEORV32_UART0, buffer, max_size, echo)
/**@}*/

/**********************************************************************//**
 * @name UART1: Backward compatibility Wrapper, #neorv32_uart_h
 **************************************************************************/
/**@{*/
#define neorv32_uart1_available()                  neorv32_uart_available(NEORV32_UART1)
#define neorv32_uart1_get_rx_fifo_depth()          neorv32_uart_get_rx_fifo_depth(NEORV32_UART1)
#define neorv32_uart1_get_tx_fifo_depth()          neorv32_uart_get_tx_fifo_depth(NEORV32_UART1)
#define neorv32_uart1_setup(baudrate, irq_mask)    neorv32_uart_setup(NEORV32_UART1, baudrate, irq_mask)
#define neorv32_uart1_disable()                    neorv32_uart_disable(NEORV32_UART1)
#define neorv32_uart1_enable()                     neorv32_uart_enable(NEORV32_UART1)
#define neorv32_uart1_rtscts_disable()             neorv32_uart_rtscts_disable(NEORV32_UART1)
#define neorv32_uart1_rtscts_enable()              neorv32_uart_rtscts_enable(NEORV32_UART1)
#define neorv32_uart1_putc(c)                      neorv32_uart_putc(NEORV32_UART1, c)
#define neorv32_uart1_tx_busy()                    neorv32_uart_tx_busy(NEORV32_UART1)
#define neorv32_uart1_getc()                       neorv32_uart_getc(NEORV32_UART1)
#define neorv32_uart1_char_received()              neorv32_uart_char_received(NEORV32_UART1)
#define neorv32_uart1_char_received_get()          neorv32_uart_char_received_get(NEORV32_UART1)
#define neorv32_uart1_puts(s)                      neorv32_uart_puts(NEORV32_UART1, s)
#define neorv32_uart1_printf(...)                  neorv32_uart_printf(NEORV32_UART1, __VA_ARGS__)
#define neorv32_uart1_scan(buffer, max_size, echo) neorv32_uart_scan(NEORV32_UART1, buffer, max_size, echo)
/**@}*/

/**********************************************************************//**
 * Print string (zero-terminated) via UART0. Print full line break "\r\n" for every '\n'.
 * @note This function is blocking.
 * @warning This function is deprecated!
 * @param[in] s Pointer to string.
 **************************************************************************/
inline void __attribute__((deprecated("Use 'neorv32_uart0_puts()' instead."))) neorv32_uart0_print(const char *s) {
  neorv32_uart0_puts(s);
}

/**********************************************************************//**
 * Print string (zero-terminated) via UART1. Print full line break "\r\n" for every '\n'.
 * @note This function is blocking.
 * @warning This function is deprecated!
 * @param[in] s Pointer to string.
 **************************************************************************/
inline void __attribute__((deprecated("Use 'neorv32_uart0_puts()' instead."))) neorv32_uart1_print(const char *s) {
  neorv32_uart1_puts(s);
}


// ================================================================================================
// Custom Functions Unit (CFU)
// ================================================================================================

/**********************************************************************//**
 * @name Backward-compatibility layer (before version v1.7.8.2)
 * @warning This function is deprecated!
 **************************************************************************/
/**@{*/
/** R3-type CFU custom instruction 0 (funct3 = 000) */
#define neorv32_cfu_cmd0(funct7, rs1, rs2) neorv32_cfu_r3_instr(funct7, 0, rs1, rs2)
/** R3-type CFU custom instruction 1 (funct3 = 001) */
#define neorv32_cfu_cmd1(funct7, rs1, rs2) neorv32_cfu_r3_instr(funct7, 1, rs1, rs2)
/** R3-type CFU custom instruction 2 (funct3 = 010) */
#define neorv32_cfu_cmd2(funct7, rs1, rs2) neorv32_cfu_r3_instr(funct7, 2, rs1, rs2)
/** R3-type CFU custom instruction 3 (funct3 = 011) */
#define neorv32_cfu_cmd3(funct7, rs1, rs2) neorv32_cfu_r3_instr(funct7, 3, rs1, rs2)
/** R3-type CFU custom instruction 4 (funct3 = 100) */
#define neorv32_cfu_cmd4(funct7, rs1, rs2) neorv32_cfu_r3_instr(funct7, 4, rs1, rs2)
/** R3-type CFU custom instruction 5 (funct3 = 101) */
#define neorv32_cfu_cmd5(funct7, rs1, rs2) neorv32_cfu_r3_instr(funct7, 5, rs1, rs2)
/** R3-type CFU custom instruction 6 (funct3 = 110) */
#define neorv32_cfu_cmd6(funct7, rs1, rs2) neorv32_cfu_r3_instr(funct7, 6, rs1, rs2)
/** R3-type CFU custom instruction 7 (funct3 = 111) */
#define neorv32_cfu_cmd7(funct7, rs1, rs2) neorv32_cfu_r3_instr(funct7, 7, rs1, rs2)
/**@}*/


// ================================================================================================
// CPU Core
// ================================================================================================

/**********************************************************************//**
 * Get current system time from time[h] CSR.
 * @note This function requires the MTIME system timer to be implemented.
 * @return Current system time (64 bit).
 **************************************************************************/
inline uint64_t __attribute__((deprecated("Use 'neorv32_mtime_get_time()' instead."))) neorv32_cpu_get_systime(void) {
  return neorv32_mtime_get_time();
}

/**********************************************************************//**
 * Enable global CPU interrupts (via MIE flag in mstatus CSR).
 * @note Interrupts are always enabled when the CPU is in user-mode.
 **************************************************************************/
inline void __attribute__ ((always_inline, deprecated("Use 'neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE)' instead."))) neorv32_cpu_eint(void) {
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);
}

/**********************************************************************//**
 * Disable global CPU interrupts (via MIE flag in mstatus CSR).
 * @note Interrupts are always enabled when the CPU is in user-mode.
 **************************************************************************/
inline void __attribute__ ((always_inline, deprecated("Use 'neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE)' instead."))) neorv32_cpu_dint(void) {
  neorv32_cpu_csr_clr(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);
}


#endif // neorv32_legacy_h
