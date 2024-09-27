-- ================================================================================ --
-- NEORV32 CPU - Co-Processor: RISC-V Scalar Cryptography ('Zk*') ISA Extension     --
-- -------------------------------------------------------------------------------- --
-- Supported sub-extensions:                                                        --
-- + Zbkx: crossbar permutation                                                     --
-- + Zknh: NIST suite's hash function                                               --
-- + Zknd: NIST suite's AES decryption                                              --
-- + Zkne: NIST suite's AES encryption                                              --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu_cp_crypto is
  generic (
    EN_ZBKX : boolean; -- enable crossbar permutation extension
    EN_ZKNH : boolean; -- enable NIST hash extension
    EN_ZKNE : boolean; -- enable NIST AES encryption
    EN_ZKND : boolean  -- enable NIST AES decryption
  );
  port (
    -- global control --
    clk_i   : in  std_ulogic; -- global clock, rising edge
    rstn_i  : in  std_ulogic; -- global reset, low-active, async
    ctrl_i  : in  ctrl_bus_t; -- main control bus
    -- data input --
    rs1_i   : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 1
    rs2_i   : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 2
    -- result and status --
    res_o   : out std_ulogic_vector(XLEN-1 downto 0); -- operation result
    valid_o : out std_ulogic -- data output valid
  );
end neorv32_cpu_cp_crypto;

architecture neorv32_cpu_cp_crypto_rtl of neorv32_cpu_cp_crypto is

  -- ----------------------------------------------------------------------------------------
  -- look-up tables (ROMs)
  -- ----------------------------------------------------------------------------------------

  -- s-box --
  type sbox_rom_t is array (0 to 511) of std_ulogic_vector(7 downto 0);
  constant sbox_c : sbox_rom_t := (
    -- forward --
    x"63", x"7c", x"77", x"7b", x"f2", x"6b", x"6f", x"c5", x"30", x"01", x"67", x"2b", x"fe", x"d7", x"ab", x"76",
    x"ca", x"82", x"c9", x"7d", x"fa", x"59", x"47", x"f0", x"ad", x"d4", x"a2", x"af", x"9c", x"a4", x"72", x"c0",
    x"b7", x"fd", x"93", x"26", x"36", x"3f", x"f7", x"cc", x"34", x"a5", x"e5", x"f1", x"71", x"d8", x"31", x"15",
    x"04", x"c7", x"23", x"c3", x"18", x"96", x"05", x"9a", x"07", x"12", x"80", x"e2", x"eb", x"27", x"b2", x"75",
    x"09", x"83", x"2c", x"1a", x"1b", x"6e", x"5a", x"a0", x"52", x"3b", x"d6", x"b3", x"29", x"e3", x"2f", x"84",
    x"53", x"d1", x"00", x"ed", x"20", x"fc", x"b1", x"5b", x"6a", x"cb", x"be", x"39", x"4a", x"4c", x"58", x"cf",
    x"d0", x"ef", x"aa", x"fb", x"43", x"4d", x"33", x"85", x"45", x"f9", x"02", x"7f", x"50", x"3c", x"9f", x"a8",
    x"51", x"a3", x"40", x"8f", x"92", x"9d", x"38", x"f5", x"bc", x"b6", x"da", x"21", x"10", x"ff", x"f3", x"d2",
    x"cd", x"0c", x"13", x"ec", x"5f", x"97", x"44", x"17", x"c4", x"a7", x"7e", x"3d", x"64", x"5d", x"19", x"73",
    x"60", x"81", x"4f", x"dc", x"22", x"2a", x"90", x"88", x"46", x"ee", x"b8", x"14", x"de", x"5e", x"0b", x"db",
    x"e0", x"32", x"3a", x"0a", x"49", x"06", x"24", x"5c", x"c2", x"d3", x"ac", x"62", x"91", x"95", x"e4", x"79",
    x"e7", x"c8", x"37", x"6d", x"8d", x"d5", x"4e", x"a9", x"6c", x"56", x"f4", x"ea", x"65", x"7a", x"ae", x"08",
    x"ba", x"78", x"25", x"2e", x"1c", x"a6", x"b4", x"c6", x"e8", x"dd", x"74", x"1f", x"4b", x"bd", x"8b", x"8a",
    x"70", x"3e", x"b5", x"66", x"48", x"03", x"f6", x"0e", x"61", x"35", x"57", x"b9", x"86", x"c1", x"1d", x"9e",
    x"e1", x"f8", x"98", x"11", x"69", x"d9", x"8e", x"94", x"9b", x"1e", x"87", x"e9", x"ce", x"55", x"28", x"df",
    x"8c", x"a1", x"89", x"0d", x"bf", x"e6", x"42", x"68", x"41", x"99", x"2d", x"0f", x"b0", x"54", x"bb", x"16",
    -- inverse --
    x"52", x"09", x"6a", x"d5", x"30", x"36", x"a5", x"38", x"bf", x"40", x"a3", x"9e", x"81", x"f3", x"d7", x"fb",
    x"7c", x"e3", x"39", x"82", x"9b", x"2f", x"ff", x"87", x"34", x"8e", x"43", x"44", x"c4", x"de", x"e9", x"cb",
    x"54", x"7b", x"94", x"32", x"a6", x"c2", x"23", x"3d", x"ee", x"4c", x"95", x"0b", x"42", x"fa", x"c3", x"4e",
    x"08", x"2e", x"a1", x"66", x"28", x"d9", x"24", x"b2", x"76", x"5b", x"a2", x"49", x"6d", x"8b", x"d1", x"25",
    x"72", x"f8", x"f6", x"64", x"86", x"68", x"98", x"16", x"d4", x"a4", x"5c", x"cc", x"5d", x"65", x"b6", x"92",
    x"6c", x"70", x"48", x"50", x"fd", x"ed", x"b9", x"da", x"5e", x"15", x"46", x"57", x"a7", x"8d", x"9d", x"84",
    x"90", x"d8", x"ab", x"00", x"8c", x"bc", x"d3", x"0a", x"f7", x"e4", x"58", x"05", x"b8", x"b3", x"45", x"06",
    x"d0", x"2c", x"1e", x"8f", x"ca", x"3f", x"0f", x"02", x"c1", x"af", x"bd", x"03", x"01", x"13", x"8a", x"6b",
    x"3a", x"91", x"11", x"41", x"4f", x"67", x"dc", x"ea", x"97", x"f2", x"cf", x"ce", x"f0", x"b4", x"e6", x"73",
    x"96", x"ac", x"74", x"22", x"e7", x"ad", x"35", x"85", x"e2", x"f9", x"37", x"e8", x"1c", x"75", x"df", x"6e",
    x"47", x"f1", x"1a", x"71", x"1d", x"29", x"c5", x"89", x"6f", x"b7", x"62", x"0e", x"aa", x"18", x"be", x"1b",
    x"fc", x"56", x"3e", x"4b", x"c6", x"d2", x"79", x"20", x"9a", x"db", x"c0", x"fe", x"78", x"cd", x"5a", x"f4",
    x"1f", x"dd", x"a8", x"33", x"88", x"07", x"c7", x"31", x"b1", x"12", x"10", x"59", x"27", x"80", x"ec", x"5f",
    x"60", x"51", x"7f", x"a9", x"19", x"b5", x"4a", x"0d", x"2d", x"e5", x"7a", x"9f", x"93", x"c9", x"9c", x"ef",
    x"a0", x"e0", x"3b", x"4d", x"ae", x"2a", x"f5", x"b0", x"c8", x"eb", x"bb", x"3c", x"83", x"53", x"99", x"61",
    x"17", x"2b", x"04", x"7e", x"ba", x"77", x"d6", x"26", x"e1", x"69", x"14", x"63", x"55", x"21", x"0c", x"7d"
  );

  -- ----------------------------------------------------------------------------------------
  -- helper functions
  -- ----------------------------------------------------------------------------------------

  -- byte-wise vector look-up --
  function xperm8_f(vec : std_ulogic_vector(31 downto 0); sel : std_ulogic_vector(7 downto 0)) return std_ulogic_vector is
    variable res_v : std_ulogic_vector(7 downto 0);
  begin
    if (sel(7 downto 2) /= "000000") then -- index out of range
      res_v := (others => '0');
    else
      case sel(1 downto 0) is
        when "00"   => res_v := vec(7 downto 0);
        when "01"   => res_v := vec(15 downto 8);
        when "10"   => res_v := vec(23 downto 16);
        when others => res_v := vec(31 downto 24);
      end case;
    end if;
    return res_v;
  end function xperm8_f;

  -- nibble-wise vector look-up --
  function xperm4_f(vec : std_ulogic_vector(31 downto 0); sel : std_ulogic_vector(3 downto 0)) return std_ulogic_vector is
    variable res_v : std_ulogic_vector(3 downto 0);
  begin
    if (sel(3) /= '0') then -- index out of range
      res_v := (others => '0');
    else
      case sel(2 downto 0) is
        when "000"  => res_v := vec(3 downto 0);
        when "001"  => res_v := vec(7 downto 4);
        when "010"  => res_v := vec(11 downto 8);
        when "011"  => res_v := vec(15 downto 12);
        when "100"  => res_v := vec(19 downto 16);
        when "101"  => res_v := vec(23 downto 20);
        when "110"  => res_v := vec(27 downto 24);
        when others => res_v := vec(31 downto 28);
      end case;
    end if;
    return res_v;
  end function xperm4_f;

  -- logical shift left --
  function lsl_f(data : std_ulogic_vector(31 downto 0); shamt : natural range 0 to 31) return std_ulogic_vector is
    variable res_v : std_ulogic_vector(31 downto 0);
  begin
    res_v := std_ulogic_vector(shift_left(unsigned(data), shamt));
    return res_v;
  end function lsl_f;

  -- logical shift right --
  function lsr_f(data : std_ulogic_vector(31 downto 0); shamt : natural range 0 to 31) return std_ulogic_vector is
    variable res_v : std_ulogic_vector(31 downto 0);
  begin
    res_v := std_ulogic_vector(shift_right(unsigned(data), shamt));
    return res_v;
  end function lsr_f;

  -- rotate right --
  function ror_f(data : std_ulogic_vector(31 downto 0); shamt : natural range 0 to 31) return std_ulogic_vector is
    variable res_v : std_ulogic_vector(31 downto 0);
  begin
    res_v := std_ulogic_vector(rotate_right(unsigned(data), shamt));
    return res_v;
  end function ror_f;

  -- multiply by 2 in Galois field (2^8) --
  function xt2_f(a : std_ulogic_vector(7 downto 0)) return std_ulogic_vector is
    variable res_v : std_ulogic_vector(7 downto 0);
  begin
    res_v := (a(6 downto 0) & '0') xor ("000" & a(7) & a(7) & '0' & a(7) & a(7)); -- XOR with 0x1B if a(7) is set
    return res_v;
  end function xt2_f;

  -- multiply 8-bit field element by 4-bit value for AES MixCols step --
  function gfmul_f(x : std_ulogic_vector(7 downto 0); y : std_ulogic_vector(3 downto 0)) return std_ulogic_vector is
    variable res_v : std_ulogic_vector(7 downto 0);
  begin
    res_v := (others => '0');
    if (y(0) = '1') then
      res_v := res_v xor x;
    end if;
    if (y(1) = '1') then
      res_v := res_v xor xt2_f(x);
    end if;
    if (y(2) = '1') then
      res_v := res_v xor xt2_f(xt2_f(x));
    end if;
    if (y(3) = '1') then
      res_v := res_v xor xt2_f(xt2_f(xt2_f(x)));
    end if;
    return res_v;
  end function gfmul_f;

  -- ----------------------------------------------------------------------------------------
  -- logic
  -- ----------------------------------------------------------------------------------------

  -- instruction decoder --
  constant cmd_xperm_c  : natural := 0;
  constant cmd_sha256_c : natural := 1;
  constant cmd_sha512_c : natural := 2;
  constant cmd_aesenc_c : natural := 3;
  constant cmd_aesdec_c : natural := 4;
  --
  signal cmd : std_ulogic_vector(4 downto 0);
  signal cmd_valid : std_ulogic;

  -- controller --
  type state_t is (S_IDLE, S_BUSY, S_DONE);
  signal state   : state_t;
  signal done    : std_ulogic;
  signal rs1     : std_ulogic_vector(31 downto 0);
  signal rs2     : std_ulogic_vector(31 downto 0);
  signal funct12 : std_ulogic_vector(11 downto 0);
  signal funct3  : std_ulogic_vector(2 downto 0);
  signal out_sel : std_ulogic_vector(1 downto 0);

  -- aes core --
  type aes_t is record
    dec  : std_ulogic; -- 0 = encryption, 1 = decryption
    mid  : std_ulogic; -- 0 = final round, 1 = middle round
    bs   : std_ulogic_vector(1 downto 0);
    si   : std_ulogic_vector(7 downto 0);
    so   : std_ulogic_vector(7 downto 0);
    mix1 : std_ulogic_vector(31 downto 0);
    mix2 : std_ulogic_vector(31 downto 0);
    rot  : std_ulogic_vector(31 downto 0);
    res  : std_ulogic_vector(31 downto 0);
  end record;
  signal aes : aes_t;

  -- permutation core --
  signal xperm_res, xperm4_res, xperm8_res : std_ulogic_vector(31 downto 0);

  -- sha core --
  signal sha_res : std_ulogic_vector(31 downto 0);

begin

  -- Instruction Decode ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cmd(cmd_xperm_c)  <= '1' when EN_ZBKX and (ctrl_i.ir_opcode(5) = '1') and (ctrl_i.ir_funct12(11 downto 5) = "0010100") and
                                ((ctrl_i.ir_funct3 = "100") or (ctrl_i.ir_funct3 = "010")) else '0';

  cmd(cmd_sha256_c) <= '1' when EN_ZKNH and (ctrl_i.ir_opcode(5) = '0') and (ctrl_i.ir_funct12(11 downto 2) = "0001000000") and
                                (ctrl_i.ir_funct3 = "001") else '0';

  cmd(cmd_sha512_c) <= '1' when EN_ZKNH and (ctrl_i.ir_opcode(5) = '1') and (ctrl_i.ir_funct12(11 downto 8) = "0101") and
                                (ctrl_i.ir_funct12(7 downto 6) /= "10") and (ctrl_i.ir_funct3 = "000") else '0';

  cmd(cmd_aesenc_c) <= '1' when EN_ZKNE and (ctrl_i.ir_opcode(5) = '1') and (ctrl_i.ir_funct3 = "000") and
                                (ctrl_i.ir_funct12(9 downto 7) = "100") and (ctrl_i.ir_funct12(5) = '1') else '0';

  cmd(cmd_aesdec_c) <= '1' when EN_ZKND and (ctrl_i.ir_opcode(5) = '1') and (ctrl_i.ir_funct3 = "000") and
                                (ctrl_i.ir_funct12(9 downto 7) = "101") and (ctrl_i.ir_funct12(5) = '1') else '0';

  -- valid instruction? --
  cmd_valid <= '1' when (ctrl_i.alu_cp_alu = '1') and (or_reduce_f(cmd) = '1') else '0';


  -- Controller -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  control: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      rs1     <= (others => '0');
      rs2     <= (others => '0');
      funct3  <= (others => '0');
      funct12 <= (others => '0');
      done    <= '0';
      state   <= S_IDLE;
    elsif rising_edge(clk_i) then
      -- operand gating / buffer --
      if (cmd_valid = '1') then
        rs1     <= rs1_i;
        rs2     <= rs2_i;
        funct3  <= ctrl_i.ir_funct3;
        funct12 <= ctrl_i.ir_funct12;
      end if;
      -- arbiter state machine --
      done <= '0'; -- default
      case state is
        -- wait for operation trigger --
        when S_IDLE =>
          if (cmd_valid = '1') then -- trigger new operation
            if (cmd(cmd_aesenc_c) = '1') or (cmd(cmd_aesdec_c) = '1') then
              state <= S_BUSY;
            else -- super fast
              done  <= '1';
              state <= S_IDLE;
            end if;
          end if;
        -- delay cycle --
        when S_BUSY =>
          state <= S_DONE;
        -- S_DONE: final step & enable output for one cycle --
        when others =>
          done  <= '1';
          state <= S_IDLE;
      end case;
    end if;
  end process control;

  -- processing done (high one cycle before actual data output) --
  valid_o <= done;


  -- Output Select --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  result: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      res_o <= (others => '0');
    elsif rising_edge(clk_i) then
      res_o <= (others => '0'); -- default
      if (done = '1') then
        case out_sel is
          when "00"   => res_o <= xperm_res;
          when "01"   => res_o <= aes.res;
          when others => res_o <= sha_res;
        end case;
      end if;
    end if;
  end process result;

  -- function unit select --
  out_sel <= funct12(8) & funct12(5);


  -- Crossbar Permutations ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  xperm_enabled:
  if EN_ZBKX generate

    -- byte-wise vector look-up --
    xperm8_gen:
    for i in 0 to 3 generate
      xperm8_res(8*i+7 downto 8*i+0) <= xperm8_f(rs1, rs2(8*i+7 downto 8*i+0));
    end generate;

    -- nibble-wise vector look-up --
    xperm4_gen:
    for i in 0 to 7 generate
      xperm4_res(4*i+3 downto 4*i+0) <= xperm4_f(rs1, rs2(4*i+3 downto 4*i+0));
    end generate;

    -- operation select --
    xperm_res <= xperm8_res when (funct3(2) = '1') else xperm4_res;

  end generate;

  xperm_disabled:
  if not EN_ZBKX generate
    xperm_res <= (others => '0');
  end generate;


  -- NIST Hash Functions --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  sha_enabled:
  if EN_ZKNH generate
    sha_core: process(funct12, rs1, rs2)
    begin
      if (funct12(10) = '0') then -- sha256
        case funct12(1 downto 0) is
          when "00"   => sha_res <= ror_f(rs1,  2) xor ror_f(rs1, 13) xor ror_f(rs1, 22); -- sha256sum0
          when "01"   => sha_res <= ror_f(rs1,  6) xor ror_f(rs1, 11) xor ror_f(rs1, 25); -- sha256sum1
          when "10"   => sha_res <= ror_f(rs1,  7) xor ror_f(rs1, 18) xor lsr_f(rs1,  3); -- sha256sig0
          when others => sha_res <= ror_f(rs1, 17) xor ror_f(rs1, 19) xor lsr_f(rs1, 10); -- sha256sig1
        end case;
      else -- sha512
        case funct12(7 downto 5) is
          when "000"  => sha_res <= lsl_f(rs1, 25) xor lsl_f(rs1, 30) xor lsr_f(rs1, 28) xor lsr_f(rs2,  7) xor lsr_f(rs2,  2) xor lsl_f(rs2,  4); -- sha512sum0r
          when "001"  => sha_res <= lsl_f(rs1, 23) xor lsr_f(rs1, 14) xor lsr_f(rs1, 18) xor lsr_f(rs2,  9) xor lsl_f(rs2, 18) xor lsl_f(rs2, 14); -- sha512sum1r
          when "010"  => sha_res <= lsr_f(rs1,  1) xor lsr_f(rs1,  7) xor lsr_f(rs1,  8) xor lsl_f(rs2, 31) xor lsl_f(rs2, 25) xor lsl_f(rs2, 24); -- sha512sig0l
          when "011"  => sha_res <= lsl_f(rs1,  3) xor lsr_f(rs1,  6) xor lsr_f(rs1, 19) xor lsr_f(rs2, 29) xor lsl_f(rs2, 26) xor lsl_f(rs2, 13); -- sha512sig1l
          when "110"  => sha_res <= lsr_f(rs1,  1) xor lsr_f(rs1,  7) xor lsr_f(rs1,  8) xor lsl_f(rs2, 31) xor lsl_f(rs2, 24); -- sha512sig0h
          when others => sha_res <= lsl_f(rs1,  3) xor lsr_f(rs1,  6) xor lsr_f(rs1, 19) xor lsr_f(rs2, 29) xor lsl_f(rs2, 13); -- sha512sig1h
        end case;
      end if;
    end process sha_core;
  end generate;

  sha_disabled:
  if not EN_ZKNH generate
    sha_res <= (others => '0');
  end generate;


  -- NIST AES Encryption/Decryption ---------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  aes_enabled:
  if EN_ZKNE or EN_ZKND generate

    -- operation select --
    aes.bs  <= funct12(11 downto 10); -- byte select
    aes.mid <= funct12(6); -- 0 = final round, 1 = middle round
    aes.dec <= '1' when (EN_ZKNE = false) else '0' when (EN_ZKND = false) else funct12(7); -- 0 = encrypt, 1 = decrypt

    -- select byte from rs2 --
    with aes.bs select aes.si <=
      rs2(07 downto 00) when "00",
      rs2(15 downto 08) when "01",
      rs2(23 downto 16) when "10",
      rs2(31 downto 24) when others;

    -- s-box look-up --
    sbox_lookup: process(clk_i)
    begin
      if rising_edge(clk_i) then -- ROM access; try to infer memory primitives
        aes.so <= sbox_c(to_integer(unsigned(aes.dec & aes.si))); -- aes.dec = 0 -> fwd-s-box, aes.dec = 1 -> inv-s-box
      end if;
    end process sbox_lookup;

    -- mix columns --
    mix_columns: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        aes.mix1 <= (others => '0');
      elsif rising_edge(clk_i) then
        if (aes.dec = '1') then -- decrypt
          aes.mix1(31 downto 24) <= gfmul_f(aes.so, x"b");
          aes.mix1(23 downto 16) <= gfmul_f(aes.so, x"d");
          aes.mix1(15 downto 08) <= gfmul_f(aes.so, x"9");
          aes.mix1(07 downto 00) <= gfmul_f(aes.so, x"e");
        else -- encrypt
          aes.mix1(31 downto 24) <= gfmul_f(aes.so, x"3");
          aes.mix1(23 downto 16) <= aes.so;
          aes.mix1(15 downto 08) <= aes.so;
          aes.mix1(07 downto 00) <= gfmul_f(aes.so, x"2");
        end if;
      end if;
    end process mix_columns;

    -- middle / final round --
    aes.mix2 <= aes.mix1 when (aes.mid = '1') else x"000000" & aes.so;

    -- rotate by multiples of 8 --
    with aes.bs select aes.rot <=
      aes.mix2(31 downto 0)                          when "00",
      aes.mix2(23 downto 0) & aes.mix2(31 downto 24) when "01",
      aes.mix2(15 downto 0) & aes.mix2(31 downto 16) when "10",
      aes.mix2(07 downto 0) & aes.mix2(31 downto 08) when others;

    -- final XOR --
    aes.res <= rs1 xor aes.rot;

  end generate;

  aes_disabled:
  if (not EN_ZKNE) and (not EN_ZKND) generate
    aes.bs   <= (others => '0');
    aes.mid  <= '0';
    aes.dec  <= '0';
    aes.so   <= (others => '0');
    aes.mix1 <= (others => '0');
    aes.mix2 <= (others => '0');
    aes.res  <= (others => '0');
  end generate;


end neorv32_cpu_cp_crypto_rtl;
