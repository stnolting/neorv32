-- ================================================================================ --
-- NEORV32 - Simple Combinatorial XBUS / Wishbone Gateway                           --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity xbus_gateway is
  generic (
    -- device enable, address size in bytes and base address (word-aligned) --
    DEV_0_EN : boolean := false; DEV_0_SIZE : natural := 0; DEV_0_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_1_EN : boolean := false; DEV_1_SIZE : natural := 0; DEV_1_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_2_EN : boolean := false; DEV_2_SIZE : natural := 0; DEV_2_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_3_EN : boolean := false; DEV_3_SIZE : natural := 0; DEV_3_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_4_EN : boolean := false; DEV_4_SIZE : natural := 0; DEV_4_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_5_EN : boolean := false; DEV_5_SIZE : natural := 0; DEV_5_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_6_EN : boolean := false; DEV_6_SIZE : natural := 0; DEV_6_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_7_EN : boolean := false; DEV_7_SIZE : natural := 0; DEV_7_BASE : std_ulogic_vector(31 downto 0) := (others => '0')
  );
  port (
    -- host port --
    host_req_i  : in  xbus_req_t;
    host_rsp_o  : out xbus_rsp_t;
    -- device ports --
    dev_0_req_o : out xbus_req_t; dev_0_rsp_i : in xbus_rsp_t;
    dev_1_req_o : out xbus_req_t; dev_1_rsp_i : in xbus_rsp_t;
    dev_2_req_o : out xbus_req_t; dev_2_rsp_i : in xbus_rsp_t;
    dev_3_req_o : out xbus_req_t; dev_3_rsp_i : in xbus_rsp_t;
    dev_4_req_o : out xbus_req_t; dev_4_rsp_i : in xbus_rsp_t;
    dev_5_req_o : out xbus_req_t; dev_5_rsp_i : in xbus_rsp_t;
    dev_6_req_o : out xbus_req_t; dev_6_rsp_i : in xbus_rsp_t;
    dev_7_req_o : out xbus_req_t; dev_7_rsp_i : in xbus_rsp_t
  );
end xbus_gateway;

architecture xbus_gateway_rtl of xbus_gateway is

  -- module configuration --
  constant num_devs_c : natural := 8; -- number of device ports

  -- list of device base address and address size --
  type dev_en_list_t   is array (0 to num_devs_c-1) of boolean;
  type dev_base_list_t is array (0 to num_devs_c-1) of std_ulogic_vector(31 downto 0);
  type dev_size_list_t is array (0 to num_devs_c-1) of natural;
  constant dev_en_list_c   : dev_en_list_t   := (DEV_0_EN, DEV_1_EN, DEV_2_EN, DEV_3_EN, DEV_4_EN, DEV_5_EN, DEV_6_EN, DEV_7_EN);
  constant dev_base_list_c : dev_base_list_t := (DEV_0_BASE, DEV_1_BASE, DEV_2_BASE, DEV_3_BASE, DEV_4_BASE, DEV_5_BASE, DEV_6_BASE, DEV_7_BASE);
  constant dev_size_list_c : dev_size_list_t := (DEV_0_SIZE, DEV_1_SIZE, DEV_2_SIZE, DEV_3_SIZE, DEV_4_SIZE, DEV_5_SIZE, DEV_6_SIZE, DEV_7_SIZE);

  -- device ports combined as arrays --
  type dev_req_t is array (0 to num_devs_c-1) of xbus_req_t;
  type dev_rsp_t is array (0 to num_devs_c-1) of xbus_rsp_t;
  signal dev_req : dev_req_t;
  signal dev_rsp : dev_rsp_t;

  -- device access-enable --
  signal acc_en : std_ulogic_vector(num_devs_c-1 downto 0);

begin

  -- combine device ports --
  dev_0_req_o <= dev_req(0); dev_rsp(0) <= dev_0_rsp_i;
  dev_1_req_o <= dev_req(1); dev_rsp(1) <= dev_1_rsp_i;
  dev_2_req_o <= dev_req(2); dev_rsp(2) <= dev_2_rsp_i;
  dev_3_req_o <= dev_req(3); dev_rsp(3) <= dev_3_rsp_i;
  dev_4_req_o <= dev_req(4); dev_rsp(4) <= dev_4_rsp_i;
  dev_5_req_o <= dev_req(5); dev_rsp(5) <= dev_5_rsp_i;
  dev_6_req_o <= dev_req(6); dev_rsp(6) <= dev_6_rsp_i;
  dev_7_req_o <= dev_req(7); dev_rsp(7) <= dev_7_rsp_i;

  -- device select --
  acc_en_gen:
  for i in 0 to num_devs_c-1 generate
    acc_en(i) <= '1' when (dev_size_list_c(i) > 0) and dev_en_list_c(i) and
                          (unsigned(host_req_i.addr) >= unsigned(dev_base_list_c(i))) and
                          (unsigned(host_req_i.addr) < (unsigned(dev_base_list_c(i)) + dev_size_list_c(i))) else '0';
  end generate;

  -- request ("fire and forget") --
  bus_request_gen:
  for i in 0 to num_devs_c-1 generate
    bus_request: process(host_req_i, acc_en)
    begin
      dev_req(i) <= xbus_req_terminate_c; -- default: disabled
      if dev_en_list_c(i) then
        dev_req(i)     <= host_req_i;
        dev_req(i).cyc <= host_req_i.cyc and acc_en(i);
        dev_req(i).stb <= host_req_i.stb and acc_en(i);
      end if;
    end process bus_request;
  end generate;

  -- response --
  bus_response: process(dev_rsp, acc_en)
    variable tmp_v : xbus_rsp_t;
  begin
    tmp_v.data := (others => '0');
    tmp_v.ack  := '0';
    tmp_v.err  := '0';
    for i in 0 to num_devs_c-1 loop -- OR all enabled response buses
      if dev_en_list_c(i) then
        tmp_v.data := tmp_v.data or dev_rsp(i).data;
        tmp_v.ack  := tmp_v.ack  or dev_rsp(i).ack;
        tmp_v.err  := tmp_v.err  or dev_rsp(i).err;
      end if;
    end loop;
    host_rsp_o <= tmp_v;
  end process bus_response;

end xbus_gateway_rtl;
