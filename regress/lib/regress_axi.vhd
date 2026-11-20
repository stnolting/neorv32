-- NEORV32 regression - small AXI4 memory-mapped device for the bridge and Vivado wrapper tests.
-- Read data is a function of the address. Addresses with upper nibble E answer SLVERR. A wait
-- pattern from an internal LFSR varies the ready and valid timing, so that stalls are exercised.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity regress_axi_slave is
  port (
    clk      : in  std_logic;
    resetn   : in  std_logic;
    awaddr   : in  std_logic_vector(31 downto 0);
    awlen    : in  std_logic_vector(7 downto 0);
    awvalid  : in  std_logic;
    awready  : out std_logic;
    wlast    : in  std_logic;
    wvalid   : in  std_logic;
    wready   : out std_logic;
    araddr   : in  std_logic_vector(31 downto 0);
    arlen    : in  std_logic_vector(7 downto 0);
    arvalid  : in  std_logic;
    arready  : out std_logic;
    rdata    : out std_logic_vector(31 downto 0);
    rresp    : out std_logic_vector(1 downto 0);
    rlast    : out std_logic;
    rvalid   : out std_logic;
    rready   : in  std_logic;
    bresp    : out std_logic_vector(1 downto 0);
    bvalid   : out std_logic;
    bready   : in  std_logic
  );
end entity;

architecture sim of regress_axi_slave is
  signal lfsr : unsigned(15 downto 0) := x"ACE1";
  signal stall : std_logic;
  signal rbusy, wbusy, rv, bv : std_logic := '0';
  signal raddr : unsigned(31 downto 0);
  signal rcnt : unsigned(7 downto 0);
  signal werr : std_logic := '0';
begin

  process(clk)
  begin
    if rising_edge(clk) then
      lfsr <= lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
    end if;
  end process;
  stall <= '1' when lfsr(1 downto 0) = "00" else '0';

  arready <= (not rbusy) and (not stall);
  awready <= (not wbusy) and (not stall);
  wready  <= wbusy and (not stall) and (not bv);
  rvalid  <= rv;
  bvalid  <= bv;
  rdata   <= std_logic_vector(raddr xor x"A5A55A5A");
  rlast   <= '1' when rcnt = 0 else '0';
  rresp   <= "10" when raddr(31 downto 28) = x"E" else "00";
  bresp   <= "10" when werr = '1' else "00";

  process(clk, resetn)
  begin
    if resetn = '0' then
      rbusy <= '0';
      wbusy <= '0';
      rv    <= '0';
      bv    <= '0';
      raddr <= (others => '0');
      rcnt  <= (others => '0');
    elsif rising_edge(clk) then
      if (rbusy = '0') and (arvalid = '1') and (stall = '0') then
        rbusy <= '1';
        raddr <= unsigned(araddr);
        rcnt  <= unsigned(arlen);
      end if;
      if (rbusy = '1') then
        if (rv = '1') and (rready = '1') then
          rv <= '0';
          if rcnt = 0 then
            rbusy <= '0';
          else
            rcnt  <= rcnt - 1;
            raddr <= raddr + 4;
          end if;
        elsif (rv = '0') and (stall = '0') then
          rv <= '1';
        end if;
      end if;
      if (wbusy = '0') and (awvalid = '1') and (stall = '0') then
        wbusy <= '1';
        werr  <= '0';
        if awaddr(31 downto 28) = x"E" then
          werr <= '1';
        end if;
      end if;
      if (wbusy = '1') and (bv = '0') and (wvalid = '1') and (stall = '0') and (wlast = '1') then
        bv <= '1';
      end if;
      if (bv = '1') and (bready = '1') then
        bv <= '0';
        wbusy <= '0';
      end if;
    end if;
  end process;

end architecture;
