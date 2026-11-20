    -- one cycle of a random main control bus, CSR accesses on a given list
    type csr_list_t is array (natural range <>) of std_ulogic_vector(11 downto 0);
    procedure ctrl_random(l : csr_list_t) is
      variable v : std_ulogic_vector(63 downto 0) := randv(64);
    begin
      ctrl_i.csr_addr  <= l(rndn(l'length - 1) + l'low);
      ctrl_i.csr_we    <= v(0) and v(1);
      ctrl_i.csr_re    <= v(2);
      ctrl_i.csr_wdata <= randv(32);
      ctrl_i.cnt_event <= v(16 downto 8);
      ctrl_i.cpu_priv  <= v(17);
      ctrl_i.cpu_debug <= v(18) and v(19) and v(20);
      ctrl_i.cpu_trap  <= v(21) and v(22) and v(23);
      ctrl_i.cpu_exec  <= v(24);
      ctrl_i.lsu_rd    <= v(25);
      ctrl_i.lsu_wr    <= v(26);
      ctrl_i.lsu_priv  <= v(27);
      tick;
    end procedure;
