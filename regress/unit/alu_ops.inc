    -- ALU co-processor operation: expects signal cp_done in the testbench
    constant corner_c : suv32_list_t := (x"00000000", x"00000001", x"FFFFFFFF", x"80000000", x"7FFFFFFF",
                                         x"AAAAAAAA", x"55555555", x"0000FFFF", x"FFFF0000", x"00000002",
                                         x"80000001", x"000000FF", x"FF000000", x"01234567", x"FEDCBA98");
    variable op_count : natural := 0;
    procedure cp(o5 : std_ulogic; f3 : std_ulogic_vector(2 downto 0); f12 : std_ulogic_vector(11 downto 0);
                 a, b : std_ulogic_vector(31 downto 0); limit : natural := 45) is
    begin
      ctrl_i.ir_opcode <= '0' & o5 & "10011";
      ctrl_i.ir_funct3 <= f3;
      ctrl_i.ir_funct12 <= f12;
      ctrl_i.rf_rs2 <= f12(4 downto 0);
      rs1_i <= a;
      rs2_i <= b;
      ctrl_i.alu_cp_alu <= '1';
      tick;
      ctrl_i.alu_cp_alu <= '0';
      op_count := op_count + 1;
      if (op_count mod 37) = 0 then -- abort every now and then
        tick(2);
        ctrl_i.cpu_trap <= '1';
        tick;
        ctrl_i.cpu_trap <= '0';
        tick(2);
      else
        for i in 1 to limit loop
          tick;
          exit when cp_done = '1';
        end loop;
        tick;
      end if;
    end procedure;
    -- every funct7 and funct3, with fixed and random operands
    procedure cp_sweep(rounds : natural) is
      variable f7 : std_ulogic_vector(6 downto 0);
      variable f5 : std_ulogic_vector(4 downto 0);
    begin
      for r in 0 to rounds loop
        for o in 0 to 1 loop
          for f in 0 to 127 loop
            f7 := std_ulogic_vector(to_unsigned(f, 7));
            for t in 0 to 7 loop
              f5 := randv(5);
              if (t mod 2) = 0 then
                f5 := std_ulogic_vector(to_unsigned((f + r) mod 8, 5));
              end if;
              cp(std_ulogic'val(2 + o), std_ulogic_vector(to_unsigned(t, 3)), f7 & f5, randv(32), randv(32), 40);
            end loop;
          end loop;
        end loop;
      end loop;
    end procedure;
    -- every encoding once: each opcode bit 5, funct3 and funct12
    procedure cp_exhaustive(limit : natural) is
    begin
      for o in 0 to 1 loop
        for t in 0 to 7 loop
          for f in 0 to 4095 loop
            cp(std_ulogic'val(2 + o), std_ulogic_vector(to_unsigned(t, 3)), std_ulogic_vector(to_unsigned(f, 12)), randv(32), randv(32), limit);
          end loop;
        end loop;
      end loop;
    end procedure;
    -- corner operands on one operation
    procedure cp_corners(o5 : std_ulogic; f3 : std_ulogic_vector(2 downto 0); f12 : std_ulogic_vector(11 downto 0)) is
    begin
      for a in corner_c'range loop
        for b in corner_c'range loop
          cp(o5, f3, f12, corner_c(a), corner_c(b));
        end loop;
      end loop;
    end procedure;
