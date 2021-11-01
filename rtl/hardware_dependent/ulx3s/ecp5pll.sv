// (c)EMARD
// License=BSD

// parametric ECP5 PLL generator in systemverilog
// actual frequency can be equal or higher than requested
// to see actual frequencies
// trellis log/stdout : search for "MHz", "Derived", "frequency"
// to see actual phase shifts
// diamond log/*.mrp  : search for "Phase", "Desired"

module ecp5pll
#(
  parameter integer in_hz      =  25000000,
  parameter integer out0_hz    =  25000000,
  parameter integer out0_deg   =         0, // keep 0
  parameter integer out0_tol_hz=         0, // tolerance: if freq differs more, then error
  parameter integer out1_hz    =         0,
  parameter integer out1_deg   =         0,
  parameter integer out1_tol_hz=         0,
  parameter integer out2_hz    =         0,
  parameter integer out2_deg   =         0,
  parameter integer out2_tol_hz=         0,
  parameter integer out3_hz    =         0,
  parameter integer out3_deg   =         0,
  parameter integer out3_tol_hz=         0,
  parameter integer reset_en   =         0,
  parameter integer standby_en =         0,
  parameter integer dynamic_en =         0
)
(
  input        clk_i,
  output [3:0] clk_o,
  input        reset,
  input        standby,
  input  [1:0] phasesel,
  input        phasedir, phasestep, phaseloadreg,
  output       locked
);

  localparam PFD_MIN =   3125000;
  localparam PFD_MAX = 400000000;
  localparam VCO_MIN = 400000000;
  localparam VCO_MAX = 800000000;
  localparam VCO_OPTIMAL = (VCO_MIN+VCO_MAX)/2;

  function integer abs(input integer x);
    abs = x > 0 ? x : -x;
  endfunction

  function integer F_ecp5pll(input integer x);
    integer input_div, input_div_min, input_div_max;
    integer output_div, output_div_min, output_div_max;
    integer feedback_div, feedback_div_min, feedback_div_max;
    integer fvco, fout;
    integer error, error_prev;
    integer params_fvco;
    integer div1, div2, div3;

    integer params_refclk_div;
    integer params_feedback_div;
    integer params_output_div;

    params_fvco = 0;
    error_prev = 999999999;
    input_div_min = in_hz/PFD_MAX;
    if(input_div_min < 1)
      input_div_min = 1;
    input_div_max = in_hz/PFD_MIN;
    if(input_div_max > 128)
      input_div_max = 128;
    for(input_div = input_div_min; input_div <= input_div_max; input_div=input_div+1)
    begin
      if(out0_hz / 1000000 * input_div < 2000)
        feedback_div = out0_hz * input_div / in_hz;
      else
        feedback_div = out0_hz / in_hz * input_div;
      feedback_div_min = feedback_div;
      feedback_div_max = feedback_div+1;
      if(feedback_div_min < 1)
        feedback_div_min = 1;
      if(feedback_div_max > 80)
        feedback_div_max = 80;
      for(feedback_div = feedback_div_min; feedback_div <= feedback_div_max; feedback_div = feedback_div+1)
      begin
        output_div_min = (VCO_MIN/feedback_div) / (in_hz/input_div);
        if(output_div_min < 1)
          output_div_min = 1;
        output_div_max = (VCO_MAX/feedback_div) / (in_hz/input_div);
        if(output_div_max > 128)
          output_div_max = 128;
        fout = in_hz * feedback_div / input_div;
        for(output_div = output_div_min; output_div <= output_div_max; output_div=output_div+1)
        begin
          fvco = fout * output_div;
          error = abs(fout-out0_hz)
                + (out1_hz > 0 ? abs(fvco/(fvco >= out1_hz ? fvco/out1_hz : 1)-out1_hz) : 0)
                + (out2_hz > 0 ? abs(fvco/(fvco >= out2_hz ? fvco/out2_hz : 1)-out2_hz) : 0)
                + (out3_hz > 0 ? abs(fvco/(fvco >= out3_hz ? fvco/out3_hz : 1)-out3_hz) : 0);
          if( error < error_prev
          || (error == error_prev && abs(fvco-VCO_OPTIMAL) < abs(params_fvco-VCO_OPTIMAL)) )
          begin
            error_prev              = error;
            params_refclk_div       = input_div;
            params_feedback_div     = feedback_div;
            params_output_div       = output_div;
            params_fvco             = fvco;
          end
        end
      end
    end
    // FIXME in the future when yosys supports struct
    if(x==0)
      F_ecp5pll = params_refclk_div;
    if(x==1)
      F_ecp5pll = params_feedback_div;
    if(x==2)
      F_ecp5pll = params_output_div;
  endfunction

  function integer F_primary_phase(input integer output_div, deg);
    integer phase_compensation;
    integer phase_count_x8;

    phase_compensation = (output_div+1)/2*8-8+output_div/2*8; // output_div/2*8 = 180 deg shift
    phase_count_x8     = phase_compensation + 8*output_div*deg/360;
    if(phase_count_x8 > 1023)
      phase_count_x8 = phase_count_x8 % (output_div*8); // wraparound 360 deg
    F_primary_phase = phase_count_x8;
  endfunction

  // FIXME it is inefficient to call F_ecp5pll multiple times
  localparam params_refclk_div       = F_ecp5pll(0);
  localparam params_feedback_div     = F_ecp5pll(1);
  localparam params_output_div       = F_ecp5pll(2);
  localparam params_fout             = in_hz * params_feedback_div / params_refclk_div;
  localparam params_fvco             = params_fout * params_output_div;

  localparam params_primary_phase_x8 = F_ecp5pll(3);
  localparam params_primary_cphase   = F_primary_phase(params_output_div, out0_deg) / 8;
  localparam params_primary_fphase   = F_primary_phase(params_output_div, out0_deg) % 8;

  function integer F_secondary_divisor(input integer sfreq);
    F_secondary_divisor = 1;
    if(sfreq > 0)
      if(params_fvco >= sfreq)
        F_secondary_divisor = params_fvco/sfreq;
  endfunction

  function integer F_secondary_phase(input integer sfreq, sphase);
    integer div, freq;
    integer phase_compensation, phase_count_x8;

    phase_count_x8 = 0;
    if(sfreq > 0)
    begin
      div = 1;
      if(params_fvco >= sfreq)
        div = params_fvco/sfreq;
      freq = params_fvco/div;
      phase_compensation = div*8-8;
      phase_count_x8 = phase_compensation + 8*div*sphase/360;
      if(phase_count_x8 > 1023)
        phase_count_x8 = phase_count_x8 % (div*8); // wraparound 360 deg
    end

    F_secondary_phase = phase_count_x8;
  endfunction

  localparam params_secondary1_div      = F_secondary_divisor(out1_hz);
  localparam params_secondary1_cphase   = F_secondary_phase  (out1_hz, out1_deg) / 8;
  localparam params_secondary1_fphase   = F_secondary_phase  (out1_hz, out1_deg) % 8;
  localparam params_secondary2_div      = F_secondary_divisor(out2_hz);
  localparam params_secondary2_cphase   = F_secondary_phase  (out2_hz, out2_deg) / 8;
  localparam params_secondary2_fphase   = F_secondary_phase  (out2_hz, out2_deg) % 8;
  localparam params_secondary3_div      = F_secondary_divisor(out3_hz);
  localparam params_secondary3_cphase   = F_secondary_phase  (out3_hz, out3_deg) / 8;
  localparam params_secondary3_fphase   = F_secondary_phase  (out3_hz, out3_deg) % 8;

  // check if generated frequencies are out of range
  localparam error_out0_hz =               abs(out0_hz - params_fout)                         > out0_tol_hz;
  localparam error_out1_hz = out1_hz > 0 ? abs(out1_hz - params_fvco / params_secondary1_div) > out1_tol_hz : 0;
  localparam error_out2_hz = out2_hz > 0 ? abs(out2_hz - params_fvco / params_secondary2_div) > out2_tol_hz : 0;
  localparam error_out3_hz = out3_hz > 0 ? abs(out3_hz - params_fvco / params_secondary3_div) > out3_tol_hz : 0;
  // diamond: won't compile this, comment it out. Workaround follows using division by zero

  if(error_out0_hz) $error("out0_hz tolerance exceeds out0_tol_hz");
  if(error_out1_hz) $error("out1_hz tolerance exceeds out1_tol_hz");
  if(error_out2_hz) $error("out2_hz tolerance exceeds out2_tol_hz");
  if(error_out3_hz) $error("out3_hz tolerance exceeds out3_tol_hz");

  // diamond: trigger error with division by zero, doesn't accept $error()
  localparam trig_out0_hz = error_out0_hz ? 1/0 : 0;
  localparam trig_out1_hz = error_out1_hz ? 1/0 : 0;
  localparam trig_out2_hz = error_out2_hz ? 1/0 : 0;
  localparam trig_out3_hz = error_out3_hz ? 1/0 : 0;

  wire [1:0] PHASESEL_HW = phasesel-1;
  wire CLKOP; // internal

  // TODO: frequencies in MHz if passed as "attributes"
  // will appear in diamond *.mrp file like "Output Clock(P) Frequency (MHz):"
  // but I don't know how to pass string parameters for this:
  // (* FREQUENCY_PIN_CLKI="025.000000" *)
  // (* FREQUENCY_PIN_CLKOP="023.345678" *)
  // (* FREQUENCY_PIN_CLKOS="034.234567" *)
  // (* FREQUENCY_PIN_CLKOS2="111.345678" *)
  // (* FREQUENCY_PIN_CLKOS3="123.456789" *)
  (* ICP_CURRENT="12" *) (* LPF_RESISTOR="8" *) (* MFG_ENABLE_FILTEROPAMP="1" *) (* MFG_GMCREF_SEL="2" *)
  EHXPLLL
  #(
    .CLKI_DIV     (params_refclk_div),
    .CLKFB_DIV    (params_feedback_div),
    .FEEDBK_PATH  ("CLKOP"),

    .OUTDIVIDER_MUXA("DIVA"),
    .CLKOP_ENABLE ("ENABLED"),
    .CLKOP_DIV    (params_output_div),
    .CLKOP_CPHASE (params_primary_cphase),
    .CLKOP_FPHASE (params_primary_fphase),

    .OUTDIVIDER_MUXB("DIVB"),
    .CLKOS_ENABLE (out1_hz > 0 ? "ENABLED" : "DISABLED"),
    .CLKOS_DIV    (params_secondary1_div),
    .CLKOS_CPHASE (params_secondary1_cphase),
    .CLKOS_FPHASE (params_secondary1_fphase),

    .OUTDIVIDER_MUXC("DIVC"),
    .CLKOS2_ENABLE(out2_hz > 0 ? "ENABLED" : "DISABLED"),
    .CLKOS2_DIV   (params_secondary2_div),
    .CLKOS2_CPHASE(params_secondary2_cphase),
    .CLKOS2_FPHASE(params_secondary2_fphase),

    .OUTDIVIDER_MUXD("DIVD"),
    .CLKOS3_ENABLE(out3_hz > 0 ? "ENABLED" : "DISABLED"),
    .CLKOS3_DIV   (params_secondary3_div),
    .CLKOS3_CPHASE(params_secondary3_cphase),
    .CLKOS3_FPHASE(params_secondary3_fphase),

    .INTFB_WAKE   ("DISABLED"),
    .STDBY_ENABLE (standby_en ? "ENABLED" : "DISABLED"),
    .PLLRST_ENA   (standby_en ? "ENABLED" : "DISABLED"),
    .DPHASE_SOURCE(standby_en ? "ENABLED" : "DISABLED"),
    .PLL_LOCK_MODE(0)
  )
  pll_inst
  (
    .RST(1'b0),
    .STDBY(1'b0),
    .CLKI(clk_i),
    .CLKOP(CLKOP),
    .CLKOS (clk_o[1]),
    .CLKOS2(clk_o[2]),
    .CLKOS3(clk_o[3]),
    .CLKFB(CLKOP),
    .CLKINTFB(),
    .PHASESEL1(PHASESEL_HW[1]),
    .PHASESEL0(PHASESEL_HW[0]),
    .PHASEDIR(phasedir),
    .PHASESTEP(phasestep),
    .PHASELOADREG(phaseloadreg),
    .PLLWAKESYNC(1'b0),
    .ENCLKOP(1'b0),
    .ENCLKOS(1'b0),
    .ENCLKOS2(1'b0),
    .ENCLKOS3(1'b0),
    .LOCK(locked)
  );
  assign clk_o[0] = CLKOP;

endmodule
