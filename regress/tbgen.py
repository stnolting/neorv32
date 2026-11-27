#!/usr/bin/env python3
"""Turn a regress/unit/*.vhdt stimulus file into a complete VHDL testbench.

A .vhdt file only holds what is specific to a test. This script reads the DUT's entity
from the RTL and adds the rest, so that every DUT output is traced without exception:

  --! dut neorv32_uart            entity under test
  --! tb  name                    optional testbench name, default tb_<file name>
  --! clock clk_i                 optional, default clk_i if the DUT has it
  --! reset rstn_i                optional, default rstn_i if the DUT has it
  --! generic NAME=VALUE          default of a testbench generic (one per line)
  --! base NAME=VALUE ...         run-time settings shared by every config
  --! config name NAME=VALUE ...  one test; NAME is a generic or an @option
  --! param NAME : type := value  an extra testbench generic
  --! include FILE                 insert a file from regress/unit here
  --! decl                        architecture declarations follow
  --! arch                        concurrent statements follow
  --! stim                        statements of the stimulus process follow

Options: @stop=TIME (simulation limit), @prog=NAME (program image), @pflags=TEXT,
@expect=TEXT (the simulator output must contain TEXT).

The generated testbench provides, inside the stimulus process:
  tick(n)  chk(cond, msg)  rnd(v)  randv(n)  rndn(max)  and, if the DUT has bus_req_i and
  bus_rsp_o or the req_stb_i style bus, wr(addr, data[, ben]), rd(addr, data), rd(addr) and
  fuzz(abits, n). A clkgen_i input is driven by a neorv32_sys_clock instance.
The process must end by calling done.
"""
import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RTL = ['rtl/core/*.vhd', 'rtl/system_integration/*.vhd', 'rtl/test_setups/*.vhd', 'sim/*.vhd']


def strip_comments(text):
    return re.sub(r'--[^\n]*', '', text)


def split_top(text, sep=';'):
    """Split at sep where parentheses are balanced."""
    parts, depth, cur = [], 0, ''
    for ch in text:
        if ch == '(':
            depth += 1
        elif ch == ')':
            depth -= 1
        if ch == sep and depth == 0:
            parts.append(cur)
            cur = ''
        else:
            cur += ch
    if cur.strip():
        parts.append(cur)
    return parts


def paren_body(text, start):
    """Return the text inside the parenthesis that opens at or after start."""
    pos = text.index('(', start)
    depth = 0
    for i in range(pos, len(text)):
        if text[i] == '(':
            depth += 1
        elif text[i] == ')':
            depth -= 1
            if depth == 0:
                return text[pos + 1:i]
    raise SystemExit('unbalanced parenthesis')


def parse_entity(body):
    generics, ports = [], []
    m = re.search(r'^\s*generic\s*\(', body, re.I | re.M)
    if m:
        for item in split_top(paren_body(body, m.start())):
            if ':' not in item:
                continue
            names, rest = item.split(':', 1)
            default = None
            if ':=' in ':' + rest:
                rest, default = (':' + rest).split(':=', 1)
                rest = rest[1:]
                default = default.strip()
            for name in names.split(','):
                generics.append((name.strip(), ' '.join(rest.split()), default))
    m = re.search(r'^\s*port\s*\(', body, re.I | re.M)
    if m:
        for item in split_top(paren_body(body, m.start())):
            if ':' not in item:
                continue
            names, rest = item.split(':', 1)
            words = rest.split(None, 1)
            direction, kind = words[0].lower(), ' '.join(words[1].split())
            default = None
            if ':=' in kind:
                kind, default = (x.strip() for x in kind.split(':=', 1))
            for name in names.split(','):
                ports.append((name.strip(), direction, kind, default))
    return generics, ports


def load_design():
    entities, records = {}, {}
    for pattern in RTL:
        for path in glob.glob(os.path.join(ROOT, pattern)):
            text = strip_comments(open(path, encoding='latin-1').read())
            for m in re.finditer(r'^(?:[^\n]*;)?\s*entity\s+(\w+)\s+is\b(.*?)^\s*end\s+entity', text, re.S | re.I | re.M):
                entities[m.group(1).lower()] = parse_entity(m.group(2))
            for m in re.finditer(r'type\s+(\w+)\s+is\s+record(.*?)end\s+record', text, re.S | re.I):
                fields = []
                for line in m.group(2).split(';'):
                    if ':' in line:
                        names, kind = line.split(':', 1)
                        for name in names.split(','):
                            fields.append((name.strip(), ' '.join(kind.split())))
                records[m.group(1).lower()] = fields
    return entities, records


def flatten(name, kind, records):
    base = re.sub(r'\s*\(.*', '', kind).lower()
    if base in records:
        out = []
        for field, ftype in records[base]:
            out += flatten(f'{name}.{field}', ftype, records)
        return out
    if base in ('std_ulogic', 'std_logic', 'std_ulogic_vector', 'std_logic_vector'):
        return [(name, kind)]
    raise SystemExit(f'cannot trace {name} of type {kind}')


def init_value(kind, records):
    base = re.sub(r'\s*\(.*', '', kind).lower()
    if base.endswith('_vector'):
        return "(others => '0')"
    if base in ('std_ulogic', 'std_logic'):
        return "'0'"
    if base in records:
        return '(' + ', '.join(f'{f} => {init_value(t, records)}' for f, t in records[base]) + ')'
    raise SystemExit(f'no initial value for type {kind}')


def parse_spec(path):
    spec = {'tb': None, 'clock': None, 'reset': None, 'generic': {}, 'base': {}, 'configs': [], 'param': [],
            'decl': [], 'arch': [], 'stim': []}
    section = None
    lines = []
    for raw in open(path, encoding='utf-8'):
        m = re.match(r'\s*--!\s*include\s+(\S+)', raw)
        if m:
            lines += open(os.path.join(os.path.dirname(path), m.group(1)), encoding='utf-8').readlines()
        else:
            lines.append(raw)
    for raw in lines:
        line = raw.rstrip('\n')
        m = re.match(r'\s*--!\s*(\w+)\s*(.*)$', line)
        if not m:
            if section:
                spec[section].append(line)
            continue
        key, rest = m.group(1).lower(), m.group(2).strip()
        if key in ('decl', 'arch', 'stim'):
            section = key
        elif key in ('dut', 'tb', 'clock', 'reset'):
            spec[key] = rest
        elif key == 'generic':
            name, value = rest.split('=', 1)
            spec['generic'][name.strip()] = value.strip()
        elif key == 'param':
            spec['param'].append(rest)
        elif key == 'base':
            spec['base'].update(parse_kv(rest))
        elif key == 'config':
            words = rest.split(None, 1)
            spec['configs'].append((words[0], parse_kv(words[1] if len(words) > 1 else '')))
        else:
            raise SystemExit(f'{path}: unknown directive {key}')
    if 'dut' not in spec:
        raise SystemExit(f'{path}: no --! dut line')
    return spec


def parse_kv(text):
    """NAME=VALUE pairs; a value may be quoted to hold spaces."""
    return {m.group(1): m.group(2).strip('"')
            for m in re.finditer(r'(@?\w+)=("[^"]*"|\S+)', text)}


TEMPLATE = '''-- GENERATED by regress/tbgen.py from {source}. Do not edit.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.textio.all;

library neorv32;
use neorv32.neorv32_package.all;
use neorv32.regress_pkg.all;
use neorv32.regress_bus_pkg.all;
use neorv32.jtag_dmi_pkg.all;

entity {tb} is
{generics}end entity;

architecture sim of {tb} is

{signals}
  signal tb_done : boolean := false;
{decl}
begin

{clock}
  dut: entity neorv32.{dut}
{gmap}{pmap}

  tracer: process
    file f : text open write_mode is "trace.txt";
    variable l : line;
    variable cyc : natural := 0;
{tvars}
  begin
{theader}
    loop
      wait until falling_edge({tclk});
{tbody}
      cyc := cyc + 1;
      if tb_done then
        write(l, string'("# END "));
        write(l, cyc);
        writeline(f, l);
        file_close(f);
        std.env.finish;
      end if;
    end loop;
  end process;

{arch}
  stimulus: process
    variable rng : rng_t := rng_seed_f(SEED);
    procedure tick(n : natural := 1) is
    begin
      for i in 1 to n loop
        wait until rising_edge({sclk});
      end loop;
    end procedure;
    procedure chk(c : boolean; msg : string) is
    begin
      assert c report "[TB] check failed: " & msg severity failure;
    end procedure;
    procedure rnd(variable v : out std_ulogic_vector) is
    begin
      rng_next(rng, v);
    end procedure;
    impure function randv(n : natural) return std_ulogic_vector is
      variable v : std_ulogic_vector(n-1 downto 0);
    begin
      rng_next(rng, v);
      return v;
    end function;
    impure function rndn(max : natural) return natural is
      variable v : std_ulogic_vector(31 downto 0);
    begin
      rng_next(rng, v);
      return to_integer(unsigned(v(30 downto 0))) mod (max + 1);
    end function;
    procedure done is
    begin
      tb_done <= true;
      wait;
    end procedure;
{bus}{user_procs}
  begin
{stim}
    done;
  end process;

end architecture;
'''

BUS = '''    procedure rd(a : std_ulogic_vector(31 downto 0); variable d : out std_ulogic_vector(31 downto 0)) is
      variable n : natural := 0;
    begin
      bus_req_i.addr <= a;
      bus_req_i.rw   <= '0';
      bus_req_i.ben  <= (others => '0');
      bus_req_i.stb  <= '1';
      tick;
      bus_req_i.stb  <= '0';
      loop
        wait until falling_edge({sclk});
        exit when (bus_rsp_o.ack = '1') or (bus_rsp_o.err = '1');
        n := n + 1;
        assert n < 100000 report "[TB] bus read timeout" severity failure;
      end loop;
      d := bus_rsp_o.data;
      tick;
    end procedure;
    procedure rd(a : std_ulogic_vector(31 downto 0)) is
      variable d : std_ulogic_vector(31 downto 0);
    begin
      rd(a, d);
    end procedure;
    procedure wr(a, d : std_ulogic_vector(31 downto 0); b : std_ulogic_vector(3 downto 0) := "1111") is
      variable n : natural := 0;
    begin
      bus_req_i.addr <= a;
      bus_req_i.data <= d;
      bus_req_i.rw   <= '1';
      bus_req_i.ben  <= b;
      bus_req_i.stb  <= '1';
      tick;
      bus_req_i.stb  <= '0';
      loop
        wait until falling_edge({sclk});
        exit when (bus_rsp_o.ack = '1') or (bus_rsp_o.err = '1');
        n := n + 1;
        assert n < 100000 report "[TB] bus write timeout" severity failure;
      end loop;
      tick;
    end procedure;
'''

MEMBUS = '''    procedure rd(a : std_ulogic_vector(31 downto 0); variable d : out std_ulogic_vector(31 downto 0)) is
      variable n : natural := 0;
    begin
      req_addr_i <= a(req_addr_i'range);
      req_rw_i   <= '0';
      req_ben_i  <= (others => '0');
      req_stb_i  <= '1';
      tick;
      req_stb_i  <= '0';
      loop
        wait until falling_edge({sclk});
        exit when rsp_ack_o = '1';
        n := n + 1;
        assert n < 1000 report "[TB] bus read timeout" severity failure;
      end loop;
      d := rsp_data_o;
      tick;
    end procedure;
    procedure rd(a : std_ulogic_vector(31 downto 0)) is
      variable d : std_ulogic_vector(31 downto 0);
    begin
      rd(a, d);
    end procedure;
    procedure wr(a, d : std_ulogic_vector(31 downto 0); b : std_ulogic_vector(3 downto 0) := "1111") is
      variable n : natural := 0;
    begin
      req_addr_i <= a(req_addr_i'range);
{wdata}      req_rw_i   <= '1';
      req_ben_i  <= b;
      req_stb_i  <= '1';
      tick;
      req_stb_i  <= '0';
      loop
        wait until falling_edge({sclk});
        exit when rsp_ack_o = '1';
        n := n + 1;
        assert n < 10 report "[TB] bus write timeout" severity failure;
      end loop;
      tick;
    end procedure;
'''

FUZZ = '''    -- n random register accesses in the lower 2**abits bytes
    procedure fuzz(abits : natural; n : natural) is
      variable a, d : std_ulogic_vector(31 downto 0);
    begin
      for i in 1 to n loop
        a := (others => '0');
        a(abits - 1 downto 2) := randv(abits - 2);
        d := randv(32);
        if rndn(1) = 1 then
          wr(a, d);
        else
          rd(a);
        end if;
        tick(rndn(3));
      end loop;
    end procedure;
'''


def generate(path, entities, records):
    spec = parse_spec(path)
    dut = spec['dut'].lower()
    if dut not in entities:
        raise SystemExit(f'{path}: no entity {dut}')
    generics, ports = entities[dut]
    tb = spec['tb'] or 'tb_' + os.path.splitext(os.path.basename(path))[0]
    pnames = {p[0].lower() for p in ports}
    clk = spec['clock'] or ('clk_i' if 'clk_i' in pnames else 'tb_clk')
    rst = spec['reset'] or ('rstn_i' if 'rstn_i' in pnames else None)
    if rst == 'none':
        rst = None

    # generics
    lines = ['  generic (', '    SEED : natural := 1;', '    CFG  : natural := 0']
    for extra in spec['param']:
        lines[-1] += ';'
        lines.append('    ' + extra)
    for name, kind, default in generics:
        value = spec['generic'].get(name, default)
        if value is None:
            raise SystemExit(f'{path}: generic {name} needs a --! generic default')
        lines[-1] += ';'
        lines.append(f'    {name} : {kind} := {value}')
    gtext = '\n'.join(lines) + '\n  );\n'

    # signals: one per port, inputs start inactive and resets start active
    sigs = []
    if clk not in pnames:
        sigs.append(f"  signal {clk} : std_ulogic := '0';")
    for name, direction, kind, default in ports:
        low = name.lower()
        if direction == 'in' and low == (rst or '').lower():
            init = "'0'"
        else:
            init = init_value(kind, records)
        if direction == 'out':
            sigs.append(f'  signal {name} : {kind};')
        else:
            sigs.append(f'  signal {name} : {kind} := {init};')

    gmap = ''
    if generics:
        gmap = '  generic map (\n' + ',\n'.join(f'    {g[0]} => {g[0]}' for g in generics) + '\n  )\n'
    pmap = '  port map (\n' + ',\n'.join(f'    {p[0]} => {p[0]}' for p in ports) + '\n  );'

    # tracing: every output, record fields flattened
    traced = []
    for name, direction, kind, _ in ports:
        if direction != 'in':
            traced += flatten(name, kind, records)
    tvars, theader, tbody = [], [], []
    for i, (name, kind) in enumerate(traced):
        tvars.append(f'    variable t{i} : {kind_of(name, kind)};')
        theader.append(f'    write(l, string\'("# S {i} {name}"));\n    writeline(f, l);')
        tbody.append(f'      if ({name} /= t{i}) or (cyc = 0) then\n'
                     f'        t{i} := {name};\n'
                     f'        write(l, cyc);\n'
                     f'        write(l, string\'(" {i} "));\n'
                     f'        write(l, vstr({name}));\n'
                     f'        writeline(f, l);\n'
                     f'      end if;')

    clock = f'  {clk} <= not {clk} after 5 ns;\n'
    if rst:
        clock += f"  {rst} <= '0', '1' after 45 ns;\n"

    user_procs = ''
    stim = spec['stim']
    if stim and stim[0].strip() == '-- procs':
        end = [s.strip() for s in stim].index('-- end procs')
        user_procs = '\n'.join(stim[1:end])
        stim = stim[end + 1:]
    bus = ''
    if {'bus_req_i', 'bus_rsp_o'} <= pnames:
        bus = BUS.format(sclk=clk) + FUZZ
    elif {'req_stb_i', 'rsp_ack_o'} <= pnames:
        wdata = '      req_data_i <= d;\n' if 'req_data_i' in pnames else ''
        bus = MEMBUS.format(sclk=clk, wdata=wdata) + FUZZ
    if 'clkgen_i' in pnames:
        clock += ('\n  clkgen_inst: entity neorv32.neorv32_sys_clock\n'
                  f'    port map (clk_i => {clk}, rstn_i => {rst}, clk_en_o => clkgen_i);\n')
    return tb, spec, TEMPLATE.format(
        source=os.path.relpath(path, ROOT).replace('\\', '/'), tb=tb, generics=gtext, dut=dut,
        signals='\n'.join(sigs), decl='\n'.join(spec['decl']), clock=clock, gmap=gmap, pmap=pmap,
        tvars='\n'.join(tvars), theader='\n'.join(theader), tbody='\n'.join(tbody), tclk=clk,
        sclk=clk, arch='\n'.join(spec['arch']), bus=bus, user_procs=user_procs, stim='\n'.join(stim))


def kind_of(name, kind):
    """Declared subtype of a traced object, usable before the port's generics change."""
    return f"{name}'subtype"


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    entities, records = load_design()
    for path in sys.argv[2:]:
        tb, _, text = generate(path, entities, records)
        with open(os.path.join(out, tb + '.vhd'), 'w', newline='\n') as f:
            f.write(text)


if __name__ == '__main__':
    main()
