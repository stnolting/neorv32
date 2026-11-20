#!/usr/bin/env python3
"""NEORV32 behavioural regression driver. See regress/README.md.

  regress/run.py                    build, run every test, compare with the baselines
  regress/run.py --filter uart      run only tests whose name matches a regular expression
  regress/run.py --update           accept the current behaviour as the new baseline

Exit status is 0 only if every test matches its baseline. With --advisory it is 0 unless the
run itself breaks. If regress/cover exists, line coverage is measured and checked as well.
"""
import argparse
import collections
import glob
import hashlib
import json
import multiprocessing
import os
import re
import shutil
import subprocess
import sys
import time
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import tbgen  # noqa: E402

COVER = None  # optional line coverage, see regress/cover
if os.path.exists(os.path.join(HERE, 'cover', 'cover.py')):
    sys.path.insert(0, os.path.join(HERE, 'cover'))
    import cover as COVER  # noqa: E402

GOLDEN = os.path.join(HERE, 'golden')
TEST_DIRS = ['unit', 'system']  # block-level tests, and whole-processor tests if present
GHDL_FLAGS = ['--std=08', '--work=neorv32', '--workdir=work', '--ieee=standard']
RUN_FLAGS = ['--max-stack-alloc=0', '--ieee-asserts=disable', '--assert-level=error']
SIG_VERSION = 1
MAX_GLOBAL_WINDOWS = 512   # cycle resolution of a mismatch, for all signals together
MAX_SIGNAL_WINDOWS = 32    # cycle resolution of a mismatch, per signal


def log(text):
    print(text, flush=True)


def sh(cmd, cwd=None, check=True, **kw):
    r = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, errors='replace', **kw)
    if check and r.returncode:
        sys.stdout.write(r.stdout[-6000:])
        raise SystemExit(r.stdout[-1500:] + f'\ncommand failed ({r.returncode}): ' + ' '.join(cmd)[:200])
    return r.stdout


# ---- test list ----------------------------------------------------------------------------------

def discover(pattern):
    entities, records = tbgen.load_design()
    tests, gens = [], {}
    paths = [p for d in TEST_DIRS for p in sorted(glob.glob(os.path.join(HERE, d, '*.vhdt')))]
    for path in paths:
        tb, spec, text = tbgen.generate(path, entities, records)
        gens[tb] = text
        configs = spec['configs'] or [(os.path.splitext(os.path.basename(path))[0], {})]
        for name, kv in configs:
            merged = dict(spec['base'])
            merged.update(kv)
            tests.append({'name': name, 'tb': tb,
                          'args': sorted(f'-g{k}={v}' for k, v in merged.items() if k[0] != '@'),
                          'stop': merged.get('@stop', '50ms'),
                          'prog': merged.get('@prog'), 'pflags': merged.get('@pflags', ''),
                          'march': merged.get('@march'), 'mabi': merged.get('@mabi'),
                          'expect': merged.get('@expect'),
                          'source': os.path.relpath(path, ROOT).replace('\\', '/')})
    names = collections.Counter(t['name'] for t in tests)
    dup = [n for n, c in names.items() if c > 1]
    if dup:
        raise SystemExit(f'duplicate test names: {dup}')
    if COVER:
        tests.append(dict(COVER.CANARY))
    selected = [t for t in tests if re.search(pattern, t['name'])] if pattern else tests
    return selected, gens, tests


# ---- software -----------------------------------------------------------------------------------

def build_programs(tests, build):
    done = {}
    for t in tests:
        if not t['prog']:
            continue
        key = '|'.join(str(t[k]) for k in ('prog', 'pflags', 'march', 'mabi'))
        if key in done:
            t['image'] = done[key]
            continue
        src = os.path.join(HERE, 'sw', t['prog'])
        if not os.path.isdir(src):
            src = os.path.join(ROOT, 'sw', 'example', t['prog'])
        tag = hashlib.sha256(key.encode()).hexdigest()[:10]
        dst = os.path.join(build, 'sw', f"{t['prog']}-{tag}")
        shutil.rmtree(dst, ignore_errors=True)
        shutil.copytree(src, dst)
        flags = ['-DUART0_SIM_MODE', '-DUART1_SIM_MODE'] + t['pflags'].split()
        extra = ([f"MARCH={t['march']}"] if t['march'] else []) + ([f"MABI={t['mabi']}"] if t['mabi'] else [])
        sh(['make', f'NEORV32_HOME={ROOT}', 'RISCV_PREFIX=riscv-none-elf-'] + extra + [
            f'USER_FLAGS+={" ".join(flags)} -ffile-prefix-map={ROOT}=.', 'hex'], cwd=dst,
           env=dict(os.environ, SOURCE_DATE_EPOCH='0'))  # fixes __DATE__ and __TIME__
        t['image'] = done[key] = os.path.join(dst, 'neorv32_raw_exe.hex')


def build_rtl(tests, gens, build, incremental):
    gen = os.path.join(build, 'gen')
    if not incremental:
        for name in ('work', 'gen', 'src'):
            shutil.rmtree(os.path.join(build, name), ignore_errors=True)
        for pattern in ('*.o', '*.gcno', '*.gcda', 'e~*'):
            for path in glob.glob(os.path.join(build, pattern)):
                os.remove(path)
    os.makedirs(gen, exist_ok=True)
    os.makedirs(os.path.join(build, 'work'), exist_ok=True)
    for tb, text in gens.items():
        path = os.path.join(gen, tb + '.vhd')
        if not os.path.exists(path) or open(path).read() != text:
            with open(path, 'w', newline='\n') as f:
                f.write(text)
    # compile copies on the build file system: faster, and independent of where the repository is
    core = [re.sub(r'^.*?/core/', 'rtl/core/', x.strip()) for x in open(os.path.join(ROOT, 'rtl', 'file_list_core.f')) if x.strip()]
    core = ['regress/lib/imem_from_file.vhd' if x.endswith('neorv32_imem_image.vhd') else x for x in core]
    rest = ['rtl/system_integration/xbus2axi4_bridge.vhd', 'rtl/system_integration/neorv32_litex_core_complex.vhd',
            'rtl/system_integration/neorv32_vivado_ip.vhd']
    rest += sorted(os.path.relpath(f, ROOT) for f in glob.glob(os.path.join(ROOT, 'rtl', 'test_setups', '*.vhd')))
    rest.append('sim/jtag_dmi_pkg.vhd')
    rest += [f'sim/{n}.vhd' for n in ('psram_model', 'sim_uart_rx', 'xbus_fmem', 'xbus_gateway', 'xbus_memory')]
    rest += [f'regress/lib/{n}.vhd' for n in ('regress_pkg', 'regress_bus', 'regress_axi')]
    if COVER:
        rest.append(COVER.CANARY['source'])
    src = []
    for f in core + rest:
        dst = os.path.join(build, 'src', f)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        if not os.path.exists(dst) or open(dst, 'rb').read() != open(os.path.join(ROOT, f), 'rb').read():
            shutil.copy(os.path.join(ROOT, f), dst)
        src.append(os.path.relpath(dst, build))
    tops = sorted({t['tb'] for t in tests})
    src += [os.path.join('gen', tb + '.vhd') for tb in tops if tb != 'canary_tb']
    # GHDL orders design units by wall-clock analysis time. Under WSL the clock can step back, and
    # then a unit reads as older than one it depends on. That is rare and a clean start clears it.
    for attempt in (1, 2, 3):
        try:
            sh(['ghdl', '-a'] + GHDL_FLAGS + (COVER.ANALYSE_FLAGS if COVER else []) + src, cwd=build)
            for tb in tops:
                sh(['ghdl', '-e'] + GHDL_FLAGS + (COVER.ELAB_FLAGS if COVER else []) + [tb], cwd=build)
            return
        except SystemExit:
            if attempt == 3:
                raise
            log(f'build failed, attempt {attempt}: starting it again')
            shutil.rmtree(os.path.join(build, 'work'))
            os.makedirs(os.path.join(build, 'work'))


# ---- one test -----------------------------------------------------------------------------------

def normalise_stdout(text):
    """Drop what a comment-only edit moves: source positions. Simulation times stay."""
    return re.sub(r'\S+\.vhdl?:\d+:\d+:', '', text)


def digest(trace):
    """Reduce a trace to per-signal hashes, change counts and hash windows."""
    names, cycles = {}, None
    with open(trace, 'rb') as f:
        f.seek(0, 2)
        size = f.tell()
        f.seek(max(0, size - 200))
        tail = f.read().splitlines()
    if tail and tail[-1].startswith(b'# END '):
        cycles = int(tail[-1].split()[2])
    horizon = cycles or 1 << 40
    glob_win = sig_win = 1
    while horizon / glob_win > MAX_GLOBAL_WINDOWS:
        glob_win *= 2
    while horizon / sig_win > MAX_SIGNAL_WINDOWS:
        sig_win *= 2
    hashes, counts, windows = {}, {}, {}
    gcrc = {}
    with open(trace, 'rb') as f:
        for line in f:
            if line[0] == 35:  # '#'
                if line.startswith(b'# S '):
                    _, _, idx, name = line.split()
                    names[idx] = name.decode()
                    hashes[idx] = hashlib.sha1()
                    counts[idx] = 0
                    windows[idx] = {}
                continue
            cyc, idx, _ = line.split(b' ', 2)
            n = int(cyc)
            rest = cyc + b' ' + line[len(cyc) + 1 + len(idx) + 1:]
            hashes[idx].update(rest)
            counts[idx] += 1
            w = n // sig_win
            win = windows[idx]
            win[w] = zlib.crc32(rest, win.get(w, 0))
            g = n // glob_win
            gcrc[g] = zlib.crc32(line, gcrc.get(g, 0))
    signals = []
    for idx, name in names.items():
        wins = ','.join(f'{w}:{c & 0xffffff:06x}' for w, c in sorted(windows[idx].items()))
        signals.append((name, counts[idx], hashes[idx].hexdigest()[:16], wins))
    glist = ','.join(f'{g}:{c & 0xffffff:06x}' for g, c in sorted(gcrc.items()))
    return {'cycles': cycles, 'gwin': glob_win, 'swin': sig_win, 'global': glist, 'signals': signals}


def sig_text(t, d, stdout, image, extra):
    out = [f'version {SIG_VERSION}', f"test {t['name']}", f"tb {t['tb']}",
           f"args {' '.join(t['args'])}", f'image {image}', f"cycles {d['cycles']}",
           f"window {d['gwin']} {d['swin']}", f'stdout {stdout}', f"global {d['global']}"]
    out += [f'file {n} {h}' for n, h in extra]
    out += [f'signal {n} {c} {h} {w}' for n, c, h, w in d['signals']]
    return '\n'.join(out) + '\n'


def run_one(job):
    t, build, keep = job
    rundir = os.path.join(build, 'run', t['name'])
    shutil.rmtree(rundir, ignore_errors=True)
    os.makedirs(rundir)
    image = 'none'
    if t.get('image'):
        shutil.copy(t['image'], os.path.join(rundir, 'image.hex'))
        image = hashlib.sha256(open(t['image'], 'rb').read()).hexdigest()[:16]
    start = time.time()
    cmd = [os.path.join(build, t['tb'])] + t['args'] + [f"--stop-time={t['stop']}"] + RUN_FLAGS
    with open(os.path.join(rundir, 'stdout.txt'), 'wb') as out, open(os.path.join(rundir, 'stderr.txt'), 'wb') as err:
        try:
            rc = subprocess.run(cmd, cwd=rundir, stdout=out, stderr=err, timeout=7200).returncode
        except subprocess.TimeoutExpired:
            rc = 124
    res = {'name': t['name'], 'rc': rc, 'seconds': round(time.time() - start, 1), 'source': t['source']}
    if t['tb'] == 'canary_tb':
        res['sig'] = None
        return res
    text = normalise_stdout(open(os.path.join(rundir, 'stdout.txt'), encoding='latin-1').read())
    stdout = f"{hashlib.sha1(text.encode('latin-1')).hexdigest()[:16]} {text.count(chr(10))}"
    trace = os.path.join(rundir, 'trace.txt')
    if not os.path.exists(trace):
        res['sig'] = None
        res['error'] = 'no trace written'
        return res
    d = digest(trace)
    res['cycles'] = d['cycles']
    extra = []
    for path in sorted(glob.glob(os.path.join(rundir, 'neorv32*'))):
        extra.append((os.path.basename(path), hashlib.sha1(open(path, 'rb').read()).hexdigest()[:16]))
    res['sig'] = sig_text(t, d, stdout, image, extra)
    if d['cycles'] is None:
        res['error'] = 'did not reach the end of its stimulus'
    if t['expect'] and t['expect'] not in text:
        res['error'] = f"expected output is missing: {t['expect']}"
    if not keep:
        os.remove(trace)
    return res


# ---- baselines ----------------------------------------------------------------------------------
# One file per test source: regress/unit/uart.vhdt has its baselines in regress/golden/uart.sig,
# one section per test, each starting with a "version" line.

def golden_name(source):
    return os.path.splitext(os.path.basename(source))[0] + '.sig'


def load_golden():
    """Return {test name: signature text} from all baseline files."""
    out = {}
    for path in sorted(glob.glob(os.path.join(GOLDEN, '*.sig'))):
        cur = None
        for line in open(path).read().splitlines():
            if line.startswith('version '):
                cur = [line]
            elif cur is not None and line.strip():
                cur.append(line)
                if line.startswith('test '):
                    out[line[5:]] = cur
    return {name: '\n'.join(lines) + '\n' for name, lines in out.items()}


def golden_files(sections, source_of):
    """Group signatures into baseline files. Tests that no longer exist are dropped."""
    files = {}
    for name in sorted(sections):
        if name in source_of:
            files.setdefault(golden_name(source_of[name]), []).append(sections[name])
    return {fname: '\n'.join(parts) for fname, parts in files.items()}


def write_golden(directory, files, prune):
    os.makedirs(directory, exist_ok=True)
    for fname, text in files.items():
        with open(os.path.join(directory, fname), 'w', newline='\n') as f:
            f.write(text)
    if prune:
        for path in glob.glob(os.path.join(directory, '*.sig')):
            if os.path.basename(path) not in files:
                os.remove(path)


# ---- comparison ---------------------------------------------------------------------------------

def parse_sig(text):
    s = {'signals': {}, 'files': {}}
    for line in text.splitlines():
        key, _, rest = line.partition(' ')
        if key == 'signal':
            p = rest.split(' ')
            s['signals'][p[0]] = (int(p[1]), p[2], p[3] if len(p) > 3 else '')
        elif key == 'file':
            n, h = rest.split()
            s['files'][n] = h
        else:
            s[key] = rest
    return s


def first_diff(a, b):
    """First window index whose hash differs between two 'index:crc,...' lists."""
    da = dict(x.split(':') for x in a.split(',') if x)
    db = dict(x.split(':') for x in b.split(',') if x)
    bad = [int(k) for k in set(da) | set(db) if da.get(k) != db.get(k)]
    return min(bad) if bad else None


def compare(old_text, new_text):
    """Explain the difference between a baseline and this run. Empty means identical."""
    if old_text == new_text:
        return []
    old, new = parse_sig(old_text), parse_sig(new_text)
    out = []
    for key in ('args', 'image', 'tb'):
        if old.get(key) != new.get(key):
            out.append(f'{key} changed: {old.get(key)} -> {new.get(key)}')
    if old.get('cycles') != new.get('cycles'):
        out.append(f"run length changed: {old.get('cycles')} -> {new.get('cycles')} cycles")
    if old.get('stdout') != new.get('stdout'):
        out.append(f"simulator output changed: {old.get('stdout')} -> {new.get('stdout')}")
    same_win = old.get('window') == new.get('window')
    if same_win:
        g = first_diff(old['global'], new['global'])
        if g is not None:
            size = int(new['window'].split()[0])
            out.append(f'first difference at about cycle {g * size} (within {size} cycles)')
    swin = int(new['window'].split()[1])
    diffs = []
    for name in sorted(set(old['signals']) | set(new['signals'])):
        a, b = old['signals'].get(name), new['signals'].get(name)
        if a is None or b is None:
            diffs.append((-1, f'signal {name} was added or removed'))
        elif a != b:
            w = first_diff(a[2], b[2]) if same_win else None
            where = f'first differs in cycles {w * swin}..{(w + 1) * swin - 1}' if w is not None else 'differs'
            diffs.append((w if w is not None else 1 << 40,
                          f'signal {name} {where} (changes {a[0]} -> {b[0]})'))
    diffs.sort()
    out += [d[1] for d in diffs[:12]]
    if len(diffs) > 12:
        out.append(f'... and {len(diffs) - 12} more signals')
    for name in sorted(set(old['files']) | set(new['files'])):
        if old['files'].get(name) != new['files'].get(name):
            out.append(f'output file {name} changed')
    return out or ['signature text changed']


# ---- main ---------------------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--filter', help='regular expression on test names')
    ap.add_argument('--update', action='store_true', help='accept current behaviour as the baseline')
    ap.add_argument('--jobs', type=int, default=os.cpu_count())
    ap.add_argument('--build', default=os.environ.get('NEORV32_REGRESS_BUILD',
                                                      os.path.expanduser('~/.cache/neorv32-regress/build')))
    ap.add_argument('--report', help='report directory, default <build>/report')
    ap.add_argument('--keep-traces', action='store_true', help='keep full traces under <build>/run')
    ap.add_argument('--incremental', action='store_true', help='keep the previous build and its coverage counts')
    ap.add_argument('--advisory', action='store_true', help='report differences but exit with status 0')
    ap.add_argument('--list', action='store_true', help='list test names and exit')
    args = ap.parse_args()

    tests, gens, all_tests = discover(args.filter)
    source_of = {t['name']: t['source'] for t in all_tests if t['name'] != 'canary'}
    if args.list:
        for t in tests:
            print(t['name'])
        return 0
    if not tests:
        raise SystemExit('no test matches')
    full = not args.filter
    build = args.build
    report = args.report or os.path.join(build, 'report')
    shutil.rmtree(report, ignore_errors=True)
    os.makedirs(report)
    os.makedirs(build, exist_ok=True)
    golden = load_golden()

    t0 = time.time()
    build_programs(tests, build)
    build_rtl(tests, gens, build, args.incremental)
    log(f'built {len({t["tb"] for t in tests})} testbenches in {time.time() - t0:.0f} s')

    with multiprocessing.Pool(args.jobs) as pool:
        results = []
        jobs = [(t, build, args.keep_traces) for t in tests]
        for res in pool.imap_unordered(run_one, jobs):
            results.append(res)
            log(f"  ran {res['name']}: rc={res['rc']} {res.get('cycles')} cycles {res['seconds']} s")
    results.sort(key=lambda r: r['name'])

    failed, passed, accepted = [], 0, {}
    diag = os.path.join(report, 'diagnostics')
    for res in results:
        if res['name'] == 'canary':
            if res['rc'] != 0:
                failed.append((res['name'], [f"canary run failed, rc={res['rc']}"]))
            continue
        problems = []
        if res['rc'] != 0:
            problems.append(f"simulator exit status {res['rc']}")
        if res.get('error'):
            problems.append(res['error'])
        if res['sig'] is not None:
            if not problems:
                accepted[res['name']] = res['sig']  # a test that ran to its end can become the baseline
            if args.update:
                pass
            elif res['name'] in golden:
                problems += compare(golden[res['name']], res['sig'])
            else:
                problems.append('no baseline: run regress/run.py --update')
        if problems:
            failed.append((res['name'], problems))
            d = os.path.join(diag, res['name'])
            os.makedirs(d, exist_ok=True)
            rundir = os.path.join(build, 'run', res['name'])
            for fname in ('stdout.txt', 'stderr.txt'):
                if os.path.exists(os.path.join(rundir, fname)):
                    shutil.copy(os.path.join(rundir, fname), d)
            if res['sig']:
                with open(os.path.join(d, 'actual.sig'), 'w') as f:
                    f.write(res['sig'])
        else:
            passed += 1

    if full and not args.update:
        for name in sorted(set(golden) - set(source_of)):
            failed.append((name, ['baseline has no test: run regress/run.py --update']))

    # The baselines as they would be after accepting this run. The report always holds them, so
    # they can be copied into regress/golden without running anything locally.
    files = golden_files({**golden, **accepted}, source_of)
    write_golden(os.path.join(report, 'golden'), files, False)
    if args.update:
        write_golden(GOLDEN, files, True)

    cov = None
    cov_fail = []
    if COVER:
        log('computing coverage')
        merged = COVER.coverage(build, args.jobs)
        cov = COVER.coverage_report(merged, report)
        if full:
            cov_fail += COVER.canary_check(merged)
            cov_fail += [f'uncovered: {u}' for u in cov['uncovered']]
            cov_fail += [f'stale waiver: {s}' for s in cov['stale']]

    suite = hashlib.sha256()
    for r in results:
        if r['sig']:
            suite.update(r['sig'].encode())
    signature = suite.hexdigest()
    with open(os.path.join(report, 'signatures.txt'), 'w') as f:
        for r in results:
            if r['sig']:
                f.write(f"{hashlib.sha256(r['sig'].encode()).hexdigest()} {r['name']}\n")
        f.write(f'{signature} SUITE\n')

    changed = [name for name, _ in failed]
    if not failed and not cov_fail:
        verdict = 'Nothing changed: every test behaves exactly as recorded.'
    elif failed:
        verdict = (f'**{len(failed)} of {len(results)} tests behave differently than recorded.** '
                   'If you meant to change that behaviour, accept it: run `python3 regress/run.py --update`, '
                   'or copy the `golden` folder of the `regress-report` artifact over `regress/golden`. '
                   'If not, the list below says where to look. See `regress/README.md`.')
    else:
        verdict = 'Behaviour is unchanged, but line coverage has a gap.'
    md = ['# NEORV32 regression', '', verdict, '',
          f'- tests run: {len(results)}, unchanged {passed}, changed {len(failed)}',
          f'- suite signature: `{signature}`']
    if cov:
        md.append(f"- line coverage: {cov['percent']}% ({cov['hit']} of {cov['total']} lines), "
                  f"waived {cov['waived']}, uncovered {cov['missed']}")
    for name, problems in failed:
        md += ['', f'## Changed: {name}', '', f'Test file: `{source_of.get(name, "none")}`', ''] + [f'- {p}' for p in problems]
    if cov_fail:
        md += ['', '## Coverage problems', ''] + [f'- `{c}`' for c in cov_fail[:200]]
        if len(cov_fail) > 200:
            md.append(f'- ... and {len(cov_fail) - 200} more, see uncovered.txt')
    if cov:
        md += ['', '<details><summary>Coverage per file</summary>', '', '```'] + cov['table'] + ['```', '', '</details>']
    text = '\n'.join(md) + '\n'
    with open(os.path.join(report, 'summary.md'), 'w') as f:
        f.write(text)
    with open(os.path.join(report, 'results.json'), 'w') as f:
        json.dump([{k: v for k, v in r.items() if k != 'sig'} for r in results], f, indent=1)
    if os.environ.get('GITHUB_STEP_SUMMARY'):
        with open(os.environ['GITHUB_STEP_SUMMARY'], 'a') as f:
            f.write(text)
    if os.environ.get('GITHUB_ACTIONS') and changed:
        shown = ', '.join(changed[:8]) + (' ...' if len(changed) > 8 else '')
        print(f'::warning title=Regression::{len(changed)} tests behave differently than recorded: {shown}')

    if cov:
        log(cov['table'][-1])
    for name, problems in failed:
        log(f'FAIL {name}')
        for p in problems:
            log(f'    {p}')
    for c in cov_fail[:40]:
        log(f'COVERAGE {c}')
    if len(cov_fail) > 40:
        log(f'COVERAGE ... {len(cov_fail) - 40} more')
    extra = f', {len(cov_fail)} coverage problems' if COVER else ''
    log(f'{len(results)} tests, {len(failed)} changed{extra}, {time.time() - t0:.0f} s, report in {report}')
    if args.advisory:
        return 0
    return 1 if failed or cov_fail else 0


def advisory_main():
    """Like main, but a run that cannot even start is reported instead of failing the job."""
    try:
        return main()
    except SystemExit as stop:
        if isinstance(stop.code, int) or stop.code is None:
            raise
        text = '\n'.join([
            '# NEORV32 regression', '',
            '**The tests could not be built or run**, so nothing was compared. Usually a port or a',
            'generic of a tested module changed, and its test file in `regress/unit` needs the same change.',
            '', '```', str(stop.code), '```', ''])
        print(text)
        if os.environ.get('GITHUB_STEP_SUMMARY'):
            with open(os.environ['GITHUB_STEP_SUMMARY'], 'a') as f:
                f.write(text)
        if os.environ.get('GITHUB_ACTIONS'):
            print('::warning title=Regression::the tests could not be built or run, see the job summary')
        return 0


if __name__ == '__main__':
    sys.exit(advisory_main() if '--advisory' in sys.argv else main())
