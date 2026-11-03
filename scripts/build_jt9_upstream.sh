#!/usr/bin/env bash
# Build the upstream `jt9` and `wsprd` the upstream baseline is measured
# against (docs/notes/upstream/, scripts/upstream-baseline.py).
#
# Usage:
#   scripts/build_jt9_upstream.sh [WSJT-X-repo] [tag]
#     WSJT-X-repo  a clone that has the tag   (default ../WSJT-X)
#     tag          the release to build       (default v3.2.0-rc1)
#   JT9_TRACE=1 scripts/build_jt9_upstream.sh ../WSJT-X v3.2.0-rc1
#     builds into build-<tag>-trace with WRITE statements only (no decoder logic changes) that
#     append the FT8 decoder's stages to ./ft8_trace.dat in jt9's working directory: STAGE
#     (nzhsym), PASS/NCAND, CAND (what sync8 offered), GATE (ft8b's hard-sync bail-out), DEC (which
#     LLR/AP pass, BP or OSD, hard errors), FAIL (every pass failed), REJ, SUB (subtractions).
#     Read by bench/ft8-compare/jt9_trace.py. Never use this binary for timings.
#   JT9_ORACLE_MAX_DRIFT=1 scripts/build_jt9_upstream.sh ../WSJT-X v3.3.0-beta1
#     builds into build-<tag>-maxdrift with ONE added line: the Q65 `max_drift` that
#     jt9 takes from `params%max_drift` (always 0 on the command line, so Q65's stage 5,
#     the w3sz drift compensation, never runs) is overridden by the environment variable
#     MFSK_ORACLE_MAX_DRIFT when it is set. Nothing in the decoder changes; it is the only
#     way to drive stage 5 from jt9, for comparing mfsk-core's `max_drift` with upstream's.
#
# Output: target/upstream/build-<tag>/{jt9,wsprd}, and their sha256 on stdout.
# The source is exported with `git archive`, so the clone is not touched.
#
# WHY A SCRIPT
#
# The upstream baseline is only reproducible if the binary is. The one used
# for FT8_BENCHMARK_LOG.md §13 (2026-09-24) was built by hand in a temporary
# directory, and the edits it needed were recorded in prose only. When it
# was needed again it had to be rebuilt from that prose. These are those
# edits. None of them touches a decoder: each only drops a component that
# jt9 and wsprd do not link and that this host cannot configure.
#   - Qt5::WebSockets, used by the GUI and the TCI test simulator only,
#     is not installed here; dropped from find_package and wsjt_qt's link.
#   - map65 needs PortAudio, also not installed; gated behind
#     WSJT_SKIP_MAP65, including its install rule.
#   - docs and manpages need asciidoctor / a2x; turned off by upstream's own
#     options.
# CMake still stops at tests/unit/tci (it wants WebSockets), after it has
# written the Makefiles jt9 and wsprd need, so its exit status is ignored
# and the build's is checked instead.
#
# Check the result against the known counts: `jt9 -8 -d1/-d2/-d3` on
# embedded-poc/assets/qso3_busy.wav decodes 14 / 20 / 21 with v3.2.0-rc1.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WSJTX="${1:-$REPO_ROOT/../WSJT-X}"
TAG="${2:-v3.2.0-rc1}"

OUT="$REPO_ROOT/target/upstream"
LABEL="$TAG${JT9_ORACLE_MAX_DRIFT:+-maxdrift}${JT9_TRACE:+-trace}"
SRC="$OUT/wsjtx-$LABEL"
BUILD="$OUT/build-$LABEL"
mkdir -p "$OUT"
rm -rf "$SRC" "$BUILD"
mkdir -p "$SRC" "$BUILD"

git -C "$WSJTX" archive "$TAG" | tar -x -C "$SRC"
echo "source: $TAG = $(git -C "$WSJTX" rev-parse "$TAG^{commit}")"

if [ -n "${JT9_ORACLE_MAX_DRIFT:-}" ]; then
python3 - "$SRC" <<'PY'
import sys
path = sys.argv[1] + "/lib/decoder_engine.f90"
s = open(path).read()
old = "request%q65%max_drift_symbol_rates=params%max_drift"
if s.count(old) != 1:
    sys.exit(f"{path}: expected exactly one '{old}'")
new = (old + "\n"
       "    block\n"
       "      character(len=16) :: oracle_v\n"
       "      integer :: oracle_st, oracle_md\n"
       "      call get_environment_variable('MFSK_ORACLE_MAX_DRIFT', oracle_v, status=oracle_st)\n"
       "      if (oracle_st == 0) then\n"
       "         read(oracle_v, *, iostat=oracle_st) oracle_md\n"
       "         if (oracle_st == 0) request%q65%max_drift_symbol_rates = oracle_md\n"
       "      end if\n"
       "    end block")
open(path, "w").write(s.replace(old, new, 1))
PY
fi

if [ -n "${JT9_TRACE:-}" ]; then
python3 - "$SRC" <<'PY'
import sys
src = sys.argv[1] + "/lib/"
def tr(fields):
    return ("       open(unit=87,file='ft8_trace.dat',position='append',status='unknown')\n"
            f"       write(87,*) {fields}\n"
            "       close(87)\n")
def patch(path, old, new):
    s = open(path).read()
    if s.count(old) != 1:
        sys.exit(f"{path}: expected exactly one of: {old!r} (found {s.count(old)})")
    open(path, "w").write(s.replace(old, new, 1))
d = src + "ft8_decode.f90"
patch(d, "    call ft8apset(mycall12,hiscall12,ncontest,apsym2,aph10)\n",
      tr("'STAGE',nzhsym,ndec_early") + "    call ft8apset(mycall12,hiscall12,ncontest,apsym2,aph10)\n")
patch(d, "      call timer('sync8   ',0)\n", tr("'PASS',nzhsym,ipass,ndecodes") + "      call timer('sync8   ',0)\n")
patch(d, "      call timer('sync8   ',1)\n", "      call timer('sync8   ',1)\n" + tr("'NCAND',nzhsym,ipass,ncand"))
patch(d, "        call timer('ft8b    ',0)\n", tr("'CAND',nzhsym,ipass,icand,sync,f1,xdt") + "        call timer('ft8b    ',0)\n")
patch(d, "             call subtractft8(dd,itone_save(1,i),f1_save(i),xdt_save(i),  &\n",
      tr("'SUB',nzhsym,f1_save(i),xdt_save(i)-0.5,'early'") + "             call subtractft8(dd,itone_save(1,i),f1_save(i),xdt_save(i),  &\n")
patch(d, "          call subtractft8(dd,itone_save(1,i),f1_save(i),xdt_save(i),.true.)\n",
      tr("'SUB',nzhsym,f1_save(i),xdt_save(i)-0.5,'late'") + "          call subtractft8(dd,itone_save(1,i),f1_save(i),xdt_save(i),.true.)\n")
b = src + "ft8/ft8b.f90"
patch(b, "  if(nsync.le.syncmin) then ! bail out\n    nbadcrc=1\n",
      "  if(nsync.le.syncmin) then ! bail out\n" + tr("'GATE',nzhsym,f1,xdt-0.5,nsync,syncmin") + "    nbadcrc=1\n")
patch(b, "     nbadcrc=0\n     call get_ft8_tones_from_77bits(message77,itone)\n",
      "     nbadcrc=0\n" + tr("'DEC',nzhsym,imetric,ipass,iaptype,ntype,nharderrors,dmin,nsync,f1,xdt-0.5,trim(msg37)") + "     call get_ft8_tones_from_77bits(message77,itone)\n")
patch(b, "    if(nsync.le.10 .and. xsnr.lt.-25.0) then    !bail out, likely false decode\n       nbadcrc=1\n",
      "    if(nsync.le.10 .and. xsnr.lt.-25.0) then    !bail out, likely false decode\n" + tr("'REJ',nzhsym,f1,xdt-0.5,nsync,xsnr") + "       nbadcrc=1\n")
patch(b, "    if(xsnr .lt. -25.0) xsnr=-25.0\n    return\n  enddo\n  return\n",
      "    if(xsnr .lt. -25.0) xsnr=-25.0\n    return\n  enddo\n" + tr("'FAIL',nzhsym,imetric,f1,xdt-0.5,nsync,nharderrors,dmin") + "  return\n")
dump = ("       open(unit=87,file='ft8_trace.dat',position='append',status='unknown')\n"
        + "".join(f"       write(87,'(a,2i4,f9.3,f8.4,i3,174f8.3)') 'LLR',nzhsym,imetric,f1,xdt-0.5,{k},{n}\n"
                  for k, n in enumerate(["llra", "llrb", "llrc", "llrd", "llre"], 1))
        + "       close(87)\n")
patch(b, "  llre=scalefac*bmete\n", "  llre=scalefac*bmete\n" + dump)
cs_dump = ("  block\n"
           "    character(len=32) :: cenv\n"
           "    integer :: ienv, ik, ii\n"
           "    real :: fenv\n"
           "    call get_environment_variable('JT9_TRACE_CS_F',cenv,status=ienv)\n"
           "    if(ienv.eq.0) then\n"
           "      read(cenv,*) fenv\n"
           "      if(abs(f1-fenv).lt.0.6) then\n"
           "        open(unit=87,file='ft8_trace.dat',position='append',status='unknown')\n"
           "        write(87,'(a,2i4,f9.3,f8.4)') 'CSHDR',nzhsym,imetric,f1,xdt-0.5\n"
           "        do ik=1,79\n"
           "          write(87,'(a,i3,16f11.4)') 'CS',ik,(real(cs(ii,ik)),aimag(cs(ii,ik)),ii=0,7)\n"
           "        enddo\n"
           "        close(87)\n"
           "      endif\n"
           "    endif\n"
           "  end block\n")
patch(b, "  do nsym=1,3\n", cs_dump + "  do nsym=1,3\n")
sync_dump = ("  block\n"
             "    character(len=32) :: cenv\n"
             "    integer :: ienv, ibb, jj\n"
             "    real :: fenv\n"
             "    call get_environment_variable('JT9_TRACE_SYNC_F',cenv,status=ienv)\n"
             "    if(ienv.eq.0) then\n"
             "      read(cenv,*) fenv\n"
             "      ibb=nint(fenv/df)\n"
             "      open(unit=87,file='ft8_trace.dat',position='append',status='unknown')\n"
             "      write(87,'(a,2f14.5,2i5,2f14.5)') 'SYNCHDR',base,base2,jpeak(ibb),jpeak2(ibb),red(ibb),red2(ibb)\n"
             "      write(87,'(a,125f12.3)') 'SYNC2D',(sync2d(ibb,jj),jj=-JZ,JZ)\n"
             "      close(87)\n"
             "    endif\n"
             "  end block\n")
patch(src + "ft8/sync8.f90", "  red2=red2/base2\n", "  red2=red2/base2\n" + sync_dump)
resid_dump = ("   block\n"
              "     character(len=32) :: rfn\n"
              "     write(rfn,'(a,i2.2,a,i1,a)') 'resid_',nzhsym,'_',ipass,'.bin'\n"
              "     open(unit=88,file=rfn,access='stream',form='unformatted',status='replace')\n"
              "     write(88) dd\n"
              "     close(88)\n"
              "   end block\n")
patch(d, "   enddo  ! ipass\n", resid_dump + "   enddo  ! ipass\n")
PY
fi

sed -i 's/ Qt5::WebSockets//' "$SRC/CMakeLists.txt"
# v3.2.0-rc1 lists `... LinguistTools WebSockets REQUIRED`; v3.3.0-beta1 `... LinguistTools WebSockets Concurrent REQUIRED`
sed -i 's/ LinguistTools WebSockets / LinguistTools /' "$SRC/CMake/Dependencies.cmake"
python3 - "$SRC" <<'PY'
import sys
src = sys.argv[1]
def patch(path, old, new):
    s = open(path).read()
    if old not in s:
        sys.exit(f"{path}: expected text not found; this script targets v3.2.0-rc1 and v3.3.0-beta1")
    open(path, "w").write(s.replace(old, new, 1))
patch(f"{src}/CMakeLists.txt",
      "find_package (Portaudio REQUIRED)\nadd_subdirectory (map65)",
      "if (NOT WSJT_SKIP_MAP65)\nfind_package (Portaudio REQUIRED)\nadd_subdirectory (map65)\nendif ()")
old = ("install (TARGETS map65\n"
       "  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime\n"
       "  BUNDLE DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime\n"
       "  )")
patch(f"{src}/CMake/Install.cmake", old, "if (TARGET map65)\n" + old + "\nendif ()")
PY

( cd "$BUILD" && cmake -DCMAKE_BUILD_TYPE=Release -DWSJT_SKIP_MAP65=ON -DWSJT_SKIP_QMAP=ON \
    -DWSJT_GENERATE_DOCS=OFF -DWSJT_SKIP_MANPAGES=ON "$SRC" > cmake.log 2>&1 ) || true
( cd "$BUILD" && make -j"$(nproc)" jt9 wsprd > make.log 2>&1 ) || {
  echo "error: build failed, see $BUILD/make.log" >&2
  exit 1
}
sha256sum "$BUILD/jt9" "$BUILD/wsprd"
