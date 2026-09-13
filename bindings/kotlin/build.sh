#!/usr/bin/env bash
# Build the JNI shim + Kotlin binding and run the JVM test.
#
# Needs: a JDK (JAVA_HOME), kotlinc on PATH, and a C compiler.
#
# Every stage is wrapped in `timeout` and announces itself before it
# starts. That is not defensive dressing: the first CI run of this
# script sat in one step for 72 minutes with no output and had to be
# cancelled, so there was nothing to diagnose from. A stage that hangs
# now fails with a message naming the stage.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HERE="$ROOT/bindings/kotlin"
OUT="$HERE/build"
mkdir -p "$OUT"

: "${JAVA_HOME:?set JAVA_HOME to a JDK}"
command -v kotlinc >/dev/null || { echo "kotlinc not on PATH"; exit 1; }

# Per-stage ceilings. Generous against a cold cache on a 2-core runner,
# and still far short of "nobody notices until the job times out".
: "${MFSK_KT_CARGO_TIMEOUT:=900}"
: "${MFSK_KT_KOTLINC_TIMEOUT:=900}"
: "${MFSK_KT_CC_TIMEOUT:=120}"
: "${MFSK_KT_JAVA_TIMEOUT:=300}"

stage() { echo; echo "==> $*"; }

stage "cargo build -p mfsk-ffi --release"
timeout "$MFSK_KT_CARGO_TIMEOUT" \
    cargo build --manifest-path "$ROOT/Cargo.toml" -p mfsk-ffi --release \
    || { echo "::error::cargo build timed out or failed"; exit 1; }

# Compile to a classes directory rather than a fat jar. `-include-runtime`
# repackages the whole Kotlin stdlib on every run, which is the slowest
# thing this script could do and buys nothing: the stdlib jar is already
# on disk next to kotlinc, and `java` takes it on the classpath.
KOTLIN_HOME="$(cd "$(dirname "$(command -v kotlinc)")/.." && pwd)"
STDLIB="$KOTLIN_HOME/lib/kotlin-stdlib.jar"
[ -f "$STDLIB" ] || { echo "::error::kotlin-stdlib.jar not found at $STDLIB"; exit 1; }

stage "kotlinc (binding + test) → $OUT/classes"
# No pipe. Piping kotlinc's output through `grep` hid whatever it was
# doing during that 72-minute run, and a filter that removes warnings
# is not worth being unable to see the compiler at all.
timeout "$MFSK_KT_KOTLINC_TIMEOUT" \
    kotlinc -nowarn -d "$OUT/classes" "$HERE/src/Mfsk.kt" "$HERE/test/MfskTest.kt" \
    || { echo "::error::kotlinc timed out or failed"; exit 1; }

stage "cc (JNI shim)"
# Built against the committed header, so this is another compiler
# reading `mfsk.h` as a real translation unit — the check that found
# MFSK_DECODE_FLAG_HASH_RESOLVED missing from it.
timeout "$MFSK_KT_CC_TIMEOUT" \
    cc -shared -fPIC -O2 -Wall -Wextra -Werror \
    -o "$OUT/libmfsk_jni.so" "$HERE/native/mfsk_jni.c" \
    -I"$JAVA_HOME/include" -I"$JAVA_HOME/include/linux" \
    -I"$ROOT/mfsk-ffi/include" \
    -L"$ROOT/target/release" -lmfsk \
    -Wl,-rpath,"$ROOT/target/release" \
    || { echo "::error::cc timed out or failed"; exit 1; }

stage "java (JVM test)"
timeout "$MFSK_KT_JAVA_TIMEOUT" \
    java -Djava.library.path="$OUT" \
         -cp "$OUT/classes:$STDLIB" \
         io.github.mfskcore.MfskTestKt \
    || { echo "::error::the JVM test timed out or failed"; exit 1; }
