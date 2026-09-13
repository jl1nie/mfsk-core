#!/usr/bin/env bash
# Build the JNI shim + Kotlin binding and run the JVM test.
#
# Needs: a JDK (JAVA_HOME), kotlinc on PATH, and a C compiler.
# `cargo build -p mfsk-ffi --release` is run for you.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HERE="$ROOT/bindings/kotlin"
OUT="$HERE/build"
mkdir -p "$OUT"

: "${JAVA_HOME:?set JAVA_HOME to a JDK}"
command -v kotlinc >/dev/null || { echo "kotlinc not on PATH"; exit 1; }

echo "==> cargo build -p mfsk-ffi --release"
cargo build --manifest-path "$ROOT/Cargo.toml" -p mfsk-ffi --release

echo "==> kotlinc (binding + test)"
kotlinc "$HERE/src/Mfsk.kt" "$HERE/test/MfskTest.kt" \
    -include-runtime -d "$OUT/mfsk.jar" 2>&1 | grep -v '^warning:' || true

echo "==> cc (JNI shim)"
# Built against the committed header, so this is another compiler
# reading `mfsk.h` as a real translation unit — the check that caught a
# macro-generated function missing from it.
cc -shared -fPIC -O2 -Wall -Wextra -Werror \
    -o "$OUT/libmfsk_jni.so" "$HERE/native/mfsk_jni.c" \
    -I"$JAVA_HOME/include" -I"$JAVA_HOME/include/linux" \
    -I"$ROOT/mfsk-ffi/include" \
    -L"$ROOT/target/release" -lmfsk \
    -Wl,-rpath,"$ROOT/target/release"

echo "==> java (JVM test)"
java -Djava.library.path="$OUT" \
     -cp "$OUT/mfsk.jar" \
     io.github.mfskcore.MfskTestKt
