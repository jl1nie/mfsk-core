import { readFileSync, writeFileSync } from "node:fs";
import { resolve } from "node:path";

const wsjtxRoot = process.argv[2];
const output = process.argv[3];
if (!wsjtxRoot || !output) {
  throw new Error(
    "usage: node tools/generate_ldpc240_74.mjs <wsjtx-root> <output.rs>",
  );
}

const sourceDir = resolve(wsjtxRoot, "lib", "fst4");
const parity = readFileSync(
  resolve(sourceDir, "ldpc_240_74_parity.f90"),
  "utf8",
);
const generator = readFileSync(
  resolve(sourceDir, "ldpc_240_74_generator.f90"),
  "utf8",
);

function dataValues(name) {
  const match = parity.match(
    new RegExp(`data\\s+${name}\\s*\\/([\\s\\S]*?)\\/`, "i"),
  );
  if (!match) {
    throw new Error(`missing Fortran data block ${name}`);
  }
  return [...match[1].matchAll(/-?\d+/g)].map((item) => Number(item[0]));
}

function rows(values, width, count, convert) {
  if (values.length !== width * count) {
    throw new Error(
      `expected ${count}x${width} values, received ${values.length}`,
    );
  }
  return Array.from({ length: count }, (_, row) =>
    values
      .slice(row * width, (row + 1) * width)
      .map(convert),
  );
}

const mn = rows(dataValues("Mn"), 3, 240, (value) => value - 1);
const nm = rows(
  dataValues("Nm"),
  5,
  166,
  (value) => (value === 0 ? 0 : value - 1),
);
const nrw = dataValues("nrw");
if (nrw.length !== 166) {
  throw new Error(`expected 166 row weights, received ${nrw.length}`);
}

const hexRows = [...generator.matchAll(/"([0-9a-f]{19})"/gi)].map(
  (match) => match[1],
);
if (hexRows.length !== 166) {
  throw new Error(`expected 166 generator rows, received ${hexRows.length}`);
}
const genParity = hexRows.map((hex) => {
  const bits = [];
  for (let index = 0; index < hex.length; index += 1) {
    const nibble = Number.parseInt(hex[index], 16);
    const width = index === hex.length - 1 ? 2 : 4;
    for (let bit = 0; bit < width; bit += 1) {
      bits.push((nibble >> (3 - bit)) & 1);
    }
  }
  if (bits.length !== 74) {
    throw new Error(`generator row has ${bits.length} bits`);
  }
  return bits;
});

function rustArray(name, type, values) {
  const body = values
    .map((row) =>
      Array.isArray(row)
        ? `    [${row.join(", ")}],`
        : `    ${row},`,
    )
    .join("\n");
  return `pub const ${name}: ${type} = [\n${body}\n];\n`;
}

const rust = `//! Generated from pinned WSJT-X v3.0.2:
//! \`lib/fst4/ldpc_240_74_parity.f90\` and
//! \`lib/fst4/ldpc_240_74_generator.f90\`.
//! Fortran indices are converted from 1-based to 0-based.

${rustArray("MN", "[[u8; 3]; 240]", mn)}
${rustArray("NM", "[[u8; 5]; 166]", nm)}
${rustArray("NRW", "[u8; 166]", nrw)}
${rustArray("GEN_PARITY", "[[u8; 74]; 166]", genParity)}
`;

writeFileSync(resolve(output), rust, "utf8");
