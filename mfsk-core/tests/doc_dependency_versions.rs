//! The `Cargo.toml` snippets the user documentation shows name the crate's
//! own `major.minor` (#708). Doctests compile the Rust fences, never the
//! TOML ones, so a snippet still saying `0.12` after two releases went
//! unnoticed; this reads them.
//!
//! Scope: the crate rustdoc (`src/lib.rs`), `README.md` and every
//! `docs/reference/*.md`. `CHANGELOG.md` and `docs/historical/` quote old
//! versions on purpose and are not read.

use std::fs;
use std::path::{Path, PathBuf};

const ROOT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/..");

fn documents() -> Vec<PathBuf> {
    let root = Path::new(ROOT);
    let mut v = vec![root.join("mfsk-core/src/lib.rs"), root.join("README.md")];
    let mut reference: Vec<PathBuf> = fs::read_dir(root.join("docs/reference"))
        .expect("docs/reference")
        .map(|e| e.expect("entry").path())
        .filter(|p| p.extension().is_some_and(|x| x == "md"))
        .collect();
    reference.sort();
    v.extend(reference);
    v
}

/// The version a line's `mfsk-core = "X"` or `mfsk-core = { version = "X", … }`
/// names, if it is one (a `git =` or `path =` dependency names none).
fn named_version(line: &str) -> Option<&str> {
    let rest = line.split_once("mfsk-core = ")?.1.trim_start();
    let rest = if let Some(table) = rest.strip_prefix('{') {
        table.split_once("version = ")?.1
    } else {
        rest
    };
    rest.strip_prefix('"')?.split('"').next()
}

#[test]
fn the_parser_reads_both_forms() {
    assert_eq!(named_version(r#"mfsk-core = "0.14""#), Some("0.14"));
    assert_eq!(
        named_version(r#"//! mfsk-core = { version = "0.12", features = ["ft8"] }"#),
        Some("0.12")
    );
    assert_eq!(
        named_version(
            r#"mfsk-core = { git = "https://github.com/jl1nie/mfsk-core", branch = "main" }"#
        ),
        None
    );
    assert_eq!(
        named_version(r#"mfsk-core = { path = "../mfsk-core" }"#),
        None
    );
}

#[test]
fn documented_dependency_versions_are_the_crates() {
    let pkg = env!("CARGO_PKG_VERSION");
    let mut parts = pkg.split('.');
    let want = format!("{}.{}", parts.next().unwrap(), parts.next().unwrap());

    let mut seen = 0;
    let mut stale = Vec::new();
    for doc in documents() {
        let text = fs::read_to_string(&doc).unwrap_or_else(|e| panic!("{}: {e}", doc.display()));
        for (i, line) in text.lines().enumerate() {
            if let Some(v) = named_version(line) {
                seen += 1;
                if v != want && !v.starts_with(&format!("{want}.")) {
                    stale.push(format!("{}:{}: {v}", doc.display(), i + 1));
                }
            }
        }
    }
    assert!(
        seen > 0,
        "no dependency snippet found: the scan reads nothing"
    );
    assert!(
        stale.is_empty(),
        "snippets naming a version other than {want} ({pkg}):\n{}",
        stale.join("\n")
    );
}
