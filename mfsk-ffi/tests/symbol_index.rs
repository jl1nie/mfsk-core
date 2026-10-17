//! `BINDINGS.md` §2.12 (and its `.ja.md` twin) lists every function the header
//! exports, grouped, with a count per group and in total. It drifted once (it said
//! 100 while the header had 124, 2026-10-17); this keeps it equal to the header.

use std::collections::BTreeSet;

fn exported() -> BTreeSet<String> {
    let header = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/include/mfsk.h"))
        .expect("include/mfsk.h is committed");
    // Drop comments, then take every `mfsk_name(`.
    let mut code = String::new();
    let mut rest = header.as_str();
    while let Some(i) = rest.find("/*") {
        code.push_str(&rest[..i]);
        rest = rest[i..].split_once("*/").map_or("", |(_, r)| r);
    }
    code.push_str(rest);
    names(&code, "(")
}

/// Every `mfsk_[a-z0-9_]+` in `text` that is followed by `end`.
fn names(text: &str, end: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    let mut s = text;
    while let Some(i) = s.find("mfsk_") {
        let tail = &s[i..];
        let n = tail
            .find(|c: char| !(c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_'))
            .unwrap_or(tail.len());
        if tail[n..].trim_start().starts_with(end) {
            out.insert(tail[..n].to_string());
        }
        s = &tail[n.max(1)..];
    }
    out
}

fn check(doc: &str, total_marker: impl Fn(usize) -> String) {
    let path = format!("{}/../docs/reference/{doc}", env!("CARGO_MANIFEST_DIR"));
    let text = std::fs::read_to_string(&path).expect(&path);
    let start = text.find("### 2.12").expect("section 2.12");
    let end = start + text[start..].find("\n## 3.").expect("section 3 after 2.12");
    let sec = &text[start..end];
    let listed = names(sec, "`");
    let header = exported();
    assert_eq!(
        listed.symmetric_difference(&header).collect::<Vec<_>>(),
        Vec::<&String>::new(),
        "{doc} §2.12 and include/mfsk.h differ"
    );
    let mut sum = 0;
    for row in sec
        .lines()
        .filter(|l| l.starts_with("| ") && l.contains(") | "))
    {
        let (label, syms) = row[2..].split_once(" | ").unwrap();
        let count: usize = label[label.rfind('(').unwrap() + 1..label.rfind(')').unwrap()]
            .parse()
            .unwrap();
        assert_eq!(count, syms.matches("`mfsk_").count(), "{doc}: {label}");
        sum += count;
    }
    assert_eq!(sum, header.len(), "{doc}: group counts");
    assert!(sec.contains(&total_marker(header.len())), "{doc}: total");
}

#[test]
fn symbol_index_equals_the_header() {
    check("BINDINGS.md", |n| format!("{n} exported functions"));
    check("BINDINGS.ja.md", |n| {
        format!("エクスポートされる関数は {n} 個")
    });
}
