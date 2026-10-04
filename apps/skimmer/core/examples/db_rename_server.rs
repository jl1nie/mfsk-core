// SPDX-License-Identifier: GPL-3.0-or-later
//! Record what one server name heard under another: `db_rename_server DB FROM TO`
//! (FROM may be `<none>` for the decodes made before servers had names). A compact copy of
//! the file is made first, beside it, and the recording skimmer may keep running.

use std::path::PathBuf;

use skimmer_core::store;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let [db, from, to] = a.as_slice() else {
        eprintln!(
            "usage: db_rename_server DB FROM TO   (FROM <none> for the decodes with no server name)"
        );
        std::process::exit(2);
    };
    let db = PathBuf::from(db);
    let from = if from == "<none>" { "" } else { from.as_str() };
    let before = store::info(&db).unwrap_or_else(|e| fail(&e));
    println!("{} decodes; servers: {:?}", before.decodes, before.servers);
    let copy = PathBuf::from(format!("{}.before-rename", db.display()));
    if copy.exists() {
        fail(&format!(
            "{} exists already; move it away first",
            copy.display()
        ));
    }
    store::backup(&db, &copy).unwrap_or_else(|e| fail(&e));
    println!("copy: {}", copy.display());
    let n = store::rename_server(&db, from, to).unwrap_or_else(|e| fail(&e));
    let after = store::info(&db).unwrap_or_else(|e| fail(&e));
    println!(
        "renamed {n} decodes; now {} decodes; servers: {:?}",
        after.decodes, after.servers
    );
}

fn fail(e: &str) -> ! {
    eprintln!("{e}");
    std::process::exit(1);
}
