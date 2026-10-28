//! An application can build the rows and results a decoder returns, to test
//! the code that consumes them, without decoding audio (#712).
//!
//! `Row` and `SlotResult` are `#[non_exhaustive]`, so outside the crate a
//! struct expression is closed to them; this file is outside the crate and
//! uses only the public API (no `internal-testing` path).

use mfsk_core::ProtocolId;
use mfsk_core::decoder::{BudgetReport, Row, RowDetail, SlotResult};
use mfsk_core::prelude::Decoded;

/// What a display or logger under test would do with a period's result.
fn lines(r: &SlotResult<()>) -> Vec<String> {
    r.rows
        .iter()
        .map(|row| format!("{:>4.0} {}", row.decoded.freq_hz, row.decoded.text))
        .collect()
}

fn decoded(text: &str, freq_hz: f32) -> Decoded {
    Decoded::new(text, freq_hz, 0.1, -12.0, ProtocolId::Ft8)
}

#[test]
fn an_untyped_row_for_a_display_test() {
    let row = Row::new(decoded("CQ K1ABC FN42", 1500.0), ());
    assert_eq!(row.decoded.text, "CQ K1ABC FN42");
    assert_eq!(row.detail, RowDetail::default());
}

#[test]
fn a_typed_row_carries_the_callers_native_data() {
    #[derive(Debug, PartialEq)]
    struct Native {
        candidate: u32,
    }
    let mut row = Row::new(decoded("K1ABC W9XYZ EN37", 800.0), Native { candidate: 7 });
    let mut detail = RowDetail::default();
    detail.pass = 2;
    detail.hash_resolved = true;
    detail.delivery = Some(0);
    // Assigning the detail from outside the crate is what is checked: it compiles.
    row.detail = detail;
    assert_eq!(row.native, Native { candidate: 7 });
}

#[test]
fn an_empty_result_has_no_budget_and_no_search() {
    let r = SlotResult::<()>::new(Vec::new());
    assert!(r.rows.is_empty());
    assert_eq!(r.budget, BudgetReport::default());
    assert_eq!(r.candidates, None);
    assert!(lines(&r).is_empty());
}

#[test]
fn a_result_with_rows_and_explicit_metadata() {
    let mut r = SlotResult::new(vec![
        Row::new(decoded("CQ K1ABC FN42", 1500.0), ()),
        Row::new(decoded("CQ K1ABC FN42", 2100.0), ()),
    ]);
    let mut budget = BudgetReport::default();
    budget.exhausted = true;
    budget.candidates_skipped = 3;
    r.budget = budget;
    // Assigning the metadata from outside the crate is what is checked: it compiles.
    r.candidates = Some(42);
    assert_eq!(
        lines(&r),
        ["1500 CQ K1ABC FN42", "2100 CQ K1ABC FN42"],
        "one message at two frequencies is two rows"
    );
}
