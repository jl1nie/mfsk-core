#!/usr/bin/env python3
"""ハムフェア2026 資料のテキストとレイアウトを行き来させるビルドスクリプト.

    python3 build.py stamp     # レイアウト HTML に data-t 目印を振る（初回のみ）
    python3 build.py extract   # HTML → content.md（現在の文言を吸い出す）
    python3 build.py build     # content.md → HTML → PDF（+ はみ出し検査）

content.md が文言の正本. build は HTML の文字列を content.md で上書きする.
"""
import re, sys, subprocess, shutil
from pathlib import Path
from bs4 import BeautifulSoup, NavigableString, Tag

HERE = Path(__file__).resolve().parent
CONTENT = HERE / "content.md"
CHROME_CANDIDATES = [
    "/opt/pw-browsers/chromium-1194/chrome-linux/chrome",
    shutil.which("chromium"), shutil.which("chromium-browser"),
    shutil.which("google-chrome"), shutil.which("chrome"),
]

DOCS = [
    {"file": "flipchart.html", "prefix": "F", "title": "フリップ資料（A4横10ページ・ブースでめくる用）"},
    {"file": "leaflet.html",   "prefix": "L", "title": "配布リーフレット（A4縦両面1枚・持ち帰り用）"},
]

BLOCK = {"div","p","ul","ol","table","section","header","footer","tr","tbody","thead",
         "h1","h2","h3","h4","h5","h6","li","td","th"}
SKIP_CLASS = {"pnum", "a"}          # ページ番号・矢印記号は文言ではない
PAGENO = re.compile(r"^\d+\s*/\s*\d+$")

LABELS = {  # class / tag → MD に出す日本語ラベル
    "kicker":"キッカー", "h1sub":"URL行", "tagline":"キャッチ", "modes":"モード一覧",
    "sv":"要点の見出し", "sl":"要点の説明", "whoami":"署名", "qcap":"QRの説明",
    "sub":"右肩の補足", "bt":"図・主文", "bs":"図・副文", "al":"図・矢印ラベル",
    "nopc":"図の注記", "mn":"モード名", "md":"モードの説明",
    "kv":"数値", "kl":"数値の説明", "note":"注記", "takeaway":"まとめ帯",
    "punch":"強調ボックス", "lead":"リード文", "kick":"キッカー", "tl":"キャッチ",
    "url":"URL行", "t":"CTA・見出し", "u":"CTA・URL", "d":"CTA・補足", "s":"右肩の補足",
}
TAG_LABELS = {"h1":"大見出し","h2":"見出し","h3":"小見出し","table":"表","ul":"箇条書き",
              "p":"本文","span":"テキスト","div":"テキスト","footer":"フッタ"}


# ---------------------------------------------------------------- inline 変換
def inline_to_md(node, br):
    if isinstance(node, NavigableString):
        # ノーブレークスペースは詰め幅の調整に使っているので、実体参照として残す
        text = str(node).replace("\u00a0", "\x01")
        return re.sub(r"\s+", " ", text).replace("\x01", "&nbsp;")
    if node.name == "br":
        return br
    if node.name in ("svg",):
        return ""
    inner = "".join(inline_to_md(c, br) for c in node.children)
    cls = " ".join(node.get("class", []))
    if node.name in ("b", "strong"):              return f"**{inner}**"
    if node.name == "code":                       return f"`{inner}`"
    if node.name == "em":                         return f"++{inner}++"
    if node.name == "small":                      return f"^{inner}^"
    if node.name == "span" and cls == "hl":       return f"=={inner}=="
    attrs = "".join(f' {k}="{" ".join(v) if isinstance(v, list) else v}"'
                    for k, v in node.attrs.items())
    return f"<{node.name}{attrs}>{inner}</{node.name}>"


def md_to_html(text, br):
    out = text.replace("\n", "<br>") if br == "\n" else text
    # 等幅の中身は先に退避する。`lib/*.f90` のような文字列が他の記法に巻き込まれるため
    spans = []
    def stash(m):
        spans.append(m.group(1))
        return f"\x00{len(spans) - 1}\x00"
    out = re.sub(r"`(.+?)`", stash, out)
    out = re.sub(r"==(.+?)==", r'<span class="hl">\1</span>', out)
    out = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", out)
    out = re.sub(r"\+\+(.+?)\+\+", r"<em>\1</em>", out)
    out = re.sub(r"\^(.+?)\^", r"<small>\1</small>", out)
    return re.sub(r"\x00(\d+)\x00", lambda m: f"<code>{spans[int(m.group(1))]}</code>", out)


def set_inner(el, html):
    el.clear()
    # list() が要る: append は元の親から外すので、生のリストを辿ると1つ飛ばしになる
    for child in list(BeautifulSoup(html, "html.parser").contents):
        el.append(child)


# ------------------------------------------------------------- 走査（共通ルール）
def is_leaf(el):
    if el.name in ("table", "ul", "ol"):
        return True
    if any(isinstance(c, Tag) and c.name in BLOCK for c in el.children):
        return False
    return bool(el.get_text(strip=True))


def walk(el, out):
    """文言の入る末端要素を文書順に集める."""
    for child in el.children:
        if not isinstance(child, Tag) or child.name == "svg":
            continue
        cls = set(child.get("class", []))
        if cls & SKIP_CLASS:
            continue
        if is_leaf(child):
            if child.name == "span" and PAGENO.match(child.get_text(strip=True)):
                continue
            out.append(child)
        else:
            walk(child, out)


def label_for(el):
    for c in el.get("class", []):
        if c in LABELS:
            return LABELS[c]
    return TAG_LABELS.get(el.name, el.name)


def soup_of(path):
    return BeautifulSoup(path.read_text(encoding="utf-8"), "html.parser")


def write_soup(soup, path):
    path.write_text(soup.decode(formatter="html5"), encoding="utf-8")


# ------------------------------------------------------------------- 各サブコマンド
def cmd_stamp():
    for doc in DOCS:
        path = HERE / doc["file"]
        soup = soup_of(path)
        for pi, page in enumerate(soup.select(".page"), 1):
            fields = []
            walk(page, fields)
            for fi, el in enumerate(fields, 1):
                el["data-t"] = f"{doc['prefix']}{pi:02d}-{fi:02d}"
        write_soup(soup, path)
        print(f"stamp: {doc['file']} — {len(soup.select('[data-t]'))} 箇所")


def table_to_md(el):
    rows = []
    for tr in el.select("tr"):
        cells = tr.find_all(["th", "td"])
        rows.append(["".join(inline_to_md(x, "<br>") for x in c.children).strip()
                     for c in cells])
    if not rows:
        return ""
    width = max(len(r) for r in rows)
    rows = [r + [""] * (width - len(r)) for r in rows]
    has_head = bool(el.find("thead"))
    lines = []
    if has_head:
        lines.append("| " + " | ".join(rows[0]) + " |")
        lines.append("|" + "---|" * width)
        body = rows[1:]
    else:
        lines.append("| " + " | ".join([""] * width) + " |")
        lines.append("|" + "---|" * width)
        body = rows
    lines += ["| " + " | ".join(r) + " |" for r in body]
    return "\n".join(lines)


def list_to_md(el):
    return "\n".join("- " + "".join(inline_to_md(x, "<br>") for x in li.children).strip()
                     for li in el.find_all("li", recursive=False))


def field_to_md(el):
    if el.name == "table":
        return table_to_md(el)
    if el.name in ("ul", "ol"):
        return list_to_md(el)
    md = "".join(inline_to_md(c, "\n") for c in el.children)
    return re.sub(r" *\n *", "\n", md).strip()


def cmd_extract():
    out = ["# mfsk-core — ハムフェア2026 資料テキスト", "",
           "このファイルが**文言の正本**です。ここを直して `python3 build.py build` を走らせると、",
           "HTML と PDF の両方が作り直されます。レイアウト（判型・段組・色・表の列幅）は",
           "HTML 側が持っているので、このファイルでは文言だけを扱います。", "",
           "## 編集のしかた", "",
           "- `### F02-05 · 本文` のような見出しの**次の行から次の見出しまで**が、その箇所の文言です。",
           "  見出しの行（ID とラベル）は**変えないでください** — 差し込み先を見失います。",
           "- 装飾は次の記法で書けます。",
           "  `**太字**` / `` `等幅` `` / `==マーカー==` / `++アクセント色++` / `^小さめ^`（数値の単位）",
           "- **改行したいところで改行**してください。そのまま改行として出ます。",
           "  表と箇条書きの中だけは、行が壊れるので改行に `<br>` を使ってください。",
           "- 表は Markdown の表のまま編集できます。行の増減も可能です（列は増やせません）。",
           "- 箇条書きは `- ` で始まる行です。項目の増減も可能です。",
           "- ここに書いていない HTML タグはそのまま素通しします。分からないものは触らないのが安全です。", "",
           "## 直したあと", "",
           "- 文言を増やすとページからはみ出すことがあります。`build` が各ページの超過量を報告するので、",
           "  `over` が 0 でない行が出たら、その分だけ削るか、HTML 側の `class=\"page dense\"` で寸法を落とします。",
           "- 数字を直すときは `README.md` の出典表も一緒に確認してください。", ""]

    for doc in DOCS:
        soup = soup_of(HERE / doc["file"])
        out += ["---", "", f"# {doc['title']}", ""]
        for page in soup.select(".page"):
            head = page.select_one(".ph h2, .mast h1, .backhead h1")
            pid = page.select_one("[data-t]")["data-t"].split("-")[0]
            name = head.get_text(strip=True) if head else "表紙"
            out += [f"## {pid} — {name}", ""]
            fields = []
            walk(page, fields)
            for el in fields:
                out += [f"### {el['data-t']} · {label_for(el)}", "", field_to_md(el), ""]
    CONTENT.write_text("\n".join(out) + "\n", encoding="utf-8")
    print(f"extract: {CONTENT.name} — {len(re.findall(r'^### ', CONTENT.read_text(), re.M))} 箇所")


def parse_content():
    fields, key = {}, None
    for line in CONTENT.read_text(encoding="utf-8").splitlines():
        m = re.match(r"^### ([A-Z]\d+-\d+)(?: · .*)?$", line)
        if m:
            key = m.group(1); fields[key] = []
        elif key is not None:
            if line.startswith(("## ", "# ", "---")):
                key = None
            else:
                fields[key].append(line)
    return {k: "\n".join(v).strip() for k, v in fields.items()}


def apply_table(el, md):
    rows = [[c.strip() for c in ln.strip().strip("|").split("|")]
            for ln in md.splitlines() if ln.strip().startswith("|")]
    rows = [r for r in rows if not all(set(c) <= set("-: ") for c in r)]
    if not rows:
        return
    head_cells = el.select("thead th")
    body = rows
    if head_cells:
        for th, txt in zip(head_cells, rows[0]):
            set_inner(th, md_to_html(txt, "<br>"))
        body = rows[1:]
    tbody = el.find("tbody") or el
    proto = tbody.find("tr")
    proto_cells = proto.find_all(["td", "th"]) if proto else []
    soup = BeautifulSoup("", "html.parser")
    new_rows = []
    for r in body:
        tr = soup.new_tag("tr")
        for i, txt in enumerate(r):
            src = proto_cells[i] if i < len(proto_cells) else None
            td = soup.new_tag("td")
            if src is not None and src.get("class"):
                td["class"] = src["class"]
            set_inner(td, md_to_html(txt, "<br>"))
            tr.append(td)
        new_rows.append(tr)
    for tr in tbody.find_all("tr"):
        tr.decompose()
    for tr in new_rows:
        tbody.append(tr)


def apply_list(el, md):
    items = [ln[2:].strip() for ln in md.splitlines() if ln.strip().startswith("- ")]
    proto = el.find("li")
    attrs = dict(proto.attrs) if proto else {}
    soup = BeautifulSoup("", "html.parser")
    new = []
    for txt in items:
        li = soup.new_tag("li")
        for k, v in attrs.items():
            li[k] = v
        set_inner(li, md_to_html(txt, "<br>"))
        new.append(li)
    for li in el.find_all("li"):
        li.decompose()
    for li in new:
        el.append(li)


def chrome():
    for c in CHROME_CANDIDATES:
        if c and Path(c).exists():
            return c
    sys.exit("Chromium が見つかりません（PDF 生成に必要）")


def check_fit(html):
    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        print("  （playwright 未導入のため、はみ出し検査はスキップ）")
        return
    with sync_playwright() as p:
        b = p.chromium.launch(executable_path=chrome(), args=["--no-sandbox"])
        pg = b.new_page(); pg.emulate_media(media="print")
        pg.goto("file://" + str(html))
        rows = pg.evaluate("""()=>[...document.querySelectorAll('.page')].map((page,i)=>{
          const mm = px => Math.round(px / 3.7795);
          // ページ自体の溢れと、中で潰れて欠けた要素の両方を見る。
          // .body は flex なので、溢れがページではなく内側の箱の切り落としとして出ることがある
          const inner = [...page.querySelectorAll('*')]
            .filter(el => el.scrollHeight - el.clientHeight > 6)
            // 本当に切り落とされる箱だけを見る。overflow:visible の要素は
            // 行送りの端数ではみ出して見えるだけで、印刷結果は欠けない
            .filter(el => el.classList.contains('body') ||
                          getComputedStyle(el).overflowY !== 'visible')
            .map(el => (el.className && el.className.baseVal === undefined
                        ? String(el.className).split(' ')[0] : el.tagName.toLowerCase())
                       + ' ' + mm(el.scrollHeight - el.clientHeight) + 'mm');
          return {page: i + 1, over: mm(page.scrollHeight - page.clientHeight), inner};
        })""")
        b.close()
    ok = True
    for r in rows:
        if r["over"] > 0:
            ok = False
            print(f"  ⚠ {html.name} p{r['page']}: ページから {r['over']}mm はみ出しています")
        for item in r["inner"]:
            ok = False
            print(f"  ⚠ {html.name} p{r['page']}: 「{item}」分が箱に収まらず欠けています")
    if ok:
        print(f"  ✓ {html.name}: 全ページ収まっています")


def cmd_build():
    if not CONTENT.exists():
        sys.exit("content.md がありません。先に `python3 build.py extract` を実行してください")
    fields = parse_content()
    for doc in DOCS:
        html = HERE / doc["file"]
        soup = soup_of(html)
        used = 0
        for el in soup.select("[data-t]"):
            md = fields.get(el["data-t"])
            if md is None:
                print(f"  ⚠ {el['data-t']} が content.md にありません（HTML の内容を残します）")
                continue
            if el.name == "table":
                apply_table(el, md)
            elif el.name in ("ul", "ol"):
                apply_list(el, md)
            else:
                set_inner(el, md_to_html(md, "\n"))
            used += 1
        write_soup(soup, html)
        pdf = html.with_suffix(".pdf")
        subprocess.run([chrome(), "--headless", "--disable-gpu", "--no-sandbox",
                        "--no-pdf-header-footer", f"--print-to-pdf={pdf}",
                        f"file://{html}"], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"build: {html.name} ← {used} 箇所 / {pdf.name} を生成")
        check_fit(html)


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "build"
    {"stamp": cmd_stamp, "extract": cmd_extract, "build": cmd_build}[cmd]()
