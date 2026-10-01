#!/bin/bash
# Compiles the LaTeX documents of docs/ into PDFs next to the sources.
# If babel has no Italian (package texlive-lang-italian missing), it compiles a temporary copy with English babel
# and the Italian names redefined; the .tex files are never modified.
#     bash docs/compila_pdf.sh [file.tex ...]        (default: every .tex in docs/)
set -e
cd "$(dirname "$0")"
FILES=("$@")
[ ${#FILES[@]} -eq 0 ] && FILES=(*.tex)
ITALIAN=$(kpsewhich italian.ldf || true)
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
for f in "${FILES[@]}"; do
    base="${f%.tex}"
    src="$f"
    if [ -z "$ITALIAN" ] && grep -q '\[italian\]{babel}' "$f"; then
        sed -e 's/\\usepackage\[italian\]{babel}/\\usepackage[english]{babel}\n\\addto\\captionsenglish{\\renewcommand{\\contentsname}{Indice}\\renewcommand{\\figurename}{Figura}\\renewcommand{\\tablename}{Tabella}\\renewcommand{\\chaptername}{Capitolo}\\renewcommand{\\abstractname}{Sommario}\\renewcommand{\\bibname}{Bibliografia}}/' \
            "$f" > "$TMP/$f"
        src="$TMP/$f"
        echo "$f: babel italiano non installato, compilo con i nomi italiani ridefiniti"
    else
        cp "$f" "$TMP/$f"
        src="$TMP/$f"
    fi
    cp -r figure_* "$TMP/" 2>/dev/null || true
    (cd "$TMP" && for i in 1 2; do pdflatex -interaction=nonstopmode -halt-on-error "$f" > "$base.stdout" 2>&1 || { grep -m3 -A4 '^!' "$base.log"; exit 1; }; done)
    cp "$TMP/$base.pdf" "$base.pdf"
    echo "$base.pdf: $(grep -o 'Output written.*' "$TMP/$base.log")"
done
