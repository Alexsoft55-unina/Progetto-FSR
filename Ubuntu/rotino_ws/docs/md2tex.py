"""Minimal Markdown -> LaTeX converter for the documents of docs/ (pandoc is not installed here).

Handles what these documents use: headings, paragraphs, **bold**, *italic*, `code`, links, $inline$ and
$$display$$ math, bullet and numbered lists, tables, fenced code blocks, horizontal rules, images.
The preamble is the one pandoc produced for the first version of Tecniche_di_controllo.tex.

    python3 docs/md2tex.py docs/Tecniche_di_controllo.md      # -> docs/Tecniche_di_controllo.tex
"""

import os
import re
import sys

PREAMBLE_FROM = 'Tecniche_di_controllo.tex'     # the preamble (up to \begin{document}) is kept from here
SPECIAL = {'\\': r'\textbackslash{}', '{': r'\{', '}': r'\}', '_': r'\_', '%': r'\%', '&': r'\&', '#': r'\#',
           '~': r'\textasciitilde{}', '^': r'\^{}', '$': r'\$'}


# characters the documents use that pdflatex (utf8, T1) does not know
UNICODE_DEFS = {
    0x2212: r'\ensuremath{-}', 0x2192: r'\ensuremath{\rightarrow}', 0x2264: r'\ensuremath{\le}',
    0x2265: r'\ensuremath{\ge}', 0x2248: r'\ensuremath{\approx}', 0x207B: r'\textsuperscript{-}',
    0x03C4: r'\ensuremath{\tau}', 0x03B8: r'\ensuremath{\theta}', 0x03C6: r'\ensuremath{\varphi}',
    0x03BE: r'\ensuremath{\xi}', 0x03C1: r'\ensuremath{\rho}', 0x03C9: r'\ensuremath{\omega}',
    0x03C8: r'\ensuremath{\psi}', 0x0394: r'\ensuremath{\Delta}', 0x03B1: r'\ensuremath{\alpha}',
    0x03BB: r'\ensuremath{\lambda}', 0x03BC: r'\ensuremath{\mu}', 0x03C3: r'\ensuremath{\sigma}',
    0x1E61: r'\ensuremath{\dot s}', 0x010B: r'\ensuremath{\dot c}',
    0x2500: '-', 0x2502: '|', 0x250C: '+', 0x2510: '+', 0x2514: '+', 0x2518: '+', 0x251C: '+', 0x2524: '+',
    0x252C: '+', 0x2534: '+', 0x253C: '+', 0x25BA: '>', 0x25C4: '<', 0x25BC: 'v', 0x25B2: '^',
}
UNICODE_PREAMBLE = '\n'.join('\\DeclareUnicodeCharacter{%04X}{%s}' % (k, v) for k, v in UNICODE_DEFS.items())


def combining(text, in_math=False):
    """Letter + combining dot / diaeresis -> \\dot / \\ddot (plain text) or the bare letter (verbatim)."""
    if in_math is None:
        return text.replace('\u0307', '').replace('\u0308', '')
    text = re.sub('(.)\u0307', lambda m: r'\(\dot{' + _math_letter(m.group(1)) + r'}\)', text)
    return re.sub('(.)\u0308', lambda m: r'\(\ddot{' + _math_letter(m.group(1)) + r'}\)', text)


def _math_letter(ch):
    v = UNICODE_DEFS.get(ord(ch))
    return v[len(r'\ensuremath{'):-1] if v and v.startswith(r'\ensuremath') else ch


def escape(text):
    return ''.join(SPECIAL.get(c, c) for c in text)


def slug(text):
    s = re.sub(r'[^\w\s-]', '', text.lower(), flags=re.UNICODE)
    return re.sub(r'\s+', '-', s.strip())


def inline(text):
    """Inline markup outside math; math ($...$) is copied verbatim."""
    out = []
    for i, part in enumerate(re.split(r'(\$[^$]+\$)', text)):
        if i % 2:
            out.append(r'\(' + part[1:-1] + r'\)')
            continue
        tokens = re.split(r'(`[^`]+`|!\[[^\]]*\]\([^)]+\)|\[[^\]]+\]\([^)]+\))', part)
        for j, tok in enumerate(tokens):
            if j % 2 == 0:
                t = combining(escape(tok))
                t = re.sub(r'\*\*(.+?)\*\*', r'\\textbf{\1}', t)
                t = re.sub(r'(?<![\w*])\*(?!\s)(.+?)(?<!\s)\*(?![\w*])', r'\\emph{\1}', t)
                out.append(t)
            elif tok.startswith('`'):
                out.append(r'\texttt{' + escape(tok[1:-1]) + '}')
            elif tok.startswith('!'):
                path = re.match(r'!\[[^\]]*\]\(([^)]+)\)', tok).group(1)
                out.append(r'\includegraphics[width=\linewidth]{' + path + '}')
            else:
                label, url = re.match(r'\[([^\]]+)\]\(([^)]+)\)', tok).groups()
                out.append(inline(label) + r' (\texttt{' + escape(url) + '})')
    return ''.join(out)


def table(rows):
    cells = [[c.strip() for c in r.strip().strip('|').split('|')] for r in rows if not re.match(r'^\|[\s:|-]+\|$', r.strip())]
    n = max(len(r) for r in cells)
    col = r'>{\raggedright\arraybackslash}p{(\linewidth - %d\tabcolsep) * \real{%.4f}}' % (2 * n, 1.0 / n)
    lines = [r'\begin{longtable}[]{@{}' + ''.join(col for _ in range(n)) + '@{}}', r'\toprule\noalign{}']
    head, body = cells[0], cells[1:]
    lines.append(' & '.join(inline(c) for c in head) + r' \\')
    lines += [r'\midrule\noalign{}', r'\endhead']
    for r in body:
        r = r + [''] * (n - len(r))
        lines.append(' & '.join(inline(c) for c in r) + r' \\')
    lines += [r'\bottomrule\noalign{}', r'\end{longtable}']
    return '\n'.join(lines)


def convert(md):
    lines = md.split('\n')
    out, i = [], 0
    levels = {1: 'section', 2: 'subsection', 3: 'subsubsection', 4: 'paragraph'}
    while i < len(lines):
        ln = lines[i]
        if ln.startswith('```'):
            j = i + 1
            while j < len(lines) and not lines[j].startswith('```'):
                j += 1
            out.append('\\begin{verbatim}\n' + combining('\n'.join(lines[i + 1:j]), None) + '\n\\end{verbatim}')
            i = j + 1
            continue
        m = re.match(r'^(#{1,4}) (.*)$', ln)
        if m:
            title = m.group(2)
            out.append('\\%s{%s}\\label{%s}' % (levels[len(m.group(1))], inline(title), slug(title)))
            i += 1
            continue
        if ln.strip() == '---':
            out.append(r'\begin{center}\rule{0.5\linewidth}{0.5pt}\end{center}')
            i += 1
            continue
        if ln.strip().startswith('$$'):
            block = ln.strip()
            while not (block.endswith('$$') and len(block) > 2) and i + 1 < len(lines):
                i += 1
                block += '\n' + lines[i].strip()
            out.append(r'\[' + block[2:-2].strip() + r'\]')
            i += 1
            continue
        if ln.strip().startswith('|'):
            rows = []
            while i < len(lines) and lines[i].strip().startswith('|'):
                rows.append(lines[i])
                i += 1
            out.append(table(rows))
            continue
        m = re.match(r'^(\s*)([-*]|\d+\.) (.*)$', ln)
        if m:
            env = 'enumerate' if m.group(2)[0].isdigit() else 'itemize'
            items = []
            while i < len(lines):
                m = re.match(r'^(\s*)([-*]|\d+\.) (.*)$', lines[i])
                if m:
                    items.append(m.group(3))
                elif lines[i].startswith('  ') and lines[i].strip() and items:
                    items[-1] += ' ' + lines[i].strip()
                else:
                    break
                i += 1
            out.append('\\begin{%s}\n\\tightlist\n' % env
                       + '\n'.join(r'\item ' + _para(it) for it in items) + '\n\\end{%s}' % env)
            continue
        if not ln.strip():
            i += 1
            continue
        para = [ln]
        i += 1
        while i < len(lines) and lines[i].strip() and not re.match(r'^(#{1,4} |```|\||\s*([-*]|\d+\.) |---\s*$)', lines[i]):
            para.append(lines[i])
            i += 1
        out.append(_para(' '.join(p.strip() for p in para)))
    return '\n\n'.join(out)


def _para(text):
    """A paragraph may hold display math $$...$$ in the middle."""
    parts = re.split(r'(\$\$.+?\$\$)', text)
    return ''.join(r'\[' + p[2:-2].strip() + r'\]' if p.startswith('$$') else inline(p) for p in parts)


def main(path):
    here = os.path.dirname(os.path.abspath(path))
    with open(os.path.join(here, PREAMBLE_FROM)) as f:
        src = f.read()
    preamble = src[:src.index('\\begin{document}')]
    preamble = re.sub(r'% md2tex unicode.*?% end md2tex unicode\n', '', preamble, flags=re.S)
    preamble += '% md2tex unicode\n' + UNICODE_PREAMBLE + '\n% end md2tex unicode\n'
    if '\\usepackage{graphicx}' not in preamble:
        preamble = preamble.replace('\\usepackage{bookmark}', '\\usepackage{graphicx}\n\\usepackage{bookmark}')
    with open(path) as f:
        body = convert(f.read())
    target = os.path.splitext(path)[0] + '.tex'
    with open(target, 'w') as f:
        f.write(preamble + '\\begin{document}\n\n' + body + '\n\n\\end{document}\n')
    print(target)


if __name__ == '__main__':
    main(sys.argv[1])
