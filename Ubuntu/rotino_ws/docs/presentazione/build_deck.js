// Genera RoTino_Controllo.pptx (pptxgenjs). Prima: python3 modello_lagrangiano.py && python3 render_eq.py
// Uso: NODE_PATH=<cartella con node_modules> node build_deck.js <cartella della skill pptx>
const fs = require('fs');
const path = require('path');
const pptxgen = require('pptxgenjs');

const SKILL = process.argv[2];
const { applyTheme } = require(path.join(SKILL, 'scripts', 'apply_theme.js'));

const HERE = __dirname;
const WS = path.resolve(HERE, '..', '..');
const OUT = path.join(HERE, 'RoTino_Controllo.pptx');
const EQ = JSON.parse(fs.readFileSync(path.join(HERE, 'eq', 'manifest.json')));
const MOD = JSON.parse(fs.readFileSync(path.join(HERE, 'modello.json')));

const THEME = {
  name: 'RoTino',
  headFontFace: 'Cambria',
  bodyFontFace: 'Calibri',
  colors: {
    dk1: '1B2329', lt1: 'FFFFFF', dk2: '263238', lt2: 'EEF2F4',
    accent1: 'C0392B', accent2: '1F6F78', accent3: 'B9770E', accent4: '4A6274',
    accent5: '7B8A97', accent6: '2E86AB', hlink: '1F6F78', folHlink: '4A6274',
  },
};
const HEX = THEME.colors;

const pres = new pptxgen();
pres.layout = 'LAYOUT_WIDE';            // 13.333 x 7.5 in
pres.theme = { headFontFace: THEME.headFontFace, bodyFontFace: THEME.bodyFontFace };
pres.author = 'Alessando Prisco';
pres.title = 'RoTino: modellazione e controllo';
const C = pres.SchemeColor;
const SW = 13.333;

// sezioni: colore e nome breve per l'etichetta in alto a destra
const SEC = {
  0: { title: 'Apertura', color: C.text2, tag: '' },
  1: { title: '1 · Modellazione', color: C.accent4, tag: '1 · Modellazione' },
  2: { title: '2 · C-space e DoF', color: C.accent3, tag: '2 · C-space e DoF' },
  3: { title: '3 · Controllo PID con ZMP', color: C.accent1, tag: '3 · PID con ZMP' },
  4: { title: '4 · MPC + TV-LQR', color: C.accent2, tag: '4 · MPC + TV-LQR' },
  5: { title: '5 · Confronto', color: C.accent5, tag: '5 · Confronto' },
};

// ------------------------------------------------------------------ layout
pres.defineSlideMaster({
  title: 'TITOLO',
  background: { color: C.text2 },
  objects: [
    { placeholder: { options: { name: 'title', type: 'title', x: 0.8, y: 2.0, w: 7.4, h: 1.9, fontSize: 40, bold: true, align: 'left',
      color: C.background1, valign: 'bottom', margin: 0 }, text: '' } },
    { placeholder: { options: { name: 'body', type: 'body', x: 0.8, y: 4.1, w: 7.4, h: 1.2, fontSize: 18, align: 'left',
      color: C.background2, valign: 'top', margin: 0 }, text: '' } },
  ],
});
pres.defineSlideMaster({
  title: 'SEZIONE',
  background: { color: C.text2 },
  objects: [
    { placeholder: { options: { name: 'title', type: 'title', x: 3.1, y: 2.25, w: 9.4, h: 1.2, fontSize: 40, bold: true, align: 'left',
      color: C.background1, valign: 'bottom', margin: 0 }, text: '' } },
    { placeholder: { options: { name: 'body', type: 'body', x: 3.1, y: 3.6, w: 9.0, h: 1.8, fontSize: 18, align: 'left',
      color: C.background2, valign: 'top', margin: 0 }, text: '' } },
  ],
  slideNumber: { x: 12.3, y: 6.95, w: 0.6, h: 0.3, fontSize: 10, color: C.background2, align: 'right' },
});
pres.defineSlideMaster({
  title: 'CONTENUTO',
  background: { color: C.background1 },
  margin: [0.5, 0.6, 0.6, 0.6],
  objects: [
    { placeholder: { options: { name: 'title', type: 'title', x: 0.6, y: 0.35, w: 9.8, h: 0.8, fontSize: 30, bold: true, align: 'left',
      color: C.text2, valign: 'middle', margin: 0 }, text: '' } },
    { text: { text: 'RoTino · modellazione e controllo', options: { x: 0.6, y: 7.0, w: 6, h: 0.3, fontSize: 10,
      color: C.accent4, margin: 0 } } },
  ],
  slideNumber: { x: 12.1, y: 7.0, w: 0.6, h: 0.3, fontSize: 10, color: C.accent4, align: 'right' },
});

// ------------------------------------------------------------------ helper
let currentSec = 0;
const declared = new Set();
function newSlide(master, sec) {
  if (!declared.has(sec)) {
    pres.addSection({ title: SEC[sec].title });
    declared.add(sec);
  }
  currentSec = sec;
  return pres.addSlide({ masterName: master, sectionTitle: SEC[sec].title });
}

// markup minimo: **grassetto**, _{pedice}, ^{apice}; '\n' va a capo (pptxgenjs, se trova '\n' dentro un run,
// mette breakLine su ogni pezzo e spezza anche dopo il run successivo)
function richLine(text, base) {
  const parts = String(text).split(/(\*\*[^*]+\*\*|_\{[^}]+\}|\^\{[^}]+\})/).filter((p) => p !== '');
  return parts.map((p) => {
    if (p.startsWith('**')) return { text: p.slice(2, -2), options: { ...base, bold: true } };
    if (p.startsWith('_{')) return { text: p.slice(2, -1), options: { ...base, subscript: true } };
    if (p.startsWith('^{')) return { text: p.slice(2, -1), options: { ...base, superscript: true } };
    return { text: p, options: { ...base } };
  });
}
function rich(text, base = {}) {
  const lines = String(text).split('\n');
  const runs = [];
  lines.forEach((l, i) => {
    const r = richLine(l, base);
    if (r.length === 0) r.push({ text: ' ', options: { ...base } });
    if (i < lines.length - 1) r[r.length - 1].options.breakLine = true;
    runs.push(...r);
  });
  return runs;
}

function text(slide, t, x, y, w, h, o = {}) {
  const { fontSize = 15, color = C.text1, bold = false, italic = false, align = 'left', valign = 'top',
    fontFace, margin = 0 } = o;
  const base = { color, fontSize, bold, italic };
  if (fontFace) base.fontFace = fontFace;
  slide.addText(rich(t, base), { x, y, w, h, align, valign, margin, isTextBox: true, fit: 'none' });
}

function bullets(slide, items, x, y, w, h, o = {}) {
  const { fontSize = 15, color = C.text1, gap = 6, bulletColor } = o;
  const runs = [];
  items.forEach((it, i) => {
    const item = typeof it === 'string' ? { t: it } : it;
    const r = rich(item.t, { color, fontSize });
    const bullet = item.plain ? false : { indent: 16, ...(bulletColor ? {} : {}) };
    r[0].options = { ...r[0].options, bullet, paraSpaceAfter: gap, indentLevel: item.level || 0 };
    if (i < items.length - 1) r[r.length - 1].options.breakLine = true;
    runs.push(...r);
  });
  slide.addText(runs, { x, y, w, h, valign: 'top', margin: 0, isTextBox: true, fit: 'none' });
}

function card(slide, x, y, w, h, fill = C.background2, name) {
  slide.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, fill: { color: fill }, line: { color: fill, width: 0 },
    rectRadius: 0.08, objectName: name });
}

function badge(slide, x, y, d, label, fill, fontSize) {
  slide.addShape(pres.shapes.OVAL, { x, y, w: d, h: d, fill: { color: fill }, line: { color: fill, width: 0 } });
  slide.addText(String(label), { x, y, w: d, h: d, align: 'center', valign: 'middle', margin: 0, isTextBox: true,
    color: C.background1, bold: true, fontSize: fontSize || Math.round(d * 22) });
}

// card con numero/etichetta in cerchio, titolo e testo
function infoCard(slide, x, y, w, h, label, title, body, color, o = {}) {
  card(slide, x, y, w, h);
  const d = 0.46;
  badge(slide, x + 0.2, y + 0.2, d, label, color, o.badgeSize || 13);
  text(slide, title, x + 0.8, y + 0.2, w - 1.0, 0.46, { fontSize: o.titleSize || 15, bold: true, color: C.text2,
    valign: 'middle' });
  if (body) {
    if (Array.isArray(body)) bullets(slide, body, x + 0.25, y + 0.8, w - 0.45, h - 0.95, { fontSize: o.size || 13, gap: 4 });
    else text(slide, body, x + 0.25, y + 0.8, w - 0.45, h - 0.95, { fontSize: o.size || 13 });
  }
}

// numero grande con didascalia
function stat(slide, x, y, w, value, caption, color, o = {}) {
  text(slide, value, x, y, w, 0.75, { fontSize: o.size || 34, bold: true, color, fontFace: THEME.headFontFace,
    valign: 'bottom' });
  text(slide, caption, x, y + 0.8, w, o.capH || 0.7, { fontSize: o.capSize || 12, color: C.accent4 });
}

// formula in un riquadro (x, y, w, h): scala uniforme, mai oltre maxScale; align: left | center
function eq(slide, name, x, y, w, h, o = {}) {
  const m = EQ[name];
  if (!m) throw new Error('formula mancante: ' + name);
  const s = Math.min(w / m.w, h / m.h, o.maxScale || 1.7);
  const ew = m.w * s, eh = m.h * s;
  const ex = o.align === 'center' ? x + (w - ew) / 2 : x;
  const ey = o.valign === 'middle' ? y + (h - eh) / 2 : y;
  slide.addImage({ path: path.join(HERE, 'eq', name + '.png'), x: ex, y: ey, w: ew, h: eh, altText: 'Formula: ' + name });
  return { x: ex, y: ey, w: ew, h: eh };
}

function header(slide, title) {
  slide.addText(title, { placeholder: 'title' });
  const sec = SEC[currentSec];
  if (sec.tag) {
    slide.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 10.55, y: 0.52, w: 2.18, h: 0.42, fill: { color: sec.color },
      line: { color: sec.color, width: 0 }, rectRadius: 0.21 });
    slide.addText(sec.tag, { x: 10.55, y: 0.52, w: 2.18, h: 0.42, align: 'center', valign: 'middle', margin: 0,
      fontSize: 12, bold: true, color: C.background1, isTextBox: true });
  }
}

function line(slide, x1, y1, x2, y2, o = {}) {
  slide.addShape(pres.shapes.LINE, {
    x: Math.min(x1, x2), y: Math.min(y1, y2), w: Math.max(Math.abs(x2 - x1), 0.0001), h: Math.max(Math.abs(y2 - y1), 0.0001),
    flipV: (x2 - x1) * (y2 - y1) < 0 ? true : undefined,
    flipH: undefined,
    line: { color: o.color || C.text2, width: o.width || 1.5, dashType: o.dash, endArrowType: o.arrow ? 'triangle' : undefined,
      beginArrowType: o.arrowBegin ? 'triangle' : undefined },
  });
}

// freccia orizzontale o verticale fra due punti (sempre verso il secondo)
function arrow(slide, x1, y1, x2, y2, color) {
  const o = { color: color || C.accent4, width: 1.5, arrow: true };
  if (x2 < x1 || y2 < y1) {
    // pptxgenjs disegna le linee da (x, y) a (x+w, y+h): per andare "indietro" uso la punta all'inizio
    slide.addShape(pres.shapes.LINE, { x: Math.min(x1, x2), y: Math.min(y1, y2), w: Math.max(Math.abs(x2 - x1), 0.0001),
      h: Math.max(Math.abs(y2 - y1), 0.0001), line: { color: o.color, width: o.width, beginArrowType: 'triangle' } });
  } else line(slide, x1, y1, x2, y2, o);
}

function block(slide, x, y, w, h, t, o = {}) {
  const fill = o.fill || C.background2;
  slide.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, fill: { color: fill },
    line: { color: o.border || fill, width: o.border ? 1.25 : 0 }, rectRadius: 0.08 });
  text(slide, t, x + 0.08, y, w - 0.16, h, { fontSize: o.fontSize || 12, color: o.color || C.text1, align: 'center',
    valign: 'middle', bold: o.bold });
}

function table(slide, rows, x, y, w, colW, o = {}) {
  const head = o.headFill || SEC[currentSec].color;
  const fs = o.fontSize || 12;
  const data = rows.map((r, i) => r.map((c, j) => ({
    text: rich(c, { fontSize: fs, color: i === 0 ? C.background1 : C.text1, bold: i === 0 }),
    options: {
      fill: { color: i === 0 ? head : (i % 2 === 0 ? C.background2 : C.background1) },
      align: (o.align && o.align[j]) || 'left', valign: 'middle', margin: [3, 6, 3, 6],
    },
  })));
  slide.addTable(data, { x, y, w, colW, rowH: o.rowH || 0.3, border: { type: 'solid', pt: 0.75, color: HEX.lt2 } });
}

function pngSize(file) {
  const b = fs.readFileSync(file);
  return { w: b.readUInt32BE(16), h: b.readUInt32BE(20) };
}

function image(slide, file, x, y, w, h, alt) {
  const s = pngSize(file);
  const k = Math.min(w / s.w, h / s.h);
  const iw = s.w * k, ih = s.h * k;
  slide.addImage({ path: file, x: x + (w - iw) / 2, y: y + (h - ih) / 2, w: iw, h: ih, altText: alt });
}

// schema di RoTino nella posa nominale: (ox, oy) centro ruota, s pollici per metro
function drawRobot(slide, ox, oy, s, o = {}) {
  const a = 0.0919 * s, r = 0.06 * s;
  const leg = o.leg || C.accent1, torso = o.torso || C.accent5, ink = o.ink || C.text2;
  const K = [ox - a, oy - a], H = [ox, oy - 2 * a];
  if (o.ground !== false) line(slide, ox - 2.2 * r, oy + r, ox + 2.6 * r, oy + r, { color: ink, width: 1.5 });
  slide.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: ox - 0.05 * s, y: H[1] - 0.055 * s, w: 0.15 * s, h: 0.11 * s,
    fill: { color: torso }, line: { color: torso, width: 0 }, rectRadius: 0.06 });
  slide.addShape(pres.shapes.RECTANGLE, { x: ox + 0.01 * s, y: H[1] + 0.027 * s, w: 0.07 * s, h: 0.036 * s,
    fill: { color: ink }, line: { color: ink, width: 0 } });
  line(slide, ox, oy, K[0], K[1], { color: leg, width: o.legW || 6 });
  line(slide, K[0], K[1], H[0], H[1], { color: leg, width: o.legW || 6 });
  slide.addShape(pres.shapes.OVAL, { x: ox - r, y: oy - r, w: 2 * r, h: 2 * r, fill: { color: o.wheel || C.text1 },
    line: { color: o.wheel || C.text1, width: 0 } });
  const hub = 0.025 * s;
  slide.addShape(pres.shapes.OVAL, { x: ox - hub, y: oy - hub, w: 2 * hub, h: 2 * hub, fill: { color: C.accent2 },
    line: { color: C.accent2, width: 0 } });
  for (const p of [K, H]) {
    const j = 0.012 * s;
    slide.addShape(pres.shapes.OVAL, { x: p[0] - j, y: p[1] - j, w: 2 * j, h: 2 * j, fill: { color: C.background1 },
      line: { color: ink, width: 1.25 } });
  }
  return { K, H, r };
}

function placeholderChart(slide, x, y, w, h, file, metric) {
  slide.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, fill: { color: C.background2 },
    line: { color: C.accent5, width: 1.25, dashType: 'dash' }, rectRadius: 0.06 });
  const exists = fs.existsSync(path.join(WS, 'benchmark_runs', 'suite_pid_mpc', file));
  if (!exists) console.warn('ATTENZIONE: grafico non trovato', file);
  text(slide, 'Grafico da inserire', x + 0.3, y + h / 2 - 0.75, w - 0.6, 0.4, { fontSize: 16, bold: true,
    color: C.accent4, align: 'center' });
  text(slide, 'benchmark_runs/suite_pid_mpc/' + file, x + 0.3, y + h / 2 - 0.3, w - 0.6, 0.35, { fontSize: 11,
    color: C.accent4, align: 'center' });
  text(slide, 'Metriche: ' + metric, x + 0.3, y + h / 2 + 0.1, w - 0.6, 0.6, { fontSize: 12, color: C.text1,
    align: 'center' });
}

function keyMessage(slide, x, y, w, h) {
  card(slide, x, y, w, h);
  badge(slide, x + 0.2, y + (h - 0.42) / 2, 0.42, '!', C.text2, 14);
  text(slide, '**Messaggio chiave:** da scrivere dopo aver inserito i grafici.', x + 0.8, y, w - 1.0, h,
    { fontSize: 14, color: C.accent4, valign: 'middle', italic: true });
}

function legendPidMpc(slide, x, y) {
  badge(slide, x, y, 0.26, '', C.accent1);
  text(slide, 'PID', x + 0.34, y - 0.02, 0.6, 0.3, { fontSize: 12, bold: true, color: C.accent1 });
  badge(slide, x + 0.95, y, 0.26, '', C.accent2);
  text(slide, 'MPC', x + 1.29, y - 0.02, 0.7, 0.3, { fontSize: 12, bold: true, color: C.accent2 });
}

function sectionSlide(sec, num, title, body, notes) {
  const s = newSlide('SEZIONE', sec);
  s.addText(title, { placeholder: 'title' });
  s.addText(body, { placeholder: 'body' });
  badge(s, 0.95, 2.35, 1.7, num, SEC[sec].color, 60);
  s.addNotes(notes);
  return s;
}

const fmt = (v, d = 4) => v.toFixed(d).replace('.', ',');
const P = MOD.params, K = MOD.compact;

// ================================================================== 0. apertura
{
  const s = newSlide('TITOLO', 0);
  s.addText('RoTino: modellazione e controllo di un robot bipede su ruote', { placeholder: 'title' });
  s.addText('Dinamica di Eulero-Lagrange, spazio delle configurazioni, PID con lo ZMP e MPC + TV-LQR', { placeholder: 'body' });
  text(s, 'Alessando Prisco · Progetto FSR · ottobre 2026', 0.8, 6.4, 7.4, 0.4, { fontSize: 14, color: C.background2 });
  drawRobot(s, 10.3, 5.25, 18, { leg: C.accent1, torso: C.accent5, ink: C.background2, wheel: C.text1, legW: 9 });
  s.addNotes('Presentazione del progetto RoTino: un robot bipede su ruote da 4,29 kg simulato in Gazebo Fortress con ROS 2 Humble. '
    + 'Il percorso è in cinque parti: il modello dinamico, lo spazio delle configurazioni, le due leggi di controllo progettate '
    + '(PID con lo ZMP e MPC con TV-LQR) e l\'impostazione del confronto fra le due.');
}
{
  const s = newSlide('CONTENUTO', 0);
  header(s, 'Il percorso');
  const items = [
    ['Modellazione', 'Ipotesi, coordinate, Eulero-Lagrange, matrici M, C, G, B e riduzione al VL-WIP', C.accent4],
    ['C-space e DoF', 'Gradi di libertà, topologia, vincoli olonomi e anolonomi, sottoattuazione', C.accent3],
    ['PID con lo ZMP', 'Capture Point, ZMP come ingresso, anteprima LIPM, gambe e salto', C.accent1],
    ['MPC + TV-LQR', 'LQR schedulato, MPC del corpo superiore, VMC e stima dello stato', C.accent2],
    ['Confronto', 'Metodo e struttura dei grafici PID contro MPC', C.text2],
  ];
  const x0 = 0.9, dx = 2.42, y = 2.3;
  line(s, x0 + 0.45, y + 0.45, x0 + 4 * dx + 0.45, y + 0.45, { color: C.accent5, width: 2 });
  items.forEach(([t, d, c], i) => {
    const x = x0 + i * dx;
    badge(s, x, y, 0.9, i + 1, c, 28);
    text(s, t, x - 0.1, y + 1.15, 2.2, 0.45, { fontSize: 17, bold: true, color: C.text2 });
    text(s, d, x - 0.1, y + 1.65, 2.15, 1.6, { fontSize: 13, color: C.accent4 });
  });
  text(s, 'Codice: workspace ROS 2 rotino_ws (rotino_description, rotino_pid, rotino_mpc, rotino_benchmark)',
    0.8, 6.2, 11.5, 0.4, { fontSize: 12, color: C.accent4, italic: true });
  s.addNotes('Le cinque parti seguono l\'ordine logico del progetto: prima il modello e le sue proprietà, poi la struttura '
    + 'dello spazio delle configurazioni, poi le due leggi di controllo e infine il confronto. Entrambe le leggi usano la '
    + 'stessa libreria di modello (rotino_description) e lo stesso URDF, e il confronto cambia solo il nodo del controllore.');
}

// ================================================================== 1. modellazione
sectionSlide(1, '1', 'Modellazione del robot',
  'Ipotesi, coordinate generalizzate, equazioni di Eulero-Lagrange, matrici M, C, G e B, riduzione al pendolo inverso a lunghezza variabile (VL-WIP)',
  'Il modello segue la struttura di Matlab/SMC/init.m (Cui et al., 2022): quattro coordinate planari e Lagrangiana simbolica. '
  + 'I parametri però sono quelli di RoTino, letti dall\'URDF: init.m usa quelli del robot del paper (torso da 60 kg). '
  + 'Il calcolo è rifatto in SymPy in docs/presentazione/modello_lagrangiano.py.');
{
  const s = newSlide('CONTENUTO', 1);
  header(s, 'RoTino in numeri');
  table(s, [
    ['Simbolo', 'Valore', 'Origine (URDF)'],
    ['m_{w}', '0,38 kg', 'ruota 0,32 + mozzo 0,06'],
    ['I_{w}', '2,595·10^{-3} kg·m²', 'i_{yy} ruota, con armatura'],
    ['r · d', '0,06 m · 0,294 m', 'raggio, carreggiata'],
    ['m_{b}', '3,53 kg', 'totale 4,29 kg meno le ruote'],
    ['m_{3}', '2,85 kg', 'torso 2,50 + zavorra 0,35'],
    ['m_{2} · m_{1}', '0,18 · 0,16 kg', 'coscia · stinco, per gamba'],
    ['l_{1} = l_{2}', '0,130 m', '√2 · 0,0919, segmenti a 45°'],
    ['μ', '2,2', 'attrito ruota-suolo'],
    ['τ_{w,max} · τ_{leg,max}', '18 · 60 N·m', 'limiti URDF (controllo: 10 N·m)'],
  ], 0.6, 1.55, 6.3, [1.7, 2.0, 2.6], { rowH: 0.42, fontSize: 13 });
  infoCard(s, 7.3, 1.55, 5.43, 1.55, '1', 'Ruote pesanti in rotazione',
    '2I_{w}/r² = 1,44 kg: l\'inerzia delle ruote riportata alla traslazione vale il 41 % di m_{b}.', C.accent4);
  infoCard(s, 7.3, 3.25, 5.43, 1.55, '2', 'Baricentro del torso avanzato',
    '(+27,5; −5,5) mm dall\'anca: in equilibrio il torso è ruotato all\'indietro di circa 4,7°.', C.accent4);
  infoCard(s, 7.3, 4.95, 5.43, 1.55, '3', 'Inerzia reale più piccola',
    'I_{y} = 0,0174 kg·m² dai tensori URDF contro m_{b}l²/3 = 0,0311 della Tabella 1 del paper (×1,8).', C.accent4);
  s.addNotes('Tutti i parametri vengono dall\'URDF tramite WBRModel: nessun valore è copiato a mano. Tre aspetti pesano sul '
    + 'controllo. Le ruote sono il 18 % della massa, ma la loro inerzia di rotolamento vale il 41 % del corpo superiore, quindi '
    + 'non si può trascurare. Il baricentro del torso è davanti all\'anca, perciò in equilibrio il torso non è orizzontale. '
    + 'L\'inerzia reale del corpo superiore è 1,8 volte più piccola di quella che assume il paper.');
}
{
  const s = newSlide('CONTENUTO', 1);
  header(s, 'Ipotesi del modello');
  const hyp = [
    ['Piano sagittale', 'Le due gambe si muovono insieme: masse e inerzie di gamba e ruota contano due volte.'],
    ['Corpi rigidi, giunti ideali', 'Stinco e coscia come aste con baricentro a metà, torso con baricentro (a_{3}, b_{3}).'],
    ['Puro rotolamento', 'Suolo piano, nessun distacco: s_{w} = r q_{w}, la ruota resta a quota r.'],
    ['Attuazione in coppia', 'Ruote, ginocchia e anche ricevono coppie dirette (ros2_control, interfaccia effort).'],
  ];
  hyp.forEach(([t, d], i) => {
    const y = 1.55 + i * 1.2;
    badge(s, 0.6, y + 0.05, 0.6, i + 1, C.accent4, 18);
    text(s, t, 1.45, y, 5.6, 0.4, { fontSize: 17, bold: true, color: C.text2 });
    text(s, d, 1.45, y + 0.42, 5.7, 0.7, { fontSize: 14 });
  });
  card(s, 7.6, 1.55, 5.13, 4.6);
  text(s, 'Cosa si trascura', 7.9, 1.75, 4.6, 0.45, { fontSize: 18, bold: true, color: C.text2 });
  bullets(s, [
    'Imbardata e rollio: trattati a parte (modello di imbardata disaccoppiato)',
    'Slittamento e perdita di contatto delle ruote',
    'Smorzamento dei giunti (0,8 N·m·s/rad): i controllori lo compensano',
    'Dinamica dei motori, ritardi, campionamento a 500 Hz',
    'Camber e larghezza della ruota cilindrica (56 mm)',
  ], 7.9, 2.35, 4.6, 3.6, { fontSize: 14, gap: 8 });
  text(s, 'Struttura: Matlab/SMC/init.m (Cui et al., 2022) · Parametri: URDF di RoTino', 0.6, 6.45, 12, 0.35,
    { fontSize: 12, italic: true, color: C.accent4 });
  s.addNotes('Le ipotesi sono quelle di init.m, con due adattamenti. Il robot è ridotto al piano sagittale sommando le due '
    + 'gambe, e il baricentro del torso non è a metà di un\'asta sopra l\'anca ma nel punto reale dato da base e zavorra. '
    + 'Ciò che si trascura viene recuperato dai controllori: lo smorzamento dei giunti è compensato in feedforward, '
    + 'l\'imbardata ha un anello separato.');
}
{
  const s = newSlide('CONTENUTO', 1);
  header(s, 'Coordinate generalizzate');
  const ox = 2.55, oy = 5.6, sc = 14;
  const R = drawRobot(s, ox, oy, sc, { legW: 7 });
  // riferimenti degli angoli
  line(s, ox, oy, ox + 1.35, oy, { color: C.accent4, width: 1, dash: 'dash' });
  line(s, R.H[0], R.H[1], R.H[0], R.H[1] - 1.55, { color: C.accent4, width: 1, dash: 'dash' });
  text(s, 'q_{1}', ox - 1.05, oy - 0.95, 0.5, 0.35, { fontSize: 16, bold: true, color: C.accent1 });
  text(s, 'q_{2}', R.K[0] - 0.62, R.K[1] - 0.2, 0.5, 0.35, { fontSize: 16, bold: true, color: C.accent1 });
  text(s, 'q_{3}', R.H[0] + 0.12, R.H[1] - 1.5, 0.5, 0.35, { fontSize: 16, bold: true, color: C.accent1 });
  text(s, 'q_{w}', ox + 0.95, oy + 0.3, 0.5, 0.35, { fontSize: 16, bold: true, color: C.accent2 });
  text(s, 'x', ox + 1.4, oy - 0.18, 0.3, 0.3, { fontSize: 12, color: C.accent4 });
  text(s, 'stinco', R.K[0] - 0.85, R.K[1] + 0.35, 0.8, 0.3, { fontSize: 11, color: C.accent4 });
  text(s, 'coscia', R.K[0] + 0.6, R.K[1] - 0.15, 0.8, 0.3, { fontSize: 11, color: C.accent4 });
  text(s, 'torso', R.H[0] - 0.7, R.H[1] - 1.2, 0.8, 0.3, { fontSize: 11, color: C.accent4 });
  table(s, [
    ['q', 'Significato', 'Attuazione', 'Posa nominale'],
    ['q_{1}', 'angolo assoluto dello stinco dall\'orizzontale', '**passivo**', '135°'],
    ['q_{w}', 'rotazione assoluta della ruota, s_{w} = r q_{w}', 'τ_{w} ruota', '0'],
    ['q_{2}', 'ginocchio relativo (coscia = q_{1} + q_{2})', 'τ_{k} ginocchio', '−90°'],
    ['q_{3}', 'beccheggio del torso dalla verticale', 'τ_{h} anca', '0'],
  ], 5.0, 1.55, 7.73, [0.7, 3.73, 1.8, 1.5], { rowH: 0.5, fontSize: 13, align: ['center', 'left', 'left', 'center'] });
  infoCard(s, 5.0, 4.4, 7.73, 1.9, '4−3', 'Quattro coordinate, tre motori',
    'Il torso si comanda con l\'anca, ma l\'equilibrio dell\'insieme passa solo dalle ruote. Dall\'URDF: '
    + 'q_{1} = 135° − (θ_{pitch} + q_{hip} + q_{knee}), q_{2} = q_{knee} − 90°, q_{3} = θ_{pitch}.', C.accent4, { badgeSize: 11 });
  s.addNotes('Le coordinate sono le stesse di init.m: q1 assoluto dello stinco, che non ha motore proprio; qw rotazione della '
    + 'ruota legata all\'avanzamento dal vincolo di rotolamento; q2 ginocchio relativo; q3 beccheggio assoluto del torso. '
    + 'Nella posa nominale di RoTino (giunti URDF a zero) lo stinco sale indietro a 135° e la coscia torna in avanti a 45°, '
    + 'così la ruota sta esattamente sotto l\'anca. Il legame con le variabili dell\'URDF è scritto in basso.');
}
{
  const s = newSlide('CONTENUTO', 1);
  header(s, 'Cinematica dei baricentri ed energie');
  eq(s, 'cinematica', 0.6, 1.5, 6.6, 2.75);
  card(s, 7.6, 1.5, 5.13, 2.75);
  text(s, 'Parametri (gambe sommate)', 7.85, 1.65, 4.7, 0.4, { fontSize: 15, bold: true, color: C.text2 });
  bullets(s, [
    `m_{1}, m_{2}, m_{3}, m_{w} = ${fmt(P.m_1, 2)}; ${fmt(P.m_2, 2)}; ${fmt(P.m_3, 2)}; ${fmt(P.m_w, 2)} kg`,
    `I_{1}, I_{2} = ${fmt(P.I_1 * 1e4, 2)}; ${fmt(P.I_2 * 1e4, 2)} ·10^{-4} kg·m²`,
    `I_{3} = ${fmt(P.I_3 * 1e3, 2)}·10^{-3}, I_{w} = ${fmt(P.I_w * 1e3, 2)}·10^{-3} kg·m²`,
    `(a_{3}, b_{3}) = (+${fmt(P.a_3 * 1e3, 1)}; ${fmt(P.b_3 * 1e3, 1)}) mm; in init.m (0; l_{3}/2)`,
  ], 7.85, 2.15, 4.75, 2.0, { fontSize: 13, gap: 5 });
  eq(s, 'energie', 0.6, 4.55, 9.0, 1.6);
  text(s, 'Velocità dei baricentri: ṗ_{i} = (∂p_{i}/∂q) q̇, calcolate in modo simbolico come in init.m.', 0.6, 6.35, 12, 0.4,
    { fontSize: 13, italic: true, color: C.accent4 });
  s.addNotes('Le posizioni dei baricentri si scrivono a catena partendo dal centro ruota, che avanza di r per q_w. La sola '
    + 'differenza da init.m è l\'ultima riga: il baricentro del torso è spostato di (a3, b3) nella terna del torso, perché '
    + 'in RoTino è 27,5 mm avanti e 5,5 mm sotto l\'anca. L\'energia cinetica somma traslazione e rotazione di ogni corpo, '
    + 'la potenziale è solo gravitazionale.');
}
{
  const s = newSlide('CONTENUTO', 1);
  header(s, 'Equazioni di Eulero-Lagrange');
  eq(s, 'eulero_lagrange', 0.6, 1.5, 12.1, 0.85, { align: 'center', maxScale: 1.75 });
  eq(s, 'definizioni_MCG', 0.6, 2.65, 12.1, 1.5, { align: 'center', maxScale: 1.55 });
  const cw = 3.85;
  infoCard(s, 0.6, 4.55, cw, 1.85, 'M', 'Inerzia M(q)',
    'Simmetrica e definita positiva; dipende da q_{1}, q_{2}, q_{3} ma non da q_{w}.', C.accent4);
  infoCard(s, 0.6 + cw + 0.28, 4.55, cw, 1.85, 'C', 'Coriolis e centrifughi',
    'Termini in q̇_{i}q̇_{j} e q̇_{i}², costruiti con i simboli di Christoffel.', C.accent4);
  infoCard(s, 0.6 + 2 * (cw + 0.28), 4.55, cw, 1.85, 'G', 'Gravità G(q)',
    'G_{w} = 0: su suolo piano la ruota non sente la gravità.', C.accent4);
  s.addNotes('La forma standard dei manipolatori: M q̈ + C q̇ + G = B τ. M è la Hessiana dell\'energia cinetica rispetto alle '
    + 'velocità; C si costruisce con i simboli di Christoffel di prima specie, il che garantisce la proprietà di passività; '
    + 'G è il gradiente del potenziale. Tutto è calcolato in SymPy e verificato contro la forma diretta d/dt ∂L/∂q̇ − ∂L/∂q.');
}
{
  const s = newSlide('CONTENUTO', 1);
  header(s, 'La matrice di inerzia M(q)');
  eq(s, 'M_compatta', 0.6, 1.5, 12.1, 1.6, { align: 'center' });
  eq(s, 'M_def', 0.6, 3.25, 12.1, 0.95, { align: 'center' });
  const cw = 3.85;
  infoCard(s, 0.6, 4.5, cw, 1.9, '22', 'M_{22} costante',
    `Massa traslante più inerzia delle ruote: ${fmt(K.M22)} kg·m².`, C.accent4, { badgeSize: 12 });
  infoCard(s, 0.6 + cw + 0.28, 4.5, cw, 1.9, '2j', 'Accoppiamenti con q_{w}',
    'M_{12}, M_{23}, M_{24}: avanzare muove gamba e torso. È l\'origine della fase non minima.', C.accent4, { badgeSize: 12 });
  infoCard(s, 0.6 + 2 * (cw + 0.28), 4.5, cw, 1.9, '44', 'M_{44} costante',
    `Torso rigido attorno all'anca: I_{3} + m_{3}(a_{3}² + b_{3}²) = ${fmt(K.M44)} kg·m².`, C.accent4, { badgeSize: 12 });
  s.addNotes(`Raccogliendo i coefficienti la matrice diventa leggibile. k1 = ${fmt(K.k1)} e k2 = ${fmt(K.k2)} kg·m sono i `
    + 'momenti statici della catena sopra la ruota e sopra il ginocchio; ψ proietta il baricentro del torso. La forma '
    + 'compatta è verificata nello script contro quella estesa prodotta da SymPy. La riga e la colonna di q_w legano '
    + 'l\'avanzamento alla postura: è questo accoppiamento inerziale che rende il sistema a fase non minima.');
}
{
  const s = newSlide('CONTENUTO', 1);
  header(s, 'Coriolis, centrifughi e gravità');
  eq(s, 'C_compatta', 0.6, 1.5, 7.6, 1.75);
  eq(s, 'G_compatta', 8.6, 1.5, 4.1, 1.75);
  infoCard(s, 0.6, 3.75, 5.9, 2.6, 'q̇', 'Struttura dei termini',
    ['Solo termini quadratici nelle velocità, nessuno in q̇_{w}: q_{w} è una coordinata ciclica',
      'Riga 2 (ruota): spinta delle masse in rotazione sull\'avanzamento',
      'G dipende solo dalla postura: equilibrio con il baricentro sopra il contatto'], C.accent4);
  infoCard(s, 6.83, 3.75, 5.9, 2.6, 'i', 'Nota su init.m (valutazione)',
    ['La riga CdQ = ∂(∂L/∂q̇)/∂q · q̇ non sottrae ∂T/∂q',
      'Il calc_dynamics.m generato contiene invece il termine giusto (riga 3: +k sin q_{2} q̇_{1}²)',
      'Qui C è costruita con i simboli di Christoffel e verificata contro d/dt ∂L/∂q̇ − ∂L/∂q'], C.accent1);
  s.addNotes('Il vettore C q̇ contiene termini centrifughi e di Coriolis, tutti quadratici nelle velocità. La coordinata q_w '
    + 'non compare né in M né in G: è ciclica, il che riflette l\'omogeneità del suolo. Nota tecnica: in init.m la riga che '
    + 'calcola CdQ omette il termine −∂T/∂q dell\'equazione di Lagrange. Il file calc_dynamics.m presente nella cartella è '
    + 'però corretto, quindi è stato generato da una versione diversa dello script: va allineato.');
}
{
  const s = newSlide('CONTENUTO', 1);
  header(s, 'Valori nella posa nominale q₀');
  eq(s, 'M_num', 0.6, 1.5, 12.1, 1.45, { align: 'center' });
  const cond = MOD.M_eig[3] / MOD.M_eig[0];
  const st = [
    ['4,29 kg', 'massa totale (verificata)'],
    [fmt(cond, 0), 'numero di condizionamento di M(q_{0})'],
    [`${fmt(MOD.Z_C * 1e3, 0)} mm`, 'Z_{C}: baricentro del corpo superiore sopra l\'asse'],
    [`+${fmt(MOD.S_C * 1e3, 1)} mm`, 'S_{C}: davanti all\'asse a torso orizzontale'],
  ];
  st.forEach(([v, c], i) => stat(s, 0.6 + i * 3.1, 3.1, 2.9, v, c, C.accent4, { size: 32 }));
  eq(s, 'proprieta_M', 0.6, 4.85, 6.6, 1.5);
  card(s, 7.6, 4.75, 5.13, 1.75);
  text(s, '**Verifiche automatiche** (modello_lagrangiano.py)', 7.85, 4.88, 4.7, 0.35, { fontSize: 13, color: C.text2 });
  bullets(s, ['M simmetrica, definita positiva; Ṁ − 2C antisimmetrica',
    'S_{C}, Z_{C} uguali a WBRModel.equivalent_centroid',
    'forma compatta = forma estesa SymPy'], 7.85, 5.3, 4.75, 1.15, { fontSize: 12, gap: 3 });
  s.addNotes('Nella posa nominale M ha autovalori fra 0,007 e 0,148 kg·m², con un condizionamento intorno a 21. Il modello '
    + 'planare restituisce lo stesso baricentro del corpo superiore di WBRModel, la libreria usata dai controllori: '
    + `S_C = ${fmt(MOD.S_C * 1e3, 1)} mm e Z_C = ${fmt(MOD.Z_C * 1e3, 1)} mm. Il baricentro sta davanti all'asse: `
    + 'per stare in equilibrio il robot deve ruotare il torso indietro.');
}
{
  const s = newSlide('CONTENUTO', 1);
  header(s, 'Matrice di ingresso B e sottoattuazione');
  eq(s, 'B_matrice', 0.6, 1.5, 7.3, 1.45);
  card(s, 8.2, 1.5, 4.53, 1.45);
  text(s, 'La colonna di ogni motore è il gradiente dell\'angolo relativo che muove: la riga di q_{1} non è nulla, il sistema è **non collocato**.',
    8.4, 1.6, 4.15, 1.25, { fontSize: 13, valign: 'middle' });
  eq(s, 'B_check', 0.6, 3.35, 8.0, 1.0);
  infoCard(s, 0.6, 4.65, 12.13, 1.75, '!', 'Valutazione: la B di wbr_terms.m ha due segni da rivedere',
    'Ruotando tutto il robot come un corpo rigido nessun angolo relativo cambia, quindi B^{T}δq deve essere nullo. '
    + 'B_{init.m} = [−1 0 −1; 1 0 0; 0 1 −1; 0 0 1] dà −2δ sulle colonne di ruota e anca: le reazioni dei motori non sono '
    + 'uguali e opposte con le convenzioni di init.m (q_{3} e q_{w} orari, q_{1} antiorario).', C.accent1);
  s.addNotes('Ogni motore esercita una coppia uguale e opposta su due corpi, quindi la sua colonna di B è il gradiente '
    + 'dell\'angolo relativo fra quei corpi. Un controllo semplice: un moto rigido di tutto il robot non deve produrre lavoro '
    + 'dei motori. La B costruita qui supera il test; quella di wbr_terms.m no, sulle colonne della ruota e dell\'anca. '
    + 'È una correzione che conviene riportare nel modello MATLAB.');
}
{
  const s = newSlide('CONTENUTO', 1);
  header(s, 'Linearizzazione parziale non collocata');
  eq(s, 'schur', 0.6, 1.5, 12.1, 1.95, { align: 'center' });
  const cw = 3.85;
  infoCard(s, 0.6, 3.85, cw, 2.55, 'D', 'D invertibile',
    'Tre motori impongono le tre accelerazioni attive q̈_{w}, q̈_{2}, q̈_{3}. In posa nominale det D = 1,30.', C.accent4);
  infoCard(s, 0.6 + cw + 0.28, 3.85, cw, 2.55, 'q₁', 'Dinamica interna',
    'q_{1} segue dalla prima riga: lì resta l\'instabilità del pendolo, che va stabilizzata attraverso le uscite scelte.', C.accent4);
  infoCard(s, 0.6 + 2 * (cw + 0.28), 3.85, cw, 2.55, 'M̄', 'Complemento di Schur',
    'M̄ = M_{aa} − M_{au}M_{uu}^{-1}M_{ua} è definita positiva: in q_{0} gli elementi diagonali valgono 0,009; 0,026; 0,010 kg·m².', C.accent4);
  s.addNotes('È lo schema di wbr_terms.m: si separa la coordinata passiva q1 dalle tre attive. Con il complemento di Schur '
    + 'si trova la mappa τ = D⁻¹(M̄ v + h̄) che impone le accelerazioni attive. Il sistema è non collocato perché i motori '
    + 'agiscono anche sulla riga passiva. Valori calcolati con la B corretta della slide precedente.');
}
{
  const s = newSlide('CONTENUTO', 1);
  header(s, 'Dal modello completo al VL-WIP');
  text(s, 'Il corpo superiore (torso, cosce, stinchi) diventa un punto di massa m_{b} = 3,53 kg nel suo baricentro: '
    + 'l = √(S_{C}² + Z_{C}²) = 0,163 m, θ = atan2(S_{C}, Z_{C}). Le gambe regolano l, le ruote bilanciano θ.',
    0.6, 1.5, 12.1, 0.8, { fontSize: 15 });
  eq(s, 'vlwip_nl', 0.6, 2.45, 12.1, 1.45, { align: 'center' });
  const cw = 3.85;
  infoCard(s, 0.6, 4.25, cw, 2.15, '4→3', 'Meno coordinate',
    'Da [q_{1}, q_{w}, q_{2}, q_{3}] più l\'imbardata a [s, θ, φ]: la postura entra solo tramite l e I_{y}(l).', C.accent4, { badgeSize: 11 });
  infoCard(s, 0.6 + cw + 0.28, 4.25, cw, 2.15, 'φ', 'Imbardata disaccoppiata',
    'M diagonale a blocchi e C_{3} = 0: equilibrio e sterzata si controllano separatamente.', C.accent4);
  infoCard(s, 0.6 + 2 * (cw + 0.28), 4.25, cw, 2.15, '−', 'Cosa si perde',
    'La dinamica di l (l̇, l̈), la rotazione del torso rispetto al pendolo equivalente, la cedevolezza delle gambe.', C.accent4);
  s.addNotes('I controllori non usano il modello a quattro coordinate ma il pendolo inverso su ruote a lunghezza variabile '
    + 'del paper (eq. 4-5). Il centroide equivalente si calcola con la stessa cinematica dell\'URDF. Rispetto al modello '
    + 'completo si perde la dinamica della lunghezza, accettabile finché le gambe si muovono lentamente rispetto '
    + 'all\'equilibrio: non durante la spinta del salto.');
}
{
  const s = newSlide('CONTENUTO', 1);
  header(s, 'Linearizzazione e proprietà strutturali');
  eq(s, 'vlwip_lin', 0.6, 1.45, 12.1, 1.1, { align: 'center' });
  eq(s, 'fase_non_minima', 0.6, 2.75, 12.1, 0.7, { align: 'center' });
  table(s, [
    ['l [m]', 'a_{1} [m/s²]', 'a_{2} [s^{-2}]', 'b_{1}', 'b_{2}', '√a_{2} [rad/s]'],
    ['0,2188', '−12,02', '89,17', '8,055', '−38,20', '9,44'],
    ['**0,1627**', '**−10,60**', '**105,82**', '**7,933**', '**−50,15**', '**10,29**'],
    ['0,1061', '−7,86', '120,35', '7,379', '−68,44', '10,97'],
    ['0,0738', '−5,32', '116,94', '6,564', '−80,40', '10,81'],
  ], 0.6, 3.75, 6.9, [1.1, 1.2, 1.2, 1.0, 1.1, 1.3], { rowH: 0.42, fontSize: 12, align: ['center', 'center', 'center', 'center', 'center', 'center'] });
  stat(s, 7.9, 3.65, 2.4, '+10,29', 'rad/s, polo instabile: diverge in 97 ms', C.accent1, { size: 30 });
  stat(s, 10.4, 3.65, 2.4, '±6,23', 'rad/s, zeri: fase non minima', C.accent1, { size: 30 });
  stat(s, 7.9, 5.25, 4.9, '6,14 m/s²/rad', 'g_{st} = a_{1} − b_{1}a_{2}/b_{2}: l\'inclinazione è l\'ingresso della traslazione', C.accent4, { size: 26 });
  s.addNotes('Linearizzando attorno a θ = 0 si ottengono le equazioni 13-14 del paper; i coefficienti dipendono dalla '
    + 'lunghezza l, quindi dalla postura. Il polo instabile è a circa 10 rad/s, cioè il robot cade in un decimo di secondo. '
    + 'Lo zero reale positivo dice che per andare avanti le ruote devono prima arretrare: nessun controllore causale lo '
    + 'evita. A regime, tenere il pendolo inclinato di θ produce un\'accelerazione g_st θ: è il meccanismo su cui si reggono '
    + 'sia il PID sia l\'MPC.');
}

// ================================================================== 2. C-space e DoF
sectionSlide(2, '2', 'Spazio delle configurazioni e gradi di libertà',
  'Conteggio dei DoF, topologia del C-space, vincoli olonomi e anolonomi, sottoattuazione',
  'Questa parte riprende docs/Analisi_DoF_Topologia.md e la collega al modello appena visto: il robot libero ha 12 gradi '
  + 'di libertà, il modello planare 4, il pendolo sagittale 2.');
{
  const s = newSlide('CONTENUTO', 2);
  header(s, 'Quanti gradi di libertà');
  eq(s, 'dof', 0.6, 1.5, 12.1, 0.85, { align: 'center' });
  const parts = [
    ['6', 'Torso', 'corpo rigido libero: 3 traslazioni e 3 rotazioni'],
    ['2', 'Anche', 'giunti rotoidali, limiti ±0,9 rad'],
    ['2', 'Ginocchia', 'giunti rotoidali, limiti ±0,9 rad'],
    ['2', 'Ruote', 'giunti continui, senza limiti'],
  ];
  parts.forEach(([n, t, d], i) => {
    const x = 0.6 + i * 3.1;
    card(s, x, 2.75, 2.85, 2.6);
    badge(s, x + 0.95, 2.95, 0.95, n, C.accent3, 32);
    text(s, t, x + 0.2, 4.0, 2.45, 0.4, { fontSize: 17, bold: true, color: C.text2, align: 'center' });
    text(s, d, x + 0.2, 4.45, 2.45, 0.8, { fontSize: 13, color: C.accent4, align: 'center' });
  });
  text(s, '**12 DoF** senza vincoli di contatto, contati come base flottante più giunti (Grübler esteso).', 0.6, 5.7, 12.1, 0.5,
    { fontSize: 16, color: C.text2 });
  s.addNotes('Si conta il robot prima di imporre il contatto con il suolo. Il torso è una base flottante con sei gradi di '
    + 'libertà, a cui si sommano i sei giunti: due anche, due ginocchia, due ruote. Il vettore delle coordinate generalizzate '
    + 'ha dodici componenti.');
}
{
  const s = newSlide('CONTENUTO', 2);
  header(s, 'Topologia del C-space');
  eq(s, 'cspace', 0.6, 1.5, 12.1, 0.6, { align: 'center' });
  const cw = 3.85, y = 2.5, h = 3.3;
  const label = (x, t, d) => {
    text(s, t, x + 0.25, y + 1.6, cw - 0.5, 0.4, { fontSize: 16, bold: true, color: C.text2, align: 'center' });
    text(s, d, x + 0.25, y + 2.1, cw - 0.5, 1.1, { fontSize: 13 });
  };
  // SE(3): terna di assi
  card(s, 0.6, y, cw, h);
  const ax = 0.6 + cw / 2 - 0.15, ay = y + 1.15;
  line(s, ax, ay, ax + 0.8, ay, { color: C.accent1, width: 2.5, arrow: true });
  line(s, ax, ay, ax, ay - 0.8, { color: C.accent2, width: 2.5, arrow: true });
  line(s, ax, ay, ax - 0.45, ay + 0.35, { color: C.accent3, width: 2.5, arrow: true });
  label(0.6, 'SE(3) ≅ ℝ³ × SO(3)', 'Torso libero. SO(3) si rappresenta con i quaternioni per evitare il gimbal lock.');
  // intervalli
  const x2 = 0.6 + cw + 0.28;
  card(s, x2, y, cw, h);
  for (let i = 0; i < 4; i++) {
    const ly = y + 0.35 + i * 0.3, lx = x2 + cw / 2 - 0.8;
    line(s, lx, ly, lx + 1.6, ly, { color: C.accent3, width: 2.5 });
    line(s, lx, ly - 0.1, lx, ly + 0.1, { color: C.text2, width: 2 });
    line(s, lx + 1.6, ly - 0.1, lx + 1.6, ly + 0.1, { color: C.text2, width: 2 });
  }
  label(x2, '[q^{−}, q^{+}]⁴ ≅ ℝ⁴', 'Anche e ginocchia hanno fine corsa: ogni giunto è un intervallo chiuso, non una circonferenza.');
  // toro
  const x3 = 0.6 + 2 * (cw + 0.28);
  card(s, x3, y, cw, h);
  for (const dx of [-1.0, 0.25]) {
    s.addShape(pres.shapes.OVAL, { x: x3 + cw / 2 + dx, y: y + 0.4, w: 0.75, h: 0.75, fill: { color: C.background2 },
      line: { color: C.accent3, width: 3 } });
  }
  text(s, '×', x3 + cw / 2 - 0.25, y + 0.52, 0.5, 0.5, { fontSize: 20, bold: true, color: C.text2, align: 'center' });
  label(x3, 'T² = S¹ × S¹', 'Le ruote girano senza limiti: due circonferenze, cioè un toro.');
  text(s, 'Stesso numero di DoF non significa stessa topologia: ℝ¹² e SE(3) × ℝ⁴ × T² non si deformano l\'uno nell\'altro.',
    0.6, 6.1, 12.1, 0.5, { fontSize: 14, italic: true, color: C.accent4 });
  s.addNotes('La topologia dice di che forma è lo spazio delle configurazioni. Il torso vive in SE(3); i giunti limitati sono '
    + 'intervalli, quindi equivalgono a ℝ; le ruote, che girano all\'infinito, sono circonferenze. Il prodotto è SE(3) per ℝ⁴ '
    + 'per il toro T². Due spazi con dodici dimensioni non sono per forza equivalenti.');
}
{
  const s = newSlide('CONTENUTO', 2);
  header(s, 'Vincoli olonomi e anolonomi');
  eq(s, 'pfaff', 0.6, 1.5, 7.2, 1.7);
  infoCard(s, 8.15, 1.5, 4.58, 2.1, 'H', 'Olonomi (integrabili)',
    'Contatto con il suolo piano: quota dell\'asse fissata a r e rollio legato alle gambe. Riducono le coordinate.', C.accent3);
  infoCard(s, 8.15, 3.8, 4.58, 2.1, 'NH', 'Anolonomi (non integrabili)',
    'Rotolamento senza slittamento: limitano le velocità, non le configurazioni raggiungibili (parentesi di Lie).', C.accent3, { badgeSize: 11 });
  infoCard(s, 0.6, 3.5, 7.2, 2.4, 'B', 'Teorema di Brockett',
    'Il modello differenziale (uniciclo) non ammette una retroazione liscia e tempo-invariante che stabilizzi asintoticamente '
    + 'un punto (x_{d}, y_{d}, ψ_{d}). Per questo entrambi i controllori stabilizzano l\'equilibrio e inseguono traiettorie '
    + 'nel tempo invece di "parcheggiare" su una posa.', C.accent3);
  s.addNotes('Quando le ruote toccano il suolo compaiono due tipi di vincoli. Quelli olonomi fissano la quota e riducono le '
    + 'coordinate; quelli anolonomi, in forma Pfaffiana, vietano lo slittamento laterale ma lasciano raggiungibili tutte le '
    + 'pose. Le reazioni entrano nella dinamica tramite i moltiplicatori di Lagrange λ. Il teorema di Brockett spiega perché '
    + 'la guida planare è fatta come inseguimento di traiettoria.');
}
{
  const s = newSlide('CONTENUTO', 2);
  header(s, 'Sottoattuazione a tre livelli');
  eq(s, 'sottoattuazione', 0.6, 1.5, 12.1, 0.55, { align: 'center' });
  const lv = [
    ['12 / 6', 'Robot libero', 'n = 12 coordinate, m = 6 motori: ruote, ginocchia, anche.'],
    ['4 / 3', 'Modello planare', 'Gruppo 1: q_{1} non ha motore. Le reazioni dei motori lo muovono (non collocato).'],
    ['2 / 1', 'VL-WIP sagittale', 's e θ con un solo ingresso u = τ_{l} + τ_{r}: un motore per due coordinate.'],
  ];
  lv.forEach(([n, t, d], i) => {
    const x = 0.6 + i * 4.13;
    card(s, x, 2.4, 3.85, 2.45);
    text(s, n, x + 0.25, 2.55, 3.4, 0.75, { fontSize: 34, bold: true, color: C.accent3, fontFace: THEME.headFontFace });
    text(s, t, x + 0.25, 3.3, 3.4, 0.4, { fontSize: 16, bold: true, color: C.text2 });
    text(s, d, x + 0.25, 3.75, 3.4, 1.0, { fontSize: 13 });
    if (i < 2) arrow(s, x + 3.85, 3.6, x + 4.13, 3.6, C.accent3);
  });
  infoCard(s, 0.6, 5.15, 12.13, 1.3, '→', 'Conseguenza per il controllo',
    'Per avanzare bisogna prima inclinarsi: l\'inclinazione (o lo ZMP) è l\'ingresso virtuale della traslazione. Il PID e l\'MPC differiscono nel modo di sceglierla.',
    C.accent3);
  s.addNotes('La sottoattuazione è una proprietà del modello che si usa. Il robot libero ha sei motori per dodici coordinate; '
    + 'il modello planare tre motori per quattro coordinate; il pendolo sagittale un solo ingresso per due coordinate. '
    + 'Ne segue che la traslazione si comanda indirettamente, tramite l\'inclinazione: il PID la sceglie con lo ZMP, l\'MPC '
    + 'con lo spostamento Δs del baricentro.');
}

// ================================================================== 3. PID con ZMP
sectionSlide(3, '3', 'Controllo PID con lo ZMP',
  'rotino_pid: PI sul Capture Point, ZMP come ingresso delle ruote, anteprima LIPM, PID di rotta, PD cartesiano delle gambe e supervisore del salto',
  'Il controllore nasce dal porting di un controllore MuJoCo ed è stato riprogettato attorno allo Zero Moment Point. '
  + 'La legge di controllo è in rotino_pid/zmp_balance.py, pura numpy e testata offline; il nodo ROS è controller.py.');
{
  const s = newSlide('CONTENUTO', 3);
  header(s, 'L\'idea: lo ZMP come ingresso');
  eq(s, 'lipm', 0.6, 1.5, 6.8, 0.75);
  bullets(s, [
    'Su due ruote il poligono di appoggio è un **segmento**: longitudinalmente lo ZMP coincide con il contatto',
    'Le ruote spostano il contatto, quindi **spostano lo ZMP**: è la leva con cui si governa il baricentro',
    'ξ è il **Capture Point**: dove dovrebbe stare lo ZMP per fermare il robot',
    'Lo stato misurato è il CoM dalla cinematica e dalla posa del torso (ground truth)',
  ], 0.6, 2.55, 6.8, 3.2, { fontSize: 15, gap: 9 });
  // schema carrello-tavolo
  const gx = 8.0, gy = 5.6;
  card(s, 7.75, 1.5, 4.98, 4.65);
  line(s, gx, gy, gx + 4.4, gy, { color: C.text2, width: 2 });
  const px = 9.6, cx = 10.4, cy = 2.6;
  s.addShape(pres.shapes.OVAL, { x: px - 0.4, y: gy - 0.8, w: 0.8, h: 0.8, fill: { color: C.text1 }, line: { color: C.text1, width: 0 } });
  line(s, px, gy - 0.4, cx, cy, { color: C.accent1, width: 4 });
  s.addShape(pres.shapes.OVAL, { x: cx - 0.22, y: cy - 0.22, w: 0.44, h: 0.44, fill: { color: C.accent1 }, line: { color: C.accent1, width: 0 } });
  line(s, cx, cy, cx, gy, { color: C.accent4, width: 1, dash: 'dash' });
  line(s, px, gy + 0.2, cx, gy + 0.2, { color: C.accent4, width: 1, arrow: true });
  text(s, 'c', cx + 0.3, cy - 0.2, 0.4, 0.4, { fontSize: 18, bold: true, color: C.accent1 });
  text(s, 'p = ZMP', px - 1.5, gy - 0.4, 1.0, 0.35, { fontSize: 13, bold: true, color: C.text2, align: 'right' });
  text(s, 's = c − p', cx + 0.1, gy + 0.06, 1.3, 0.3, { fontSize: 12, color: C.accent4 });
  text(s, 'h ≈ 0,19 m', cx + 0.1, 3.9, 1.3, 0.35, { fontSize: 12, color: C.accent4 });
  text(s, 'ω = √(g/h) ≈ 7,1 rad/s', 8.0, 1.65, 4.5, 0.4, { fontSize: 14, bold: true, color: C.text2 });
  s.addNotes('Nel modello carrello-tavolo il baricentro accelera in proporzione alla sua distanza dallo ZMP. Su un robot a '
    + 'due ruote lo ZMP longitudinale non può muoversi dentro un piede: sta sotto l\'asse, e sono le ruote a spostarlo. '
    + 'Quindi lo ZMP è l\'ingresso naturale. Il Capture Point è la componente divergente del moto: se lo ZMP ci si mette '
    + 'sopra, il robot si ferma.');
}
{
  const s = newSlide('CONTENUTO', 3);
  header(s, 'Architettura degli anelli');
  const r1 = 1.75, r2 = 3.35, r3 = 4.95, bh = 0.95;
  const red = C.accent1;
  block(s, 0.6, r1, 2.05, bh, 'Percorso pianificato\n(S, trapezio, teleop)');
  block(s, 3.0, r1, 2.05, bh, 'Anteprima LIPM\nc_{ref}, c̈_{ref}');
  block(s, 5.4, r1, 2.3, bh, '(A) PI sul Capture Point\nρ = 0,4', { fill: red, color: C.background1, bold: true });
  block(s, 8.05, r1, 2.3, bh, 'PD sull\'offset ZMP\n+ feedforward VL-WIP', { fill: red, color: C.background1, bold: true });
  arrow(s, 2.65, r1 + bh / 2, 3.0, r1 + bh / 2);
  arrow(s, 5.05, r1 + bh / 2, 5.4, r1 + bh / 2);
  arrow(s, 7.7, r1 + bh / 2, 8.05, r1 + bh / 2);
  block(s, 0.6, r2, 2.05, bh, 'Traiettoria planare');
  block(s, 3.0, r2, 2.05, bh, '(C) Guida laterale\nψ_{cmd}', { border: red });
  block(s, 5.4, r2, 4.95, bh, '(B) PID di rotta + attrito di Coulomb e viscoso', { border: red });
  arrow(s, 2.65, r2 + bh / 2, 3.0, r2 + bh / 2);
  arrow(s, 5.05, r2 + bh / 2, 5.4, r2 + bh / 2);
  block(s, 0.6, r3, 2.05, bh, 'Altezza, salto\n(supervisore 7 stati)');
  block(s, 3.0, r3, 7.35, bh, '(D) PD cartesiano ruota-anca con il peso in feedforward → τ_{hip}, τ_{knee}', { border: red });
  arrow(s, 2.65, r3 + bh / 2, 3.0, r3 + bh / 2);
  block(s, 10.85, r1, 1.88, 2.55, 'τ_{l} = τ_{c} − τ_{d}\nτ_{r} = τ_{c} + τ_{d}\n\nruote, 10 N·m', { fill: C.text2, color: C.background1, bold: true });
  arrow(s, 10.35, r1 + bh / 2, 10.85, r1 + bh / 2);
  arrow(s, 10.35, r2 + bh / 2, 10.85, r2 + bh / 2);
  text(s, 'τ_{c}', 10.38, r1 + 0.08, 0.5, 0.3, { fontSize: 11, color: C.accent4 });
  text(s, 'τ_{d}', 10.38, r2 + 0.08, 0.5, 0.3, { fontSize: 11, color: C.accent4 });
  text(s, 'Stato: posa del torso da /rotino/odom e cinematica URDF, accoppiati per timestamp a 500 Hz; derivate per differenze finite.',
    0.6, 6.25, 12.1, 0.45, { fontSize: 13, italic: true, color: C.accent4 });
  s.addNotes('Quattro anelli. A: equilibrio e avanzamento con lo ZMP, sul modo comune delle ruote. B e C: rotta e guida laterale, '
    + 'sul modo differenziale. D: gambe in spazio cartesiano, con il supervisore del salto che cambia i riferimenti. '
    + 'Le coppie comune e differenziale si combinano nelle due ruote con priorità al modo comune.');
}
{
  const s = newSlide('CONTENUTO', 3);
  header(s, 'Anello A: Capture Point → ZMP → coppia');
  eq(s, 'anello_A', 0.6, 1.5, 8.2, 1.55);
  table(s, [
    ['Guadagno', 'Valore'],
    ['K_{s}, K_{sd} (per ruota)', '60 N·m/m · 5 N·m·s/m'],
    ['ρ (autorità)', '0,4'],
    ['k_{ξ}, k_{i}', '1,5 s^{-1} · 0,5 s^{-2}'],
    ['a_{max}', '3 m/s² → offset ≤ 59 mm'],
    ['λ_{s} (feedforward)', '21,8 mm per m/s²'],
    ['Saturazione', '10 N·m per ruota'],
  ], 9.1, 1.5, 3.63, [1.7, 1.93], { rowH: 0.42, fontSize: 12 });
  const cw = 2.6;
  infoCard(s, 0.6, 3.5, cw, 2.4, 'E', 'Anello esterno', 'PI sull\'errore del Capture Point: dice dove mettere lo ZMP rispetto al CoM.', C.accent1);
  infoCard(s, 0.6 + cw + 0.2, 3.5, cw, 2.4, 'I', 'Anello interno', 'PD sull\'offset s: le ruote portano il contatto dove serve.', C.accent1);
  infoCard(s, 0.6 + 2 * (cw + 0.2), 3.5, cw, 2.4, 'FF', 'Feedforward', 'Dal VL-WIP: inclinazione e coppia che tengono c̈_{ref} a regime.', C.accent1, { badgeSize: 11 });
  s.addNotes('L\'anello esterno impone una dinamica del Capture Point con un PI e ne ricava l\'offset desiderato fra CoM e ZMP, '
    + 'saturato a 59 mm. L\'anello interno è un PD sull\'offset che produce la coppia di modo comune. Il feedforward viene '
    + 'dal modello linearizzato: 21,8 mm di offset per ogni m/s² di accelerazione, più la coppia corrispondente.');
}
{
  const s = newSlide('CONTENUTO', 3);
  header(s, 'Anteprima LIPM: inclinarsi prima');
  eq(s, 'anteprima', 0.6, 1.5, 6.0, 1.25);
  bullets(s, [
    'Il riferimento del CoM è la soluzione **limitata** di c̈ = ω²(c − p_{ref})',
    'Si ottiene filtrando il percorso con un nucleo simmetrico: guarda nel **futuro** per circa 1/ω',
    'Il robot si inclina **prima** del gradino di accelerazione, come richiede la fase non minima',
    'Con i comandi da dashboard il futuro non è noto: si usa l\'accelerazione corrente filtrata (0,1 s)',
  ], 0.6, 2.85, 5.9, 3.6, { fontSize: 15, gap: 10 });
  image(s, path.join(WS, 'docs', 'figure_pid_zmp', 'trapezio_zmp.png'), 6.8, 1.4, 5.93, 5.3,
    'Trapezio 1 m/s: velocità, ZMP desiderato ed effettivo, inclinazione del CoM');
  s.addNotes('Un pendolo inverso non può seguire un gradino di accelerazione senza prima inclinarsi. L\'anteprima LIPM calcola '
    + 'il CoM di riferimento filtrando il percorso pianificato con il nucleo e alla meno omega per modulo di u: il risultato '
    + 'anticipa il moto. Nel grafico lo ZMP desiderato e quello effettivo si spostano dietro il CoM circa 0,2 s prima '
    + 'della partenza.');
}
{
  const s = newSlide('CONTENUTO', 3);
  header(s, 'Progetto robusto dei guadagni');
  infoCard(s, 0.6, 1.5, 6.3, 2.5, 'ρ', 'Perché un\'autorità ρ = 0,4',
    'La cascata LIPM pura assume un anello interno infinitamente veloce. A 500 Hz, con 1–2 campioni di ritardo e il modo '
    + 'pendolo-ruote a circa 10 rad/s, il termine ċ/ω a piena autorità diventa retroazione positiva sulla velocità delle ruote.', C.accent1);
  infoCard(s, 0.6, 4.2, 6.3, 2.2, '10', 'Dieci casi di robustezza',
    ['massa ±20 %, anca ±0,3 rad (CoM da accosciato a esteso)', 'ritardo di 1 o 2 campioni, modello discretizzato a 500 Hz'], C.accent1, { badgeSize: 12 });
  stat(s, 7.4, 1.5, 5.3, '0,69', 'smorzamento minimo su tutti i casi', C.accent1, { size: 40 });
  stat(s, 7.4, 3.0, 5.3, '−0,5 · −2,3±0,2j · −9,8 · −52', 'poli nominali ad anello chiuso [s^{-1}]', C.accent1, { size: 20 });
  stat(s, 7.4, 4.5, 5.3, '16 test', 'pytest offline: recupero robusto, trapezio < 1 cm, spinta 2,7 N·s, integrale, anteprima', C.accent1, { size: 32, capH: 0.9 });
  s.addNotes('I guadagni non sono tarati a mano in Gazebo ma scelti sul modello linearizzato e discretizzato, con posizionamento '
    + 'robusto dei poli su dieci varianti. L\'autorità ridotta ρ è la chiave: con autorità piena la cascata diventa instabile '
    + 'per guadagni interni alti. Il residuo lo recupera l\'integrale. I sedici test in test_zmp_balance.py lo verificano '
    + 'sul modello lineare.');
}
{
  const s = newSlide('CONTENUTO', 3);
  header(s, 'Rotta, guida laterale e gambe');
  eq(s, 'imbardata_pid', 0.6, 1.5, 12.1, 0.75, { align: 'center' });
  eq(s, 'gambe_pd', 0.6, 2.5, 12.1, 0.65, { align: 'center' });
  infoCard(s, 0.6, 3.5, 5.9, 2.9, 'B', 'Rotta (B) e guida (C)',
    ['K_{ψ} = 1,8 N·m/rad, K_{ω} = 0,45, K_{I} = 0,6; saturazione 2,7 N·m',
      'Attrito di strisciamento identificato in Gazebo: 0,25 N·m + 0,10 N·m·s/rad',
      'Senza attrito il robot girava a scatti (stick-slip)'], C.accent1);
  infoCard(s, 6.83, 3.5, 5.9, 2.9, 'D', 'Gambe (D)',
    ['Appoggio: K_{p} = diag(1500, 5000) N/m, K_{d} = diag(40, 80) N·s/m',
      'Salto (3000, 4000), volo (800, 800) senza sostegno',
      'Sostituisce il PD di giunto (160–200 N·m/rad) che a 500 Hz andava in ciclo limite bang-bang'], C.accent1);
  s.addNotes('La rotta usa un PID con feedforward d\'attrito sul riferimento, quindi senza chattering. La guida laterale corregge '
    + 'la rotta in base all\'errore laterale, ma solo in movimento, perché un veicolo differenziale non annulla l\'errore '
    + 'laterale da fermo. Le gambe usano un PD sulla posizione della ruota rispetto all\'anca, convertito in coppie con la '
    + 'trasposta del Jacobiano: in spazio giunti equivale a circa 50 N·m/rad e ha eliminato il ciclo limite.');
}
{
  const s = newSlide('CONTENUTO', 3);
  header(s, 'Supervisore del salto');
  const st = ['BALANCE', 'PRELOAD', 'THRUST', 'FLIGHT', 'LANDING', 'RECOVERY', 'SETTLE'];
  const bw = 1.52, gap = 0.25, y = 1.6;
  st.forEach((n, i) => {
    const x = 0.6 + i * (bw + gap);
    block(s, x, y, bw, 0.7, n, { fill: i === 0 ? C.text2 : C.accent1, color: C.background1, bold: true, fontSize: 12 });
    if (i < st.length - 1) arrow(s, x + bw, y + 0.35, x + bw + gap, y + 0.35);
  });
  line(s, 0.6 + 6 * (bw + gap) + bw / 2, y + 0.7, 0.6 + 6 * (bw + gap) + bw / 2, y + 1.0, { color: C.accent4, width: 1.5 });
  line(s, 0.6 + bw / 2, y + 1.0, 0.6 + 6 * (bw + gap) + bw / 2, y + 1.0, { color: C.accent4, width: 1.5 });
  arrow(s, 0.6 + bw / 2, y + 1.0, 0.6 + bw / 2, y + 0.7);
  table(s, [
    ['Stato', 'Riferimento (anca, ginocchio) [rad]', 'Uscita'],
    ['PRELOAD', '(0, 0) → (+0,12, −0,22) in 0,80 s, smoothstep', 'dopo 0,80 s'],
    ['THRUST', '→ (−0,27, +0,44) in 0,02 s, 1 − (1 − τ)³', 'decollo, o LANDING dopo 0,16 s'],
    ['FLIGHT', 'posa congelata al decollo, ruote smorzate', 'contatto confermato'],
    ['LANDING', '→ (+0,08, −0,20) in 0,80 s, gambe cedevoli', 'dopo 0,80 s'],
    ['RECOVERY / SETTLE', 'ritorno a (0, 0) in 2,0 s, poi assestamento', '0,30 s fermo, o 12 s'],
  ], 0.6, 3.0, 7.9, [1.9, 4.1, 1.9], { rowH: 0.48, fontSize: 12 });
  infoCard(s, 8.85, 3.0, 3.88, 3.4, '↑', 'Decollo confermato se',
    ['contatto perso (isteresi, 4 ms)', 'forza sotto soglia', 'ruota staccata dal suolo', 'v_{z} del CoM > 0,08 m/s',
      'Correzione del Capture Point scalata per fase, integrale congelato'], C.accent1);
  s.addNotes('Il salto è una macchina a sette stati. I riferimenti di giunto vengono dal controllore MuJoCo originale e sono '
    + 'convertiti in posizioni cartesiane della ruota. Il decollo richiede quattro condizioni insieme per non scambiare un '
    + 'rimbalzo per un volo. Durante le fasi di salto l\'anello del Capture Point ha autorità ridotta e l\'integrale è fermo.');
}
{
  const s = newSlide('CONTENUTO', 3);
  header(s, 'Valutazioni sul PID');
  card(s, 0.6, 1.5, 5.9, 4.6);
  text(s, 'Punti di forza', 0.85, 1.65, 5.4, 0.45, { fontSize: 18, bold: true, color: C.accent2 });
  bullets(s, [
    'Anteprima LIPM: con riferimenti noti il robot si inclina in anticipo e insegue senza sovraelongazioni',
    'Integrale sul Capture Point: annulla un offset del CoM che il modello non conosce',
    'Legge pura, separata dal nodo ROS, testata offline su casi di robustezza',
    'Guadagni da modello (poli robusti) e non da tentativi in simulazione',
  ], 0.85, 2.25, 5.45, 3.7, { fontSize: 15, gap: 10 });
  card(s, 6.83, 1.5, 5.9, 4.6);
  text(s, 'Limiti e spunti', 7.08, 1.65, 5.4, 0.45, { fontSize: 18, bold: true, color: C.accent1 });
  bullets(s, [
    'Stato dalla ground truth del simulatore: non disponibile su un robot reale',
    'ρ = 0,4 fisso: poca autorità contro le spinte. Spunto: ρ schedulato nei transitori',
    'Vincoli solo come saturazione (a_{max}); l\'anti-windup non vede il limite 10 − |τ_{d}| in curva',
    'Telemetria: z_{ref} = z in /rotino/wbr_state, quindi l\'errore di quota del benchmark non è significativo',
    'Compensazione laterale spenta: la ruota cilindrica salta di spigolo al flesso della S',
  ], 7.08, 2.25, 5.45, 3.8, { fontSize: 15, gap: 8 });
  s.addNotes('Valutazione personale. Il PID funziona bene dove il riferimento è noto in anticipo, grazie all\'anteprima. '
    + 'I limiti sono due di metodo e due di implementazione. Di metodo: lo stato viene dalla ground truth e ρ è fisso. '
    + 'Di implementazione: l\'anti-windup non tiene conto della riduzione del limite di coppia in curva, e la telemetria '
    + 'pubblica z_ref uguale a z. La compensazione laterale richiederebbe una ruota bombata nell\'URDF.');
}

// ================================================================== 4. MPC + TV-LQR
sectionSlide(4, '4', 'MPC + TV-LQR + VMC',
  'rotino_mpc: architettura di Cui et al. (Fig. 4). Equilibrio con LQR schedulato sulla lunghezza, corpo superiore con MPC vincolato, gambe con controllo a modello virtuale, stato con filtro di Kalman',
  'Il secondo controllore implementa l\'architettura del paper: si disaccoppia il robot in due modelli semplici e si dà a '
  + 'ciascuno il controllore più adatto. Codice in rotino_mpc/controller.py e solvers.py.');
{
  const s = newSlide('CONTENUTO', 4);
  header(s, 'Perché questa architettura');
  const tl = C.accent2;
  block(s, 0.6, 1.6, 2.6, 1.0, 'Riferimenti s, ṡ, z, ż\nsull\'orizzonte (0,5 s)');
  block(s, 3.6, 1.6, 2.7, 1.0, 'MPC corpo superiore\n100 Hz', { fill: tl, color: C.background1, bold: true });
  block(s, 7.0, 1.6, 2.7, 1.0, 'TV-LQR K(l)\n500 Hz', { fill: tl, color: C.background1, bold: true });
  block(s, 10.1, 1.6, 2.63, 1.0, 'Ruote τ_{l}, τ_{r}', { fill: C.text2, color: C.background1, bold: true });
  block(s, 7.0, 3.0, 2.7, 1.0, 'VMC gambe\n500 Hz', { fill: tl, color: C.background1, bold: true });
  block(s, 10.1, 3.0, 2.63, 1.0, 'Gambe τ_{hip}, τ_{knee}', { fill: C.text2, color: C.background1, bold: true });
  block(s, 3.6, 3.0, 2.7, 1.0, 'IMU + encoder\n→ Kalman', { border: tl });
  arrow(s, 3.2, 2.1, 3.6, 2.1);
  arrow(s, 6.3, 2.1, 7.0, 2.1);
  arrow(s, 9.7, 2.1, 10.1, 2.1);
  arrow(s, 9.7, 3.5, 10.1, 3.5);
  arrow(s, 6.3, 3.5, 7.0, 3.5);
  line(s, 6.6, 2.1, 6.6, 3.2, { color: C.accent4, width: 1.5 });
  arrow(s, 6.6, 3.2, 7.0, 3.2);
  text(s, 'Δs, F_{z}, piano CoM', 5.9, 1.2, 2.0, 0.3, { fontSize: 11, color: C.accent4, align: 'center' });
  const cw = 3.85;
  infoCard(s, 0.6, 4.4, cw, 2.0, 'E', 'Equilibrio: veloce e instabile',
    'Modello lineare a parametri variabili (l): LQR ricalcolato in l, una moltiplicazione matrice-vettore.', C.accent2, { badgeSize: 14 });
  infoCard(s, 0.6 + cw + 0.28, 4.4, cw, 2.0, 'P', 'Postura: lenta e vincolata',
    'Dove mettere il CoM e quanta forza chiedere alle gambe: vincoli di attrito, area di lavoro, F_{z}.', C.accent2, { badgeSize: 14 });
  infoCard(s, 0.6 + 2 * (cw + 0.28), 4.4, cw, 2.0, 'V', 'Gambe: modello virtuale',
    'Molla-smorzatore virtuale al piede più forza verticale: niente dinamica inversa in appoggio.', C.accent2);
  s.addNotes('Il paper si colloca fra il pendolo inverso puro, semplice ma che ignora il torso, e il whole-body control, '
    + 'completo ma costoso. Disaccoppia l\'equilibrio, veloce e instabile, dal posizionamento del corpo superiore, più lento '
    + 'ma vincolato. Il primo va all\'LQR a 500 Hz, il secondo all\'MPC a 100 Hz, e il VMC traduce in coppie di giunto.');
}
{
  const s = newSlide('CONTENUTO', 4);
  header(s, 'TV-LQR: pesi letti alla Bryson');
  eq(s, 'lqr', 0.6, 1.5, 5.9, 1.25);
  eq(s, 'pesi_lqr', 0.6, 3.0, 12.1, 0.75);
  table(s, [
    ['Stato', 's', 'θ', 'φ', 'ṡ', 'θ̇', 'φ̇'],
    ['q_{ii}', '30', '400', '80', '15', '6', '2'],
    ['1/√q_{ii}', '0,18 m', '0,050 rad (2,9°)', '0,11 rad', '0,26 m/s', '0,41 rad/s', '0,71 rad/s'],
  ], 0.6, 4.05, 7.6, [1.15, 0.95, 1.55, 0.95, 1.0, 1.0, 1.0], { rowH: 0.42, fontSize: 12, align: ['left', 'center', 'center', 'center', 'center', 'center', 'center'] });
  infoCard(s, 6.85, 1.5, 5.88, 1.3, 'θ', 'Priorità all\'inclinazione',
    'La posizione è accettata a decine di cm: il robot cede in traslazione per non cadere.', C.accent2);
  infoCard(s, 8.5, 4.05, 4.23, 2.35, 'R', 'Differenziale ×50',
    'Una risonanza torsionale delle gambe a 76 rad/s: con R_{d} = 100 l\'imbardata attraversa a 15,4 rad/s.', C.accent2);
  text(s, 'R su modo comune coincide con R = I; il costo si separa in un problema sagittale e uno di imbardata.',
    0.6, 5.55, 7.6, 0.8, { fontSize: 13, italic: true, color: C.accent4 });
  s.addNotes('Il funzionale quadratico si minimizza risolvendo l\'equazione algebrica di Riccati. La lettura alla Bryson dei pesi: '
    + 'uno su radice di q è lo scostamento che costa un\'unità. L\'inclinazione pesa molto più della posizione. R è scritta '
    + 'su modo comune e differenziale: il differenziale costa cinquanta volte di più per non eccitare una risonanza '
    + 'torsionale delle gambe scoperta in simulazione.');
}
{
  const s = newSlide('CONTENUTO', 4);
  header(s, 'Guadagni, poli e scheduling su l');
  eq(s, 'K_lqr', 0.6, 1.5, 7.6, 0.85);
  table(s, [
    ['l [m]', 'K_{θ} [N·m/rad]', 'K_{θ̇}', 'K_{ṡ}', 'Poli sagittali'],
    ['0,219', '21,22', '3,13', '5,51', '−139,9; −8,06; −1,15±0,68j'],
    ['**0,163**', '**20,28**', '**2,78**', '**5,64**', '**−179,5; −8,13; −1,08±0,68j**'],
    ['0,074', '18,43', '2,33', '6,32', '−281,1; −8,16; −0,86±0,63j'],
  ], 0.6, 2.7, 7.6, [1.0, 1.6, 0.9, 0.9, 3.2], { rowH: 0.45, fontSize: 12, align: ['center', 'center', 'center', 'center', 'left'] });
  stat(s, 8.6, 1.45, 4.1, '3,873 = √(30/2)', 'K_{s} non dipende da l: √(q_{s}/r_{c})', C.accent2, { size: 26 });
  stat(s, 8.6, 3.0, 4.1, '−1,08 ± 0,68j', 'ritorno in posizione (ζ = 0,85): la dinamica che si vede dopo una spinta', C.accent2, { size: 26 });
  infoCard(s, 0.6, 4.75, 12.13, 1.65, 'l', 'Comportamento invariante con l\'altezza',
    'LQRSchedule calcola K su 25 pose (ciascuna con il suo I_{y}) e interpola linearmente in l: i poli sagittali restano quasi fermi. '
    + 'Vale finché l varia lentamente: non durante la spinta del salto, dove il modello ignora l̇.', C.accent2);
  s.addNotes('Il guadagno su s coincide con la radice del rapporto fra i pesi, come in un doppio integratore. La tabella mostra '
    + 'che lungo l\'altezza i poli lenti e quello di inclinazione cambiano poco: è lo scopo dello scheduling. Il polo veloce '
    + 'a −180 rad/s sta oltre la banda del filtro su θ̇, quindi nel sistema reale è il filtro a determinarlo.');
}
{
  const s = newSlide('CONTENUTO', 4);
  header(s, 'MPC del corpo superiore');
  eq(s, 'mpc_modelli', 0.6, 1.5, 6.4, 2.1);
  eq(s, 'mpc_qp', 0.6, 3.85, 12.1, 0.7, { align: 'center' });
  stat(s, 7.5, 1.4, 2.5, '25 × 20 ms', 'orizzonte di 0,5 s, aggiornato a 100 Hz', C.accent2, { size: 26 });
  stat(s, 10.2, 1.4, 2.5, '60 iter.', 'FISTA con vincoli di box, partenza dalla soluzione precedente', C.accent2, { size: 26 });
  const cw = 5.9;
  infoCard(s, 0.6, 4.8, cw, 1.6, 'h', 'Orizzontale', 'S_{h} = diag(50, 20), W_{h} = 200: 1 cm di Δs costa quanto 2 cm di errore.', C.accent2);
  infoCard(s, 0.6 + cw + 0.33, 4.8, cw, 1.6, 'v', 'Verticale', 'S_{v} = diag(5000, 150), W_{v} = 2·10^{-3}: quota rigida, il VMC non ha molla verticale.', C.accent2);
  s.addNotes('L\'MPC predice il baricentro con due doppi integratori indipendenti, ottenuti congelando il coefficiente (g + z̈)/h '
    + 'sull\'orizzonte. Ingressi: lo spostamento Δs del CoM rispetto al contatto e la forza verticale F_z. La QP condensata ha '
    + 'solo vincoli di box, quindi basta un gradiente proiettato accelerato. Rispetto al paper la discretizzazione è esatta '
    + 'invece che di Eulero e le QP sono due, disaccoppiate.');
}
{
  const s = newSlide('CONTENUTO', 4);
  header(s, 'Vincoli e riferimento per l\'LQR');
  eq(s, 'mpc_vincoli', 0.6, 1.5, 12.1, 0.75, { align: 'center' });
  eq(s, 'xref', 0.6, 2.5, 12.1, 0.6, { align: 'center' });
  stat(s, 0.6, 3.4, 3.8, '3 cm → 10,48°', 'il tetto su Δs è il vincolo sempre attivo', C.accent2, { size: 28 });
  stat(s, 4.75, 3.4, 3.8, '1,13 m/s²', 'massima accelerazione a regime che il tetto consente', C.accent2, { size: 28 });
  stat(s, 8.9, 3.4, 3.8, '0,3 m_{b}g', 'forza minima: il contatto non si perde mai nel piano', C.accent2, { size: 28 });
  infoCard(s, 0.6, 5.0, 12.13, 1.4, 'Δs', 'Uno spostamento, due esecutori',
    'Δs (filtrato a 19 ms) diventa θ_{ref} e arretramento dell\'asse per l\'LQR, e posizione del piede rispetto al CoM per il VMC: ruote e gambe realizzano lo stesso spostamento.',
    C.accent2, { badgeSize: 12 });
  s.addNotes('Il vincolo d\'attrito del paper, μ per h, vale 36 cm: con μ = 2,2 permetterebbe 65° di inclinazione. Il vincolo '
    + 'che conta è il tetto fisso di 3 cm, aggiunto nell\'implementazione, che limita θ_ref a circa 10°. L\'LQR non insegue '
    + 'il riferimento grezzo ma il piano dell\'MPC, quindi il suo riferimento è già compatibile con i limiti fisici.');
}
{
  const s = newSlide('CONTENUTO', 4);
  header(s, 'VMC, stima dello stato e imbardata');
  eq(s, 'vmc', 0.6, 1.5, 12.1, 0.6, { align: 'center' });
  eq(s, 'kalman', 0.6, 2.35, 6.6, 1.2);
  infoCard(s, 7.5, 2.3, 5.23, 1.6, 'K', 'Kalman per asse',
    'Tempi di fusione 45 ms (IMU) e 280 ms (odometria); correzione solo in appoggio.', C.accent2);
  const cw = 3.85;
  infoCard(s, 0.6, 4.25, cw, 2.15, 'z', 'K_{p,z} = 0 in appoggio',
    'La quota la regola l\'MPC tramite F_{z}: una molla in parallelo sarebbe un secondo regolatore sulla stessa grandezza.', C.accent2);
  infoCard(s, 0.6 + cw + 0.28, 4.25, cw, 2.15, 'x', 'Piede rapido',
    'K_{p,x} = 1500 N/m, K_{d} = 40 N·s/m: il piede converge in 27 ms, molto prima del moto del CoM.', C.accent2);
  infoCard(s, 0.6 + 2 * (cw + 0.28), 4.25, cw, 2.15, 'ψ', 'Imbardata',
    'Riga φ del TV-LQR più attrito di Coulomb e viscoso e integrale con zona morta (0,035 rad).', C.accent2);
  s.addNotes('Il VMC tratta ogni gamba come una molla-smorzatore virtuale fra anca e ruota, più metà della forza verticale '
    + 'pianificata. In verticale non c\'è molla perché la quota la decide l\'MPC. Lo stato arriva da IMU ed encoder, fusi '
    + 'da tre filtri di Kalman scalari: l\'IMU domina sulle alte frequenze, l\'odometria sulle basse.');
}
{
  const s = newSlide('CONTENUTO', 4);
  header(s, 'Valutazioni su MPC + TV-LQR');
  card(s, 0.6, 1.5, 5.9, 4.6);
  text(s, 'Punti di forza', 0.85, 1.65, 5.4, 0.45, { fontSize: 18, bold: true, color: C.accent2 });
  bullets(s, [
    'Anticipo sui riferimenti: 0,5 s di orizzonte in un sistema a fase non minima',
    'Vincoli rispettati nel piano, non saturati a posteriori',
    'Un\'unica sintesi per F_{z}: appoggio, quota e spinta del salto',
    'Stima realistica (IMU + Kalman), trasferibile a un robot vero',
  ], 0.85, 2.25, 5.45, 3.7, { fontSize: 15, gap: 10 });
  card(s, 6.83, 1.5, 5.9, 4.6);
  text(s, 'Limiti e spunti', 7.08, 1.65, 5.4, 0.45, { fontSize: 18, bold: true, color: C.accent1 });
  bullets(s, [
    'Nessuna azione integrale sul sagittale: con 3 N costanti l\'errore a regime è 0,65 m (modello ridotto)',
    'Predizione senza beccheggio: l\'MPC non sa che spostare il CoM richiede prima di inclinare il pendolo',
    'Il tetto di 3 cm rende inattivo il vincolo d\'attrito del paper e limita la decelerazione',
    'Spunti: stato integrale o modello del disturbo nella QP; vincolo di inclinazione funzione di h; predizione sul VL-WIP completo',
  ], 7.08, 2.25, 5.45, 3.8, { fontSize: 15, gap: 8 });
  s.addNotes('Valutazione personale. Il punto di forza è la gestione esplicita dei vincoli e del futuro. Il limite principale è '
    + 'l\'assenza di azione integrale sul piano sagittale: un disturbo persistente lascia un errore. Il modello di '
    + 'predizione, un punto materiale, non contiene il beccheggio, che resta tutto all\'LQR. Gli spunti indicano come '
    + 'chiudere questi limiti senza cambiare l\'architettura.');
}

// ================================================================== 5. confronto (solo struttura)
sectionSlide(5, '5', 'Confronto PID – MPC',
  'Stesso URDF, stesso mondo Gazebo, stessa attuazione in coppia a 500 Hz, stessi argomenti di scenario: cambia solo il nodo del controllore. Sezione impostata: i grafici sono da inserire.',
  'Metodo: robot.launch.py avvia simulazione, robot e scenario, e riceve dal pacchetto di controllo solo il nome '
  + 'dell\'eseguibile. Il confronto si rigenera con ros2 run rotino_benchmark suite. In questa sezione ci sono la sintesi '
  + 'delle architetture e la struttura delle slide con i grafici, da completare.');
{
  const s = newSlide('CONTENUTO', 5);
  header(s, 'Le due architetture a confronto');
  table(s, [
    ['', 'PID con ZMP', 'MPC + TV-LQR + VMC'],
    ['Stato per l\'equilibrio', 'ground truth, derivate non filtrate', 'IMU + Kalman + cinematica'],
    ['Legge sulle ruote', 'PI sul Capture Point → ZMP → PD', 'LQR schedulato su l'],
    ['Uso del modello', 'feedforward VL-WIP, anteprima LIPM', 'LQR (VL-WIP) + MPC (massa concentrata)'],
    ['Traslazione', 'anteprima LIPM + integrale sul Capture Point', 'MPC con anteprima + LQR sul piano'],
    ['Limite di inclinazione', 'offset ZMP ≤ a_{max}/ω² (3 m/s²)', 'Δs ≤ 3 cm → 10,48°'],
    ['Imbardata', 'PID + attrito', 'LQR (R_{d} = 100) + attrito + integrale'],
    ['Gambe', 'PD cartesiano, 1500 / 5000 N/m', 'VMC, K_{p,z} = 0, sostegno da F_{z}'],
    ['Azione integrale', 'Capture Point, imbardata', 'solo imbardata'],
    ['Poli sagittali', '−0,5; −2,3±0,2j; −9,8; −52', '−8,13; −1,08±0,68j'],
  ], 0.6, 1.5, 12.13, [2.9, 4.6, 4.63], { rowH: 0.47, fontSize: 13, headFill: C.text2 });
  s.addNotes('Sintesi della sezione 5 di Tecniche_di_controllo.md. Le differenze principali: il PID lavora sullo ZMP con '
    + 'un\'azione integrale e lo stato ideale, l\'MPC lavora con vincoli espliciti e uno stato stimato. Nel leggere i grafici '
    + 'va ricordato che le condizioni di stima non sono le stesse.');
}
const charts = [
  ['Disturbi: spinta sul torso e salto', [
    ['spinta/plots/disturbo_e_fase.png', 'beccheggio di picco, recupero, errore di posizione max'],
    ['salto/plots/assetto.png', 'salita del CoM, beccheggio di picco, recupero'],
  ], 'Spinta: impulso di 2,7 N·s all\'indietro sul torso a 4 s dal rilascio. Salto verticale a 4 s, poi equilibrio.'],
  ['Inseguimento: trapezio e va e vieni', [
    ['trapezio/plots/inseguimento.png', 'errore di posizione e di velocità rms, beccheggio di picco'],
    ['va_e_vieni/plots/inseguimento.png', 'errore di posizione rms, coppia ruote rms'],
  ], 'Trapezio: 2 m a 1 m/s con 0,6 m/s². Va e vieni: 1 m avanti e indietro in 8 s.'],
  ['Curve a S: inseguimento e ZMP laterale', [
    ['curva_S_veloce/plots/zmp_lateral.png', 'ZMP laterale max / (d/2), carico minimo sulla ruota'],
    ['curva_S/plots/inseguimento.png', 'errore di posizione rms, beccheggio di picco'],
  ], 'S di 3 m con scarto laterale di 0,6 m in 12 s, e versione veloce: 1 m in 5 s.'],
  ['Equilibrio, altezza e bilancio finale', [
    ['equilibrio/plots/coppie_ruote.png', 'beccheggio rms a regime, coppia ruote rms'],
    ['altezza/plots/assetto.png', 'oscillazione di quota rms, beccheggio rms'],
    ['riepilogo.png', 'bilancio sulle 31 metriche chiave'],
  ], 'Equilibrio da fermo dopo il rilascio; altezza sinusoidale ±3 cm con periodo 2,2 s.'],
];
for (const [title, plots, scen] of charts) {
  const s = newSlide('CONTENUTO', 5);
  header(s, title);
  text(s, scen, 0.6, 1.4, 9.5, 0.45, { fontSize: 13, color: C.accent4, italic: true });
  legendPidMpc(s, 10.6, 1.48);
  const n = plots.length, gap = 0.3, w = (12.13 - gap * (n - 1)) / n;
  plots.forEach(([f, m], i) => placeholderChart(s, 0.6 + i * (w + gap), 2.0, w, 3.5, f, m));
  keyMessage(s, 0.6, 5.75, 12.13, 0.75);
  s.addNotes('Slide di struttura: inserire i grafici indicati da benchmark_runs/suite_pid_mpc e scrivere il messaggio chiave. '
    + 'Nei grafici del benchmark il PID è disegnato in rosso e l\'MPC in blu; la legenda della slide segue la palette della '
    + 'presentazione (PID rosso, MPC ottanio).');
}

// pptxgenjs scrive un <a:pPr> per ogni run di un paragrafo (dopo il primo vale solo l'ultimo, che ha buNone):
// tengo solo il primo, che porta il pallino e le spaziature
async function dedupeParagraphProps(file) {
  const JSZip = require('jszip');
  const zip = await JSZip.loadAsync(fs.readFileSync(file));
  const pPr = /<a:pPr\b[^>]*?(?:\/>|>[\s\S]*?<\/a:pPr>)/g;
  let fixed = 0;
  for (const name of Object.keys(zip.files).filter((n) => /^ppt\/slides\/slide\d+\.xml$/.test(n))) {
    const xml = await zip.file(name).async('string');
    const out = xml.replace(/<a:p>([\s\S]*?)<\/a:p>/g, (m, body) => {
      let seen = false;
      const b = body.replace(pPr, (p) => {
        if (!seen) { seen = true; return p; }
        fixed += 1;
        return '';
      });
      return '<a:p>' + b + '</a:p>';
    });
    zip.file(name, out);
  }
  fs.writeFileSync(file, await zip.generateAsync({ type: 'nodebuffer', compression: 'DEFLATE' }));
  return fixed;
}

(async () => {
  await pres.writeFile({ fileName: OUT });
  console.log('pPr duplicati rimossi:', await dedupeParagraphProps(OUT));
  await applyTheme(OUT, THEME);
  console.log('Scritto', OUT);
})();
