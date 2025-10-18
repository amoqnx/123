"use strict";

// ====== Utility: parsing/formatting ======
const EPS = 1e-9;

function parseNum(value) {
  if (value === undefined || value === null) return NaN;
  const v = String(value).replace(",", ".").trim();
  if (v === "") return NaN;
  const n = Number(v);
  return Number.isFinite(n) ? n : NaN;
}

function fmt(num, digits = 4) {
  if (!Number.isFinite(num)) return "NaN";
  const s = num.toFixed(digits);
  // Remove trailing zeros and possible dangling dot
  return s.replace(/\.0+$/, "").replace(/(\.[0-9]*?)0+$/, "$1");
}

function deg(rad) { return rad * 180 / Math.PI; }
function rad(degVal) { return degVal * Math.PI / 180; }
function nearZero(x, eps = EPS) { return Math.abs(x) <= eps; }

// ====== Linear algebra utilities ======
function vector(x, y) { return { x, y }; }
function add(u, v) { return { x: u.x + v.x, y: u.y + v.y }; }
function scale(v, k) { return { x: v.x * k, y: v.y * k }; }
function dot(u, v) { return u.x * v.x + u.y * v.y; }
function cross2(u, v) { return u.x * v.y - u.y * v.x; }
function length(v) { return Math.hypot(v.x, v.y); }
function angleBetween(u, v) {
  const nu = length(u), nv = length(v);
  if (nearZero(nu) || nearZero(nv)) return NaN;
  const c = dot(u, v) / (nu * nv);
  // Clamp for numerical stability
  const clamped = Math.min(1, Math.max(-1, c));
  return Math.acos(clamped);
}
function isCollinear(u, v) { return Math.abs(cross2(u, v)) <= EPS; }
function isPerpendicular(u, v) { return Math.abs(dot(u, v)) <= EPS; }

function vectorFromPoints(A, B) { return { x: B.x - A.x, y: B.y - A.y }; }

// ====== Lines (Ax + By + C = 0) ======
function generalFromTwoPoints(A, B) {
  const Acoef = A.y - B.y;
  const Bcoef = B.x - A.x;
  const Ccoef = A.x * B.y - B.x * A.y;
  return normalizeLine({ A: Acoef, B: Bcoef, C: Ccoef });
}
function generalFromPointAndDir(P, v) {
  // Direction v => normal n = (-v.y, v.x)
  const Acoef = -v.y;
  const Bcoef = v.x;
  const Ccoef = v.y * P.x - v.x * P.y; // from n·(x - P) = 0
  return normalizeLine({ A: Acoef, B: Bcoef, C: Ccoef });
}
function generalPerpThroughPoint(P, v) {
  // Perpendicular to v => normal is v
  const Acoef = v.x;
  const Bcoef = v.y;
  const Ccoef = -(v.x * P.x + v.y * P.y);
  return normalizeLine({ A: Acoef, B: Bcoef, C: Ccoef });
}
function generalFromSlopeIntercept(k, b) {
  // y = kx + b => kx - y + b = 0
  return normalizeLine({ A: k, B: -1, C: b });
}
function generalFromIntercepts(ax, by) {
  // x/a + y/b = 1 => bx + ay - ab = 0
  const Acoef = by;
  const Bcoef = ax;
  const Ccoef = -ax * by;
  return normalizeLine({ A: Acoef, B: Bcoef, C: Ccoef });
}

function normalizeLine(L) {
  const { A, B, C } = L;
  const s = Math.hypot(A, B);
  if (nearZero(s)) return { A, B, C };
  let AA = A / s, BB = B / s, CC = C / s;
  // Make the first non-zero among (A,B) positive for consistency
  if (AA < -EPS || (nearZero(AA) && BB < -EPS)) {
    AA = -AA; BB = -BB; CC = -CC;
  }
  return { A: AA, B: BB, C: CC };
}

function slopeFromGeneral(L) {
  if (nearZero(L.B)) return { k: Infinity, b: NaN };
  const k = -L.A / L.B;
  const b = -L.C / L.B;
  return { k, b };
}
function interceptsFromGeneral(L) {
  const ax = nearZero(L.A) ? NaN : -L.C / L.A;
  const by = nearZero(L.B) ? NaN : -L.C / L.B;
  return { ax, by };
}
function areParallel(L1, L2) {
  return nearZero(L1.A * L2.B - L2.A * L1.B);
}
function arePerpendicular(L1, L2) {
  return nearZero(L1.A * L2.A + L1.B * L2.B);
}
function intersection(L1, L2) {
  const D = L1.A * L2.B - L2.A * L1.B;
  if (nearZero(D)) return null;
  const Dx = -L1.C * L2.B - (-L2.C) * L1.B;
  const Dy = L1.A * -L2.C - L2.A * -L1.C;
  const x = Dx / D;
  const y = Dy / D;
  return { x, y };
}
function angleBetweenLines(L1, L2) {
  const num = Math.abs(L1.A * L2.B - L2.A * L1.B);
  const den = L1.A * L2.A + L1.B * L2.B;
  if (nearZero(den)) return Math.PI / 2; // 90°
  return Math.atan(num / Math.abs(den));
}

// ====== Plotting (Canvas 2D) ======
class Plot2D {
  /**
   * @param {HTMLCanvasElement} canvas
   */
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.setDefaultWorld();
    this.adjustDPI();
  }

  setDefaultWorld() {
    this.xmin = -10; this.xmax = 10;
    this.ymin = -10; this.ymax = 10;
  }

  adjustDPI() {
    const dpr = window.devicePixelRatio || 1;
    const rect = this.canvas.getBoundingClientRect();
    this.canvas.width = Math.round(rect.width * dpr);
    this.canvas.height = Math.round(rect.height * dpr);
    this.ctx.setTransform(1, 0, 0, 1, 0, 0);
    this.ctx.scale(dpr, dpr);
  }

  fitToPoints(points, pad = 1) {
    const xs = points.map(p => p.x);
    const ys = points.map(p => p.y);
    let xmin = Math.min(...xs, -1), xmax = Math.max(...xs, 1);
    let ymin = Math.min(...ys, -1), ymax = Math.max(...ys, 1);
    if (!isFinite(xmin) || !isFinite(xmax) || xmin === xmax) { xmin = -10; xmax = 10; }
    if (!isFinite(ymin) || !isFinite(ymax) || ymin === ymax) { ymin = -10; ymax = 10; }
    const dx = xmax - xmin, dy = ymax - ymin;
    const padX = dx * 0.1 + pad; const padY = dy * 0.1 + pad;
    this.xmin = xmin - padX; this.xmax = xmax + padX;
    this.ymin = ymin - padY; this.ymax = ymax + padY;
  }

  worldToScreen(x, y) {
    const rect = this.canvas.getBoundingClientRect();
    const w = rect.width, h = rect.height;
    const sx = (x - this.xmin) / (this.xmax - this.xmin) * w;
    const sy = h - (y - this.ymin) / (this.ymax - this.ymin) * h;
    return { x: sx, y: sy };
  }

  clear() {
    const rect = this.canvas.getBoundingClientRect();
    this.ctx.clearRect(0, 0, rect.width, rect.height);
    this.drawGrid();
    this.drawAxes();
  }

  drawGrid() {
    const rect = this.canvas.getBoundingClientRect();
    const ctx = this.ctx;
    ctx.save();
    ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--grid') || '#2a3250';
    ctx.lineWidth = 1;
    ctx.globalAlpha = 0.6;

    const niceStep = (range) => {
      const raw = range / 10; // target ~10 lines
      const pow10 = Math.pow(10, Math.floor(Math.log10(raw)));
      const steps = [1, 2, 5, 10];
      for (const s of steps) {
        if (raw <= s * pow10) return s * pow10;
      }
      return 10 * pow10;
    };

    const stepX = niceStep(this.xmax - this.xmin);
    const stepY = niceStep(this.ymax - this.ymin);

    ctx.beginPath();
    for (let x = Math.ceil(this.xmin / stepX) * stepX; x <= this.xmax; x += stepX) {
      const p1 = this.worldToScreen(x, this.ymin);
      const p2 = this.worldToScreen(x, this.ymax);
      ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y);
    }
    for (let y = Math.ceil(this.ymin / stepY) * stepY; y <= this.ymax; y += stepY) {
      const p1 = this.worldToScreen(this.xmin, y);
      const p2 = this.worldToScreen(this.xmax, y);
      ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y);
    }
    ctx.stroke();
    ctx.restore();
  }

  drawAxes() {
    const ctx = this.ctx;
    ctx.save();
    ctx.strokeStyle = 'rgba(255,255,255,0.55)';
    ctx.lineWidth = 1.3;

    // X axis
    const x0 = this.worldToScreen(this.xmin, 0);
    const x1 = this.worldToScreen(this.xmax, 0);
    ctx.beginPath(); ctx.moveTo(x0.x, x0.y); ctx.lineTo(x1.x, x1.y); ctx.stroke();

    // Y axis
    const y0 = this.worldToScreen(0, this.ymin);
    const y1 = this.worldToScreen(0, this.ymax);
    ctx.beginPath(); ctx.moveTo(y0.x, y0.y); ctx.lineTo(y1.x, y1.y); ctx.stroke();
    ctx.restore();
  }

  drawPoint(p, color = '#fff', radius = 4, label = '') {
    const ctx = this.ctx;
    const s = this.worldToScreen(p.x, p.y);
    ctx.save();
    ctx.fillStyle = color; ctx.strokeStyle = 'rgba(0,0,0,0.5)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(s.x, s.y, radius, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    if (label) {
      ctx.fillStyle = 'rgba(255,255,255,0.9)';
      ctx.font = '12px Inter, sans-serif';
      ctx.fillText(label, s.x + 6, s.y - 6);
    }
    ctx.restore();
  }

  drawSegment(p1, p2, color = '#fff', width = 2) {
    const ctx = this.ctx;
    const s1 = this.worldToScreen(p1.x, p1.y);
    const s2 = this.worldToScreen(p2.x, p2.y);
    ctx.save();
    ctx.strokeStyle = color; ctx.lineWidth = width;
    ctx.beginPath(); ctx.moveTo(s1.x, s1.y); ctx.lineTo(s2.x, s2.y); ctx.stroke();
    ctx.restore();
  }

  drawVector(origin, vec, color = '#fff', width = 3) {
    const end = { x: origin.x + vec.x, y: origin.y + vec.y };
    this.drawSegment(origin, end, color, width);
    // Arrow head
    const ah = 10; // px
    const aw = 6;  // px
    const sO = this.worldToScreen(origin.x, origin.y);
    const sE = this.worldToScreen(end.x, end.y);
    const ang = Math.atan2(sE.y - sO.y, sE.x - sO.x);
    const ctx = this.ctx;
    ctx.save();
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(sE.x, sE.y);
    ctx.lineTo(sE.x - ah * Math.cos(ang - Math.PI / 8), sE.y - ah * Math.sin(ang - Math.PI / 8));
    ctx.lineTo(sE.x - ah * Math.cos(ang + Math.PI / 8), sE.y - ah * Math.sin(ang + Math.PI / 8));
    ctx.closePath(); ctx.fill();
    ctx.restore();
  }

  drawLineGeneral(L, color = 'rgba(255,255,255,0.9)', width = 2) {
    // Find intersections with canvas bounds in world coords
    const xs = [this.xmin, this.xmax];
    const ys = [this.ymin, this.ymax];
    const points = [];

    // x = const => solve A*x + B*y + C = 0 => y = (-A*x - C)/B
    for (const x of xs) {
      if (!nearZero(L.B)) {
        const y = (-L.A * x - L.C) / L.B;
        points.push({ x, y });
      }
    }
    // y = const => solve for x
    for (const y of ys) {
      if (!nearZero(L.A)) {
        const x = (-L.B * y - L.C) / L.A;
        points.push({ x, y });
      }
    }

    // Keep only points inside bounds and take extremes along the line
    const inside = points.filter(p => p.x >= this.xmin - 1e-6 && p.x <= this.xmax + 1e-6 && p.y >= this.ymin - 1e-6 && p.y <= this.ymax + 1e-6);
    if (inside.length < 2) return;

    // Pick two farthest points to draw full segment
    let bestI = 0, bestJ = 1, bestD = -1;
    for (let i = 0; i < inside.length; i++) {
      for (let j = i + 1; j < inside.length; j++) {
        const d = (inside[i].x - inside[j].x) ** 2 + (inside[i].y - inside[j].y) ** 2;
        if (d > bestD) { bestD = d; bestI = i; bestJ = j; }
      }
    }
    this.drawSegment(inside[bestI], inside[bestJ], color, width);
  }
}

// ====== UI Builders ======
function buildNumberInput(id, label, placeholder = '', value = '') {
  return `\n    <div class="form-row">\n      <label for="${id}">${label}</label>\n      <input type="text" inputmode="decimal" id="${id}" placeholder="${placeholder}" value="${value}"/>\n    </div>\n  `;
}

function buildPointInputs(prefix, label, defaults = { x: '', y: '' }) {
  return `\n    <div class="form-row">\n      <div class="input-grid">\n        <div>\n          <label for="${prefix}x">${label} x</label>\n          <input type="text" inputmode="decimal" id="${prefix}x" placeholder="например, 1.5" value="${defaults.x}"/>\n        </div>\n        <div>\n          <label for="${prefix}y">${label} y</label>\n          <input type="text" inputmode="decimal" id="${prefix}y" placeholder="например, -2" value="${defaults.y}"/>\n        </div>\n      </div>\n    </div>\n  `;
}

function buildVectorInputs(prefix, label, defaults = { x: '', y: '' }) {
  return `\n    <div class="form-row">\n      <div class="input-grid">\n        <div>\n          <label for="${prefix}x">${label} x</label>\n          <input type="text" inputmode="decimal" id="${prefix}x" placeholder="например, 3" value="${defaults.x}"/>\n        </div>\n        <div>\n          <label for="${prefix}y">${label} y</label>\n          <input type="text" inputmode="decimal" id="${prefix}y" placeholder="например, 4" value="${defaults.y}"/>\n        </div>\n      </div>\n    </div>\n  `;
}

function buildLineABCInputs(prefix, label, defaults = { A: '', B: '', C: '' }) {
  return `\n    <div class="form-row">\n      <div class="input-3col">\n        <div>\n          <label for="${prefix}A">${label} A</label>\n          <input type="text" inputmode="decimal" id="${prefix}A" placeholder="A" value="${defaults.A}"/>\n        </div>\n        <div>\n          <label for="${prefix}B">${label} B</label>\n          <input type="text" inputmode="decimal" id="${prefix}B" placeholder="B" value="${defaults.B}"/>\n        </div>\n        <div>\n          <label for="${prefix}C">${label} C</label>\n          <input type="text" inputmode="decimal" id="${prefix}C" placeholder="C" value="${defaults.C}"/>\n        </div>\n      </div>\n      <div class="help">Формат: \(Ax + By + C = 0\).</div>\n    </div>\n  `;
}

function buildRatioInputs(prefix, labelM = 'm', labelN = 'n', defaults = { m: '', n: '' }) {
  return `\n    <div class="form-row">\n      <div class="input-grid">\n        <div>\n          <label for="${prefix}m">${labelM}</label>\n          <input type="text" inputmode="decimal" id="${prefix}m" placeholder="например, 1" value="${defaults.m}"/>\n        </div>\n        <div>\n          <label for="${prefix}n">${labelN}</label>\n          <input type="text" inputmode="decimal" id="${prefix}n" placeholder="например, 2" value="${defaults.n}"/>\n        </div>\n      </div>\n      <div class="help">Отношение \(m:n\). Для внешнего деления используйте отрицательные значения.</div>\n    </div>\n  `;
}

// ====== MathJax helper ======
async function typeset(container) {
  if (window.MathJax && window.MathJax.typesetPromise) {
    await window.MathJax.typesetPromise([container]);
  }
}

// ====== Vector Calculator Logic ======
const vectorOperationConfigs = {
  vector_from_points: {
    name: 'Координаты вектора по двум точкам',
    inputs: () => (
      buildPointInputs('A_', 'Точка A', { x: '1', y: '2' }) +
      buildPointInputs('B_', 'Точка B', { x: '5', y: '4' })
    ),
    compute: (vals) => {
      const A = { x: parseNum(vals['A_x']), y: parseNum(vals['A_y']) };
      const B = { x: parseNum(vals['B_x']), y: parseNum(vals['B_y']) };
      if (!isFinite(A.x) || !isFinite(A.y) || !isFinite(B.x) || !isFinite(B.y)) throw new Error('Введите корректные координаты точек A и B.');
      const vAB = vectorFromPoints(A, B);
      const formula = `\\[ \\overrightarrow{AB} = (x_B - x_A, \\ y_B - y_A) = (${fmt(B.x)} - ${fmt(A.x)},\\ ${fmt(B.y)} - ${fmt(A.y)}) = (${fmt(vAB.x)},\\ ${fmt(vAB.y)}) \\]`;
      const explain = `Вектор \\( \\overrightarrow{AB} \\) направлен от точки A к точке B.`;
      const result = `\\( \\overrightarrow{AB} = (${fmt(vAB.x)},\\ ${fmt(vAB.y)}) \\)`;
      const draw = (plot) => {
        plot.fitToPoints([A, B]); plot.clear();
        plot.drawPoint(A, '#fff', 4, 'A');
        plot.drawPoint(B, '#fff', 4, 'B');
        plot.drawVector(A, vAB, getCssVar('--chip-res'));
      };
      return { formula, explain, result, draw };
    }
  },
  length: {
    name: 'Длина (модуль) вектора',
    inputs: () => buildVectorInputs('V_', 'Вектор v', { x: '3', y: '4' }),
    compute: (vals) => {
      const v = { x: parseNum(vals['V_x']), y: parseNum(vals['V_y']) };
      if (!isFinite(v.x) || !isFinite(v.y)) throw new Error('Введите корректные координаты вектора v.');
      const L = length(v);
      const formula = `\\[ |\\vec v| = \\sqrt{x^2 + y^2} = \\sqrt{(${fmt(v.x)})^2 + (${fmt(v.y)})^2} = ${fmt(L)} \\]`;
      const explain = 'Длина вектора равна квадратному корню из суммы квадратов его координат.';
      const result = `\\( |\\vec v| = ${fmt(L)} \\)`;
      const draw = (plot) => {
        plot.fitToPoints([{ x: 0, y: 0 }, v]); plot.clear();
        plot.drawVector({ x: 0, y: 0 }, v, getCssVar('--chip-res'));
      };
      return { formula, explain, result, draw };
    }
  },
  add: {
    name: 'Сложение векторов',
    inputs: () => (
      buildVectorInputs('U_', 'Вектор u', { x: '2', y: '1' }) +
      buildVectorInputs('V_', 'Вектор v', { x: '1', y: '3' })
    ),
    compute: (vals) => {
      const u = { x: parseNum(vals['U_x']), y: parseNum(vals['U_y']) };
      const v = { x: parseNum(vals['V_x']), y: parseNum(vals['V_y']) };
      if (![u.x,u.y,v.x,v.y].every(Number.isFinite)) throw new Error('Введите корректные координаты векторов u и v.');
      const s = add(u, v);
      const formula = `\\[ \\vec u + \\vec v = (u_x + v_x,\\ u_y + v_y) = (${fmt(u.x)} + ${fmt(v.x)},\\ ${fmt(u.y)} + ${fmt(v.y)}) = (${fmt(s.x)},\\ ${fmt(s.y)}) \\]`;
      const explain = 'Сумма векторов складывается по координатам.';
      const result = `\\( \\vec u + \\vec v = (${fmt(s.x)},\\ ${fmt(s.y)}) \\)`;
      const draw = (plot) => {
        plot.fitToPoints([{ x: 0, y: 0 }, u, v, s]); plot.clear();
        plot.drawVector({ x: 0, y: 0 }, u, getCssVar('--chip-u'));
        plot.drawVector({ x: 0, y: 0 }, v, getCssVar('--chip-v'));
        plot.drawVector({ x: 0, y: 0 }, s, getCssVar('--chip-res'));
      };
      return { formula, explain, result, draw };
    }
  },
  scale: {
    name: 'Умножение вектора на число',
    inputs: () => (
      buildVectorInputs('V_', 'Вектор v', { x: '2', y: '-1' }) +
      buildNumberInput('K', 'Число k', 'например, 2', '2')
    ),
    compute: (vals) => {
      const v = { x: parseNum(vals['V_x']), y: parseNum(vals['V_y']) };
      const k = parseNum(vals['K']);
      if (!Number.isFinite(k) || ![v.x,v.y].every(Number.isFinite)) throw new Error('Введите корректные данные.');
      const kv = scale(v, k);
      const formula = `\\[ k\\,\\vec v = (k x,\\ k y) = (${fmt(k)}\\cdot${fmt(v.x)},\\ ${fmt(k)}\\cdot${fmt(v.y)}) = (${fmt(kv.x)},\\ ${fmt(kv.y)}) \\]`;
      const explain = 'При умножении вектора на число каждая координата умножается на это число.';
      const result = `\\( ${fmt(k)}\\,\\vec v = (${fmt(kv.x)},\\ ${fmt(kv.y)}) \\)`;
      const draw = (plot) => {
        plot.fitToPoints([{ x: 0, y: 0 }, v, kv]); plot.clear();
        plot.drawVector({ x: 0, y: 0 }, v, getCssVar('--chip-u'));
        plot.drawVector({ x: 0, y: 0 }, kv, getCssVar('--chip-res'));
      };
      return { formula, explain, result, draw };
    }
  },
  dot: {
    name: 'Скалярное произведение',
    inputs: () => (
      buildVectorInputs('U_', 'Вектор u', { x: '1', y: '2' }) +
      buildVectorInputs('V_', 'Вектор v', { x: '3', y: '4' })
    ),
    compute: (vals) => {
      const u = { x: parseNum(vals['U_x']), y: parseNum(vals['U_y']) };
      const v = { x: parseNum(vals['V_x']), y: parseNum(vals['V_y']) };
      if (![u.x,u.y,v.x,v.y].every(Number.isFinite)) throw new Error('Введите корректные координаты векторов u и v.');
      const d = dot(u, v);
      const formula = `\\[ \\vec u \\cdot \\vec v = u_x v_x + u_y v_y = ${fmt(u.x)}\\cdot${fmt(v.x)} + ${fmt(u.y)}\\cdot${fmt(v.y)} = ${fmt(d)} \\]`;
      const explain = 'Скалярное произведение равно сумме произведений соответствующих координат.';
      const result = `\\( \\vec u \\cdot \\vec v = ${fmt(d)} \\)`;
      const draw = (plot) => {
        plot.fitToPoints([{ x: 0, y: 0 }, u, v]); plot.clear();
        plot.drawVector({ x: 0, y: 0 }, u, getCssVar('--chip-u'));
        plot.drawVector({ x: 0, y: 0 }, v, getCssVar('--chip-v'));
      };
      return { formula, explain, result, draw };
    }
  },
  angle: {
    name: 'Угол между двумя векторами',
    inputs: () => (
      buildVectorInputs('U_', 'Вектор u', { x: '1', y: '0' }) +
      buildVectorInputs('V_', 'Вектор v', { x: '1', y: '1' })
    ),
    compute: (vals) => {
      const u = { x: parseNum(vals['U_x']), y: parseNum(vals['U_y']) };
      const v = { x: parseNum(vals['V_x']), y: parseNum(vals['V_y']) };
      if (![u.x,u.y,v.x,v.y].every(Number.isFinite)) throw new Error('Введите корректные координаты векторов u и v.');
      const theta = angleBetween(u, v);
      if (!Number.isFinite(theta)) throw new Error('Угол не определён для нулевого вектора.');
      const degVal = deg(theta);
      const formula = `\\[ \\cos\\,\\theta = \\frac{\\vec u\\cdot\\vec v}{|\\vec u|\\,|\\vec v|} = \\frac{${fmt(dot(u,v))}}{${fmt(length(u))}\\cdot${fmt(length(v))}} \n \\Rightarrow\\ \n\\theta = \\arccos\\,(${fmt(dot(u,v)/(length(u)*length(v)))}) \n \n = ${fmt(degVal)}^\\circ \\]`;
      const explain = 'Используем формулу через скалярное произведение и длины векторов.';
      const result = `\\( \\theta = ${fmt(degVal)}^\\circ \\)`;
      const draw = (plot) => {
        plot.fitToPoints([{ x: 0, y: 0 }, u, v]); plot.clear();
        plot.drawVector({ x: 0, y: 0 }, u, getCssVar('--chip-u'));
        plot.drawVector({ x: 0, y: 0 }, v, getCssVar('--chip-v'));
      };
      return { formula, explain, result, draw };
    }
  },
  collinear: {
    name: 'Проверка коллинеарности',
    inputs: () => (
      buildVectorInputs('U_', 'Вектор u', { x: '2', y: '1' }) +
      buildVectorInputs('V_', 'Вектор v', { x: '4', y: '2' })
    ),
    compute: (vals) => {
      const u = { x: parseNum(vals['U_x']), y: parseNum(vals['U_y']) };
      const v = { x: parseNum(vals['V_x']), y: parseNum(vals['V_y']) };
      if (![u.x,u.y,v.x,v.y].every(Number.isFinite)) throw new Error('Введите корректные координаты векторов u и v.');
      const col = isCollinear(u, v);
      const formula = `\\[ \\text{Коллинеарность: } u_x v_y - u_y v_x = 0 \n \n=> ${fmt(u.x)}\\cdot${fmt(v.y)} - ${fmt(u.y)}\\cdot${fmt(v.x)} = ${fmt(cross2(u,v))} \n \n${col ? "= 0" : "\\ne 0"} \\]`;
      const explain = 'В двумерном случае коллинеарность эквивалентна нулевому псевдоскалярному произведению ("перпендикулярному" детерминанту).';
      const result = col ? 'Да, векторы коллинеарны.' : 'Нет, векторы не коллинеарны.';
      const draw = (plot) => {
        plot.fitToPoints([{ x: 0, y: 0 }, u, v]); plot.clear();
        plot.drawVector({ x: 0, y: 0 }, u, getCssVar('--chip-u'));
        plot.drawVector({ x: 0, y: 0 }, v, getCssVar('--chip-v'));
      };
      return { formula, explain, result, draw };
    }
  },
  perpendicular: {
    name: 'Проверка перпендикулярности',
    inputs: () => (
      buildVectorInputs('U_', 'Вектор u', { x: '1', y: '2' }) +
      buildVectorInputs('V_', 'Вектор v', { x: '-2', y: '1' })
    ),
    compute: (vals) => {
      const u = { x: parseNum(vals['U_x']), y: parseNum(vals['U_y']) };
      const v = { x: parseNum(vals['V_x']), y: parseNum(vals['V_y']) };
      if (![u.x,u.y,v.x,v.y].every(Number.isFinite)) throw new Error('Введите корректные координаты векторов u и v.');
      const perp = isPerpendicular(u, v);
      const formula = `\\[ \\text{Перпендикулярность: } \\vec u\\cdot\\vec v = 0 \n \n=> ${fmt(u.x)}\\cdot${fmt(v.x)} + ${fmt(u.y)}\\cdot${fmt(v.y)} = ${fmt(dot(u,v))} \n \n${perp ? "= 0" : "\\ne 0"} \\]`;
      const explain = 'Перпендикулярность эквивалентна нулевому скалярному произведению.';
      const result = perp ? 'Да, векторы перпендикулярны.' : 'Нет, векторы не перпендикулярны.';
      const draw = (plot) => {
        plot.fitToPoints([{ x: 0, y: 0 }, u, v]); plot.clear();
        plot.drawVector({ x: 0, y: 0 }, u, getCssVar('--chip-u'));
        plot.drawVector({ x: 0, y: 0 }, v, getCssVar('--chip-v'));
      };
      return { formula, explain, result, draw };
    }
  }
};

// ====== Coordinate Calculator Logic ======
const coordOperationConfigs = {
  distance: {
    name: 'Расстояние между двумя точками',
    inputs: () => (
      buildPointInputs('A_', 'Точка A', { x: '1', y: '2' }) +
      buildPointInputs('B_', 'Точка B', { x: '5', y: '6' })
    ),
    compute: (vals) => {
      const A = { x: parseNum(vals['A_x']), y: parseNum(vals['A_y']) };
      const B = { x: parseNum(vals['B_x']), y: parseNum(vals['B_y']) };
      if (![A.x,A.y,B.x,B.y].every(Number.isFinite)) throw new Error('Введите корректные координаты точек A и B.');
      const d = Math.hypot(B.x - A.x, B.y - A.y);
      const formula = `\\[ d = \\sqrt{(x_2-x_1)^2+(y_2-y_1)^2} = \\sqrt{(${fmt(B.x)}-${fmt(A.x)})^2 + (${fmt(B.y)}-${fmt(A.y)})^2} = ${fmt(d)} \\]`;
      const explain = 'Применяем формулу расстояния между двумя точками на плоскости.';
      const result = `\\( d = ${fmt(d)} \\)`;
      const draw = (plot) => {
        plot.fitToPoints([A, B]); plot.clear();
        plot.drawPoint(A, '#fff', 4, 'A');
        plot.drawPoint(B, '#fff', 4, 'B');
        plot.drawSegment(A, B, getCssVar('--chip-res'));
      };
      return { formula, explain, result, draw };
    }
  },
  midpoint: {
    name: 'Середина отрезка',
    inputs: () => (
      buildPointInputs('A_', 'Точка A', { x: '0', y: '0' }) +
      buildPointInputs('B_', 'Точка B', { x: '6', y: '4' })
    ),
    compute: (vals) => {
      const A = { x: parseNum(vals['A_x']), y: parseNum(vals['A_y']) };
      const B = { x: parseNum(vals['B_x']), y: parseNum(vals['B_y']) };
      if (![A.x,A.y,B.x,B.y].every(Number.isFinite)) throw new Error('Введите корректные координаты точек A и B.');
      const M = { x: (A.x + B.x) / 2, y: (A.y + B.y) / 2 };
      const formula = `\\[ M \\big( \\tfrac{x_1+x_2}{2},\\ \\tfrac{y_1+y_2}{2} \\big) = ( \\tfrac{${fmt(A.x)}+${fmt(B.x)}}{2},\\ \\tfrac{${fmt(A.y)}+${fmt(B.y)}}{2} ) = (${fmt(M.x)},\\ ${fmt(M.y)}) \\]`;
      const explain = 'Координаты середины — средние арифметические соответствующих координат.';
      const result = `\\( M=(${fmt(M.x)},\\ ${fmt(M.y)}) \\)`;
      const draw = (plot) => {
        plot.fitToPoints([A, B, M]); plot.clear();
        plot.drawPoint(A, '#fff', 4, 'A');
        plot.drawPoint(B, '#fff', 4, 'B');
        plot.drawSegment(A, B, getCssVar('--chip-aux'));
        plot.drawPoint(M, getCssVar('--chip-res'), 5, 'M');
      };
      return { formula, explain, result, draw };
    }
  },
  divide_ratio: {
    name: 'Деление отрезка в заданном отношении',
    inputs: () => (
      buildPointInputs('A_', 'Точка A', { x: '0', y: '0' }) +
      buildPointInputs('B_', 'Точка B', { x: '6', y: '3' }) +
      buildRatioInputs('R_', 'm', 'n', { m: '1', n: '2' })
    ),
    compute: (vals) => {
      const A = { x: parseNum(vals['A_x']), y: parseNum(vals['A_y']) };
      const B = { x: parseNum(vals['B_x']), y: parseNum(vals['B_y']) };
      const m = parseNum(vals['R_m']);
      const n = parseNum(vals['R_n']);
      if (![A.x,A.y,B.x,B.y,m,n].every(Number.isFinite)) throw new Error('Введите корректные данные.');
      if (nearZero(m + n)) throw new Error('m + n не должно равняться 0.');
      const P = { x: (n * A.x + m * B.x) / (m + n), y: (n * A.y + m * B.y) / (m + n) };
      const formula = `\\[ P=\\Big( \\tfrac{n x_A + m x_B}{m+n},\\ \\tfrac{n y_A + m y_B}{m+n} \\Big) = \n ( \\tfrac{${fmt(n)}\\cdot${fmt(A.x)} + ${fmt(m)}\\cdot${fmt(B.x)}}{${fmt(m+n)}}, \\tfrac{${fmt(n)}\\cdot${fmt(A.y)} + ${fmt(m)}\\cdot${fmt(B.y)}}{${fmt(m+n)}} ) = (${fmt(P.x)},\\ ${fmt(P.y)}) \\]`;
      const explain = 'Используем формулу внутреннего (или внешнего при отрицательных m или n) деления отрезка.';
      const result = `\\( P=(${fmt(P.x)},\\ ${fmt(P.y)}) \\)`;
      const draw = (plot) => {
        plot.fitToPoints([A, B, P]); plot.clear();
        plot.drawSegment(A, B, getCssVar('--chip-aux'));
        plot.drawPoint(A, '#fff', 4, 'A');
        plot.drawPoint(B, '#fff', 4, 'B');
        plot.drawPoint(P, getCssVar('--chip-res'), 5, 'P');
      };
      return { formula, explain, result, draw };
    }
  },
  triangle_area: {
    name: 'Площадь треугольника по трём точкам',
    inputs: () => (
      buildPointInputs('A_', 'Точка A', { x: '0', y: '0' }) +
      buildPointInputs('B_', 'Точка B', { x: '6', y: '0' }) +
      buildPointInputs('C_', 'Точка C', { x: '2', y: '3' })
    ),
    compute: (vals) => {
      const A = { x: parseNum(vals['A_x']), y: parseNum(vals['A_y']) };
      const B = { x: parseNum(vals['B_x']), y: parseNum(vals['B_y']) };
      const C = { x: parseNum(vals['C_x']), y: parseNum(vals['C_y']) };
      if (![A.x,A.y,B.x,B.y,C.x,C.y].every(Number.isFinite)) throw new Error('Введите корректные координаты точек A, B, C.');
      const area2 = Math.abs((B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x));
      const S = 0.5 * area2;
      const formula = `\\[ S = \\tfrac{1}{2} | (x_B-x_A)(y_C-y_A) - (y_B-y_A)(x_C-x_A) | = ${fmt(S)} \\]`;
      const explain = 'Площадь равна половине модуля векторного произведения \\((\\overrightarrow{AB}, \\overrightarrow{AC})\\).';
      const result = `\\( S = ${fmt(S)} \\)`;
      const draw = (plot) => {
        plot.fitToPoints([A, B, C]); plot.clear();
        plot.drawSegment(A, B, getCssVar('--chip-aux'));
        plot.drawSegment(B, C, getCssVar('--chip-aux'));
        plot.drawSegment(C, A, getCssVar('--chip-aux'));
        plot.drawPoint(A, '#fff', 4, 'A');
        plot.drawPoint(B, '#fff', 4, 'B');
        plot.drawPoint(C, '#fff', 4, 'C');
      };
      return { formula, explain, result, draw };
    }
  },
  centroid: {
    name: 'Центроид треугольника',
    inputs: () => (
      buildPointInputs('A_', 'Точка A', { x: '0', y: '0' }) +
      buildPointInputs('B_', 'Точка B', { x: '6', y: '0' }) +
      buildPointInputs('C_', 'Точка C', { x: '2', y: '6' })
    ),
    compute: (vals) => {
      const A = { x: parseNum(vals['A_x']), y: parseNum(vals['A_y']) };
      const B = { x: parseNum(vals['B_x']), y: parseNum(vals['B_y']) };
      const C = { x: parseNum(vals['C_x']), y: parseNum(vals['C_y']) };
      if (![A.x,A.y,B.x,B.y,C.x,C.y].every(Number.isFinite)) throw new Error('Введите корректные координаты точек A, B, C.');
      const G = { x: (A.x + B.x + C.x) / 3, y: (A.y + B.y + C.y) / 3 };
      const formula = `\\[ G=\\Big( \\tfrac{x_A+x_B+x_C}{3},\\ \\tfrac{y_A+y_B+y_C}{3} \\Big) = (${fmt(G.x)},\\ ${fmt(G.y)}) \\]`;
      const explain = 'Центроид — точка пересечения медиан, среднее арифметическое координат вершин.';
      const result = `\\( G=(${fmt(G.x)},\\ ${fmt(G.y)}) \\)`;
      const draw = (plot) => {
        plot.fitToPoints([A, B, C, G]); plot.clear();
        plot.drawSegment(A, B, getCssVar('--chip-aux'));
        plot.drawSegment(B, C, getCssVar('--chip-aux'));
        plot.drawSegment(C, A, getCssVar('--chip-aux'));
        plot.drawPoint(G, getCssVar('--chip-res'), 5, 'G');
        plot.drawPoint(A, '#fff', 4, 'A');
        plot.drawPoint(B, '#fff', 4, 'B');
        plot.drawPoint(C, '#fff', 4, 'C');
      };
      return { formula, explain, result, draw };
    }
  },
  line_through_two_points: {
    name: 'Прямая через две точки',
    inputs: () => (
      buildPointInputs('A_', 'Точка A', { x: '0', y: '0' }) +
      buildPointInputs('B_', 'Точка B', { x: '4', y: '3' })
    ),
    compute: (vals) => {
      const A = { x: parseNum(vals['A_x']), y: parseNum(vals['A_y']) };
      const B = { x: parseNum(vals['B_x']), y: parseNum(vals['B_y']) };
      if (![A.x,A.y,B.x,B.y].every(Number.isFinite)) throw new Error('Введите корректные координаты точек A и B.');
      if (nearZero(A.x - B.x) && nearZero(A.y - B.y)) throw new Error('Точки A и B должны различаться.');
      const L = generalFromTwoPoints(A, B);
      const { k, b } = slopeFromGeneral(L);
      const { ax, by } = interceptsFromGeneral(L);
      const formula = `\\[ \n  Ax + By + C = 0,\\ \ A=${fmt(L.A)},\\ B=${fmt(L.B)},\\ C=${fmt(L.C)} \\\\ \n  ${Number.isFinite(k) ? `y = ${fmt(k)}x + ${fmt(b)}` : '\\text{Вертикальная прямая}' } \\\\ \n  ${Number.isFinite(ax) && Number.isFinite(by) ? `\\frac{x}{${fmt(ax)}} + \\frac{y}{${fmt(by)}} = 1` : ''} \n\\]`;
      const explain = 'Строим общее уравнение по двум точкам и переводим к другим формам.';
      const result = `\\( ${fmt(L.A)}x + ${fmt(L.B)}y + ${fmt(L.C)} = 0 \\)`;
      const draw = (plot) => {
        plot.fitToPoints([A, B]); plot.clear();
        plot.drawLineGeneral(L, getCssVar('--chip-res'));
        plot.drawPoint(A, '#fff', 4, 'A');
        plot.drawPoint(B, '#fff', 4, 'B');
      };
      return { formula, explain, result, draw };
    }
  },
  line_point_direction: {
    name: 'Прямая: точка + направляющий вектор',
    inputs: () => (
      buildPointInputs('P_', 'Точка P', { x: '1', y: '2' }) +
      buildVectorInputs('V_', 'Вектор направления v', { x: '2', y: '1' })
    ),
    compute: (vals) => {
      const P = { x: parseNum(vals['P_x']), y: parseNum(vals['P_y']) };
      const v = { x: parseNum(vals['V_x']), y: parseNum(vals['V_y']) };
      if (![P.x,P.y,v.x,v.y].every(Number.isFinite)) throw new Error('Введите корректные данные.');
      if (nearZero(v.x) && nearZero(v.y)) throw new Error('Направляющий вектор не должен быть нулевым.');
      const L = generalFromPointAndDir(P, v);
      const { k, b } = slopeFromGeneral(L);
      const formula = `\\[ \n  \n  \text{Параметрически: } x=${fmt(P.x)}+t\\cdot${fmt(v.x)},\\ \ y=${fmt(P.y)}+t\\cdot${fmt(v.y)} \\\\ \n  Ax + By + C = 0: \\ ${fmt(L.A)}x + ${fmt(L.B)}y + ${fmt(L.C)} = 0 \\\\ \n  ${Number.isFinite(k) ? `y = ${fmt(k)}x + ${fmt(b)}` : '\\text{Вертикальная прямая}' } \n\\]`;
      const explain = 'Нормаль к прямой перпендикулярна направляющему вектору: n = (−v_y, v_x).';
      const result = `\\( ${fmt(L.A)}x + ${fmt(L.B)}y + ${fmt(L.C)} = 0 \\)`;
      const draw = (plot) => {
        plot.fitToPoints([P, { x: P.x + v.x, y: P.y + v.y }]); plot.clear();
        plot.drawLineGeneral(L, getCssVar('--chip-res'));
        plot.drawPoint(P, '#fff', 4, 'P');
      };
      return { formula, explain, result, draw };
    }
  },
  line_point_perp_vector: {
    name: 'Прямая: через точку перпендикулярно вектору',
    inputs: () => (
      buildPointInputs('P_', 'Точка P', { x: '1', y: '2' }) +
      buildVectorInputs('V_', 'Вектор v (нормаль)', { x: '1', y: '-2' })
    ),
    compute: (vals) => {
      const P = { x: parseNum(vals['P_x']), y: parseNum(vals['P_y']) };
      const v = { x: parseNum(vals['V_x']), y: parseNum(vals['V_y']) };
      if (![P.x,P.y,v.x,v.y].every(Number.isFinite)) throw new Error('Введите корректные данные.');
      if (nearZero(v.x) && nearZero(v.y)) throw new Error('Вектор не должен быть нулевым.');
      const L = generalPerpThroughPoint(P, v);
      const { k, b } = slopeFromGeneral(L);
      const formula = `\\[ \n  v_x(x-${fmt(P.x)}) + v_y(y-${fmt(P.y)}) = 0 \\\\ \n  ${fmt(v.x)}(x-${fmt(P.x)}) + ${fmt(v.y)}(y-${fmt(P.y)}) = 0 \\\\ \n  Ax + By + C = 0: \\ ${fmt(L.A)}x + ${fmt(L.B)}y + ${fmt(L.C)} = 0 \\\\ \n  ${Number.isFinite(k) ? `y = ${fmt(k)}x + ${fmt(b)}` : '\\text{Вертикальная прямая}' } \n\\]`;
      const explain = 'Нормаль совпадает с заданным вектором, проходящим через точку.';
      const result = `\\( ${fmt(L.A)}x + ${fmt(L.B)}y + ${fmt(L.C)} = 0 \\)`;
      const draw = (plot) => {
        plot.fitToPoints([P, { x: P.x + v.x, y: P.y + v.y }]); plot.clear();
        plot.drawLineGeneral(L, getCssVar('--chip-res'));
        plot.drawPoint(P, '#fff', 4, 'P');
      };
      return { formula, explain, result, draw };
    }
  },
  line_intercepts: {
    name: 'Прямая в отрезках',
    inputs: () => (
      buildNumberInput('A_int', 'Пересечение с осью x (a)', 'например, 4', '4') +
      buildNumberInput('B_int', 'Пересечение с осью y (b)', 'например, 3', '3')
    ),
    compute: (vals) => {
      const ax = parseNum(vals['A_int']);
      const by = parseNum(vals['B_int']);
      if (![ax, by].every(Number.isFinite)) throw new Error('Введите корректные значения a и b.');
      if (nearZero(ax) || nearZero(by)) throw new Error('a и b не должны равняться 0.');
      const L = generalFromIntercepts(ax, by);
      const formula = `\\[ \\frac{x}{${fmt(ax)}} + \\frac{y}{${fmt(by)}} = 1 \\ \Rightarrow \\ ${fmt(L.A)}x + ${fmt(L.B)}y + ${fmt(L.C)} = 0 \\]`;
      const explain = 'Форма в отрезках пересекает оси в точках (a,0) и (0,b).';
      const result = `\\( ${fmt(L.A)}x + ${fmt(L.B)}y + ${fmt(L.C)} = 0 \\)`;
      const draw = (plot) => {
        const P = { x: ax, y: 0 }, Q = { x: 0, y: by };
        plot.fitToPoints([P, Q]); plot.clear();
        plot.drawLineGeneral(L, getCssVar('--chip-res'));
        plot.drawPoint(P, '#fff', 4, 'a');
        plot.drawPoint(Q, '#fff', 4, 'b');
      };
      return { formula, explain, result, draw };
    }
  },
  line_slope: {
    name: 'Прямая с угловым коэффициентом',
    inputs: () => (
      buildNumberInput('K', 'k (угловой коэффициент)', 'например, 1.5', '1') +
      buildNumberInput('B', 'b (свободный член)', 'например, -2', '0')
    ),
    compute: (vals) => {
      const k = parseNum(vals['K']);
      const b = parseNum(vals['B']);
      if (![k, b].every(Number.isFinite)) throw new Error('Введите корректные значения k и b.');
      const L = generalFromSlopeIntercept(k, b);
      const formula = `\\[ y = ${fmt(k)}x + ${fmt(b)} \\ \Rightarrow \\ ${fmt(L.A)}x + ${fmt(L.B)}y + ${fmt(L.C)} = 0 \\]`;
      const explain = 'Стандартная форма с угловым коэффициентом.';
      const result = `\\( ${fmt(L.A)}x + ${fmt(L.B)}y + ${fmt(L.C)} = 0 \\)`;
      const draw = (plot) => {
        const p1 = { x: 0, y: b }, p2 = { x: 1, y: k + b };
        plot.fitToPoints([p1, p2]); plot.clear();
        plot.drawLineGeneral(L, getCssVar('--chip-res'));
      };
      return { formula, explain, result, draw };
    }
  },
  lines_parallel: {
    name: 'Параллельность двух прямых',
    inputs: () => (
      buildLineABCInputs('L1_', 'Прямая 1', { A: '1', B: '-2', C: '3' }) +
      buildLineABCInputs('L2_', 'Прямая 2', { A: '2', B: '-4', C: '-1' })
    ),
    compute: (vals) => {
      const L1 = normalizeLine({ A: parseNum(vals['L1_A']), B: parseNum(vals['L1_B']), C: parseNum(vals['L1_C']) });
      const L2 = normalizeLine({ A: parseNum(vals['L2_A']), B: parseNum(vals['L2_B']), C: parseNum(vals['L2_C']) });
      if (![L1.A,L1.B,L1.C,L2.A,L2.B,L2.C].every(Number.isFinite)) throw new Error('Введите корректные коэффициенты прямых.');
      const par = areParallel(L1, L2);
      const formula = `\\[ \n  \n  A_1B_2 - A_2B_1 = ${fmt(L1.A)}\\cdot${fmt(L2.B)} - ${fmt(L2.A)}\\cdot${fmt(L1.B)} = ${fmt(L1.A*L2.B - L2.A*L1.B)} \n  \n${par ? "= 0" : "\\ne 0"} \n\\]`;
      const explain = 'Параллельность: детерминант нормалей равен 0 (со-направленные нормали).';
      const result = par ? 'Да, прямые параллельны.' : 'Нет, прямые не параллельны.';
      const draw = (plot) => {
        plot.fitToPoints([{ x: -5, y: 0 }, { x: 5, y: 0 }]); plot.clear();
        plot.drawLineGeneral(L1, getCssVar('--chip-u'));
        plot.drawLineGeneral(L2, getCssVar('--chip-v'));
      };
      return { formula, explain, result, draw };
    }
  },
  lines_perp: {
    name: 'Перпендикулярность двух прямых',
    inputs: () => (
      buildLineABCInputs('L1_', 'Прямая 1', { A: '1', B: '1', C: '0' }) +
      buildLineABCInputs('L2_', 'Прямая 2', { A: '1', B: '-1', C: '0' })
    ),
    compute: (vals) => {
      const L1 = normalizeLine({ A: parseNum(vals['L1_A']), B: parseNum(vals['L1_B']), C: parseNum(vals['L1_C']) });
      const L2 = normalizeLine({ A: parseNum(vals['L2_A']), B: parseNum(vals['L2_B']), C: parseNum(vals['L2_C']) });
      if (![L1.A,L1.B,L1.C,L2.A,L2.B,L2.C].every(Number.isFinite)) throw new Error('Введите корректные коэффициенты прямых.');
      const perp = arePerpendicular(L1, L2);
      const formula = `\\[ A_1A_2 + B_1B_2 = ${fmt(L1.A)}\\cdot${fmt(L2.A)} + ${fmt(L1.B)}\\cdot${fmt(L2.B)} = ${fmt(L1.A*L2.A + L1.B*L2.B)} \n ${perp ? "= 0" : "\\ne 0"} \\]`;
      const explain = 'Перпендикулярность: скалярное произведение нормалей равно 0.';
      const result = perp ? 'Да, прямые перпендикулярны.' : 'Нет, прямые не перпендикулярны.';
      const draw = (plot) => {
        plot.fitToPoints([{ x: -5, y: 0 }, { x: 5, y: 0 }]); plot.clear();
        plot.drawLineGeneral(L1, getCssVar('--chip-u'));
        plot.drawLineGeneral(L2, getCssVar('--chip-v'));
      };
      return { formula, explain, result, draw };
    }
  },
  lines_intersection: {
    name: 'Точка пересечения двух прямых',
    inputs: () => (
      buildLineABCInputs('L1_', 'Прямая 1', { A: '1', B: '-1', C: '0' }) +
      buildLineABCInputs('L2_', 'Прямая 2', { A: '0', B: '1', C: '-2' })
    ),
    compute: (vals) => {
      const L1 = normalizeLine({ A: parseNum(vals['L1_A']), B: parseNum(vals['L1_B']), C: parseNum(vals['L1_C']) });
      const L2 = normalizeLine({ A: parseNum(vals['L2_A']), B: parseNum(vals['L2_B']), C: parseNum(vals['L2_C']) });
      if (![L1.A,L1.B,L1.C,L2.A,L2.B,L2.C].every(Number.isFinite)) throw new Error('Введите корректные коэффициенты прямых.');
      const P = intersection(L1, L2);
      if (!P) throw new Error('Прямые параллельны или совпадают — точки пересечения нет (или бесконечно много).');
      const formula = `\\[ P = (x, y), \n \n x = \\frac{\\begin{vmatrix}-C_1 & B_1 \\ \\ -C_2 & B_2\\end{vmatrix}}{\\begin{vmatrix}A_1 & B_1 \\ \\ A_2 & B_2\\end{vmatrix}}, \ \ y = \\frac{\\begin{vmatrix}A_1 & -C_1 \\ \\ A_2 & -C_2\\end{vmatrix}}{\\begin{vmatrix}A_1 & B_1 \\ \\ A_2 & B_2\\end{vmatrix}} \n \n = (${fmt(P.x)},\\ ${fmt(P.y)}) \\]`;
      const explain = 'Используем формулы Крамера для решения системы двух уравнений.';
      const result = `\\( P=(${fmt(P.x)},\\ ${fmt(P.y)}) \\)`;
      const draw = (plot) => {
        plot.fitToPoints([P, { x: -5, y: 0 }, { x: 5, y: 0 }]); plot.clear();
        plot.drawLineGeneral(L1, getCssVar('--chip-u'));
        plot.drawLineGeneral(L2, getCssVar('--chip-v'));
        plot.drawPoint(P, getCssVar('--chip-res'), 5, 'P');
      };
      return { formula, explain, result, draw };
    }
  },
  lines_angle: {
    name: 'Угол между двумя прямыми',
    inputs: () => (
      buildLineABCInputs('L1_', 'Прямая 1', { A: '1', B: '-1', C: '0' }) +
      buildLineABCInputs('L2_', 'Прямая 2', { A: '1', B: '1', C: '0' })
    ),
    compute: (vals) => {
      const L1 = normalizeLine({ A: parseNum(vals['L1_A']), B: parseNum(vals['L1_B']), C: parseNum(vals['L1_C']) });
      const L2 = normalizeLine({ A: parseNum(vals['L2_A']), B: parseNum(vals['L2_B']), C: parseNum(vals['L2_C']) });
      if (![L1.A,L1.B,L1.C,L2.A,L2.B,L2.C].every(Number.isFinite)) throw new Error('Введите корректные коэффициенты прямых.');
      const theta = angleBetweenLines(L1, L2);
      const degVal = deg(theta);
      const num = L1.A * L2.B - L2.A * L1.B;
      const den = L1.A * L2.A + L1.B * L2.B;
      const formula = `\\[ \\tan\\,\\theta = \\Bigg|\\frac{A_1B_2 - A_2B_1}{A_1A_2 + B_1B_2}\\Bigg| = \\Bigg|\\frac{${fmt(num)}}{${fmt(den)}}\\Bigg| \n \n=> \\ \theta = ${fmt(degVal)}^\\circ \\]`;
      const explain = 'Используем формулу через коэффициенты общих уравнений прямых.';
      const result = `\\( \\theta = ${fmt(degVal)}^\\circ \\)`;
      const draw = (plot) => {
        plot.fitToPoints([{ x: -5, y: 0 }, { x: 5, y: 0 }]); plot.clear();
        plot.drawLineGeneral(L1, getCssVar('--chip-u'));
        plot.drawLineGeneral(L2, getCssVar('--chip-v'));
      };
      return { formula, explain, result, draw };
    }
  }
};

// ====== DOM helpers ======
function getValues(container) {
  const inputs = container.querySelectorAll('input');
  const map = {};
  inputs.forEach((i) => { map[i.id] = i.value; });
  return map;
}

function setHTML(el, html) { el.innerHTML = html; }
function setText(el, text) { el.textContent = text; }
function getCssVar(name) {
  const v = getComputedStyle(document.documentElement).getPropertyValue(name);
  return v && v.trim() ? v.trim() : '#fff';
}

function showError(el, message) {
  setHTML(el, `<span style="color: var(--danger)">${message}</span>`);
}

// ====== Tabs ======
function setupTabs() {
  const tabs = document.querySelectorAll('.tab');
  tabs.forEach(tab => {
    tab.addEventListener('click', () => {
      tabs.forEach(t => t.classList.remove('active'));
      tab.classList.add('active');
      const targetSel = tab.getAttribute('data-tab-target');
      document.querySelectorAll('.tab-panel').forEach(p => p.classList.remove('active'));
      const target = document.querySelector(targetSel);
      if (target) target.classList.add('active');
      // resize canvases on tab switch
      vectorPlot.adjustDPI();
      coordPlot.adjustDPI();
      // Repaint last results if any
      if (typeof lastVectorDraw === 'function') { vectorPlot.clear(); lastVectorDraw(vectorPlot); }
      if (typeof lastCoordDraw === 'function') { coordPlot.clear(); lastCoordDraw(coordPlot); }
    });
  });
}

// ====== Vector UI ======
let vectorPlot, coordPlot;
let lastVectorDraw = null, lastCoordDraw = null;

function setupVectorUI() {
  const opSelect = document.getElementById('vector-operation');
  const inputsBox = document.getElementById('vector-inputs');
  const formulaBox = document.getElementById('vector-formula');
  const explainBox = document.getElementById('vector-explain');
  const resultBox = document.getElementById('vector-result');
  const btnCalc = document.getElementById('vector-calc');
  const btnReset = document.getElementById('vector-reset');

  const renderInputs = () => {
    const cfg = vectorOperationConfigs[opSelect.value];
    if (!cfg) return;
    setHTML(inputsBox, cfg.inputs());
  };

  opSelect.addEventListener('change', () => { renderInputs(); setHTML(formulaBox, ''); setHTML(explainBox, ''); setHTML(resultBox, ''); });
  renderInputs();

  const doCalc = async () => {
    const cfg = vectorOperationConfigs[opSelect.value];
    if (!cfg) return;
    try {
      const vals = getValues(inputsBox);
      const { formula, explain, result, draw } = cfg.compute(vals);
      setHTML(formulaBox, formula);
      setHTML(explainBox, explain);
      setHTML(resultBox, result);
      await typeset(formulaBox); await typeset(explainBox); await typeset(resultBox);
      lastVectorDraw = draw;
      vectorPlot.adjustDPI();
      vectorPlot.clear();
      draw(vectorPlot);
    } catch (e) {
      showError(resultBox, e.message || String(e));
    }
  };

  btnCalc.addEventListener('click', doCalc);
  // Enter to calculate
  inputsBox.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') { e.preventDefault(); doCalc(); }
  });

  btnReset.addEventListener('click', () => {
    renderInputs();
    setHTML(formulaBox, ''); setHTML(explainBox, ''); setHTML(resultBox, '');
    lastVectorDraw = null; vectorPlot.adjustDPI(); vectorPlot.clear();
  });
}

// ====== Coordinates UI ======
function setupCoordUI() {
  const opSelect = document.getElementById('coord-operation');
  const inputsBox = document.getElementById('coord-inputs');
  const formulaBox = document.getElementById('coord-formula');
  const explainBox = document.getElementById('coord-explain');
  const resultBox = document.getElementById('coord-result');
  const btnCalc = document.getElementById('coord-calc');
  const btnReset = document.getElementById('coord-reset');

  const renderInputs = () => {
    const cfg = coordOperationConfigs[opSelect.value];
    if (!cfg) return;
    setHTML(inputsBox, cfg.inputs());
  };

  opSelect.addEventListener('change', () => { renderInputs(); setHTML(formulaBox, ''); setHTML(explainBox, ''); setHTML(resultBox, ''); });
  renderInputs();

  const doCalc = async () => {
    const cfg = coordOperationConfigs[opSelect.value];
    if (!cfg) return;
    try {
      const vals = getValues(inputsBox);
      const { formula, explain, result, draw } = cfg.compute(vals);
      setHTML(formulaBox, formula);
      setHTML(explainBox, explain);
      setHTML(resultBox, result);
      await typeset(formulaBox); await typeset(explainBox); await typeset(resultBox);
      lastCoordDraw = draw;
      coordPlot.adjustDPI();
      coordPlot.clear();
      draw(coordPlot);
    } catch (e) {
      showError(resultBox, e.message || String(e));
    }
  };
  btnCalc.addEventListener('click', doCalc);
  // Enter to calculate
  inputsBox.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') { e.preventDefault(); doCalc(); }
  });

  btnReset.addEventListener('click', () => {
    renderInputs();
    setHTML(formulaBox, ''); setHTML(explainBox, ''); setHTML(resultBox, '');
    lastCoordDraw = null; coordPlot.adjustDPI(); coordPlot.clear();
  });
}

// ====== Bootstrapping ======
window.addEventListener('DOMContentLoaded', () => {
  setupTabs();
  vectorPlot = new Plot2D(document.getElementById('vector-canvas'));
  coordPlot = new Plot2D(document.getElementById('coord-canvas'));
  vectorPlot.clear(); coordPlot.clear();
  setupVectorUI();
  setupCoordUI();
  // Redraw on resize
  window.addEventListener('resize', () => {
    if (vectorPlot) { vectorPlot.adjustDPI(); vectorPlot.clear(); if (typeof lastVectorDraw === 'function') lastVectorDraw(vectorPlot); }
    if (coordPlot) { coordPlot.adjustDPI(); coordPlot.clear(); if (typeof lastCoordDraw === 'function') lastCoordDraw(coordPlot); }
  });
});
