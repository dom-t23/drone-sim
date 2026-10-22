// The drone's eye: a tiny software renderer for the onboard camera and a gate detector
// that works on the pixels it draws. Pure JavaScript, no DOM, so it runs in Node tests.
//
// Vision mode, step 3. Until now the camera was a model: it took the true gate corners,
// projected them, and added noise and the odd dropout. With pixels, the camera draws a
// real 320 x 180 colour picture of what the drone would see (sky, a textured ground,
// orange gate frames with their depth and shading, grey legs and dark pillars, with a
// depth buffer so things really hide each other) plus sensor noise. The detector never
// sees the course: it finds orange pixels, groups them into blobs, fits a quadrilateral
// to each blob's outline and refines the four sides by line fitting to sub-pixel corners.
// Gates often line up one behind another, so a near and a far frame can touch in the
// picture and make one blob; those are taken apart by sliding the outline the drone
// expects to see over the blob. Which gate each quad is (data association) is decided by
// associate(), from where the drone believes the gates are; it is never told.

export const EYE = {
  W: 320,
  H: 180,
  near: 0.1, // m: clip plane
  noise: 7, // sensor noise, 0-255 levels, roughly 1 sigma
  minPixels: 40, // smaller blobs are ignored
  minSide: 10, // px: a quad smaller than this is too far away to fit well
  fill: [0.1, 0.62], // blob pixels / quad area for a frame (a ring), not a solid shape
  maxRms: 0.9, // px: the four sides must be straight lines
  band: [-1.6, 1.0], // px either side of a side's line (outward +) whose edge pixels it fits
  barRatio: 0.28 / 3.76, // frame bar width / outer size
  solid: 0.65, // share of each side that must be frame, just inside the edge...
  clear: 0.6, // ...and not frame, just outside it
  alone: 0.9, // a small quad: share of each side with no frame a few pixels further out
  splitMinSide: 20, // px: smallest predicted gate it tries to take out of a blob
  searchMax: 30, // px: how far a predicted outline is slid when taking a blob apart
  outside: (side) => 2.5 + 0.075 * side, // px the blob's outline may stick out of its quad
};

const GATE_RGB = [255, 104, 26];
const LEG_RGB = [96, 108, 124];
const PILLAR_RGB = [44, 54, 72];
const SUN = (() => { const l = Math.hypot(0.4, 0.8, 0.3); return [0.4 / l, 0.8 / l, 0.3 / l]; })();

// A box: centre, three unit axes, half sizes, colour.
function box(c, a0, a1, a2, h, rgb, gate = -1) {
  return { c, a: [a0, a1, a2], h, rgb, gate };
}

// Everything the camera can see that isn't sky or ground, as boxes: each gate frame as four
// bars, its two legs, and the scenery pillars.
export function buildScene(gates, pillars, { inner, bar, legRadius }) {
  const boxes = [], s = inner, t = bar;
  for (const g of gates) {
    const { right: r, up: u, normal: n, pos: p } = g;
    const at = (x, y) => [p[0] + r[0] * x + u[0] * y, p[1] + r[1] * x + u[1] * y, p[2] + r[2] * x + u[2] * y];
    for (const [w, hh, x, y] of [[s + 2 * t, t, 0, (s + t) / 2], [s + 2 * t, t, 0, -(s + t) / 2], [t, s, (s + t) / 2, 0], [t, s, -(s + t) / 2, 0]]) {
      boxes.push(box(at(x, y), r, u, n, [w / 2, hh / 2, t / 2], GATE_RGB, g.id));
    }
    for (const side of [-1, 1]) {
      const c = at(side * (s + t) / 2, -(s / 2 + t));
      if (c[1] < 0.1) continue;
      boxes.push(box([c[0], c[1] / 2, c[2]], [1, 0, 0], [0, 1, 0], [0, 0, 1], [legRadius, c[1] / 2, legRadius], LEG_RGB));
    }
  }
  for (const q of pillars) {
    // three.js yaw about +Y: local x maps to (cos, 0, -sin), local z to (sin, 0, cos)
    const c = Math.cos(q.yaw), sn = Math.sin(q.yaw);
    boxes.push(box(q.pos, [c, 0, -sn], [0, 1, 0], [sn, 0, c], [q.size[0] / 2, q.size[1] / 2, q.size[2] / 2], PILLAR_RGB));
  }
  // a bounding radius for quick culling
  for (const b of boxes) b.r = Math.hypot(b.h[0], b.h[1], b.h[2]);
  return boxes;
}

// Pixel coordinates <-> image-plane coordinates (x/z, y/z), pixel centres at +0.5.
export function toPlane(u, v, tanV, W = EYE.W, H = EYE.H) {
  const tanH = tanV * W / H;
  return [(u / W * 2 - 1) * tanH, (1 - v / H * 2) * tanV];
}
export function toPixel(x, y, tanV, W = EYE.W, H = EYE.H) {
  const tanH = tanV * W / H;
  return [(x / tanH + 1) * W / 2, (1 - y / tanV) * H / 2];
}

const NOISE = (() => {
  const t = new Float32Array(1 << 16);
  let a = 12345;
  const r = () => { a = (Math.imul(a, 1664525) + 1013904223) >>> 0; return a / 4294967296; };
  for (let i = 0; i < t.length; i++) t[i] = (r() + r() + r() - 1.5) * 2 * EYE.noise;
  return t;
})();

// Small deterministic hash for the ground texture.
function hash2(i, j) {
  let h = Math.imul(i, 374761393) + Math.imul(j, 668265263);
  h = Math.imul(h ^ (h >>> 13), 1274126177);
  return ((h ^ (h >>> 16)) >>> 0) / 4294967296;
}

// Draw the camera's picture. cam: { pos, fwd, up, right } (world), tanV: tan(vfov / 2).
// Returns { rgb (RGBA bytes), W, H, gate (which gate each pixel shows, -1 for none) }.
export function renderView(cam, boxes, tanV, rand, img = null) {
  const W = EYE.W, H = EYE.H, N = W * H, tanH = tanV * W / H;
  if (!img) img = { rgb: new Uint8ClampedArray(N * 4), depth: new Float32Array(N), gate: new Int16Array(N), W, H };
  const { rgb, depth, gate } = img;
  depth.fill(0); // stores 1/z: 0 is infinitely far
  gate.fill(-1);
  const { pos, fwd, up, right } = cam;
  // sky and ground, in 2 x 2 pixel blocks (it's smooth, and has no orange in it): ray = fwd + right x + up y
  for (let j = 0; j < H; j += 2) {
    const y = (1 - (j + 1) / H * 2) * tanV;
    const bx = fwd[0] + up[0] * y, by = fwd[1] + up[1] * y, bz = fwd[2] + up[2] * y;
    for (let i = 0; i < W; i += 2) {
      const x = ((i + 1) / W * 2 - 1) * tanH;
      const dx = bx + right[0] * x, dy = by + right[1] * x, dz = bz + right[2] * x;
      let r, g, b;
      if (dy >= -1e-4 || pos[1] <= 0) {
        const s = Math.min(1, dy / Math.sqrt(dx * dx + dy * dy + dz * dz) * 2.5);
        r = 150 - 90 * s; g = 186 - 70 * s; b = 222 - 20 * s;
      } else {
        const tt = -pos[1] / dy;
        const n = hash2(Math.floor((pos[0] + dx * tt) * 0.5), Math.floor((pos[2] + dz * tt) * 0.5));
        const fog = Math.min(1, tt * Math.sqrt(dx * dx + dy * dy + dz * dz) / 160);
        const sh = (0.75 + 0.35 * n) * (1 - fog);
        r = 78 * sh + 120 * fog; g = 96 * sh + 150 * fog; b = 54 * sh + 170 * fog;
      }
      const k = (j * W + i) * 4;
      rgb[k] = rgb[k + 4] = rgb[k + W * 4] = rgb[k + W * 4 + 4] = r;
      rgb[k + 1] = rgb[k + 5] = rgb[k + W * 4 + 1] = rgb[k + W * 4 + 5] = g;
      rgb[k + 2] = rgb[k + 6] = rgb[k + W * 4 + 2] = rgb[k + W * 4 + 6] = b;
      rgb[k + 3] = rgb[k + 7] = rgb[k + W * 4 + 3] = rgb[k + W * 4 + 7] = 255;
    }
  }
  // boxes, face by face, with a depth buffer
  const camv = (p) => {
    const dx = p[0] - pos[0], dy = p[1] - pos[1], dz = p[2] - pos[2];
    return [dx * right[0] + dy * right[1] + dz * right[2], dx * up[0] + dy * up[1] + dz * up[2], dx * fwd[0] + dy * fwd[1] + dz * fwd[2]];
  };
  for (const b of boxes) {
    const cc = camv(b.c);
    if (cc[2] < -b.r || cc[2] - b.r > 200) continue;
    if (Math.abs(cc[0]) - b.r > (cc[2] + b.r) * tanH + 0.5 || Math.abs(cc[1]) - b.r > (cc[2] + b.r) * tanV + 0.5) continue;
    for (let f = 0; f < 6; f++) {
      const ax = f >> 1, sg = f & 1 ? -1 : 1, nrm = b.a[ax];
      const fc = [b.c[0] + nrm[0] * sg * b.h[ax], b.c[1] + nrm[1] * sg * b.h[ax], b.c[2] + nrm[2] * sg * b.h[ax]];
      // facing the camera?
      if ((fc[0] - pos[0]) * nrm[0] * sg + (fc[1] - pos[1]) * nrm[1] * sg + (fc[2] - pos[2]) * nrm[2] * sg >= 0) continue;
      const a1 = b.a[(ax + 1) % 3], a2 = b.a[(ax + 2) % 3], h1 = b.h[(ax + 1) % 3], h2 = b.h[(ax + 2) % 3];
      const quad = [[1, 1], [-1, 1], [-1, -1], [1, -1]].map(([p, q]) => camv([
        fc[0] + a1[0] * p * h1 + a2[0] * q * h2, fc[1] + a1[1] * p * h1 + a2[1] * q * h2, fc[2] + a1[2] * p * h1 + a2[2] * q * h2,
      ]));
      const lit = 0.5 + 0.5 * Math.max(0, sg * (nrm[0] * SUN[0] + nrm[1] * SUN[1] + nrm[2] * SUN[2]));
      fillPolygon(img, clipNear(quad), tanH, tanV, [b.rgb[0] * lit, b.rgb[1] * lit, b.rgb[2] * lit], b.gate);
    }
  }
  // sensor noise: a fixed table of Gaussian-ish samples read from a random offset each frame
  const off = Math.floor(rand() * NOISE.length), mask = NOISE.length - 1;
  for (let k = 0, m = off; k < N * 4; k += 4, m += 3) {
    const e = NOISE[m & mask];
    rgb[k] += e + NOISE[(m + 1) & mask] * 0.4; rgb[k + 1] += e; rgb[k + 2] += e + NOISE[(m + 2) & mask] * 0.4;
  }
  return img;
}

// Sutherland-Hodgman against the near plane, in camera coordinates.
function clipNear(poly) {
  const out = [], n = EYE.near;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length];
    if (a[2] >= n) out.push(a);
    if ((a[2] >= n) !== (b[2] >= n)) {
      const t = (n - a[2]) / (b[2] - a[2]);
      out.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, n]);
    }
  }
  return out;
}

// Fill a convex polygon (camera coordinates, in front of the near plane), fan-triangulated.
function fillPolygon(img, poly, tanH, tanV, rgb, gateId) {
  if (poly.length < 3) return;
  const { W, H } = img;
  const P = poly.map(([x, y, z]) => [(x / z / tanH + 1) * W / 2, (1 - y / z / tanV) * H / 2, 1 / z]);
  for (let i = 1; i + 1 < P.length; i++) fillTriangle(img, P[0], P[i], P[i + 1], rgb, gateId);
}

function fillTriangle(img, a, b, c, rgb, gateId) {
  const { W, H, rgb: px, depth, gate } = img;
  const area = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
  if (Math.abs(area) < 1e-9) return;
  const x0 = Math.max(0, Math.floor(Math.min(a[0], b[0], c[0]))), x1 = Math.min(W - 1, Math.ceil(Math.max(a[0], b[0], c[0])));
  const y0 = Math.max(0, Math.floor(Math.min(a[1], b[1], c[1]))), y1 = Math.min(H - 1, Math.ceil(Math.max(a[1], b[1], c[1])));
  const inv = 1 / area;
  for (let j = y0; j <= y1; j++) {
    const py = j + 0.5;
    for (let i = x0; i <= x1; i++) {
      const pxx = i + 0.5;
      const w0 = ((b[0] - pxx) * (c[1] - py) - (b[1] - py) * (c[0] - pxx)) * inv;
      const w1 = ((c[0] - pxx) * (a[1] - py) - (c[1] - py) * (a[0] - pxx)) * inv;
      const w2 = 1 - w0 - w1;
      if (w0 < 0 || w1 < 0 || w2 < 0) continue;
      const iz = w0 * a[2] + w1 * b[2] + w2 * c[2], k = j * W + i;
      if (iz <= depth[k]) continue;
      depth[k] = iz;
      gate[k] = gateId;
      const fog = Math.min(1, 1 / iz / 160);
      px[k * 4] = rgb[0] * (1 - fog) + 120 * fog; px[k * 4 + 1] = rgb[1] * (1 - fog) + 150 * fog; px[k * 4 + 2] = rgb[2] * (1 - fog) + 170 * fog;
    }
  }
}

// ---------- the detector ----------

// Is this pixel gate orange? Bright red, much less blue, green in between: the hue of the
// frames whether they're in sun or shade, and nothing in the sky, grass or pillars.
export function isOrange(r, g, b) {
  return r > 80 && r - b > 0.55 * r && g > 0.18 * r && g < 0.72 * r;
}

// Find the gates in a picture: orange blobs, each fitted with a quadrilateral. A blob that
// isn't one clean quad (usually a near gate with a far one overlapping it in the picture,
// which is common: gates line up one behind the other) is taken apart using pred, where
// the drone expects gates to be ([{ id, corners }] in pixels, as for associate). Returns
// [{ corners: [[u, v] x4, round the outline], pixels, rms, fill, split }], pixel units.
export function detectGates(img, pred = null) {
  const { W, H, rgb } = img, N = W * H;
  // 0: not orange, 1: orange, 2: orange and already in a blob
  if (!img.mask) { img.mask = new Uint8Array(N); img.blob = new Int32Array(N); }
  const mask = img.mask, blob = img.blob, quads = [], stack = [];
  for (let k = 0; k < N; k++) mask[k] = isOrange(rgb[k * 4], rgb[k * 4 + 1], rgb[k * 4 + 2]) ? 1 : 0;
  for (let k0 = 0; k0 < N; k0++) {
    if (mask[k0] !== 1) continue;
    // flood fill one blob (8-connected, so thin far frames hold together)
    const pts = [];
    let touches = false;
    stack.push(k0); mask[k0] = 2; blob[k0] = k0;
    while (stack.length) {
      const k = stack.pop(), i = k % W, j = (k - i) / W;
      pts.push(k);
      if (i === 0 || j === 0 || i === W - 1 || j === H - 1) touches = true;
      for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) {
        const ii = i + di, jj = j + dj;
        if ((di || dj) && ii >= 0 && jj >= 0 && ii < W && jj < H) {
          const kk = jj * W + ii;
          if (mask[kk] === 1) { mask[kk] = 2; blob[kk] = k0; stack.push(kk); }
        }
      }
    }
    if (pts.length < EYE.minPixels) continue; // too small to fit
    const edges = edgesOf(pts, mask, W, H);
    // a blob cut off by the edge of the picture can't be a whole gate, but it can hold one
    const q = touches ? null : fitBlob(pts, edges, mask, W, H, (k) => blob[k] === k0);
    const found = q ? [q] : splitBlob(pts, edges, mask, W, H, pred);
    for (const f of found) { f.at = k0; quads.push(f); } // at: one of the blob's pixels
  }
  return quads;
}

// Edge points of a blob: for each blob pixel side that faces a non-blob pixel, the
// midpoint of that pixel side (so a line through them is the boundary itself).
function edgesOf(pts, mask, W, H) {
  const edges = [];
  for (const k of pts) {
    const i = k % W, j = (k - i) / W;
    if (i > 0 && !mask[k - 1]) edges.push([i, j + 0.5]);
    if (i < W - 1 && !mask[k + 1]) edges.push([i + 1, j + 0.5]);
    if (j > 0 && !mask[k - W]) edges.push([i + 0.5, j]);
    if (j < H - 1 && !mask[k + W]) edges.push([i + 0.5, j + 1]);
  }
  return edges;
}

// Monotone-chain convex hull of [x, y] points.
function hull(points) {
  const p = points.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], hi = [];
  for (const q of p) { while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop(); lo.push(q); }
  for (let i = p.length - 1; i >= 0; i--) { const q = p[i]; while (hi.length >= 2 && cross(hi[hi.length - 2], hi[hi.length - 1], q) <= 0) hi.pop(); hi.push(q); }
  return lo.slice(0, -1).concat(hi.slice(0, -1));
}

const area2 = (q) => q.reduce((s, p, i) => { const n = q[(i + 1) % q.length]; return s + p[0] * n[1] - n[0] * p[1]; }, 0);
const sidesOf = (q) => q.map((p, i) => Math.hypot(q[(i + 1) % 4][0] - p[0], q[(i + 1) % 4][1] - p[1]));

// The usual case, one gate on its own: rough corners from the blob's convex hull, refined,
// then checked.
function fitBlob(pts, edges, mask, W, H, own) {
  // outline points: the outer corners of each row's end pixels
  const rows = new Map();
  for (const k of pts) {
    const i = k % W, j = (k - i) / W, r = rows.get(j);
    if (!r) rows.set(j, [i, i]); else { if (i < r[0]) r[0] = i; if (i > r[1]) r[1] = i; }
  }
  const outline = [];
  for (const [j, [a, b]] of rows) outline.push([a, j], [a, j + 1], [b + 1, j], [b + 1, j + 1]);
  const hl = hull(outline);
  if (hl.length < 4) return null;
  // diagonal: the farthest pair; the other two corners: farthest from it on each side
  let A = 0, C = 0, best = -1;
  for (let i = 0; i < hl.length; i++) for (let j = i + 1; j < hl.length; j++) {
    const d = (hl[i][0] - hl[j][0]) ** 2 + (hl[i][1] - hl[j][1]) ** 2;
    if (d > best) { best = d; A = i; C = j; }
  }
  const pa = hl[A], pc = hl[C];
  let B = -1, D = -1, sb = 0, sd = 0;
  for (let i = 0; i < hl.length; i++) {
    const s = (pc[0] - pa[0]) * (hl[i][1] - pa[1]) - (pc[1] - pa[1]) * (hl[i][0] - pa[0]);
    if (s > sb) { sb = s; B = i; }
    if (s < sd) { sd = s; D = i; }
  }
  if (B < 0 || D < 0) return null;
  const q = refine([pa, hl[B], pc, hl[D]], edges);
  if (!q) return null;
  // the outline must BE a quadrilateral: no part of the blob's hull may stick out of the
  // fitted quad
  const out = EYE.outside(Math.min(...sidesOf(q.corners)));
  const [qx, qy] = centre(q.corners);
  for (let s = 0; s < 4; s++) {
    const a = q.corners[s], b = q.corners[(s + 1) % 4], L = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const side = (p) => ((p[0] - a[0]) * (b[1] - a[1]) - (p[1] - a[1]) * (b[0] - a[0])) / L;
    const sg = side([qx, qy]) > 0 ? -1 : 1; // outward positive
    for (const p of hl) if (sg * side(p) > out) return null;
  }
  return verify(q, mask, W, H, own);
}

const centre = (q) => [(q[0][0] + q[1][0] + q[2][0] + q[3][0]) / 4, (q[0][1] + q[1][1] + q[2][1] + q[3][1]) / 4];

// Refine rough corners: fit a line to each side's outer edge points (total least squares)
// and put the corners where the lines meet. Returns { corners (clockwise on screen), rms }.
function refine(rough, edges, again = true) {
  if (area2(rough) < 0) rough = [rough[0], rough[3], rough[2], rough[1]];
  const sides = sidesOf(rough);
  if (Math.min(...sides) < EYE.minSide) return null;
  const [cx, cy] = centre(rough);
  const lines = [];
  let sq = 0, count = 0;
  for (let s = 0; s < 4; s++) {
    const p = rough[s], q = rough[(s + 1) % 4], L = sides[s];
    const tx = (q[0] - p[0]) / L, ty = (q[1] - p[1]) / L;
    // outward normal (away from the quad's centre)
    let nx = ty, ny = -tx;
    if ((p[0] - cx) * nx + (p[1] - cy) * ny < 0) { nx = -nx; ny = -ny; }
    // the outermost edge point per step along the side (the outer edge, not the inner)
    const bins = new Map();
    for (const e of edges) {
      const al = (e[0] - p[0]) * tx + (e[1] - p[1]) * ty, off = (e[0] - p[0]) * nx + (e[1] - p[1]) * ny;
      if (al <= 0.12 * L || al >= 0.88 * L || off <= EYE.band[0] || off >= EYE.band[1]) continue;
      const k = Math.round(al), cur = bins.get(k);
      if (!cur || off > cur.off) bins.set(k, { e, off });
    }
    let use = [...bins.values()].map((b) => b.e);
    if (use.length < 4) return null;
    // total least squares, then once more without the points well off that line (where a
    // thin bar breaks up, the outermost point left in a step can be its inner edge)
    let line = fitLine(use);
    const kept = use.filter(([x, y]) => Math.abs(-(x - line.mx) * line.dy + (y - line.my) * line.dx) <= Math.max(0.75, 2 * line.rms));
    if (kept.length < Math.max(4, 0.6 * use.length)) return null;
    if (kept.length < use.length) { use = kept; line = fitLine(use); }
    sq += line.rms * line.rms * use.length;
    count += use.length;
    lines.push(line);
  }
  const rms = Math.sqrt(sq / count);
  if (rms > EYE.maxRms) return null;
  const corners = [];
  for (let s = 0; s < 4; s++) {
    // corner s is where side s-1 meets side s
    const a = lines[(s + 3) % 4], b = lines[s];
    const det = a.dx * b.dy - a.dy * b.dx;
    if (Math.abs(det) < 0.2) return null; // nearly parallel sides: not a quad
    const t = ((b.mx - a.mx) * b.dy - (b.my - a.my) * b.dx) / det;
    const c = [a.mx + a.dx * t, a.my + a.dy * t];
    if (Math.hypot(c[0] - rough[s][0], c[1] - rough[s][1]) > 4 + 0.08 * Math.min(...sides)) return null;
    corners.push(c);
  }
  // once more from the better corners: a rough corner a pixel out can tilt a side enough
  // to drop part of its true edge out of the band
  return again ? refine(corners, edges, false) ?? { corners, rms } : { corners, rms };
}

// Total least squares line through points: centroid, direction and rms distance.
function fitLine(pts) {
  let mx = 0, my = 0;
  for (const [x, y] of pts) { mx += x; my += y; }
  mx /= pts.length; my /= pts.length;
  let sxx = 0, sxy = 0, syy = 0;
  for (const [x, y] of pts) { const a = x - mx, b = y - my; sxx += a * a; sxy += a * b; syy += b * b; }
  const th = 0.5 * Math.atan2(2 * sxy, sxx - syy), dx = Math.cos(th), dy = Math.sin(th);
  let sq = 0;
  for (const [x, y] of pts) { const r = -(x - mx) * dy + (y - my) * dx; sq += r * r; }
  return { mx, my, dx, dy, rms: Math.sqrt(sq / pts.length) };
}

// Is this quad a gate frame? Inside the picture, big enough, convex; every side solid frame
// from end to end just inside its line (a gate's bar runs the whole side, while the hull of
// two overlapping gates bridges gaps); mostly not frame just outside it (so it is the outer
// edge, not an inner one); and a ring, not a solid shape. own(k), for a quad fitted to one
// blob, says whether pixel k is in that blob. Returns the quad with its fill, or null.
function verify(q, mask, W, H, own = null) {
  const c = q.corners, sides = sidesOf(c);
  if (Math.min(...sides) < EYE.minSide) return null;
  for (const p of c) if (!(p[0] > 1 && p[1] > 1 && p[0] < W - 1 && p[1] < H - 1)) return null;
  const turn = (i) => { const a = c[i], b = c[(i + 1) % 4], d = c[(i + 2) % 4]; return (b[0] - a[0]) * (d[1] - b[1]) - (b[1] - a[1]) * (d[0] - b[0]); };
  if (![0, 1, 2, 3].every((i) => turn(i) > 0)) return null; // convex, clockwise on screen
  const meanSide = sides.reduce((a, b) => a + b, 0) / 4, inset = Math.max(0.8, 0.5 * EYE.barRatio * meanSide);
  const [qx, qy] = centre(c);
  // a far gate's thin frame tangled with others can still make a plausible quad: small
  // ones must stand alone, with no other frame just outside them
  const small = meanSide < EYE.splitMinSide;
  const at = (x, y) => { x = Math.floor(x); y = Math.floor(y); return x >= 0 && y >= 0 && x < W && y < H && mask[y * W + x] !== 0; };
  for (let s = 0; s < 4; s++) {
    const a = c[s], b = c[(s + 1) % 4], L = sides[s];
    let nx = (b[1] - a[1]) / L, ny = -(b[0] - a[0]) / L;
    if ((a[0] - qx) * nx + (a[1] - qy) * ny < 0) { nx = -nx; ny = -ny; }
    const n = Math.max(8, Math.ceil(L));
    let solid = 0, clear = 0, alone = 0;
    for (let k = 0; k < n; k++) {
      const t = 0.08 + 0.84 * (k + 0.5) / n, x = a[0] + (b[0] - a[0]) * t, y = a[1] + (b[1] - a[1]) * t;
      if (small && !at(x + nx * 2, y + ny * 2) && !at(x + nx * 3.5, y + ny * 3.5)) alone++;
      if (at(x - nx * inset, y - ny * inset) || at(x - nx * (inset + 0.6), y - ny * (inset + 0.6))) solid++;
      if (!at(x + nx * 1.2, y + ny * 1.2)) clear++;
    }
    if (solid < EYE.solid * n || clear < EYE.clear * n) return null;
    if (small && alone < EYE.alone * n) return null;
  }
  if (own) {
    // and its opening must be clear of its own blob: a gate seen through it is a blob of
    // its own, but one tangled with this frame puts this blob's pixels across the opening
    let inside = 0;
    for (let i = 0; i < 8; i++) for (let j = 0; j < 8; j++) {
      const u = 0.25 + 0.5 * (i + 0.5) / 8, v = 0.25 + 0.5 * (j + 0.5) / 8;
      const x = Math.floor((1 - u) * (1 - v) * c[0][0] + u * (1 - v) * c[1][0] + u * v * c[2][0] + (1 - u) * v * c[3][0]);
      const y = Math.floor((1 - u) * (1 - v) * c[0][1] + u * (1 - v) * c[1][1] + u * v * c[2][1] + (1 - u) * v * c[3][1]);
      if (x >= 0 && y >= 0 && x < W && y < H && own(y * W + x)) inside++;
    }
    if (inside > 2) return null;
  }
  // fill: frame pixels inside the quad over its area
  let inside = 0;
  const x0 = Math.max(0, Math.floor(Math.min(c[0][0], c[1][0], c[2][0], c[3][0]))), x1 = Math.min(W - 1, Math.ceil(Math.max(c[0][0], c[1][0], c[2][0], c[3][0])));
  const y0 = Math.max(0, Math.floor(Math.min(c[0][1], c[1][1], c[2][1], c[3][1]))), y1 = Math.min(H - 1, Math.ceil(Math.max(c[0][1], c[1][1], c[2][1], c[3][1])));
  for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
    if (!mask[y * W + x]) continue;
    const px = x + 0.5, py = y + 0.5;
    let inq = true;
    for (let s = 0; s < 4 && inq; s++) {
      const a = c[s], b = c[(s + 1) % 4];
      if ((b[0] - a[0]) * (py - a[1]) - (b[1] - a[1]) * (px - a[0]) < 0) inq = false;
    }
    if (inq) inside++;
  }
  const fill = inside / (area2(c) / 2);
  if (!(fill >= EYE.fill[0] && fill <= EYE.fill[1])) return null;
  return { corners: c, pixels: inside, rms: q.rms, fill };
}

// A blob that isn't one clean quad, usually a near gate with a far one overlapping it in
// the picture. Take it apart with what the drone expects to see: for each gate it predicts
// near this blob, slide the predicted outline (and slightly bigger and smaller copies) over
// the blob to where it fits best (solid frame just inside every side, clear just outside),
// then refine and check it like any other quad. Map errors mostly shift a gate sideways in
// the picture without changing its size much, so this finds it even metres from the map.
function splitBlob(pts, edges, mask, W, H, pred) {
  if (!pred?.length) return [];
  let bx0 = W, bx1 = 0, by0 = H, by1 = 0;
  for (const k of pts) {
    const i = k % W, j = (k - i) / W;
    if (i < bx0) bx0 = i; if (i > bx1) bx1 = i; if (j < by0) by0 = j; if (j > by1) by1 = j;
  }
  const at = (x, y) => { x = Math.floor(x); y = Math.floor(y); return x >= 0 && y >= 0 && x < W && y < H && mask[y * W + x] !== 0; };
  const found = [];
  for (const p of pred) {
    const sides = sidesOf(p.corners), side = sides.reduce((a, b) => a + b, 0) / 4;
    if (side < EYE.splitMinSide) continue; // far gates are too thin to take apart reliably
    const [pcx, pcy] = centre(p.corners), reach = Math.min(EYE.searchMax, Math.max(18, ASSOC.reach * side));
    // only predictions that could be this blob, and whole gates the picture could hold
    if (pcx < bx0 - reach || pcx > bx1 + reach || pcy < by0 - reach || pcy > by1 + reach) continue;
    if (Math.max(...sides) > 0.8 * H) continue;
    // a gate that would still be cut off by the edge of the picture after the search
    const xs = p.corners.map((c) => c[0]), ys = p.corners.map((c) => c[1]);
    if (Math.min(...xs) < -reach || Math.min(...ys) < -reach || Math.max(...xs) > W + reach || Math.max(...ys) > H + reach) continue;
    // a blob too small to hold that gate's whole frame
    if (pts.length < 0.4 * 0.276 * side * side) continue;
    const inset = Math.max(0.8, 0.5 * EYE.barRatio * side);
    let best = null;
    for (const sc of [0.88, 1, 1.12]) {
      // sample points just inside and just outside each side of the scaled outline
      const q = p.corners.map(([x, y]) => [pcx + (x - pcx) * sc, pcy + (y - pcy) * sc]);
      const samples = [];
      for (let s = 0; s < 4; s++) {
        const a = q[s], b = q[(s + 1) % 4], L = Math.hypot(b[0] - a[0], b[1] - a[1]);
        let nx = (b[1] - a[1]) / L, ny = -(b[0] - a[0]) / L;
        if ((a[0] - pcx) * nx + (a[1] - pcy) * ny < 0) { nx = -nx; ny = -ny; }
        for (let k = 0; k < 7; k++) {
          const t = 0.1 + 0.8 * (k + 0.5) / 7, x = a[0] + (b[0] - a[0]) * t, y = a[1] + (b[1] - a[1]) * t;
          samples.push([x - nx * inset, y - ny * inset, x + nx * 1.2, y + ny * 1.2]);
        }
      }
      const score = (dx, dy) => {
        let sc = 0;
        for (const [ix, iy, ox, oy] of samples) sc += (at(ix + dx, iy + dy) ? 1 : 0) + (at(ox + dx, oy + dy) ? 0 : 0.5);
        return sc;
      };
      // coarse grid, then the pixels around the best of it
      let cb = null;
      for (let dy = -reach; dy <= reach; dy += 3) for (let dx = -reach; dx <= reach; dx += 3) {
        const sc = score(dx, dy);
        if (!cb || sc > cb.sc) cb = { sc, dx, dy };
      }
      for (let dy = cb.dy - 2; dy <= cb.dy + 2; dy++) for (let dx = cb.dx - 2; dx <= cb.dx + 2; dx++) {
        const sc = score(dx, dy);
        if (!best || sc > best.score) best = { score: sc, q: q.map(([x, y]) => [x + dx, y + dy]) };
      }
    }
    if (!best || best.score < 0.8 * 42) continue; // 28 samples, 1.5 points each
    const r = refine(best.q, edges);
    const v = r && verify(r, mask, W, H);
    if (v) { v.split = true; v.area = area2(v.corners) / 2; found.push(v); }
  }
  // biggest first; quads that overlap one already kept are dropped
  found.sort((a, b) => b.area - a.area);
  const keep = [];
  const contains = (q, p) => [0, 1, 2, 3].every((s) => {
    const a = q.corners[s], b = q.corners[(s + 1) % 4];
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= 0;
  });
  for (const f of found) {
    if (keep.some((k) => f.corners.some((p) => contains(k, p)) || k.corners.some((p) => contains(f, p)))) continue;
    keep.push(f);
  }
  return keep;
}

// Which of the predicted gates is each detected quad? pred: [{ id, corners: [[u, v] x4] }]
// in pixels, from where the drone believes the gates are. The cost of a pairing is how far
// the corners are from the prediction (over the four rotations of the corner order and
// their mirror images), as a fraction of the predicted size, plus how different the sizes
// are. Size matters most: a map error moves a gate sideways by metres, which can put a
// far gate right behind a near one in the picture, but it barely changes how far away the
// gate is, and so how big it looks. Pairs go cheapest first; a quad that two gates could
// equally well be is left out rather than guessed. Returns [{ quad, id, corners, cost }]
// with the corners in the prediction's order.
export const ASSOC = { reach: 0.9, size: 0.45, sizeWeight: 2, margin: 1.5 };
const sideLen = (c) => (Math.hypot(c[1][0] - c[0][0], c[1][1] - c[0][1]) + Math.hypot(c[2][0] - c[1][0], c[2][1] - c[1][1])
  + Math.hypot(c[3][0] - c[2][0], c[3][1] - c[2][1]) + Math.hypot(c[0][0] - c[3][0], c[0][1] - c[3][1])) / 4;
export function associate(quads, pred) {
  const pairs = [], byQuad = new Map();
  for (const q of quads) {
    const sq = sideLen(q.corners);
    for (const p of pred) {
      const sp = sideLen(p.corners), dSize = Math.abs(Math.log(sq / sp));
      if (!(dSize <= ASSOC.size)) continue;
      let best = Infinity, order = null;
      for (const mirror of [false, true]) for (let r = 0; r < 4; r++) {
        let d = 0;
        const o = [];
        for (let k = 0; k < 4; k++) {
          const c = q.corners[mirror ? (r - k + 8) % 4 : (r + k) % 4];
          o.push(c);
          d += Math.hypot(c[0] - p.corners[k][0], c[1] - p.corners[k][1]);
        }
        if (d < best) { best = d; order = o; }
      }
      const dist = best / 4 / sp;
      if (dist > ASSOC.reach) continue;
      const m = { quad: q, id: p.id, corners: order, cost: dist + ASSOC.sizeWeight * dSize };
      pairs.push(m);
      if (!byQuad.has(q)) byQuad.set(q, []);
      byQuad.get(q).push(m.cost);
    }
  }
  // ambiguous quads: a second gate fits nearly as well
  const ambiguous = new Set();
  for (const [q, costs] of byQuad) {
    costs.sort((a, b) => a - b);
    if (costs.length > 1 && costs[1] < ASSOC.margin * costs[0] + 0.05) ambiguous.add(q);
  }
  pairs.sort((a, b) => a.cost - b.cost);
  const usedQ = new Set(), usedId = new Set(), out = [];
  for (const m of pairs) {
    if (ambiguous.has(m.quad) || usedQ.has(m.quad) || usedId.has(m.id)) continue;
    usedQ.add(m.quad); usedId.add(m.id);
    out.push(m);
  }
  return out;
}
