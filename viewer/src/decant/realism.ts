/**
 * The cradle page's photographic look: neutral tone mapping (true colours: the page's
 * explanatory reds and greens stay what they are), a procedural studio environment
 * (softboxes, a ceiling bank and a window; no downloaded HDRI: the box may be offline), a key
 * light with soft shadows, a walnut tabletop that fades into the room, a contact shadow
 * under the stand, and physical materials true to each part.
 *
 * What a part is made of comes from the bake: the material it's drawn with names its
 * fabrication kind (``aluminium``: a 6061 link; ``aluminium_frame``: a 5052 plate;
 * ``plywood``; ``metal``; ``gearbox``; ``printed``; ``led``; ``glass_bottle``,
 * ``glass_clear``), and its node's extras its surface: a bought part's ``finish``
 * (``hard_anodized``, ``stainless``, ``zinc``, ``black_oxide``, ``aluminium``, ``brass``;
 * none: plain steel; ``hardware.catalog.finish``) and a printed one's ``filament``
 * (``petg_filament``, ``tpu95a_filament``, ``pla_filament``; ``hardware.bom.part_filament``).
 * A node without them (an older bake) falls back on its name.
 *
 * The baked meshes carry positions only: :func:`createLook`'s ``dress`` gives them creased
 * normals and box-projected UVs, and splits a sheet part (a laser-cut plate, the birch
 * board) into its faces and its cut edges: brushed or bead-blasted faces, a rougher laser
 * edge; birch faces, plies on the edge. The maps are drawn in canvases at start-up.
 *
 * ``quality``: ``high`` with transmission (refracting glass, wine that glows ruby where it
 * is thin) and 2k shadows; ``low`` on a software rasterizer (SwiftShader, llvmpipe) or a
 * phone: glass and wine as blended layers, 1k shadows. ``?quality=high|low`` forces either.
 */
import * as THREE from 'three';
import { toCreasedNormals } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import type { Stage } from '../scene';
import type { LoadedScene } from '../loader';

export type Quality = 'high' | 'low';

/** The print colours on offer (PETG and PLA; the TPU parts print black). */
export const PRINT_COLORS: Record<string, string> = {
  charcoal: '#2b2d31',
  black: '#141517',
  'warm grey': '#77736c',
  ivory: '#e4ddcc',
  oxblood: '#5a1a1f',
};

export interface Look {
  quality: Quality;
  env: THREE.Texture;
  /** physical materials for the baked parts, by kind, the bottle, the glass and the wine */
  materials: {
    glassGreen: THREE.MeshPhysicalMaterial;
    /** a decanter's or a glass's: blended, so the wine in it shows through */
    glassClear: THREE.MeshPhysicalMaterial;
    /** the wine in the bottle (opaque: dark green glass round it) */
    wine: THREE.MeshPhysicalMaterial;
    /** the wine in the vessel: deep (near black, ruby at its thin edges) */
    wineDeep: THREE.MeshPhysicalMaterial;
    /** the stream from the mouth: thin, ruby */
    wineThin: THREE.MeshPhysicalMaterial;
    print: THREE.MeshPhysicalMaterial;
    tpu: THREE.MeshPhysicalMaterial;
    /** stainless steel and bead-blasted aluminium, for what the page adds */
    steel: THREE.Material;
    alu: THREE.Material;
  };
  /** Swap the baked materials for physical ones, smooth the round parts, cast shadows. */
  dress(loaded: LoadedScene): void;
  /** Stand the table, the shadow and the key light's frustum round ``box`` (the cradle). */
  fit(box: THREE.Box3): void;
  /** A soft contact shadow on the table, a disc of ``radius`` at ``(x, y)`` (a vessel's foot). */
  footShadow(x: number, y: number, radius: number): void;
  setPrintColor(name: string): void;
  printColor: string;
}

function detectQuality(renderer: THREE.WebGLRenderer): Quality {
  const forced = new URLSearchParams(location.search).get('quality');
  if (forced === 'high' || forced === 'low') return forced;
  const gl = renderer.getContext();
  const info = gl.getExtension('WEBGL_debug_renderer_info');
  const name = String(info ? gl.getParameter(info.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER));
  if (/swiftshader|llvmpipe|software|basic render|softpipe/i.test(name)) return 'low';
  if (matchMedia('(max-width: 820px)').matches) return 'low';
  return 'high';
}

// -- the environment: a warm room, softboxes, a ceiling bank, strip lights and a window -------

function glow(color: THREE.ColorRepresentation, intensity: number): THREE.MeshBasicMaterial {
  const m = new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide });
  m.color.multiplyScalar(intensity);
  return m;
}

/** The room the metal and the glass reflect (world units here are metres-ish; Z up, the
 * page's world). Bright enough overhead that bead-blasted aluminium reads light grey from
 * any side, as it does in a room. */
function studioScene(): THREE.Scene {
  const scene = new THREE.Scene();
  // the room: a sphere, a warm wooden floor, warm grey walls, a light ceiling
  const room = new THREE.SphereGeometry(20, 48, 24);
  room.rotateX(Math.PI / 2);   // poles on Z
  const pos = room.attributes.position!;
  const colors = new Float32Array(pos.count * 3);
  const floor = new THREE.Color('#6b5240'), wall = new THREE.Color('#b3aba0'), ceil = new THREE.Color('#ddd8cf');
  const c = new THREE.Color();
  for (let i = 0; i < pos.count; i++) {
    const z = pos.getZ(i) / 20;
    if (z < 0) c.copy(wall).lerp(floor, Math.min(1, -z * 2.2));
    else c.copy(wall).lerp(ceil, Math.min(1, z * 1.4));
    c.toArray(colors, i * 3);
  }
  room.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  scene.add(new THREE.Mesh(room, new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.BackSide })));

  const panel = (w: number, h: number, mat: THREE.Material, p: [number, number, number]): void => {
    const m = new THREE.Mesh(new THREE.PlaneGeometry(w, h), mat);
    m.position.set(...p);
    m.lookAt(0, 0, 0);
    scene.add(m);
  };
  // the key softbox, high front-left (where the key light comes from)
  panel(10, 8, glow('#fff3e4', 9), [-6, 9, 10]);
  // a fill softbox, low front-right
  panel(8, 6, glow('#f2f4ff', 2.6), [10, 6, 3]);
  // the ceiling bank: what a satin metal's top faces show
  panel(14, 14, glow('#fffaf2', 1.4), [0, 0, 17]);
  // a wide bounce card low on the viewer's side: what an upright plate facing the camera
  // reflects (without it, the floor: aluminium read as dark steel)
  panel(26, 6, glow('#f3eee6', 1.5), [4, 18, 0.5]);
  panel(14, 5, glow('#f3eee6', 1.2), [16, 8, 0.5]);
  panel(14, 5, glow('#f3eee6', 1.0), [-14, 12, 0.5]);
  panel(20, 5, glow('#f3eee6', 0.8), [0, -18, 0.5]);
  // two strip lights behind, for rim highlights on the glass and the plates' edges
  panel(1.6, 10, glow('#ffffff', 6), [5, -12, 5]);
  panel(1.2, 8, glow('#ffffff', 4), [-11, -6, 4]);
  // a window, cool daylight, to one side
  const win = new THREE.Group();
  for (const [dx, dz] of [[-1.3, 1.6], [1.3, 1.6], [-1.3, -1.6], [1.3, -1.6]] as const) {
    const pane = new THREE.Mesh(new THREE.PlaneGeometry(2.4, 3.0), glow('#dfeaff', 3.2));
    pane.position.set(dx, 0, dz);
    pane.rotation.x = Math.PI / 2;
    win.add(pane);
  }
  win.position.set(-14, -9, 4);
  win.lookAt(0, 0, 4);
  scene.add(win);
  return scene;
}

// -- noise ------------------------------------------------------------------------------------

function hash(x: number, y: number): number {
  let h = (x * 374761393 + y * 668265263) | 0;
  h = Math.imul(h ^ (h >>> 13), 1274126177);
  return ((h ^ (h >>> 16)) >>> 0) / 4294967295;
}

function noise(x: number, y: number): number {
  const xi = Math.floor(x), yi = Math.floor(y);
  const xf = x - xi, yf = y - yi;
  const u = xf * xf * (3 - 2 * xf), v = yf * yf * (3 - 2 * yf);
  const a = hash(xi, yi), b = hash(xi + 1, yi), c = hash(xi, yi + 1), d = hash(xi + 1, yi + 1);
  return a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v;
}

/** Tileable value noise on a ``period`` × ``period`` lattice. */
function tnoise(x: number, y: number, period: number): number {
  const xi = Math.floor(x), yi = Math.floor(y);
  const xf = x - xi, yf = y - yi;
  const u = xf * xf * (3 - 2 * xf), v = yf * yf * (3 - 2 * yf);
  const w = (i: number): number => ((i % period) + period) % period;
  const a = hash(w(xi), w(yi)), b = hash(w(xi + 1), w(yi)), c = hash(w(xi), w(yi + 1)), d = hash(w(xi + 1), w(yi + 1));
  return a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v;
}

function fbm(x: number, y: number): number {
  return noise(x, y) * 0.55 + noise(x * 2.1, y * 2.1) * 0.3 + noise(x * 4.3, y * 4.3) * 0.15;
}

/** A canvas of ``w`` × ``h`` filled by ``px(x, y) -> [r, g, b]`` (0..1), as a repeating
 * texture (a small one: :func:`texFrom` takes a large one's arrays). */
function paint(w: number, h: number, px: (x: number, y: number) => [number, number, number],
  srgb = false): THREE.CanvasTexture {
  const cv = document.createElement('canvas');
  cv.width = w; cv.height = h;
  const ctx = cv.getContext('2d')!;
  const img = ctx.createImageData(w, h);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const [r, g, b] = px(x, y);
      const i = (y * w + x) * 4;
      img.data[i] = Math.round(255 * Math.min(1, Math.max(0, r)));
      img.data[i + 1] = Math.round(255 * Math.min(1, Math.max(0, g)));
      img.data[i + 2] = Math.round(255 * Math.min(1, Math.max(0, b)));
      img.data[i + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  const t = new THREE.CanvasTexture(cv);
  if (srgb) t.colorSpace = THREE.SRGBColorSpace;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = 8;
  return t;
}

const grey = (v: number): [number, number, number] => [v, v, v];

// -- the tabletop: walnut planks drawn in a canvas ----------------------------------------------

interface WoodMaps { map: THREE.CanvasTexture; bump: THREE.CanvasTexture; rough: THREE.CanvasTexture }

/** Planks along X: cathedral grain, fine pores, seams and butt joints, ``W`` × ``H`` px, one
 * tile of the tabletop (it repeats; the fog hides the repeat). */
function woodMaps(W: number, H: number, planks: number): WoodMaps {
  const light = [0.4, 0.27, 0.18], dark = [0.2, 0.12, 0.075];
  const ph = H / planks;
  const joints: number[] = [];
  for (let p = 0; p < planks; p++) joints.push(hash(p, 7) * W * 0.6 + W * 0.2);
  const map = new Float32Array(W * H * 3), bump = new Float32Array(W * H), rough = new Float32Array(W * H);
  for (let y = 0; y < H; y++) {
    const p = Math.min(planks - 1, Math.floor(y / ph));
    const yl = y - p * ph;
    const jx = joints[p]!;
    for (let x = 0; x < W; x++) {
      const seg = x < jx ? 0 : 1;        // the two boards of a plank row, end to end
      const seed = p * 13 + seg * 5;
      const tone = 0.85 + 0.3 * hash(seed, 3);
      const cx = W * (0.25 + 0.5 * hash(seed, 1));
      const warp = fbm(x / 260 + seed, yl / 55) * 34;
      const arch = ((x - cx) / W) ** 2 * (120 + 160 * hash(seed, 2));
      const ring = (yl + warp + arch + seed * 17) / (7 + 4 * hash(seed, 4));
      const f = ring - Math.floor(ring);
      const late = Math.pow(Math.sin(f * Math.PI), 6);            // the dark late wood
      const pore = noise(x / 2.2 + seed * 31, yl * 1.6) > 0.82 ? 1 : 0;
      const figure = fbm(x / 90 + seed, yl / 12) - 0.5;
      const k = Math.min(1, Math.max(0, 0.25 + 0.35 * late + 0.2 * pore + 0.35 * figure));
      // the seams: the plank's edges and the butt joint
      const edge = Math.min(yl, ph - yl), butt = Math.abs(x - jx);
      const seam = edge < 1.2 || butt < 1.0 ? 1 : edge < 2.5 || butt < 2 ? 0.45 : 0;
      const i = y * W + x;
      for (let ch = 0; ch < 3; ch++) map[i * 3 + ch] = (light[ch]! * (1 - k) + dark[ch]! * k) * tone * (1 - seam * 0.7);
      bump[i] = 0.55 - 0.25 * late - 0.25 * pore - 0.5 * seam;
      rough[i] = 0.42 + 0.22 * late + 0.25 * seam + 0.1 * pore;
    }
  }
  return { map: texFrom(W, H, map, 3, true), bump: texFrom(W, H, bump, 1), rough: texFrom(W, H, rough, 1) };
}

/** A texture from ``data`` (0..1, ``channels`` a pixel, rows from the top). */
function texFrom(w: number, h: number, data: Float32Array, channels: 1 | 3, srgb = false): THREE.CanvasTexture {
  const cv = document.createElement('canvas');
  cv.width = w; cv.height = h;
  const ctx = cv.getContext('2d')!;
  const img = ctx.createImageData(w, h);
  for (let i = 0; i < w * h; i++) {
    for (let ch = 0; ch < 3; ch++) img.data[i * 4 + ch] = 255 * data[i * channels + (channels === 3 ? ch : 0)]!;
    img.data[i * 4 + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
  const t = new THREE.CanvasTexture(cv);
  if (srgb) t.colorSpace = THREE.SRGBColorSpace;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = 8;
  return t;
}

// -- the parts' maps ----------------------------------------------------------------------------

/** Rotary-cut birch, a ``400`` mm tile: a pale cream face, faint wavy grain along u, a few
 * darker streaks. */
function birchFace(): { map: THREE.CanvasTexture; bump: THREE.CanvasTexture } {
  const N = 512;
  const field = (x: number, y: number): number => {
    const warp = tnoise(x / 64, y / 16, 8) * 10 + tnoise(x / 16, y / 6, 32) * 2;
    const line = Math.sin(((y + warp) / N) * Math.PI * 2 * 22) * 0.5 + 0.5;
    return Math.pow(line, 8) * 0.45 + tnoise(x / 8, y / 2, 64) * 0.2 + tnoise(x / 128, y / 32, 4) * 0.45;
  };
  const base = new THREE.Color('#dccdb0'), late = new THREE.Color('#c4a983');
  const c = new THREE.Color();
  const map = paint(N, N, (x, y) => {
    const k = Math.min(1, field(x, y) * 0.5);
    c.copy(base).lerp(late, k);
    return [c.r, c.g, c.b];
  }, true);
  const bump = paint(N, N, (x, y) => grey(0.5 - field(x, y) * 0.2));
  return { map, bump };
}

/** A plywood edge: ``plies`` veneers across v (0..1, the board's thickness), the face grain
 * plies light, the cross plies' end grain darker, thin glue lines between. */
function plyEdge(plies: number): THREE.CanvasTexture {
  const W = 256, H = 128;
  const along = new THREE.Color('#ddcbab'), across = new THREE.Color('#bf9f73'), glue = new THREE.Color('#7d6141');
  const c = new THREE.Color();
  const t = paint(W, H, (x, y) => {
    const v = (y + 0.5) / H * plies;
    const ply = Math.min(plies - 1, Math.floor(v));
    const f = v - ply;
    c.copy(ply % 2 === 0 ? along : across);
    const fleck = tnoise(x / (ply % 2 ? 2 : 24), y / 3, ply % 2 ? 128 : 16) * 0.12;
    c.multiplyScalar(0.94 + fleck);
    if (f < 0.07 || f > 0.93) c.lerp(glue, 0.55);
    return [c.r, c.g, c.b];
  }, true);
  t.wrapT = THREE.ClampToEdgeWrapping;
  return t;
}

/** Bead blast: a fine even grain (roughness and bump), a 60 mm tile. */
function beadGrain(): THREE.CanvasTexture {
  const N = 256;
  return paint(N, N, (x, y) => grey(0.82 + 0.18 * (hash(x, y) * 0.6 + tnoise(x / 4, y / 4, 64) * 0.4)
    - 0.1 * tnoise(x / 64, y / 64, 4)));
}

/** Brushed sheet: streaks along u (roughness and bump), a 300 × 40 mm tile. */
function brushed(): THREE.CanvasTexture {
  const W = 1024, H = 512;
  // fine streaks along u: each row its own tone, drifting slowly along the row
  return paint(W, H, (x, y) => {
    const s = hash(y, 11) * 0.55 + tnoise(x / 128, y, 8) * 0.3 + hash(x >> 3, y) * 0.15;
    return grey(0.86 + 0.14 * s);
  });
}

/** A cast housing's grain: soft pits (bump), a 40 mm tile. */
function castGrain(): THREE.CanvasTexture {
  const N = 256;
  return paint(N, N, (x, y) => grey(0.55 + 0.45 * (tnoise(x / 3, y / 3, 85) * 0.6 + tnoise(x / 9, y / 9, 28) * 0.4)));
}

// -- UVs: box-projected; a sheet part split into its faces and its edges ------------------------

interface SheetInfo { thin: number; thickness: number }

/** The geometry as triangles with normals (creased) and box-projected UVs, ``scale`` mm a
 * unit. A ``sheet`` part's triangles facing along its thinnest axis come first (group 0, its
 * faces: u along its longest axis) and its cut edges after (group 1: u along the edge, v
 * across the thickness, 0..1). */
function dressGeometry(src: THREE.BufferGeometry, scale: number, sheet: boolean): {
  geometry: THREE.BufferGeometry; info: SheetInfo | null;
} {
  // (``toCreasedNormals`` returns its geometry non-indexed: a copy, or the input itself)
  const g = !src.attributes.normal ? toCreasedNormals(src, THREE.MathUtils.degToRad(32))
    : src.index ? src.toNonIndexed() : src;
  const pos = g.attributes.position as THREE.BufferAttribute, nor = g.attributes.normal as THREE.BufferAttribute;
  g.computeBoundingBox();
  const size = g.boundingBox!.getSize(new THREE.Vector3()).toArray();
  const min = g.boundingBox!.min.toArray();
  const order = [0, 1, 2].sort((a, b) => size[a]! - size[b]!);
  const thin = order[0]!, mid = order[1]!, long = order[2]!;
  const isSheet = sheet && size[thin]! < 0.25 * size[mid]!;
  const nTri = pos.count / 3;
  const face: number[] = [], edge: number[] = [];
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3();
  const n = new THREE.Vector3();
  const uv = new Float32Array(pos.count * 2);
  const P = (i: number, k: number): number => pos.getComponent(i, k);
  for (let t = 0; t < nTri; t++) {
    const i = t * 3;
    a.fromBufferAttribute(pos, i); b.fromBufferAttribute(pos, i + 1); c.fromBufferAttribute(pos, i + 2);
    n.subVectors(b, a).cross(c.sub(a));
    const an = [Math.abs(n.x), Math.abs(n.y), Math.abs(n.z)];
    const len = Math.hypot(an[0]!, an[1]!, an[2]!) || 1;
    if (isSheet && an[thin]! / len < 0.9) {
      // an edge: u along it (the tangent in the sheet's plane), v across the thickness
      edge.push(t);
      const nl = n.getComponent(long) / len, nm = n.getComponent(mid) / len;
      for (let k = 0; k < 3; k++) {
        const along = P(i + k, long) * -nm + P(i + k, mid) * nl;
        uv[(i + k) * 2] = along / scale;
        uv[(i + k) * 2 + 1] = (P(i + k, thin) - min[thin]!) / Math.max(1e-6, size[thin]!);
      }
      continue;
    }
    face.push(t);
    // box projection: the axis the face looks along is dropped
    let d = an[0]! >= an[1]! && an[0]! >= an[2]! ? 0 : an[1]! >= an[2]! ? 1 : 2;
    if (isSheet) d = thin;
    const [u0, v0] = d === thin ? [long, mid] : d === 0 ? [1, 2] : d === 1 ? [0, 2] : [0, 1];
    for (let k = 0; k < 3; k++) {
      uv[(i + k) * 2] = P(i + k, u0) / scale;
      uv[(i + k) * 2 + 1] = P(i + k, v0) / scale;
    }
  }
  const out = new THREE.BufferGeometry();
  const tris = isSheet ? [...face, ...edge] : null;
  const take = (attr: THREE.BufferAttribute | Float32Array, items: number): THREE.BufferAttribute => {
    const arr = attr instanceof Float32Array ? attr : (attr.array as Float32Array);
    if (!tris) return new THREE.BufferAttribute(arr.slice(), items);
    const o = new Float32Array(pos.count * items);
    tris.forEach((t, j) => o.set(arr.subarray(t * 3 * items, (t + 1) * 3 * items), j * 3 * items));
    return new THREE.BufferAttribute(o, items);
  };
  out.setAttribute('position', take(pos, 3));
  out.setAttribute('normal', take(nor, 3));
  out.setAttribute('uv', take(uv, 2));
  if (isSheet) {
    out.addGroup(0, face.length * 3, 0);
    out.addGroup(face.length * 3, edge.length * 3, 1);
  }
  out.computeBoundingSphere();
  out.computeBoundingBox();
  if (g !== src) g.dispose();
  return { geometry: out, info: isSheet ? { thin, thickness: size[thin]! } : null };
}

/** Baltic birch's plies for a board ``mm`` thick (3 mm: 3 plies, 6: 5, 9: 7, 12: 9, 18: 13). */
export function pliesFor(mm: number): number {
  const n = Math.max(3, Math.round(mm / 1.35));
  return n % 2 ? n : n + 1;
}

// -- the materials ----------------------------------------------------------------------------

/** Blended glass without transmission: opaque-er at grazing angles (Fresnel), so a vessel
 * reads by its silhouette and rim as real glass does; ``edge`` the alpha at grazing. */
function fresnelGlass(m: THREE.MeshPhysicalMaterial, edge: number): THREE.MeshPhysicalMaterial {
  m.side = THREE.FrontSide;
  m.onBeforeCompile = (shader) => {
    shader.fragmentShader = shader.fragmentShader.replace('#include <opaque_fragment>', `
      float fresnelK = pow(1.0 - clamp(abs(dot(normalize(normal), normalize(vViewPosition))), 0.0, 1.0), 2.2);
      gl_FragColor = vec4(outgoingLight, mix(diffuseColor.a, ${edge.toFixed(3)}, fresnelK));
    `);
  };
  m.customProgramCacheKey = () => `fresnel-glass-${edge}`;
  return m;
}

export type Kind = 'alu_link' | 'alu_frame' | 'alu_bar' | 'plywood' | 'stainless' | 'zinc' | 'steel'
  | 'black_oxide' | 'anodized' | 'brass' | 'gearbox' | 'petg' | 'pla' | 'tpu' | 'led'
  | 'glass_green' | 'glass_clear' | 'other';

/** A baked part's kind: its material's name (the fabrication kind), then its node's extras
 * (``finish``, ``filament``), then its name. */
export function kindOf(material: string, node: string, extras: { finish?: string; filament?: string } = {}): Kind {
  const n = node.toLowerCase();
  switch (material) {
    case 'aluminium': return 'alu_link';
    case 'aluminium_frame': return 'alu_frame';
    case 'plywood': return 'plywood';
    case 'metal':
      switch (extras.finish) {
        case 'hard_anodized': return 'anodized';
        case 'black_oxide': return 'black_oxide';
        case 'zinc': return 'zinc';
        case 'stainless': return 'stainless';
        case 'aluminium': return 'alu_bar';
        case 'brass': return 'brass';
        default:
          return extras.finish === undefined && /screw|nut|washer|bolt|cap|set_screw|stud/.test(n) ? 'stainless' : 'steel';
      }
    case 'gearbox': return 'gearbox';
    case 'printed': case 'pla': case 'petg': case 'print':
      if (extras.filament) return /tpu/.test(extras.filament) ? 'tpu' : /petg/.test(extras.filament) ? 'petg' : 'pla';
      return /liner|boot|foot_pad|pad|tpu|bumper/.test(n) ? 'tpu' : 'pla';
    case 'tpu': return 'tpu';
    case 'led': return 'led';
    case 'glass_bottle': return 'glass_green';
    case 'glass_clear': return 'glass_clear';
    default: return /liner|boot/.test(n) ? 'tpu' : 'other';
  }
}

/** A texture's own copy repeating ``rx`` × ``ry`` (the maps are shared, their scale isn't). */
function tiled(t: THREE.Texture, rx = 1, ry = 1): THREE.Texture {
  const c = t.clone();
  c.repeat.set(rx, ry);
  c.needsUpdate = true;
  return c;
}

export function createLook(stage: Stage, onChange: () => void): Look {
  const { renderer, scene } = stage;
  const quality = detectQuality(renderer);
  const high = quality === 'high';
  // Khronos PBR Neutral: true base colours (the red of a toggle zone stays red, the wine
  // garnet), highlights rolled off; ACES shifted both
  renderer.toneMapping = THREE.NeutralToneMapping;
  renderer.toneMappingExposure = 1.0;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  renderer.localClippingEnabled = true;
  if (!high) renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));

  // the stage's helpers and lights out; ours in
  stage.grid.visible = false;
  for (const o of [...scene.children]) {
    if ((o as THREE.Light).isLight || o.type === 'AxesHelper') scene.remove(o);
  }
  const pmrem = new THREE.PMREMGenerator(renderer);
  const env = pmrem.fromScene(studioScene(), 0.02).texture;
  pmrem.dispose();
  scene.environment = env;
  scene.environmentIntensity = 1;
  // the room behind: a warm grey wall, lighter where the lamp falls, the table fading into it
  const ROOM = '#2c2622';
  scene.background = backdrop(ROOM);
  const fog = new THREE.Fog(ROOM, 2000, 6000);
  scene.fog = fog;

  const key = new THREE.DirectionalLight('#fff1df', 2.4);
  key.castShadow = true;
  key.shadow.mapSize.set(high ? 2048 : 1024, high ? 2048 : 1024);
  key.shadow.radius = high ? 6 : 4;
  key.shadow.bias = -0.0004;
  key.shadow.normalBias = 0.6;
  scene.add(key, key.target);
  const keyDir = new THREE.Vector3(-0.55, 0.85, 1.25).normalize();
  const rim = new THREE.DirectionalLight('#dce6ff', 0.8);
  scene.add(rim, rim.target);
  const rimDir = new THREE.Vector3(0.4, -1, 0.7).normalize();
  scene.add(new THREE.HemisphereLight('#efe6da', '#3a2a1e', 0.3));

  // the fog follows the camera: the table fades into the room past the cradle
  scene.onBeforeRender = (_r, _s, camera) => {
    const d = camera.position.distanceTo(stage.controls.target);
    fog.near = d * 1.3;
    fog.far = d * 3.4;
  };

  // the table: a walnut top, wide enough to run out of the picture into the room
  const TABLE: [number, number, number] = [5400, 3000, 32];
  const TILE: [number, number] = [1800, 1000];
  const wood = woodMaps(2048, 1152, 7);
  const rep = (t: THREE.Texture): THREE.Texture => tiled(t, TABLE[0] / TILE[0], TABLE[1] / TILE[1]);
  const tableTop = new THREE.MeshPhysicalMaterial({
    map: rep(wood.map), bumpMap: rep(wood.bump), bumpScale: 0.6, roughnessMap: rep(wood.rough), roughness: 1,
    clearcoat: 0.3, clearcoatRoughness: 0.3, envMapIntensity: 0.7,
  });
  const tableSide = new THREE.MeshPhysicalMaterial({ color: '#3b2416', roughness: 0.55, clearcoat: 0.2 });
  const table = new THREE.Mesh(new THREE.BoxGeometry(...TABLE),
    [tableSide, tableSide, tableSide, tableSide, tableTop, tableSide]);
  table.receiveShadow = true;
  table.name = 'table';
  scene.add(table);

  // the contact shadows: under the stand's footprint, under the vessel's foot
  const footprint = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({
    color: '#000', transparent: true, opacity: 0.55, depthWrite: false,
    alphaMap: blurTexture([256, 256], 14, (ctx) => ctx.fillRect(40, 40, 176, 176)),
  }));
  footprint.renderOrder = -1;
  scene.add(footprint);
  const foot = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({
    color: '#000', transparent: true, opacity: 0.4, depthWrite: false,
    alphaMap: blurTexture([128, 128], 9, (ctx) => { ctx.beginPath(); ctx.arc(64, 64, 40, 0, Math.PI * 2); ctx.fill(); }),
  }));
  foot.renderOrder = -1;
  foot.visible = false;
  scene.add(foot);

  // the maps (one each, drawn once; a material takes its own tiling)
  const bead = beadGrain(), brush = brushed(), cast = castGrain(), birch = birchFace();
  const plyMaps = new Map<number, THREE.Texture>();
  const plyMap = (n: number): THREE.Texture => {
    let t = plyMaps.get(n);
    if (!t) { t = plyEdge(n); plyMaps.set(n, t); }
    return t;
  };

  // the physical materials
  const phys = (p: THREE.MeshPhysicalMaterialParameters, envI: number): THREE.MeshPhysicalMaterial => {
    const m = new THREE.MeshPhysicalMaterial(p);
    m.envMap = env;
    m.envMapIntensity = envI;
    return m;
  };
  const glassGreen = high
    ? phys({ color: '#ffffff', metalness: 0, roughness: 0.04, transmission: 1, ior: 1.52, thickness: 8,
      attenuationColor: '#2f5a2c', attenuationDistance: 6, specularIntensity: 1, dispersion: 0.4 }, 1.3)
    : fresnelGlass(phys({ color: '#1d4426', metalness: 0, roughness: 0.05, transparent: true, opacity: 0.6,
      depthWrite: false, ior: 1.5, specularIntensity: 1, clearcoat: 1, clearcoatRoughness: 0.03 }, 1.6), 0.88);
  // the decanter's glass: blended in both (the wine in it shows through), clear with a faint
  // green edge, bright reflections, opaque at grazing angles: it reads by its rim and outline
  const glassClear = fresnelGlass(phys({ color: '#eef5f2', metalness: 0, roughness: 0.02, transparent: true,
    opacity: 0.07, depthWrite: false, ior: 1.5, specularIntensity: 1, clearcoat: 1, clearcoatRoughness: 0.01 },
  2.4), 0.62);
  // red wine: near black in depth, garnet at a thin edge, a mirror at its surface
  const wine = phys({ color: '#2a030a', metalness: 0, roughness: 0.03, clearcoat: 1, clearcoatRoughness: 0.02,
    sheen: 0.5, sheenColor: new THREE.Color('#7a0d24'), sheenRoughness: 0.35 }, 1.0);
  // (transmission: the light through ``thickness`` mm of it falls to the attenuation colour
  // to the power thickness / distance: a decanter's body near black, ruby at a thin edge)
  const wineLit = (thickness: number, distance: number): THREE.MeshPhysicalMaterial => (high
    ? phys({ color: '#ffffff', metalness: 0, roughness: 0.02, transmission: 1, ior: 1.34, thickness,
      attenuationColor: '#7b0b1f', attenuationDistance: distance, specularIntensity: 1 }, 1.1)
    : thickness > 10
      ? phys({ color: '#2a030a', metalness: 0, roughness: 0.03, clearcoat: 1, clearcoatRoughness: 0.02,
        sheen: 0.6, sheenColor: new THREE.Color('#8a1028'), sheenRoughness: 0.35 }, 1.0)
      : phys({ color: '#4d0715', metalness: 0, roughness: 0.2, emissive: '#3a0610' }, 0.35));
  const wineDeep = wineLit(36, 22), wineThin = wineLit(3, 3.5);
  const printColor = 'charcoal';
  // PETG prints with a soft gloss; PLA dries matte; both take the picked colour
  const petg = phys({ color: PRINT_COLORS[printColor], roughness: 0.4, metalness: 0, clearcoat: 0.25,
    clearcoatRoughness: 0.45, bumpMap: tiled(bead, 3, 3), bumpScale: 0.15 }, 0.6);
  const pla = phys({ color: PRINT_COLORS[printColor], roughness: 0.72, metalness: 0,
    sheen: 0.25, sheenColor: new THREE.Color('#ffffff'), sheenRoughness: 0.8,
    bumpMap: tiled(bead, 3, 3), bumpScale: 0.2 }, 0.5);
  const tpu = phys({ color: '#161616', roughness: 0.9, metalness: 0 }, 0.3);
  // bead-blasted 6061 links: an even satin, the grain in its roughness
  const aluLink = phys({ color: '#d2d5d8', metalness: 1, roughness: 0.52, roughnessMap: tiled(bead, 1, 1),
    bumpMap: tiled(bead, 1, 1), bumpScale: 0.25 }, 1.0);
  // brushed 5052 plates: streaks along the plate, a little brighter at their sheen
  const aluFrame = phys({ color: '#c9cdd1', metalness: 1, roughness: 0.36, roughnessMap: tiled(brush, 1, 6),
    bumpMap: tiled(brush, 1, 6), bumpScale: 0.02 }, 1.05);
  // a laser-cut edge: duller and darker than the face
  const aluEdge = phys({ color: '#a7abaf', metalness: 1, roughness: 0.62 }, 0.9);
  // the base board, cleats and locator: oiled Baltic birch, plies on the edges
  const birchMat = phys({ map: tiled(birch.map), bumpMap: tiled(birch.bump), bumpScale: 0.4, roughness: 0.6,
    metalness: 0, sheen: 0.2, sheenColor: new THREE.Color('#fff4e0'), sheenRoughness: 0.7 }, 0.6);
  const plyEdgeMats = new Map<number, THREE.MeshPhysicalMaterial>();
  const plyEdgeMat = (n: number): THREE.MeshPhysicalMaterial => {
    let m = plyEdgeMats.get(n);
    if (!m) {
      m = phys({ map: tiled(plyMap(n), 1, 1), roughness: 0.75, metalness: 0 }, 0.5);
      plyEdgeMats.set(n, m);
    }
    return m;
  };
  const byKind: Record<Kind, THREE.Material> = {
    alu_link: aluLink,
    alu_frame: aluFrame,
    // bought 6061 rod and tube, cold finished / extruded: brighter, smoother
    alu_bar: phys({ color: '#d4d7da', metalness: 1, roughness: 0.26 }, 1.0),
    plywood: birchMat,
    // 18-8 stainless screws, nuts and washers
    stainless: phys({ color: '#c4c7ca', metalness: 1, roughness: 0.24 }, 1.0),
    // zinc-plated steel: bluish bright
    zinc: phys({ color: '#c9d1da', metalness: 1, roughness: 0.28 }, 1.0),
    // plain steel (the output shaft): a ground bright steel
    steel: phys({ color: '#a9adb1', metalness: 1, roughness: 0.22 }, 1.0),
    // black-oxide steel (the clutch's spring plungers)
    black_oxide: phys({ color: '#1f2023', metalness: 0.75, roughness: 0.42 }, 0.9),
    // hard-anodized 6061 (the link pins, the pivot rods): a dark grey satin
    anodized: phys({ color: '#4e5053', metalness: 0.55, roughness: 0.42, clearcoat: 0.25, clearcoatRoughness: 0.5 }, 0.9),
    brass: phys({ color: '#c9a45a', metalness: 1, roughness: 0.3 }, 1.0),
    // the worm gearbox: a bare die-cast aluminium housing (the NRV050 is unpainted), its
    // casting's grain
    gearbox: phys({ color: '#a9adb1', metalness: 0.85, roughness: 0.55, bumpMap: tiled(cast, 2, 2), bumpScale: 0.15,
      clearcoat: 0.15, clearcoatRoughness: 0.6 }, 0.95),
    petg,
    pla,
    tpu,
    led: new THREE.MeshStandardMaterial({ color: '#fff3d6', emissive: '#ffd9a0', emissiveIntensity: 2.5 }),
    glass_green: glassGreen,
    glass_clear: glassClear,
    other: phys({ color: '#9a9ea3', metalness: 0.5, roughness: 0.5 }, 0.8),
  };
  const SHEETS: Partial<Record<Kind, true>> = { alu_link: true, alu_frame: true, plywood: true };
  // a map's scale: mm per UV unit
  const SCALE: Partial<Record<Kind, number>> = { alu_link: 60, alu_frame: 300, plywood: 400, gearbox: 40, petg: 30, pla: 30 };

  const look: Look = {
    quality,
    env,
    materials: { glassGreen, glassClear, wine, wineDeep, wineThin, print: pla, tpu, steel: byKind.stainless, alu: aluLink },
    printColor,

    dress(loaded) {
      const done = new Map<THREE.BufferGeometry, { geometry: THREE.BufferGeometry; info: SheetInfo | null }>();
      loaded.root.traverse((obj) => {
        const mesh = obj as THREE.Mesh;
        if (!mesh.isMesh || (mesh as unknown as THREE.Line).isLine) return;
        const old = mesh.material as THREE.MeshStandardMaterial;
        const kind = kindOf(old.name, mesh.name, mesh.userData as { finish?: string; filament?: string });
        const sheet = !!SHEETS[kind];
        // the baked meshes carry positions only: creased normals (smooth round parts, crisp
        // edges), UVs; a sheet part split into its faces and its cut edges
        let d = done.get(mesh.geometry);
        if (!d) {
          d = dressGeometry(mesh.geometry, SCALE[kind] ?? 100, sheet);
          done.set(mesh.geometry, d);
        }
        mesh.geometry = d.geometry;
        if (d.info) {
          const edgeMat = kind === 'plywood' ? plyEdgeMat(pliesFor(d.info.thickness)) : aluEdge;
          mesh.material = [byKind[kind], edgeMat];
        } else {
          mesh.material = byKind[kind];
        }
        if (old !== mesh.material) old.dispose();
        const glass = kind === 'glass_green' || kind === 'glass_clear';
        mesh.castShadow = !glass;
        mesh.receiveShadow = !glass;
      });
      for (const g of done.keys()) if (done.get(g)!.geometry !== g) g.dispose();
      if (loaded.footLine) loaded.footLine.visible = false;   // the mouth's path: a diagram's, not a photo's
      onChange();
    },

    fit(box) {
      const c = box.getCenter(new THREE.Vector3()), size = box.getSize(new THREE.Vector3());
      const floor = 0;   // the cradle stands on z = 0
      table.position.set(c.x, c.y, floor - TABLE[2] / 2);
      const reach = Math.max(size.x, size.y, size.z);
      key.target.position.set(c.x, c.y, floor + size.z * 0.35);
      key.position.copy(key.target.position).addScaledVector(keyDir, reach * 3);
      const cam = key.shadow.camera;
      const half = reach * 0.85;
      cam.left = -half; cam.right = half; cam.top = half; cam.bottom = -half;
      cam.near = reach * 0.5; cam.far = reach * 6;
      cam.updateProjectionMatrix();
      rim.target.position.copy(key.target.position);
      rim.position.copy(rim.target.position).addScaledVector(rimDir, reach * 3);
      footprint.position.set(c.x, c.y, floor + 0.3);
      footprint.scale.set(size.x * 1.25, size.y * 1.25, 1);
      onChange();
    },

    footShadow(x, y, radius) {
      foot.visible = radius > 0;
      foot.position.set(x, y, 0.35);
      foot.scale.setScalar(radius * 2 * 1.35);
      onChange();
    },

    setPrintColor(name) {
      const hex = PRINT_COLORS[name];
      if (!hex) return;
      look.printColor = name;
      petg.color.set(hex);
      pla.color.set(hex);
      onChange();
    },
  };
  return look;
}

/** A blurred dark shape for a contact shadow (``draw`` paints it into a ``size`` canvas). */
function blurTexture(size: [number, number], blur: number, draw: (ctx: CanvasRenderingContext2D) => void): THREE.CanvasTexture {
  const cv = document.createElement('canvas');
  cv.width = size[0]; cv.height = size[1];
  const ctx = cv.getContext('2d')!;
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, size[0], size[1]);
  ctx.filter = `blur(${blur}px)`;
  ctx.fillStyle = '#fff';            // an alpha map reads the green channel
  draw(ctx);
  return new THREE.CanvasTexture(cv);
}

/** The room behind the table: a warm wall, a soft pool of lamp light high in the middle,
 * ``floor`` (the fog's colour) at the bottom, so the table's far edge fades into it. */
function backdrop(floor: string): THREE.CanvasTexture {
  const cv = document.createElement('canvas');
  cv.width = 512; cv.height = 512;
  const ctx = cv.getContext('2d')!;
  const g = ctx.createLinearGradient(0, 0, 0, 512);
  g.addColorStop(0, '#4a423b');
  g.addColorStop(0.45, '#3a332d');
  g.addColorStop(0.8, floor);
  g.addColorStop(1, floor);
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, 512, 512);
  const v = ctx.createRadialGradient(256, 170, 40, 256, 220, 380);
  v.addColorStop(0, 'rgba(255,232,200,0.10)');
  v.addColorStop(1, 'rgba(0,0,0,0.25)');
  ctx.fillStyle = v;
  ctx.fillRect(0, 0, 512, 512);
  const t = new THREE.CanvasTexture(cv);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}
