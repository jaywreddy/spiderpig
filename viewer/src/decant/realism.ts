/**
 * The cradle page's photographic look: ACES tone mapping, a procedural studio
 * environment (softboxes and a window, no downloaded HDRI: the box may be offline), a
 * key light with soft shadows, a walnut tabletop drawn in a canvas, a contact shadow
 * under the stand, and physical materials per fabrication kind (the baker names each
 * material after it: ``aluminium``, ``aluminium_frame``, ``plywood``, ``metal``, ``gearbox``,
 * ``printed``, ``led``, ``glass_bottle``, ``glass_clear``; a node's name tells the
 * fasteners, the steel standoffs and the TPU liners and feet apart).
 *
 * ``quality``: ``high`` with transmission (refracting glass) and 2k shadows; ``low`` on a
 * software rasterizer (SwiftShader, llvmpipe) or a phone: glass as blended layers, 1k
 * shadows. ``?quality=high|low`` forces either.
 */
import * as THREE from 'three';
import { toCreasedNormals } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import type { Stage } from '../scene';
import type { LoadedScene } from '../loader';

export type Quality = 'high' | 'low';

/** The print colours on offer (PLA, matte). */
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
    glassClear: THREE.MeshPhysicalMaterial;
    wine: THREE.MeshPhysicalMaterial;
    print: THREE.MeshPhysicalMaterial;
    tpu: THREE.MeshPhysicalMaterial;
    /** zinc-plated steel and bead-blasted aluminium, for what the page adds */
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

// -- the environment: a dim room, two softboxes, a strip light and a window ------------------

function glow(color: THREE.ColorRepresentation, intensity: number): THREE.MeshBasicMaterial {
  const m = new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide });
  m.color.multiplyScalar(intensity);
  return m;
}

/** World units here are metres-ish; Z up (the page's world). */
function studioScene(): THREE.Scene {
  const scene = new THREE.Scene();
  // the room: a sphere, warm floor, neutral walls, a lighter ceiling
  const room = new THREE.SphereGeometry(20, 48, 24);
  room.rotateX(Math.PI / 2);   // poles on Z
  const pos = room.attributes.position!;
  const colors = new Float32Array(pos.count * 3);
  const floor = new THREE.Color('#3a2a1f'), wall = new THREE.Color('#827d76'), ceil = new THREE.Color('#aaa59c');
  const c = new THREE.Color();
  for (let i = 0; i < pos.count; i++) {
    const z = pos.getZ(i) / 20;
    if (z < 0) c.copy(wall).lerp(floor, Math.min(1, -z * 3));
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
  panel(9, 7, glow('#fff4e6', 14), [-6, 9, 10]);
  // a fill softbox, low front-right
  panel(7, 5, glow('#f2f4ff', 3), [10, 6, 2]);
  // a strip light behind, for rim highlights on the glass
  panel(1.6, 9, glow('#ffffff', 7), [4, -12, 5]);
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
  // a warm lamp glow overhead
  panel(3, 3, glow('#ffd9a8', 2.2), [0, 0, 16]);
  return scene;
}

// -- the tabletop: walnut planks drawn in a canvas ----------------------------------------------

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

function fbm(x: number, y: number): number {
  return noise(x, y) * 0.55 + noise(x * 2.1, y * 2.1) * 0.3 + noise(x * 4.3, y * 4.3) * 0.15;
}

interface WoodMaps { map: THREE.CanvasTexture; bump: THREE.CanvasTexture; rough: THREE.CanvasTexture }

/** Planks along X: cathedral grain, fine pores, seams and butt joints. ``W`` × ``H`` px
 * over the table's ``widthMm`` × ``depthMm``. */
function woodMaps(W: number, H: number, planks: number): WoodMaps {
  const make = (): [HTMLCanvasElement, CanvasRenderingContext2D, ImageData] => {
    const cv = document.createElement('canvas');
    cv.width = W; cv.height = H;
    const ctx = cv.getContext('2d')!;
    return [cv, ctx, ctx.createImageData(W, H)];
  };
  const [cMap, xMap, dMap] = make(), [cBump, xBump, dBump] = make(), [cRough, xRough, dRough] = make();
  const light = [0.52, 0.34, 0.21], dark = [0.25, 0.14, 0.08];
  const ph = H / planks;
  const joints: number[][] = [];
  for (let p = 0; p < planks; p++) joints.push([hash(p, 7) * W * 0.6 + W * 0.2, hash(p, 9)]);
  for (let y = 0; y < H; y++) {
    const p = Math.min(planks - 1, Math.floor(y / ph));
    const yl = y - p * ph;
    const [jx] = joints[p]!;
    for (let x = 0; x < W; x++) {
      const seg = x < jx! ? 0 : 1;        // the two boards of a plank row, end to end
      const seed = p * 13 + seg * 5;
      const tone = 0.85 + 0.3 * hash(seed, 3);
      const cx = W * (0.25 + 0.5 * hash(seed, 1));
      const warp = fbm(x / 260 + seed, yl / 55) * 34;
      const arch = ((x - cx) / W) ** 2 * (120 + 160 * hash(seed, 2));
      const ring = (yl + warp + arch + seed * 17) / (7 + 4 * hash(seed, 4));
      const f = ring - Math.floor(ring);
      const late = Math.pow(Math.sin(f * Math.PI), 6);            // the dark late wood
      const pores = noise(x / 2.2 + seed * 31, yl * 1.6);
      const figure = fbm(x / 90 + seed, yl / 12) - 0.5;
      let k = 0.15 + 0.55 * late + 0.25 * (pores > 0.82 ? 1 : 0) + 0.25 * figure;
      k = Math.min(1, Math.max(0, k));
      // the seams: the plank's edges and the butt joint
      const edge = Math.min(yl, ph - yl), butt = Math.abs(x - jx!);
      const seam = edge < 1.2 || butt < 1.0 ? 1 : edge < 2.5 || butt < 2 ? 0.45 : 0;
      const i = (y * W + x) * 4;
      for (let ch = 0; ch < 3; ch++) {
        const v = (light[ch]! * (1 - k) + dark[ch]! * k) * tone * (1 - seam * 0.7);
        dMap.data[i + ch] = Math.round(255 * Math.min(1, v));
      }
      dMap.data[i + 3] = 255;
      const b = 0.55 - 0.25 * late - 0.25 * (pores > 0.8 ? 1 : 0) - 0.5 * seam;
      dBump.data[i] = dBump.data[i + 1] = dBump.data[i + 2] = Math.round(255 * Math.max(0, b));
      dBump.data[i + 3] = 255;
      const r = 0.42 + 0.22 * late + 0.25 * seam + 0.1 * (pores > 0.8 ? 1 : 0);
      dRough.data[i] = dRough.data[i + 1] = dRough.data[i + 2] = Math.round(255 * Math.min(1, r));
      dRough.data[i + 3] = 255;
    }
  }
  xMap.putImageData(dMap, 0, 0); xBump.putImageData(dBump, 0, 0); xRough.putImageData(dRough, 0, 0);
  const tex = (cv: HTMLCanvasElement, srgb: boolean): THREE.CanvasTexture => {
    const t = new THREE.CanvasTexture(cv);
    if (srgb) t.colorSpace = THREE.SRGBColorSpace;
    t.anisotropy = 8;
    return t;
  };
  return { map: tex(cMap, true), bump: tex(cBump, false), rough: tex(cRough, false) };
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

function backdrop(): THREE.CanvasTexture {
  const cv = document.createElement('canvas');
  cv.width = 512; cv.height = 512;
  const ctx = cv.getContext('2d')!;
  const g = ctx.createLinearGradient(0, 0, 0, 512);
  g.addColorStop(0, '#2e2924');
  g.addColorStop(0.55, '#1d1916');
  g.addColorStop(1, '#100e0c');
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, 512, 512);
  const v = ctx.createRadialGradient(256, 210, 60, 256, 256, 420);
  v.addColorStop(0, 'rgba(255,236,210,0.07)');
  v.addColorStop(1, 'rgba(0,0,0,0.35)');
  ctx.fillStyle = v;
  ctx.fillRect(0, 0, 512, 512);
  const t = new THREE.CanvasTexture(cv);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
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

type Kind = 'alu_link' | 'alu_frame' | 'plywood' | 'zinc' | 'steel' | 'gearbox' | 'print' | 'tpu' | 'led' | 'glass_green' | 'glass_clear' | 'other';

function kindOf(material: string, node: string): Kind {
  const n = node.toLowerCase();
  switch (material) {
    case 'aluminium': return 'alu_link';
    case 'aluminium_frame': return 'alu_frame';
    case 'plywood': return 'plywood';
    case 'metal':
      return /screw|nut|washer|bolt|cap|set_screw|stud/.test(n) ? 'zinc' : 'steel';
    case 'gearbox': return 'gearbox';
    case 'printed': case 'pla': case 'petg': case 'print':
      return /liner|boot|foot_pad|pad|tpu|bumper/.test(n) ? 'tpu' : 'print';
    case 'tpu': return 'tpu';
    case 'led': return 'led';
    case 'glass_bottle': return 'glass_green';
    case 'glass_clear': return 'glass_clear';
    default: return /liner|boot/.test(n) ? 'tpu' : 'other';
  }
}

export function createLook(stage: Stage, onChange: () => void): Look {
  const { renderer, scene } = stage;
  const quality = detectQuality(renderer);
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.0;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  renderer.localClippingEnabled = true;
  if (quality === 'low') renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));

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
  scene.background = backdrop();
  const fog = new THREE.Fog('#16130f', 2000, 6000);
  scene.fog = fog;

  const key = new THREE.DirectionalLight('#fff1df', 2.6);
  key.castShadow = true;
  key.shadow.mapSize.set(quality === 'high' ? 2048 : 1024, quality === 'high' ? 2048 : 1024);
  key.shadow.radius = quality === 'high' ? 7 : 4;
  key.shadow.bias = -0.0004;
  key.shadow.normalBias = 0.6;
  scene.add(key, key.target);
  const keyDir = new THREE.Vector3(-0.55, 0.85, 1.25).normalize();
  const rim = new THREE.DirectionalLight('#dce6ff', 0.9);
  scene.add(rim, rim.target);
  const rimDir = new THREE.Vector3(0.4, -1, 0.7).normalize();
  scene.add(new THREE.HemisphereLight('#efe6da', '#3a2a1e', 0.25));

  // the fog follows the camera: the table fades into the room past the cradle
  scene.onBeforeRender = (_r, _s, camera) => {
    const d = camera.position.distanceTo(stage.controls.target);
    fog.near = d * 1.25;
    fog.far = d * 3.2;
  };

  // the table
  const TABLE: [number, number, number] = [1800, 1000, 32];
  const wood = woodMaps(2048, 1152, 7);
  const tableTop = new THREE.MeshPhysicalMaterial({
    map: wood.map, bumpMap: wood.bump, bumpScale: 0.6, roughnessMap: wood.rough, roughness: 1,
    clearcoat: 0.35, clearcoatRoughness: 0.28, envMapIntensity: 0.7,
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

  // the physical materials
  const phys = (p: THREE.MeshPhysicalMaterialParameters, envI: number): THREE.MeshPhysicalMaterial => {
    const m = new THREE.MeshPhysicalMaterial(p);
    m.envMap = env;
    m.envMapIntensity = envI;
    return m;
  };
  const high = quality === 'high';
  const glassGreen = high
    ? phys({ color: '#ffffff', metalness: 0, roughness: 0.04, transmission: 1, ior: 1.5, thickness: 7,
      attenuationColor: '#3d7a43', attenuationDistance: 9, specularIntensity: 1 }, 1.2)
    : fresnelGlass(phys({ color: '#2a5c33', metalness: 0, roughness: 0.05, transparent: true, opacity: 0.4,
      depthWrite: false, ior: 1.5, specularIntensity: 1, clearcoat: 1, clearcoatRoughness: 0.03 }, 1.6), 0.85);
  const glassClear = high
    ? phys({ color: '#ffffff', metalness: 0, roughness: 0.02, transmission: 1, ior: 1.5, thickness: 3,
      specularIntensity: 1 }, 1.3)
    : fresnelGlass(phys({ color: '#e8f0f4', metalness: 0, roughness: 0.03, transparent: true, opacity: 0.08,
      depthWrite: false, ior: 1.5, specularIntensity: 1, clearcoat: 1, clearcoatRoughness: 0.02 }, 2.2), 0.6);
  // red wine: near black in depth, garnet at a thin edge, a mirror at its surface
  const wine = phys({ color: '#2c030b', metalness: 0, roughness: 0.04, clearcoat: 1, clearcoatRoughness: 0.02,
    sheen: 0.6, sheenColor: new THREE.Color('#8a1028'), sheenRoughness: 0.35 }, 1.0);
  const printColor = 'charcoal';
  const print = phys({ color: PRINT_COLORS[printColor], roughness: 0.78, metalness: 0,
    sheen: 0.25, sheenColor: new THREE.Color('#ffffff'), sheenRoughness: 0.8 }, 0.5);
  const tpu = phys({ color: '#141414', roughness: 0.92, metalness: 0 }, 0.3);
  const byKind: Record<Kind, THREE.Material> = {
    // 6061-T6 links, bead-blasted: a fine matte satin
    alu_link: phys({ color: '#d5d8db', metalness: 1, roughness: 0.38 }, 1.1),
    // 5052 frame plates, mill / brushed finish: a touch glossier, a touch darker
    alu_frame: phys({ color: '#c8cbcf', metalness: 1, roughness: 0.27 }, 1.1),
    // the base board: routed Baltic birch plywood, oiled (no UVs on the baked meshes: a tone)
    plywood: phys({ color: '#d6b98c', metalness: 0, roughness: 0.62, sheen: 0.2,
      sheenColor: new THREE.Color('#fff4e0'), sheenRoughness: 0.7 }, 0.6),
    // zinc-plated screws, nuts and washers: bluish bright
    zinc: phys({ color: '#c9d0d8', metalness: 1, roughness: 0.24 }, 1.0),
    // 1018 steel standoffs and shafts, zinc-plated too, duller
    steel: phys({ color: '#a8abaf', metalness: 1, roughness: 0.33 }, 1.0),
    // the NRV040 worm box: cast, painted RAL 5010 gentian blue
    gearbox: phys({ color: '#0e4c86', metalness: 0.0, roughness: 0.5, clearcoat: 0.35, clearcoatRoughness: 0.45 }, 0.8),
    print,
    tpu,
    led: new THREE.MeshStandardMaterial({ color: '#fff3d6', emissive: '#ffd9a0', emissiveIntensity: 2.5 }),
    glass_green: glassGreen,
    glass_clear: glassClear,
    other: phys({ color: '#9a9ea3', metalness: 0.5, roughness: 0.5 }, 0.8),
  };

  const look: Look = {
    quality,
    env,
    materials: { glassGreen, glassClear, wine, print, tpu, steel: byKind.steel, alu: byKind.alu_link },
    printColor,

    dress(loaded) {
      const smoothed = new Map<THREE.BufferGeometry, THREE.BufferGeometry>();
      loaded.root.traverse((obj) => {
        const mesh = obj as THREE.Mesh;
        if (!mesh.isMesh || (mesh as unknown as THREE.Line).isLine) return;
        const old = mesh.material as THREE.MeshStandardMaterial;
        const kind = kindOf(old.name, mesh.name);
        mesh.material = byKind[kind];
        if (old !== mesh.material) old.dispose();
        // the baked meshes carry positions only: creased normals, smooth round parts, crisp edges
        let g = smoothed.get(mesh.geometry);
        if (!g) {
          g = mesh.geometry.attributes.normal ? mesh.geometry : toCreasedNormals(mesh.geometry, THREE.MathUtils.degToRad(32));
          smoothed.set(mesh.geometry, g);
        }
        if (g !== mesh.geometry) mesh.geometry = g;
        const glass = kind === 'glass_green' || kind === 'glass_clear';
        mesh.castShadow = !glass;
        mesh.receiveShadow = !glass;
      });
      for (const g of smoothed.keys()) if (smoothed.get(g) !== g) g.dispose();
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
      print.color.set(hex);
      onChange();
    },
  };
  return look;
}
