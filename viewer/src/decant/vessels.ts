/**
 * What the cradle pours into: decanters and wine glasses at their real sizes, each a glass
 * solid of revolution (``profile``: ``[[height_mm, outer_radius_mm], ...]`` from the base
 * to the rim), and the cavity the wine fills (the outline less the glass, from the bowl's
 * floor up).
 *
 * The list: the makers' published heights, widest diameters and capacities (RIEDEL's
 * dimension drawings, Zalto's product pages, ISO 3591's drawing, an Amazon listing for
 * the budget decanter); each profile traced from the maker's photo or drawing and scaled
 * to those published sizes (researched 2026-10-10; the sources in each entry's comments,
 * what was estimated rather than published after them). ``capacity_ml`` is what the maker
 * rates it for (a decanter: the bottle it takes), ``brim_ml`` brim-full when stated.
 */
import { profileVolume, radiusAt, type Profile } from './liquid';

export interface Vessel {
  key: string;
  name: string;
  kind: 'decanter' | 'glass';
  /** the maker's or the trade's word for the shape */
  style: string;
  /** what the maker rates it for, mL (a glass: its full capacity) */
  capacity_ml: number;
  /** brim-full, mL, when the maker states it */
  brim_ml?: number;
  /** the recommended pour, mL */
  pour_ml?: number;
  /** base to rim, outside */
  profile: Profile;
  /** the glass's wall, mm (default: 1 a glass, 2.5 a decanter) */
  wall_mm?: number;
  /** the cavity's floor over the base, mm (default: found from the profile) */
  floor_mm?: number;
  /** where the sizes come from */
  source: string;
}

/** The default: RIEDEL's Ultra decanter (225 mm tall, Ø205 mm: the cradle's bay is 260 mm wide). */
export const DEFAULT_VESSEL = 'riedel_ultra';

export const VESSELS: Vessel[] = [
  // https://www.riedel.com/en-gb/riedel-ultra-decanter/240000014
  // https://riedel.com/en/shop/decanter/ultra-decanter-240000014
  // https://cdn.riedel.com/media/61/4c/ae/1782187945/media_240000014_dimensions.c428a992.png
  // https://www.kitchenwarehouse.com.au/product/riedel-ultra-handmade-decanter-1-2l
  // traced from the maker's photo (outline extraction), scaled to the published height and max diameter (photo/published width ratio +0.7 %); estimated: profile, mouth_d_mm, neck_min_d_mm, max_d_h_mm, neck_min_h_mm, base_d_mm
  {
    key: "riedel_ultra",
    name: "RIEDEL Ultra Decanter (2400/14, handmade)",
    kind: 'decanter',
    style: "wide-base 750 ml decanter",
    capacity_ml: 750,
    profile: [[0, 28.6], [6.3, 38.8], [21.7, 84.2], [30, 95.8], [40.7, 102.5], [47.4, 100.8], [55.4, 91.2], [72.4, 52.4], [79.5, 41.3], [89.5, 32.3], [108.5, 22.9], [133.5, 18.3], [165.9, 19], [189.9, 23.9], [225, 37.1]],
    source: "https://www.riedel.com/en-gb/riedel-ultra-decanter/240000014",
  },
  // https://www.riedel.com/en-gb/shop/decanter/cabernet-decanter-144000013
  // https://cdn.riedel.com/media/bc/69/e9/1790611936/media_144000013_dimensions.a4ec6321.png
  // https://www.mcgrocer.com/products/riedel-cabernet-single-crystal-glass-decanter-1-1l-2026322942
  // traced from the maker's photo (outline extraction), scaled to the published height and max diameter (photo/published width ratio +0.1 %); estimated: profile, mouth_d_mm, neck_min_d_mm, max_d_h_mm, neck_min_h_mm
  {
    key: "riedel_cabernet",
    name: "RIEDEL Cabernet Decanter (1440/13, machine-made)",
    kind: 'decanter',
    style: "~1 L carafe",
    capacity_ml: 750,
    brim_ml: 1100,
    profile: [[0, 32.5], [3.3, 32.9], [5.3, 35.3], [46.4, 58.8], [57.1, 62], [65.1, 61.3], [77.4, 56.8], [88.1, 51.6], [120.2, 33.3], [137.9, 26.4], [152.2, 23.1], [169.6, 21.2], [191.9, 22.4], [210.3, 26.2], [230, 33.3]],
    source: "https://www.riedel.com/en-gb/shop/decanter/cabernet-decanter-144000013",
  },
  // https://www.riedel.com/en-gb/shop/decanter/cabernet-magnum-decanter-144000026
  // https://riedel.com/en/shop/decanter/cabernet-magnum-decanter-144000026
  // https://cdn.riedel.com/media/c8/55/7b/1771992893/144000026_dimensions.31310319.png
  // https://www.maisonlipari.ca/products/riedel-decanter-cabernet-magnum-crystal-v-170-cl-h-27-cm
  // traced from the maker's photo (outline extraction), scaled to the published height and max diameter (photo/published width ratio +13.9 %); estimated: profile, mouth_d_mm, neck_min_d_mm, max_d_h_mm, neck_min_h_mm, base_d_mm
  {
    key: "riedel_cabernet_magnum",
    name: "RIEDEL Cabernet Magnum Decanter (1440/26, machine-made)",
    kind: 'decanter',
    style: "magnum carafe",
    capacity_ml: 1500,
    brim_ml: 1700,
    profile: [[0, 27.1], [4, 37], [7.7, 40.2], [56.3, 67.9], [63.7, 71.4], [69.3, 72.5], [75.7, 72.3], [85, 69.5], [145.3, 36.6], [160.7, 30.7], [175.7, 27], [198, 24.7], [224.3, 26.2], [243.7, 30.2], [269, 38.3]],
    source: "https://www.riedel.com/en-gb/shop/decanter/cabernet-magnum-decanter-144000026",
  },
  // https://www.riedel.com/en-gb/riedel-ultra-magnum-decanter/240000013
  // https://riedel.com/en/shop/decanter/ultra-magnum-decanter-240000013
  // https://cdn.riedel.com/media/95/35/55/1782187945/media_240000013_dimensions.398ddb96.png
  // https://www.maisonlipari.ca/products/riedel-decanter-ultra-magnum-crystal-v-200-cl-h-24-cm
  // traced from the maker's photo (outline extraction), scaled to the published height and max diameter (photo/published width ratio +1.0 %); estimated: profile, mouth_d_mm, neck_min_d_mm, max_d_h_mm, neck_min_h_mm, base_d_mm
  {
    key: "riedel_ultra_magnum",
    name: "RIEDEL Ultra Magnum Decanter (2400/13, handmade)",
    kind: 'decanter',
    style: "wide-base magnum decanter",
    capacity_ml: 1500,
    brim_ml: 2000,
    profile: [[0, 34.2], [9, 74.9], [17, 96.7], [28.6, 114.3], [38.2, 120], [50.2, 115.6], [60.2, 102.5], [76.1, 60.9], [83.4, 48.7], [97.1, 35.4], [111.7, 27.7], [134.3, 22.5], [170.2, 22.8], [203.4, 28.7], [240, 40.8]],
    source: "https://www.riedel.com/en-gb/riedel-ultra-magnum-decanter/240000013",
  },
  // https://www.zaltoglas.at/en/products/denkart/decanter-more/
  // https://www.zaltoglas.at/wp-content/uploads/2021/04/zalto-produkte-840-axium.png
  // traced from the maker's photo (outline extraction), scaled to the published height and max diameter (photo/published width ratio +2.6 %); estimated: profile, mouth_d_mm, neck_min_d_mm, max_d_h_mm, neck_min_h_mm, base_d_mm, capacity_ml
  {
    key: "zalto_axium",
    name: "Zalto Denk'Art Axium Decanter",
    kind: 'decanter',
    style: "named model (mid-size, angular)",
    capacity_ml: 750,
    brim_ml: 1450,
    profile: [[0, 50.5], [4.1, 53.4], [14.9, 56.6], [82.8, 76.4], [91.4, 77.5], [95.8, 74.6], [134.3, 28.1], [139.5, 23.7], [146.9, 23.2], [204, 31.6]],
    source: "https://www.zaltoglas.at/en/products/denkart/decanter-more/",
  },
  // https://www.amazon.com/dp/B01AVTQ1D4
  // https://m.media-amazon.com/images/I/61MBkpjLguL._AC_SL1500_.jpg
  // https://m.media-amazon.com/images/I/71OqRfCr5cL._AC_SL1500_.jpg
  // https://shop.winetraveler.com/product/le-chateau-red-wine-decanter-carafe-aerator
  // traced from the maker's photo (outline extraction), scaled to the published height and max diameter (photo/published width ratio +5.3 %); estimated: profile, mouth_d_mm, neck_min_d_mm, max_d_h_mm, neck_min_h_mm, base_d_mm
  {
    key: "le_chateau",
    name: "Le Chateau Wine Decanter (Amazon B01AVTQ1D4, hand-blown lead-free crystal)",
    kind: 'decanter',
    style: "budget Amazon best-seller (wide base)",
    capacity_ml: 750,
    brim_ml: 1800,
    profile: [[0, 55.2], [7.6, 59.4], [23.1, 101.2], [29.6, 106.5], [37.8, 108], [48.9, 99.7], [71, 48.6], [83.4, 35.4], [99.4, 26.2], [138.8, 23.7], [215.1, 33.7], [254, 33.7]],
    source: "https://www.amazon.com/dp/B01AVTQ1D4",
  },
  // https://www.riedel.com/en-gb/shop/veritas/cabernet-merlot-644900098
  // https://cdn.riedel.com/media/f5/fd/bd/1791605595/media_644900098_dimensions.30bdeba3.png
  // traced from the maker's photo above the foot (outline extraction), scaled to the published height and bowl diameter (photo/published width ratio -1.7 %); foot from a template scaled to the foot diameter; estimated: profile, mouth_d_mm, stem_h_mm, stem_d_mm, bowl_d_h_mm
  {
    key: "riedel_veritas_cabernet",
    name: "RIEDEL Veritas Cabernet/Merlot (6449/0)",
    kind: 'glass',
    style: "Bordeaux (large red)",
    capacity_ml: 667,
    pour_ml: 125,
    profile: [[0, 45], [2.5, 45], [5.5, 41.9], [13, 9.9], [17, 5.9], [28.5, 2.5], [90.9, 4.3], [97.1, 7.5], [106.8, 20.3], [123.9, 34.8], [137, 42], [149.3, 45.7], [179.5, 47.3], [235, 37.8]],
    source: "https://www.riedel.com/en-gb/shop/veritas/cabernet-merlot-644900098",
  },
  // https://www.riedel.com/en-gb/shop/vinum/pinot-noir-burgundy-red-641600007
  // https://cdn.riedel.com/media/3f/48/c3/1780027851/media_641600007_dimensions.a86cda22.png
  // traced from the maker's photo above the foot (outline extraction), scaled to the published height and bowl diameter (photo/published width ratio -0.4 %); foot from a template scaled to the foot diameter; estimated: profile, mouth_d_mm, stem_h_mm, stem_d_mm, bowl_d_h_mm
  {
    key: "riedel_vinum_pinot",
    name: "RIEDEL Vinum Pinot Noir (Burgundy Red) (6416/07)",
    kind: 'glass',
    style: "Burgundy (balloon)",
    capacity_ml: 725,
    pour_ml: 125,
    profile: [[0, 43], [5.5, 40], [13, 9.5], [21.7, 3.8], [36, 2.7], [70.8, 3.4], [84.3, 5.9], [87.4, 8], [97.4, 27.5], [108.4, 40.7], [121.7, 49.9], [135.5, 53.7], [158.4, 52.4], [210, 35.6]],
    source: "https://www.riedel.com/en-gb/shop/vinum/pinot-noir-burgundy-red-641600007",
  },
  // https://www.zaltoglas.at/en/products/denkart/
  // https://www.zaltoglas.at/wp-content/uploads/2021/04/zalto-produkte-683-universal.png
  // traced from the maker's photo above the foot (outline extraction), scaled to the published height and bowl diameter (photo/published width ratio +3.5 %); foot from a template scaled to the foot diameter; estimated: profile, mouth_d_mm, stem_h_mm, stem_d_mm, bowl_d_h_mm, foot_d_mm
  {
    key: "zalto_universal",
    name: "Zalto Denk'Art Universal",
    kind: 'glass',
    style: "universal / all-purpose",
    capacity_ml: 555,
    profile: [[0, 45.7], [2.5, 45.7], [5.5, 42.5], [13.3, 3.3], [19.7, 2.5], [67.6, 1.8], [92.4, 2.7], [100.9, 4.3], [107.6, 9.4], [118.6, 30.8], [125.1, 40], [131.6, 44.5], [136.1, 45.5], [170.7, 42.7], [235, 34]],
    source: "https://www.zaltoglas.at/en/products/denkart/",
  },
  // https://www.zaltoglas.at/en/products/denkart/
  // https://www.zaltoglas.at/wp-content/uploads/2021/04/zalto-produkte-840-weisswein.png
  // traced from the maker's photo above the foot (outline extraction), scaled to the published height and bowl diameter (photo/published width ratio +2.5 %); foot from a template scaled to the foot diameter; estimated: profile, mouth_d_mm, stem_h_mm, stem_d_mm, bowl_d_h_mm, foot_d_mm
  {
    key: "zalto_white",
    name: "Zalto Denk'Art White Wine",
    kind: 'glass',
    style: "white wine",
    capacity_ml: 420,
    profile: [[0, 41.3], [2.5, 41.3], [5.5, 38.4], [13.3, 3.5], [19.9, 2.9], [75.6, 2.4], [94.7, 3.8], [99.7, 5.8], [106.4, 10.3], [134.2, 35.3], [139, 38.2], [145.1, 40.2], [156, 40.1], [230, 29.5]],
    source: "https://www.zaltoglas.at/en/products/denkart/",
  },
  // https://commons.wikimedia.org/wiki/File:Verre_Inao.svg
  // https://en.wikipedia.org/wiki/Wine_glass
  // https://www.iso.org/standard/9002.html
  // https://www.dimensions.com/element/iso-wine-tasting-glass
  // traced from the dimensioned INAO drawing (Commons), stem set to the standard 9 mm; estimated: profile, bowl_d_h_mm
  {
    key: "iso_3591",
    name: "ISO 3591 wine-tasting glass (e.g. Arcoroc Viticole 21.5 cl)",
    kind: 'glass',
    style: "ISO tasting glass (ISO 3591:1977 / INAO)",
    capacity_ml: 215,
    pour_ml: 50,
    wall_mm: 0.8,
    profile: [[0, 32.5], [2.5, 31.5], [9, 7.9], [13, 4.9], [19, 4.5], [51, 4.7], [54, 6.7], [60.5, 18.5], [66.5, 25.3], [75.5, 30.1], [86, 32.4], [95, 32.5], [109, 31.8], [133.5, 28.2], [155, 23]],
    source: "https://commons.wikimedia.org/wiki/File:Verre_Inao.svg",
  },
  // https://www.zaltoglas.at/en/products/denkart/
  // https://www.zaltoglas.at/wp-content/uploads/2021/04/zalto-produkte-683-bordeaux.png
  // traced from the maker's photo above the foot (outline extraction), scaled to the published height and bowl diameter (photo/published width ratio +1.8 %); foot from a template scaled to the foot diameter; estimated: profile, mouth_d_mm, stem_h_mm, stem_d_mm, bowl_d_h_mm, foot_d_mm
  {
    key: "zalto_bordeaux",
    name: "Zalto Denk'Art Bordeaux",
    kind: 'glass',
    style: "Bordeaux (alternative)",
    capacity_ml: 765,
    profile: [[0, 45.1], [2.5, 45.1], [5.5, 42], [10.5, 16.2], [14.8, 4], [25.3, 3], [92.3, 2.7], [111, 4.7], [118.5, 12], [135.6, 46.3], [142.2, 51.8], [148.4, 54], [163.5, 53.3], [240, 36.1]],
    source: "https://www.zaltoglas.at/en/products/denkart/",
  },
  // https://www.zaltoglas.at/en/products/denkart/
  // https://www.zaltoglas.at/wp-content/uploads/2021/04/zalto-produkte-683-burgunder.png
  // traced from the maker's photo above the foot (outline extraction), scaled to the published height and bowl diameter (photo/published width ratio +0.4 %); foot from a template scaled to the foot diameter; estimated: profile, mouth_d_mm, stem_h_mm, stem_d_mm, bowl_d_h_mm, foot_d_mm
  {
    key: "zalto_burgundy",
    name: "Zalto Denk'Art Burgundy",
    kind: 'glass',
    style: "Burgundy (alternative)",
    capacity_ml: 960,
    profile: [[0, 51.2], [2.5, 51.2], [5.5, 47.6], [10.5, 18.4], [16.7, 2.8], [69.8, 2.3], [98.6, 4.4], [103.9, 8.1], [108.8, 14.6], [118.3, 37], [127.5, 53], [136, 59.9], [143.5, 62.2], [157.9, 60.9], [230, 36.2]],
    source: "https://www.zaltoglas.at/en/products/denkart/",
  },
];

/** The glass's wall. */
export function wallOf(v: Vessel): number {
  return v.wall_mm ?? (v.kind === 'glass' ? 1 : 2.5);
}

/** The cavity's floor: going down from the widest point, where the vessel narrows to its
 * stem (a glass) or its foot (a decanter), plus the glass under the bowl. */
export function floorOf(v: Vessel): number {
  if (v.floor_mm !== undefined) return v.floor_mm;
  const wall = wallOf(v), p = v.profile;
  const top = p[p.length - 1]![0];
  if (v.kind === 'decanter') return Math.min(7, top - 1);    // a thick flat base
  // a glass: up its stem from the thinnest point until it flares into the bowl
  let narrow = Infinity, hn = 0;
  for (const [h, r] of p) if (r < narrow) { narrow = r; hn = h; }
  const stem = Math.max(narrow * 1.5, narrow + 2.5);
  let stemTop = hn;
  for (let h = hn; h <= top; h += 0.25) {
    if (radiusAt(p, h) > stem) break;
    stemTop = h;
  }
  return Math.min(stemTop + 2.4 * wall, top - 1);
}

/** The cavity, ``[[height, radius], ...]`` from its floor to the rim (heights over the
 * vessel's base). */
export function cavityOf(v: Vessel): Profile {
  const wall = wallOf(v), floor = floorOf(v), p = v.profile;
  const out: [number, number][] = [[floor, Math.max(0, radiusAt(p, floor) - wall)]];
  for (const [h, r] of p) if (h > floor + 1e-6) out.push([h, Math.max(0, r - wall)]);
  return out;
}

/** The vessel's height. */
export function heightOf(v: Vessel): number {
  return v.profile[v.profile.length - 1]![0];
}

/** Its widest radius (for its footprint and a riser under it). */
export function maxRadiusOf(v: Vessel): number {
  return Math.max(...v.profile.map(([, r]) => r));
}

/** What its cavity holds brim-full, mL (from the profile: a check on the stated capacity). */
export function brimMl(v: Vessel): number {
  return profileVolume(cavityOf(v)) / 1000;
}

/** A vessel that holds a bottle's ``ml``: the first decanter rated for it whose brim holds
 * it, the smallest first; else the biggest decanter (a bigger bottle is poured in turns: the
 * swap, the guide's Decanting step). */
export function vesselFor(ml: number, vessels: readonly Vessel[] = VESSELS): Vessel {
  const decanters = vessels.filter((v) => v.kind === 'decanter')
    .sort((a, b) => a.capacity_ml - b.capacity_ml || brimMl(a) - brimMl(b));
  return decanters.find((v) => v.capacity_ml >= ml && brimMl(v) >= ml)
    ?? decanters[decanters.length - 1] ?? vessels[0]!;
}

/** The bottle the cradle page opens with (``motion.bottles``' key): a magnum, poured into a
 * decanter that holds it (:func:`vesselFor`; design review round 2's M5: the page had opened
 * pouring a 6 L into a 750 ml decanter). */
export const DEFAULT_BOTTLE = 'magnum';
