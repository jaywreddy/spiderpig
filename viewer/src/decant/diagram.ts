/**
 * The cradle page's mechanism diagram: one side of the linkage in its plane, as SVG, at
 * a clip fraction. The ground pivots, the crank, every link (the cradle link picked out),
 * the bottle and the V blocks riding the cradle, the crankpin's circle, and the mouth: the
 * point the whole swing turns about.
 */
import {
  pointAt, saddleOutline, swingBounds, tiltAt, toPlane, type CradleMotion, type XY,
} from './motion';

const NS = 'http://www.w3.org/2000/svg';

export const COLORS = {
  ground: '#8a96a3',
  link: '#c7cfd8',
  cradle: '#6aa8ff',
  crank: '#f0a040',
  bottle: '#5fc488',
  saddle: '#a882ff',
  mouth: '#ff5a5a',
} as const;

function el<K extends keyof SVGElementTagNameMap>(
  tag: K, attrs: Record<string, string | number>, parent?: Element,
): SVGElementTagNameMap[K] {
  const e = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, String(v));
  parent?.appendChild(e);
  return e;
}

const pts = (ps: readonly XY[]): string => ps.map(([x, y]) => `${x.toFixed(1)},${y.toFixed(1)}`).join(' ');

export interface Diagram {
  update(f: number): void;
}

/** Draw ``m`` into ``svg`` (cleared first); ``update(f)`` poses it at clip fraction ``f``. */
export function createDiagram(svg: SVGSVGElement, m: CradleMotion): Diagram {
  svg.replaceChildren();
  const b = swingBounds(m, 25);
  const w = b.x1 - b.x0, h = b.y1 - b.y0, unit = Math.max(w, h) / 100;
  svg.setAttribute('viewBox', `${b.x0} ${-b.y1} ${w} ${h}`);
  svg.setAttribute('preserveAspectRatio', 'xMidYMid meet');
  const plane = el('g', { transform: 'scale(1,-1)' }, svg);   // y up, as the linkage's plane

  // what rides the cradle, under the links
  const bottle = el('polygon', {
    fill: COLORS.bottle, 'fill-opacity': 0.18, stroke: COLORS.bottle, 'stroke-width': unit * 0.5,
  }, plane);
  const saddles = m.saddles.map(() => el('polygon', {
    fill: COLORS.saddle, 'fill-opacity': 0.35, stroke: COLORS.saddle, 'stroke-width': unit * 0.3,
  }, plane));

  // the crankpin's circle and the mouth's (tiny) path
  const crankpin = m.crank[1];
  if (crankpin && m.points[crankpin]) {
    el('polyline', {
      points: pts([...m.points[crankpin]!, m.points[crankpin]![0]!]), fill: 'none',
      stroke: COLORS.crank, 'stroke-opacity': 0.45, 'stroke-width': unit * 0.3,
      'stroke-dasharray': `${unit * 1.5} ${unit}`,
    }, plane);
  }

  // the ground pivots
  for (const j of m.frame) {
    const p = m.points[j]?.[0];
    if (!p) continue;
    const s = unit * 2.4;
    el('polygon', {
      points: pts([[p[0], p[1]], [p[0] - s, p[1] - s * 1.6], [p[0] + s, p[1] - s * 1.6]]),
      fill: COLORS.ground, 'fill-opacity': 0.5, stroke: COLORS.ground, 'stroke-width': unit * 0.3,
    }, plane);
  }

  // the links: each body's outline edges, the cradle link picked out
  const links: { line: SVGLineElement; a: string; c: string }[] = [];
  for (const [body, spec] of Object.entries(m.links)) {
    const cradle = body === m.output_link;
    for (const [a, c] of spec.edges) {
      const line = el('line', {
        stroke: cradle ? COLORS.cradle : COLORS.link, 'stroke-width': unit * (cradle ? 1.6 : 1.4),
        'stroke-linecap': 'round', 'stroke-opacity': cradle ? 0.95 : 0.85,
      }, plane);
      links.push({ line, a, c });
    }
  }
  const crank = el('line', {
    stroke: COLORS.crank, 'stroke-width': unit * 2, 'stroke-linecap': 'round',
  }, plane);

  // the joints, then the mouth on top
  const joints = Object.keys(m.points).filter((j) => j !== m.mouth).map((j) => ({
    j, dot: el('circle', { r: unit * 0.9, fill: '#101418', stroke: '#e8edf2', 'stroke-width': unit * 0.35 }, plane),
  }));
  const mouthRing = el('circle', { r: unit * 2.2, fill: 'none', stroke: COLORS.mouth, 'stroke-width': unit * 0.5 }, plane);
  const mouthDot = el('circle', { r: unit * 1.0, fill: COLORS.mouth }, plane);
  const mouthLabel = el('text', {
    fill: COLORS.mouth, 'font-size': unit * 3.6, 'font-family': 'sans-serif',
    'text-anchor': 'middle', stroke: '#101418', 'stroke-width': unit * 0.8,
    'paint-order': 'stroke',
  }, svg);
  mouthLabel.textContent = 'mouth: stays put';

  return {
    update(f: number): void {
      const tilt = tiltAt(m, f);
      const lip = pointAt(m, m.mouth, f);
      bottle.setAttribute('points', pts(m.bottle.outline.map((p) => toPlane(lip, tilt, p))));
      m.saddles.forEach((s, i) => {
        saddles[i]!.setAttribute('points', pts(saddleOutline(s).map((p) => toPlane(lip, tilt, p))));
      });
      for (const { line, a, c } of links) {
        const pa = pointAt(m, a, f), pc = pointAt(m, c, f);
        line.setAttribute('x1', pa[0].toFixed(1)); line.setAttribute('y1', pa[1].toFixed(1));
        line.setAttribute('x2', pc[0].toFixed(1)); line.setAttribute('y2', pc[1].toFixed(1));
      }
      const [o, k] = m.crank;
      if (o && k) {
        const po = pointAt(m, o, f), pk = pointAt(m, k, f);
        crank.setAttribute('x1', po[0].toFixed(1)); crank.setAttribute('y1', po[1].toFixed(1));
        crank.setAttribute('x2', pk[0].toFixed(1)); crank.setAttribute('y2', pk[1].toFixed(1));
      }
      for (const { j, dot } of joints) {
        const p = pointAt(m, j, f);
        dot.setAttribute('cx', p[0].toFixed(1)); dot.setAttribute('cy', p[1].toFixed(1));
      }
      for (const c of [mouthRing, mouthDot]) {
        c.setAttribute('cx', lip[0].toFixed(1)); c.setAttribute('cy', lip[1].toFixed(1));
      }
      mouthLabel.setAttribute('x', lip[0].toFixed(1));
      mouthLabel.setAttribute('y', (-lip[1] - unit * 4).toFixed(1));
    },
  };
}
