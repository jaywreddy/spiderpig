// The dev tooling's npm audit at high or critical (ci.yml's test-viewer), less the advisories
// ALLOW names: npm audit has no ignore list. An entry says why its advisory doesn't reach this
// repo and when to look again; past `until` it fails, so an exception can't outlive its reason.
// What ships in the bundle stays `npm audit --omit=dev` at any severity, with no exceptions.
import { spawnSync } from 'node:child_process';

const ALLOW = [
  {
    id: 'GHSA-fc5q-39xc-vjpf',
    why: 'picomatch <= 4.0.7 (no fixed release yet) backtracks on a crafted glob pattern; '
      + "its patterns here come from vite's and vitest's config, never from a request",
    until: '2027-01-31',
  },
];
const LEVELS = new Set(['high', 'critical']);

const run = spawnSync('npm', ['audit', '--json'], { encoding: 'utf8' });
const report = JSON.parse(run.stdout || '{}');
if (report.error || !report.vulnerabilities) {
  console.error('npm audit gave no report:', report.error?.summary ?? run.stderr);
  process.exit(2);
}
const found = new Map();  // advisory id -> the advisory, once however many packages it reaches
for (const v of Object.values(report.vulnerabilities)) {
  for (const via of v.via) {  // a string is another package's finding, an object an advisory
    if (typeof via === 'object' && LEVELS.has(via.severity)) found.set(via.url.split('/').pop(), via);
  }
}
const today = new Date().toISOString().slice(0, 10);
let failed = 0;
for (const a of ALLOW) {
  if (today > a.until) {
    console.error(`${a.id}: its exception ran out on ${a.until}, check it again: ${a.why}`);
    failed++;
  } else if (!found.has(a.id)) {
    console.log(`${a.id}: no longer reported, its exception can go`);
  }
}
for (const [id, via] of found) {
  const allowed = ALLOW.find((a) => a.id === id && today <= a.until);
  const line = `${via.severity}: ${via.name} ${via.range}: ${via.title} (${via.url})`;
  if (allowed) console.log(`allowed until ${allowed.until}: ${line}`);
  else { console.error(line); failed++; }
}
process.exit(failed ? 1 : 0);
