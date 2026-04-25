import { defineConfig } from 'vite';

// Ports are driven by env so parallel worktrees don't collide:
//   VITE_PORT     — defaults to 5173 (overridden by scripts/dev.py with a
//                   per-worktree hash-derived port; user can pin via
//                   mise.local.toml [env] VITE_PORT = "...")
//   VITE_API_PORT — FastAPI port we proxy /api and /ws to (default 8000).
// strictPort=false so a stale process on the chosen port doesn't crash
// startup — Vite picks the next free one and prints the real URL.
const port = Number(process.env.VITE_PORT) || 5173;
const apiPort = Number(process.env.VITE_API_PORT) || 8000;

export default defineConfig({
  root: '.',
  build: {
    outDir: 'dist',
    emptyOutDir: true,
    target: 'es2022',
  },
  server: {
    port,
    strictPort: false,
    proxy: {
      '/api': `http://127.0.0.1:${apiPort}`,
      '/ws': { target: `ws://127.0.0.1:${apiPort}`, ws: true },
    },
  },
});
