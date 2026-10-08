import { defineConfig } from 'vitest/config';

// The viewer's unit tests (``mise run test-viewer``): plain modules in Node, no browser.
// The browser tests are Playwright's (``tests/e2e/``, ``pytest -m e2e``).
export default defineConfig({
  test: {
    include: ['src/**/*.test.ts'],
    environment: 'node',
    // every file and test by name: vitest 4+'s default reporter lists only failing files
    // outside a TTY, and tests/test_viewer_parity.py looks for ``model.test.ts`` in the output
    reporters: ['tree'],
  },
});
