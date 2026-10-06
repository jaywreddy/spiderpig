import { defineConfig } from 'vitest/config';

// The viewer's unit tests (``mise run test-viewer``): plain modules in Node, no browser.
// The browser tests are Playwright's (``tests/e2e/``, ``pytest -m e2e``).
export default defineConfig({
  test: {
    include: ['src/**/*.test.ts'],
    environment: 'node',
  },
});
