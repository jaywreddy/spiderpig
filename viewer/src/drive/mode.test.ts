import { describe, expect, it } from 'vitest';
import { loadMode } from './mode';

describe('loadMode', () => {
  it('drives a walker as before', () => {
    expect(loadMode({}, true, true, false)).toEqual(
      { product: false, panel: true, usable: true, wantPhysics: true });
    expect(loadMode({}, false, false, true)).toEqual(
      { product: false, panel: true, usable: false, wantPhysics: true });
  });
  it('turns a product (a decanting cradle) by hand: no panel, drive or physics', () => {
    expect(loadMode({ product: 'cradle' }, true, true, true)).toEqual(
      { product: true, panel: false, usable: false, wantPhysics: false });
  });
});
