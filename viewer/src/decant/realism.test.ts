import { describe, expect, it } from 'vitest';
import { kindOf, pliesFor } from './realism';

describe('kindOf: a baked part drawn as what it is made of', () => {
  it('takes the sheet kinds from the material the bake names', () => {
    expect(kindOf('aluminium', 'L.b1')).toBe('alu_link');
    expect(kindOf('aluminium_frame', 'L.frame_outer')).toBe('alu_frame');
    expect(kindOf('plywood', 'base_plate')).toBe('plywood');
  });

  it("takes a bought part's surface from its finish", () => {
    expect(kindOf('metal', 'L.pin_J_pin', { finish: 'hard_anodized' })).toBe('anodized');
    expect(kindOf('metal', 'L.handle_plunger0', { finish: 'black_oxide' })).toBe('black_oxide');
    expect(kindOf('metal', 'tie_Tf0', { finish: 'aluminium' })).toBe('alu_bar');
    expect(kindOf('metal', 'L.pin_X_clip', { finish: 'zinc' })).toBe('zinc');
    expect(kindOf('metal', 'L.riser_stud0', { finish: 'stainless' })).toBe('stainless');
  });

  it('draws a bought part with no finish as plain steel; an older bake by its name', () => {
    expect(kindOf('metal', 'L.crank_shaft', {})).toBe('steel');
    expect(kindOf('metal', 'L.crank_end_screw')).toBe('stainless');
    expect(kindOf('metal', 'L.crank_shaft')).toBe('steel');
  });

  it("takes a print's look from its filament, else its name", () => {
    expect(kindOf('printed', 'front_saddle', { filament: 'petg_filament' })).toBe('petg');
    expect(kindOf('printed', 'front_saddle_liner', { filament: 'tpu95a_filament' })).toBe('tpu');
    expect(kindOf('printed', 'decanter_riser_lift0', { filament: 'pla_filament' })).toBe('pla');
    expect(kindOf('printed', 'foot_boot_0')).toBe('tpu');
    expect(kindOf('printed', 'led_clip')).toBe('pla');
  });

  it('keeps the display glass and the light', () => {
    expect(kindOf('glass_bottle', 'bottle')).toBe('glass_green');
    expect(kindOf('glass_clear', 'decanter')).toBe('glass_clear');
    expect(kindOf('led', 'led')).toBe('led');
    expect(kindOf('gearbox', 'L.gearbox')).toBe('gearbox');
  });
});

describe('pliesFor: Baltic birch veneers by thickness', () => {
  it('is odd, three at the least', () => {
    expect(pliesFor(3)).toBe(3);
    expect(pliesFor(6)).toBe(5);
    expect(pliesFor(9)).toBe(7);
    expect(pliesFor(12)).toBe(9);
    expect(pliesFor(18)).toBe(13);
    for (const mm of [1, 4, 7.5, 15, 24]) expect(pliesFor(mm) % 2).toBe(1);
  });
});
