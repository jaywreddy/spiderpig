/**
 * The page's chrome over the canvas, for the camera (``scene.ts``) and the panels:
 * a phone or tablet layout (``COMPACT``, the same media query as ``style.css``) and the
 * screen space the top bar and the bottom dock (play bar + drive pad) take.
 */

/** Phones and tablets: the panels start collapsed and only one is open at a time. */
export const COMPACT = '(max-width: 700px), (pointer: coarse)';

export const isCompact = (): boolean => matchMedia(COMPACT).matches;

export interface Insets { top: number; bottom: number }

/** CSS px of the viewport the chrome covers at the top (the status line and, compact, the
 * panels' title bars) and at the bottom (the dock). An open panel isn't counted: it is
 * the user's to close, and the camera shouldn't jump when it opens. */
export function chromeInsets(): Insets {
  const below = (sel: string): number => document.querySelector(sel)?.getBoundingClientRect().bottom ?? 0;
  const top = Math.max(below('#status'),
    ...(isCompact() ? [below('#drive-gui > .title'), below('#tune-gui > .title')] : []));
  const dock = document.getElementById('dock');
  const bottom = dock ? Math.max(0, window.innerHeight - dock.getBoundingClientRect().top) : 0;
  return { top: Math.ceil(top), bottom: Math.ceil(bottom) };
}
