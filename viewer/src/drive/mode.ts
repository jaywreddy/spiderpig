/** What a loaded glb lets the drive mode do. A product (a decanting cradle, its root's
 * ``userData.product``) is turned by hand, not driven: no drive panel or pad, no walk to
 * drive, no physics. */
export interface LoadMode {
  product: boolean;
  panel: boolean;
  usable: boolean;
  wantPhysics: boolean;
}

export function loadMode(userData: Record<string, unknown>, sideUsable: boolean,
                         physicsOn: boolean, physicsWanted: boolean): LoadMode {
  const product = Boolean(userData.product);
  return {
    product,
    panel: !product,
    usable: sideUsable && !product,
    wantPhysics: (physicsOn || physicsWanted) && !product,
  };
}
