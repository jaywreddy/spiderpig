// The studio environment the viewer's reflections come from: three.js r160's
// ``examples/jsm/environments/RoomEnvironment.js`` (MIT, (c) 2010-2023 three.js authors;
// after model-viewer's EnvironmentScene), kept as it was at r160 with physical lights
// (the point light at 900). three's own RoomEnvironment changed after r160 (the room
// lowered 3.5, Lambert boxes, instanced lights), which washed the robot's acrylic and
// metal out (lighter, flatter highlights); this keeps the look the viewer was tuned for.
import * as THREE from 'three';

function areaLight(intensity: number): THREE.MeshBasicMaterial {
  const material = new THREE.MeshBasicMaterial();
  material.color.setScalar(intensity);
  return material;
}

type Box = [pos: [number, number, number], rotY: number, scale: [number, number, number]];

const BOXES: Box[] = [
  [[-10.906, 2.009, 1.846], -0.195, [2.328, 7.905, 4.651]],
  [[-5.607, -0.754, -0.758], 0.994, [1.970, 1.534, 3.955]],
  [[6.167, 0.857, 7.803], 0.561, [3.927, 6.285, 3.687]],
  [[-2.017, 0.018, 6.124], 0.333, [2.002, 4.566, 2.064]],
  [[2.291, -0.756, -2.621], -0.286, [1.546, 1.552, 1.496]],
  [[-2.193, -0.369, -5.547], 0.516, [3.875, 3.487, 2.986]],
];

type Light = [intensity: number, pos: [number, number, number], scale: [number, number, number]];

const LIGHTS: Light[] = [
  [50, [-16.116, 14.37, 8.208], [0.1, 2.428, 2.739]],    // -x right
  [50, [-16.109, 18.021, -8.207], [0.1, 2.425, 2.751]],  // -x left
  [17, [14.904, 12.198, -1.832], [0.15, 4.265, 6.331]],  // +x
  [43, [-0.462, 8.89, 14.520], [4.38, 5.441, 0.088]],    // +z
  [20, [3.235, 11.486, -12.541], [2.5, 2.0, 0.1]],       // -z
  [100, [0.0, 20.0, 0.0], [1.0, 0.1, 1.0]],              // +y
];

export class RoomEnvironment extends THREE.Scene {
  constructor() {
    super();
    const geometry = new THREE.BoxGeometry();
    geometry.deleteAttribute('uv');

    const mainLight = new THREE.PointLight(0xffffff, 900, 28, 2);
    mainLight.position.set(0.418, 16.199, 0.300);
    this.add(mainLight);

    const room = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ side: THREE.BackSide }));
    room.position.set(-0.757, 13.219, 0.717);
    room.scale.set(31.713, 28.305, 28.591);
    this.add(room);

    const boxMaterial = new THREE.MeshStandardMaterial();
    for (const [pos, rotY, scale] of BOXES) {
      const box = new THREE.Mesh(geometry, boxMaterial);
      box.position.set(...pos);
      box.rotation.set(0, rotY, 0);
      box.scale.set(...scale);
      this.add(box);
    }
    for (const [intensity, pos, scale] of LIGHTS) {
      const light = new THREE.Mesh(geometry, areaLight(intensity));
      light.position.set(...pos);
      light.scale.set(...scale);
      this.add(light);
    }
  }

  override dispose(): void {
    const resources = new Set<{ dispose(): void }>();
    this.traverse((object) => {
      if (object instanceof THREE.Mesh) {
        resources.add(object.geometry as THREE.BufferGeometry);
        resources.add(object.material as THREE.Material);
      }
    });
    for (const resource of resources) resource.dispose();
  }
}
