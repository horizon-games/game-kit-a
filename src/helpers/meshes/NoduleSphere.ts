import { Material, Mesh, SphereBufferGeometry } from 'three'
import { rand } from '../../utils/math'

let __geo: SphereBufferGeometry
function getGeo() {
  if (!__geo) {
    __geo = new SphereBufferGeometry(1, 32, 16)
  }
  return __geo
}

export class NoduleSphere extends Mesh {
  constructor(material: Material, nodules: number = 10, depth: number = 2) {
    super(getGeo(), material)
    if (depth <= 0) {
      return
    }
    for (let i = 0; i < nodules; i++) {
      const nodule = new NoduleSphere(material, nodules, --depth)
      nodule.position.set(rand(-1, 1), rand(-1, 1), rand(-1, 1)).normalize()
      nodule.scale.multiplyScalar(rand(0.23, 0.45))
      this.add(nodule)
    }
  }
}
