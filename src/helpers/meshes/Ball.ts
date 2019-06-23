import { Color, Mesh, MeshBasicMaterial, SphereBufferGeometry } from 'three'
import { Easing } from '~/animation/Easing'
import { AnimatedBool } from '~/utils/AnimatedBool'
import { createMaterial } from '~/utils/colors'

import { NoduleSphere } from './NoduleSphere'

export class Ball extends NoduleSphere {
  readonly color: Color
  readonly explodingState = new AnimatedBool(
    v => this.updateExplosion(v),
    false,
    0,
    Easing.Exponential.Out,
    1000
  )
  readonly validTargets: this[] = []
  collider: Mesh
  private _explosionSize: number
  constructor(
    private _radius: number,
    useCustomCollider = false,
    nodules: number = 30,
    depth: number = 4
  ) {
    super(createMaterial(0x0c152d), nodules, depth)
    const opaque = (this.material as MeshBasicMaterial).clone()
    opaque.transparent = false
    opaque.depthWrite = true
    this.color = (this.material as MeshBasicMaterial).color
    opaque.color = this.color
    this.material = opaque
    this.scale.set(_radius, _radius, _radius)

    if (useCustomCollider) {
      this.collider = new Mesh(
        new SphereBufferGeometry(0.8, 6, 4),
        new MeshBasicMaterial({
          color: 0xff0000,
          wireframe: true,
          depthTest: false,
          depthWrite: false,
          transparent: true
        })
      )
      this.add(this.collider)
    } else {
      this.collider = this
    }
  }

  explode(size: number) {
    this._explosionSize = size
    this.explodingState.pulse()
  }
  private updateExplosion(v: number) {
    const s = this._radius * (1 + v * this._explosionSize)
    this.scale.set(s, s, s)
  }
}
