import { AdditiveBlending, MeshBasicMaterial } from 'three'
import { Easing } from '../../animation/Easing'
import { AnimatedBool } from '../../utils/AnimatedBool'
import { createMaterial } from '../../utils/colors'

import { NoduleSphere } from './NoduleSphere'

export class Explosion extends NoduleSphere {
  readonly explodingState = new AnimatedBool(
    v => this.updateExplosion(v),
    false,
    15000,
    Easing.Exponential.Out
  )
  private _explosionSize: number
  private mat: MeshBasicMaterial
  constructor(
    private _radius: number,
    nodules: number = 30,
    depth: number = 4,
    size: number = 0
  ) {
    super(createMaterial(0xe7c02a), nodules, depth)
    this.mat = this.material as MeshBasicMaterial
    this.mat.blending = AdditiveBlending
    this.scale.set(_radius, _radius, _radius)
    if (size > 0) {
      this.explode(size)
    }
  }

  explode(size: number) {
    this._explosionSize = size
    this.explodingState.value = true
  }
  private updateExplosion(v: number) {
    const s = this._radius * (1 + v * this._explosionSize)
    this.scale.set(s, s, s)
    this.mat.opacity = 1 - v
    if (v === 1 && this.parent) {
      this.parent.remove(this)
    }
  }
}
