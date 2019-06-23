/*
 * Copyright (c) 2013 Google, Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

// #if ENABLE_PARTICLE

import { Color, RGBA } from '../Common/Draw'
import { Clamp, Vec2, XY } from '../Common/Math'
import { invalidParticleIndex } from '../Common/Settings'

import { ParticleGroup } from './ParticleGroup'

/**
 * The particle type. Can be combined with the | operator.
 */
export enum ParticleFlag {
  /// Water particle.
  waterParticle = 0,
  /// Removed after next simulation step.
  zombieParticle = 1 << 1,
  /// Zero velocity.
  wallParticle = 1 << 2,
  /// With restitution from stretching.
  springParticle = 1 << 3,
  /// With restitution from deformation.
  elasticParticle = 1 << 4,
  /// With viscosity.
  viscousParticle = 1 << 5,
  /// Without isotropic pressure.
  powderParticle = 1 << 6,
  /// With surface tension.
  tensileParticle = 1 << 7,
  /// Mix color between contacting particles.
  colorMixingParticle = 1 << 8,
  /// Call DestructionListener on destruction.
  destructionListenerParticle = 1 << 9,
  /// Prevents other particles from leaking.
  barrierParticle = 1 << 10,
  /// Less compressibility.
  staticPressureParticle = 1 << 11,
  /// Makes pairs or triads with other particles.
  reactiveParticle = 1 << 12,
  /// With high repulsive force.
  repulsiveParticle = 1 << 13,
  /// Call ContactListener when this particle is about to interact with
  /// a rigid body or stops interacting with a rigid body.
  /// This results in an expensive operation compared to using
  /// fixtureContactFilterParticle to detect collisions between
  /// particles.
  fixtureContactListenerParticle = 1 << 14,
  /// Call ContactListener when this particle is about to interact with
  /// another particle or stops interacting with another particle.
  /// This results in an expensive operation compared to using
  /// particleContactFilterParticle to detect collisions between
  /// particles.
  particleContactListenerParticle = 1 << 15,
  /// Call ContactFilter when this particle interacts with rigid bodies.
  fixtureContactFilterParticle = 1 << 16,
  /// Call ContactFilter when this particle interacts with other
  /// particles.
  particleContactFilterParticle = 1 << 17
}

export interface IParticleDef {
  flags?: ParticleFlag
  position?: XY
  velocity?: XY
  color?: RGBA
  lifetime?: number
  userData?: any
  group?: ParticleGroup | null
}

export class ParticleDef implements IParticleDef {
  flags: ParticleFlag = 0
  readonly position: Vec2 = new Vec2()
  readonly velocity: Vec2 = new Vec2()
  readonly color: Color = new Color(0, 0, 0, 0)
  lifetime: number = 0.0
  userData: any = null
  group: ParticleGroup | null = null
}

export function CalculateParticleIterations(
  gravity: number,
  radius: number,
  timeStep: number
): number {
  // In some situations you may want more particle iterations than this,
  // but to avoid excessive cycle cost, don't recommend more than this.
  const MAX_RECOMMENDED_PARTICLE_ITERATIONS = 8
  const RADIUS_THRESHOLD = 0.01
  const iterations = Math.ceil(
    Math.sqrt(gravity / (RADIUS_THRESHOLD * radius)) * timeStep
  )
  return Clamp(iterations, 1, MAX_RECOMMENDED_PARTICLE_ITERATIONS)
}

export class ParticleHandle {
  m_index: number = invalidParticleIndex
  GetIndex(): number {
    return this.m_index
  }
  SetIndex(index: number): void {
    this.m_index = index
  }
}

// #endif
