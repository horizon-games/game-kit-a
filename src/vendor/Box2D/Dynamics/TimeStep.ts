/*
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

import { Vec2 } from '../Common/Math'
import { MakeArray } from '../Common/Settings'

/// Profiling data. Times are in milliseconds.
export class Profile {
  step: number = 0
  collide: number = 0
  solve: number = 0
  solveInit: number = 0
  solveVelocity: number = 0
  solvePosition: number = 0
  broadphase: number = 0
  solveTOI: number = 0

  Reset() {
    this.step = 0
    this.collide = 0
    this.solve = 0
    this.solveInit = 0
    this.solveVelocity = 0
    this.solvePosition = 0
    this.broadphase = 0
    this.solveTOI = 0
    return this
  }
}

/// This is an internal structure.
export class TimeStep {
  dt: number = 0 // time step
  inv_dt: number = 0 // inverse time step (0 if dt == 0).
  dtRatio: number = 0 // dt * inv_dt0
  velocityIterations: number = 0
  positionIterations: number = 0
  // #if ENABLE_PARTICLE
  particleIterations: number = 0
  // #endif
  warmStarting: boolean = false

  Copy(step: TimeStep): TimeStep {
    this.dt = step.dt
    this.inv_dt = step.inv_dt
    this.dtRatio = step.dtRatio
    this.positionIterations = step.positionIterations
    this.velocityIterations = step.velocityIterations
    // #if ENABLE_PARTICLE
    this.particleIterations = step.particleIterations
    // #endif
    this.warmStarting = step.warmStarting
    return this
  }
}

export class Position {
  static MakeArray(length: number): Position[] {
    return MakeArray(length, (i: number): Position => new Position())
  }
  readonly c: Vec2 = new Vec2()
  a: number = 0
}

export class Velocity {
  static MakeArray(length: number): Velocity[] {
    return MakeArray(length, (i: number): Velocity => new Velocity())
  }
  readonly v: Vec2 = new Vec2()
  w: number = 0
}

export class SolverData {
  readonly step: TimeStep = new TimeStep()
  positions!: Position[]
  velocities!: Velocity[]
}
