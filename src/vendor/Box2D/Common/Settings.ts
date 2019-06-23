/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

export function Assert(condition: boolean, ...args: any[]): void {
  if (!condition) {
    // debugger;
    throw new Error(...args)
  }
}

export function Maybe<T>(value: T | undefined, def: T): T {
  return value !== undefined ? value : def
}

export const maxFloat: number = 1e37 // FLT_MAX instead of Number.MAX_VALUE;
export const epsilon: number = 1e-5 // FLT_EPSILON instead of Number.MIN_VALUE;
export const epsilon_sq: number = epsilon * epsilon
export const pi: number = 3.14159265359 // Math.PI;

/// @file
/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
///

// Collision

/// The maximum number of contact points between two convex shapes. Do
/// not change this value.
export const maxManifoldPoints: number = 2

/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because BlockAllocator has a maximum object size.
export const maxPolygonVertices: number = 8

/// This is used to fatten AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
/// This is in meters.
export const aabbExtension: number = 0.1

/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
export const aabbMultiplier: number = 2

/// A small length used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
export const linearSlop: number = 0.008 // 0.005;

/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
export const angularSlop: number = (2 / 180) * pi

/// The radius of the polygon/edge shape skin. This should not be modified. Making
/// this smaller means polygons will have an insufficient buffer for continuous collision.
/// Making it larger may create artifacts for vertex collision.
export const polygonRadius: number = 2 * linearSlop

/// Maximum number of sub-steps per contact in continuous physics simulation.
export const maxSubSteps: number = 8

// Dynamics

/// Maximum number of contacts to be handled to solve a TOI impact.
export const maxTOIContacts: number = 32

/// A velocity threshold for elastic collisions. Any collision with a relative linear
/// velocity below this threshold will be treated as inelastic.
export const velocityThreshold: number = 1

/// The maximum linear position correction used when solving constraints. This helps to
/// prevent overshoot.
export const maxLinearCorrection: number = 0.2

/// The maximum angular position correction used when solving constraints. This helps to
/// prevent overshoot.
export const maxAngularCorrection: number = (8 / 180) * pi

/// The maximum linear velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
export const maxTranslation: number = 2
export const maxTranslationSquared: number = maxTranslation * maxTranslation

/// The maximum angular velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
export const maxRotation: number = 0.5 * pi
export const maxRotationSquared: number = maxRotation * maxRotation

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
export const baumgarte: number = 0.2
export const toiBaumgarte: number = 0.75

// #if ENABLE_PARTICLE

// Particle

/// A symbolic constant that stands for particle allocation error.
export const invalidParticleIndex: number = -1

export const maxParticleIndex: number = 0x7fffffff

/// The default distance between particles, multiplied by the particle diameter.
export const particleStride: number = 0.75

/// The minimum particle weight that produces pressure.
export const minParticleWeight: number = 1.0

/// The upper limit for particle pressure.
export const maxParticlePressure: number = 0.25

/// The upper limit for force between particles.
export const maxParticleForce: number = 0.5

/// The maximum distance between particles in a triad, multiplied by the particle diameter.
export const maxTriadDistance: number = 2.0
export const maxTriadDistanceSquared: number =
  maxTriadDistance * maxTriadDistance

/// The initial size of particle data buffers.
export const minParticleSystemBufferCapacity: number = 256

/// The time into the future that collisions against barrier particles will be detected.
export const barrierCollisionTime: number = 2.5

// #endif

// Sleep

/// The time that a body must be still before it will go to sleep.
export const timeToSleep: number = 0.5

/// A body cannot sleep if its linear velocity is above this tolerance.
export const linearSleepTolerance: number = 0.01

/// A body cannot sleep if its angular velocity is above this tolerance.
export const angularSleepTolerance: number = (2 / 180) * pi

// Memory Allocation

/// Implement this function to use your own memory allocator.
export function Alloc(size: number): any {
  return null
}

/// If you implement Alloc, you should also implement this function.
// tslint:disable-next-line
export function Free(mem: any): void {}

/// Logging function.
export function Log(message: string, ...args: any[]): void {
  // console.log(message, ...args);
}

/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
export class Version {
  major: number = 0 ///< significant changes
  minor: number = 0 ///< incremental changes
  revision: number = 0 ///< bug fixes

  constructor(major: number = 0, minor: number = 0, revision: number = 0) {
    this.major = major
    this.minor = minor
    this.revision = revision
  }

  toString(): string {
    return this.major + '.' + this.minor + '.' + this.revision
  }
}

/// Current version.
export const version: Version = new Version(2, 3, 2)

export const branch: string = 'master'
export const commit: string = 'fbf51801d80fc389d43dc46524520e89043b6faf'

export function ParseInt(v: string): number {
  return parseInt(v, 10)
}

export function ParseUInt(v: string): number {
  return Math.abs(parseInt(v, 10))
}

export function MakeArray<T>(length: number, init: (i: number) => T): T[] {
  const a: T[] = []
  for (let i: number = 0; i < length; ++i) {
    a.push(init(i))
  }
  return a
}

export function MakeNullArray<T>(length: number): Array<T | null> {
  const a: Array<T | null> = []
  for (let i: number = 0; i < length; ++i) {
    a.push(null)
  }
  return a
}

export function MakeNumberArray(length: number, init: number = 0): number[] {
  const a: number[] = []
  for (let i: number = 0; i < length; ++i) {
    a.push(init)
  }
  return a
}
