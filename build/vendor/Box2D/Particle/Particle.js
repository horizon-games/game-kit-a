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
import { Color } from '../Common/Draw';
import { Clamp, Vec2 } from '../Common/Math';
import { invalidParticleIndex } from '../Common/Settings';
/**
 * The particle type. Can be combined with the | operator.
 */
export var ParticleFlag;
(function (ParticleFlag) {
    /// Water particle.
    ParticleFlag[ParticleFlag["waterParticle"] = 0] = "waterParticle";
    /// Removed after next simulation step.
    ParticleFlag[ParticleFlag["zombieParticle"] = 2] = "zombieParticle";
    /// Zero velocity.
    ParticleFlag[ParticleFlag["wallParticle"] = 4] = "wallParticle";
    /// With restitution from stretching.
    ParticleFlag[ParticleFlag["springParticle"] = 8] = "springParticle";
    /// With restitution from deformation.
    ParticleFlag[ParticleFlag["elasticParticle"] = 16] = "elasticParticle";
    /// With viscosity.
    ParticleFlag[ParticleFlag["viscousParticle"] = 32] = "viscousParticle";
    /// Without isotropic pressure.
    ParticleFlag[ParticleFlag["powderParticle"] = 64] = "powderParticle";
    /// With surface tension.
    ParticleFlag[ParticleFlag["tensileParticle"] = 128] = "tensileParticle";
    /// Mix color between contacting particles.
    ParticleFlag[ParticleFlag["colorMixingParticle"] = 256] = "colorMixingParticle";
    /// Call DestructionListener on destruction.
    ParticleFlag[ParticleFlag["destructionListenerParticle"] = 512] = "destructionListenerParticle";
    /// Prevents other particles from leaking.
    ParticleFlag[ParticleFlag["barrierParticle"] = 1024] = "barrierParticle";
    /// Less compressibility.
    ParticleFlag[ParticleFlag["staticPressureParticle"] = 2048] = "staticPressureParticle";
    /// Makes pairs or triads with other particles.
    ParticleFlag[ParticleFlag["reactiveParticle"] = 4096] = "reactiveParticle";
    /// With high repulsive force.
    ParticleFlag[ParticleFlag["repulsiveParticle"] = 8192] = "repulsiveParticle";
    /// Call ContactListener when this particle is about to interact with
    /// a rigid body or stops interacting with a rigid body.
    /// This results in an expensive operation compared to using
    /// fixtureContactFilterParticle to detect collisions between
    /// particles.
    ParticleFlag[ParticleFlag["fixtureContactListenerParticle"] = 16384] = "fixtureContactListenerParticle";
    /// Call ContactListener when this particle is about to interact with
    /// another particle or stops interacting with another particle.
    /// This results in an expensive operation compared to using
    /// particleContactFilterParticle to detect collisions between
    /// particles.
    ParticleFlag[ParticleFlag["particleContactListenerParticle"] = 32768] = "particleContactListenerParticle";
    /// Call ContactFilter when this particle interacts with rigid bodies.
    ParticleFlag[ParticleFlag["fixtureContactFilterParticle"] = 65536] = "fixtureContactFilterParticle";
    /// Call ContactFilter when this particle interacts with other
    /// particles.
    ParticleFlag[ParticleFlag["particleContactFilterParticle"] = 131072] = "particleContactFilterParticle";
})(ParticleFlag || (ParticleFlag = {}));
export class ParticleDef {
    constructor() {
        this.flags = 0;
        this.position = new Vec2();
        this.velocity = new Vec2();
        this.color = new Color(0, 0, 0, 0);
        this.lifetime = 0.0;
        this.userData = null;
        this.group = null;
    }
}
export function CalculateParticleIterations(gravity, radius, timeStep) {
    // In some situations you may want more particle iterations than this,
    // but to avoid excessive cycle cost, don't recommend more than this.
    const MAX_RECOMMENDED_PARTICLE_ITERATIONS = 8;
    const RADIUS_THRESHOLD = 0.01;
    const iterations = Math.ceil(Math.sqrt(gravity / (RADIUS_THRESHOLD * radius)) * timeStep);
    return Clamp(iterations, 1, MAX_RECOMMENDED_PARTICLE_ITERATIONS);
}
export class ParticleHandle {
    constructor() {
        this.m_index = invalidParticleIndex;
    }
    GetIndex() {
        return this.m_index;
    }
    SetIndex(index) {
        this.m_index = index;
    }
}
// #endif
//# sourceMappingURL=Particle.js.map