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
import { Color } from '../Common/Draw';
import { Transform, Vec2 } from '../Common/Math';
export var ParticleGroupFlag;
(function (ParticleGroupFlag) {
    /// Prevents overlapping or leaking.
    ParticleGroupFlag[ParticleGroupFlag["solidParticleGroup"] = 1] = "solidParticleGroup";
    /// Keeps its shape.
    ParticleGroupFlag[ParticleGroupFlag["rigidParticleGroup"] = 2] = "rigidParticleGroup";
    /// Won't be destroyed if it gets empty.
    ParticleGroupFlag[ParticleGroupFlag["particleGroupCanBeEmpty"] = 4] = "particleGroupCanBeEmpty";
    /// Will be destroyed on next simulation step.
    ParticleGroupFlag[ParticleGroupFlag["particleGroupWillBeDestroyed"] = 8] = "particleGroupWillBeDestroyed";
    /// Updates depth data on next simulation step.
    ParticleGroupFlag[ParticleGroupFlag["particleGroupNeedsUpdateDepth"] = 16] = "particleGroupNeedsUpdateDepth";
    ParticleGroupFlag[ParticleGroupFlag["particleGroupInternalMask"] = 24] = "particleGroupInternalMask";
})(ParticleGroupFlag || (ParticleGroupFlag = {}));
export class ParticleGroupDef {
    constructor() {
        this.flags = 0;
        this.groupFlags = 0;
        this.position = new Vec2();
        this.angle = 0.0;
        this.linearVelocity = new Vec2();
        this.angularVelocity = 0.0;
        this.color = new Color();
        this.strength = 1.0;
        this.shapeCount = 0;
        this.stride = 0;
        this.particleCount = 0;
        this.lifetime = 0;
        this.userData = null;
        this.group = null;
    }
}
export class ParticleGroup {
    constructor(system) {
        this.m_firstIndex = 0;
        this.m_lastIndex = 0;
        this.m_groupFlags = 0;
        this.m_strength = 1.0;
        this.m_prev = null;
        this.m_next = null;
        this.m_timestamp = -1;
        this.m_mass = 0.0;
        this.m_inertia = 0.0;
        this.m_center = new Vec2();
        this.m_linearVelocity = new Vec2();
        this.m_angularVelocity = 0.0;
        this.m_transform = new Transform();
        ///m_transform.SetIdentity();
        this.m_userData = null;
        this.m_system = system;
    }
    GetNext() {
        return this.m_next;
    }
    GetParticleSystem() {
        return this.m_system;
    }
    GetParticleCount() {
        return this.m_lastIndex - this.m_firstIndex;
    }
    GetBufferIndex() {
        return this.m_firstIndex;
    }
    ContainsParticle(index) {
        return this.m_firstIndex <= index && index < this.m_lastIndex;
    }
    GetAllParticleFlags() {
        if (!this.m_system.m_flagsBuffer.data) {
            throw new Error();
        }
        let flags = 0;
        for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
            flags |= this.m_system.m_flagsBuffer.data[i];
        }
        return flags;
    }
    GetGroupFlags() {
        return this.m_groupFlags;
    }
    SetGroupFlags(flags) {
        // DEBUG: Assert((flags & ParticleGroupFlag.particleGroupInternalMask) === 0);
        flags |= this.m_groupFlags & ParticleGroupFlag.particleGroupInternalMask;
        this.m_system.SetGroupFlags(this, flags);
    }
    GetMass() {
        this.UpdateStatistics();
        return this.m_mass;
    }
    GetInertia() {
        this.UpdateStatistics();
        return this.m_inertia;
    }
    GetCenter() {
        this.UpdateStatistics();
        return this.m_center;
    }
    GetLinearVelocity() {
        this.UpdateStatistics();
        return this.m_linearVelocity;
    }
    GetAngularVelocity() {
        this.UpdateStatistics();
        return this.m_angularVelocity;
    }
    GetTransform() {
        return this.m_transform;
    }
    GetPosition() {
        return this.m_transform.p;
    }
    GetAngle() {
        return this.m_transform.q.GetAngle();
    }
    GetLinearVelocityFromWorldPoint(worldPoint, out) {
        const s_t0 = ParticleGroup.GetLinearVelocityFromWorldPoint_s_t0;
        this.UpdateStatistics();
        ///  return m_linearVelocity + Cross(m_angularVelocity, worldPoint - m_center);
        return Vec2.AddVCrossSV(this.m_linearVelocity, this.m_angularVelocity, Vec2.SubVV(worldPoint, this.m_center, s_t0), out);
    }
    GetUserData() {
        return this.m_userData;
    }
    SetUserData(data) {
        this.m_userData = data;
    }
    ApplyForce(force) {
        this.m_system.ApplyForce(this.m_firstIndex, this.m_lastIndex, force);
    }
    ApplyLinearImpulse(impulse) {
        this.m_system.ApplyLinearImpulse(this.m_firstIndex, this.m_lastIndex, impulse);
    }
    DestroyParticles(callDestructionListener) {
        if (this.m_system.m_world.IsLocked()) {
            throw new Error();
        }
        for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
            this.m_system.DestroyParticle(i, callDestructionListener);
        }
    }
    UpdateStatistics() {
        if (!this.m_system.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_system.m_velocityBuffer.data) {
            throw new Error();
        }
        const p = new Vec2();
        const v = new Vec2();
        if (this.m_timestamp !== this.m_system.m_timestamp) {
            const m = this.m_system.GetParticleMass();
            ///  this.m_mass = 0;
            this.m_mass = m * (this.m_lastIndex - this.m_firstIndex);
            this.m_center.SetZero();
            this.m_linearVelocity.SetZero();
            for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
                ///  this.m_mass += m;
                ///  this.m_center += m * this.m_system.m_positionBuffer.data[i];
                this.m_center.SelfMulAdd(m, this.m_system.m_positionBuffer.data[i]);
                ///  this.m_linearVelocity += m * this.m_system.m_velocityBuffer.data[i];
                this.m_linearVelocity.SelfMulAdd(m, this.m_system.m_velocityBuffer.data[i]);
            }
            if (this.m_mass > 0) {
                const inv_mass = 1 / this.m_mass;
                ///this.m_center *= 1 / this.m_mass;
                this.m_center.SelfMul(inv_mass);
                ///this.m_linearVelocity *= 1 / this.m_mass;
                this.m_linearVelocity.SelfMul(inv_mass);
            }
            this.m_inertia = 0;
            this.m_angularVelocity = 0;
            for (let i = this.m_firstIndex; i < this.m_lastIndex; i++) {
                ///Vec2 p = this.m_system.m_positionBuffer.data[i] - this.m_center;
                Vec2.SubVV(this.m_system.m_positionBuffer.data[i], this.m_center, p);
                ///Vec2 v = this.m_system.m_velocityBuffer.data[i] - this.m_linearVelocity;
                Vec2.SubVV(this.m_system.m_velocityBuffer.data[i], this.m_linearVelocity, v);
                this.m_inertia += m * Vec2.DotVV(p, p);
                this.m_angularVelocity += m * Vec2.CrossVV(p, v);
            }
            if (this.m_inertia > 0) {
                this.m_angularVelocity *= 1 / this.m_inertia;
            }
            this.m_timestamp = this.m_system.m_timestamp;
        }
    }
}
ParticleGroup.GetLinearVelocityFromWorldPoint_s_t0 = new Vec2();
// #endif
//# sourceMappingURL=ParticleGroup.js.map