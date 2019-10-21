import { Manifold } from '../Collision/Collision';
import { Vec2 } from '../Common/Math';
import { ParticleGroup } from '../Particle/ParticleGroup';
import { ParticleBodyContact, ParticleContact, ParticleSystem } from '../Particle/ParticleSystem';
import { Contact } from './Contacts/Contact';
import { Fixture } from './Fixture';
import { Joint } from './Joints/Joint';
export declare class DestructionListener {
    SayGoodbyeJoint(joint: Joint): void;
    SayGoodbyeFixture(fixture: Fixture): void;
    SayGoodbyeParticleGroup(group: ParticleGroup): void;
    SayGoodbyeParticle(system: ParticleSystem, index: number): void;
}
export declare class ContactFilter {
    static readonly defaultFilter: ContactFilter;
    ShouldCollide(fixtureA: Fixture, fixtureB: Fixture): boolean;
    ShouldCollideFixtureParticle(fixture: Fixture, system: ParticleSystem, index: number): boolean;
    ShouldCollideParticleParticle(system: ParticleSystem, indexA: number, indexB: number): boolean;
}
export declare class ContactImpulse {
    normalImpulses: number[];
    tangentImpulses: number[];
    count: number;
}
export declare class ContactListener {
    static readonly defaultListener: ContactListener;
    BeginContact(contact: Contact): void;
    EndContact(contact: Contact): void;
    BeginContactFixtureParticle(system: ParticleSystem, contact: ParticleBodyContact): void;
    EndContactFixtureParticle(system: ParticleSystem, contact: ParticleBodyContact): void;
    BeginContactParticleParticle(system: ParticleSystem, contact: ParticleContact): void;
    EndContactParticleParticle(system: ParticleSystem, contact: ParticleContact): void;
    PreSolve(contact: Contact, oldManifold: Manifold): void;
    PostSolve(contact: Contact, impulse: ContactImpulse): void;
}
export declare class QueryCallback {
    ReportFixture(fixture: Fixture): boolean;
    ReportParticle(system: ParticleSystem, index: number): boolean;
    ShouldQueryParticleSystem(system: ParticleSystem): boolean;
}
export declare type QueryCallbackFunction = (fixture: Fixture) => boolean;
export declare class RayCastCallback {
    ReportFixture(fixture: Fixture, point: Vec2, normal: Vec2, fraction: number): number;
    ReportParticle(system: ParticleSystem, index: number, point: Vec2, normal: Vec2, fraction: number): number;
    ShouldQueryParticleSystem(system: ParticleSystem): boolean;
}
export declare type RayCastCallbackFunction = (fixture: Fixture, point: Vec2, normal: Vec2, fraction: number) => number;
