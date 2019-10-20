import { Color, RGBA } from '../Common/Draw';
import { Vec2, XY } from '../Common/Math';
import { ParticleGroup } from './ParticleGroup';
/**
 * The particle type. Can be combined with the | operator.
 */
export declare enum ParticleFlag {
    waterParticle = 0,
    zombieParticle = 2,
    wallParticle = 4,
    springParticle = 8,
    elasticParticle = 16,
    viscousParticle = 32,
    powderParticle = 64,
    tensileParticle = 128,
    colorMixingParticle = 256,
    destructionListenerParticle = 512,
    barrierParticle = 1024,
    staticPressureParticle = 2048,
    reactiveParticle = 4096,
    repulsiveParticle = 8192,
    fixtureContactListenerParticle = 16384,
    particleContactListenerParticle = 32768,
    fixtureContactFilterParticle = 65536,
    particleContactFilterParticle = 131072
}
export interface IParticleDef {
    flags?: ParticleFlag;
    position?: XY;
    velocity?: XY;
    color?: RGBA;
    lifetime?: number;
    userData?: any;
    group?: ParticleGroup | null;
}
export declare class ParticleDef implements IParticleDef {
    flags: ParticleFlag;
    readonly position: Vec2;
    readonly velocity: Vec2;
    readonly color: Color;
    lifetime: number;
    userData: any;
    group: ParticleGroup | null;
}
export declare function CalculateParticleIterations(gravity: number, radius: number, timeStep: number): number;
export declare class ParticleHandle {
    m_index: number;
    GetIndex(): number;
    SetIndex(index: number): void;
}
