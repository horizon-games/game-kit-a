import { Color, Mesh } from 'three';
import { AnimatedBool } from '~/utils/AnimatedBool';
import { NoduleSphere } from './NoduleSphere';
export declare class Ball extends NoduleSphere {
    private _radius;
    readonly color: Color;
    readonly explodingState: AnimatedBool;
    readonly validTargets: this[];
    collider: Mesh;
    private _explosionSize;
    constructor(_radius: number, useCustomCollider?: boolean, nodules?: number, depth?: number);
    explode(size: number): void;
    private updateExplosion;
}
