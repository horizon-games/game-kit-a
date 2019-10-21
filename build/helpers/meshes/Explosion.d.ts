import { AnimatedBool } from '../../utils/AnimatedBool';
import { NoduleSphere } from './NoduleSphere';
export declare class Explosion extends NoduleSphere {
    private _radius;
    readonly explodingState: AnimatedBool;
    private _explosionSize;
    private mat;
    constructor(_radius: number, nodules?: number, depth?: number, size?: number);
    explode(size: number): void;
    private updateExplosion;
}
