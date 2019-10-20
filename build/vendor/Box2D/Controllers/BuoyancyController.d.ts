import { Draw } from '../Common/Draw';
import { Vec2 } from '../Common/Math';
import { TimeStep } from '../Dynamics/TimeStep';
import { Controller } from './Controller';
/**
 * Calculates buoyancy forces for fluids in the form of a half
 * plane.
 */
export declare class BuoyancyController extends Controller {
    /**
     * The outer surface normal
     */
    readonly normal: Vec2;
    /**
     * The height of the fluid surface along the normal
     */
    offset: number;
    /**
     * The fluid density
     */
    density: number;
    /**
     * Fluid velocity, for drag calculations
     */
    readonly velocity: Vec2;
    /**
     * Linear drag co-efficient
     */
    linearDrag: number;
    /**
     * Angular drag co-efficient
     */
    angularDrag: number;
    /**
     * If false, bodies are assumed to be uniformly dense, otherwise
     * use the shapes densities
     */
    useDensity: boolean;
    /**
     * If true, gravity is taken from the world instead of the
     */
    useWorldGravity: boolean;
    /**
     * Gravity vector, if the world's gravity is not used
     */
    readonly gravity: Vec2;
    Step(step: TimeStep): void;
    Draw(debugDraw: Draw): void;
}
