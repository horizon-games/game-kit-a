import { Draw } from '../Common/Draw';
import { Vec2 } from '../Common/Math';
import { TimeStep } from '../Dynamics/TimeStep';
import { Controller } from './Controller';
/**
 * Applies a force every frame
 */
export declare class ConstantAccelController extends Controller {
    private static Step_s_dtA;
    /**
     * The acceleration to apply
     */
    readonly A: Vec2;
    Step(step: TimeStep): void;
    Draw(draw: Draw): void;
}
