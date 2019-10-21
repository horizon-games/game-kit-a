import { Draw } from '../Common/Draw';
import { Mat22 } from '../Common/Math';
import { TimeStep } from '../Dynamics/TimeStep';
import { Controller } from './Controller';
/**
 * Applies top down linear damping to the controlled bodies
 * The damping is calculated by multiplying velocity by a matrix
 * in local co-ordinates.
 */
export declare class TensorDampingController extends Controller {
    private static Step_s_damping;
    readonly T: Mat22;
    maxTimestep: number;
    /**
     * @see Controller::Step
     */
    Step(step: TimeStep): void;
    Draw(draw: Draw): void;
    /**
     * Sets damping independantly along the x and y axes
     */
    SetAxisAligned(xDamping: number, yDamping: number): void;
}
