import { Draw } from '../Common/Draw';
import { Vec2 } from '../Common/Math';
import { TimeStep } from '../Dynamics/TimeStep';
import { Controller } from './Controller';
/**
 * Applies a force every frame
 */
export declare class ConstantForceController extends Controller {
    /**
     * The force to apply
     */
    readonly F: Vec2;
    Step(step: TimeStep): void;
    Draw(draw: Draw): void;
}
