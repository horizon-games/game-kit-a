import { Draw } from '../Common/Draw';
import { TimeStep } from '../Dynamics/TimeStep';
import { Controller } from './Controller';
/**
 * Applies simplified gravity between every pair of bodies
 */
export declare class GravityController extends Controller {
    private static Step_s_f;
    /**
     * Specifies the strength of the gravitiation force
     */
    G: number;
    /**
     * If true, gravity is proportional to r^-2, otherwise r^-1
     */
    invSqr: boolean;
    /**
     * @see Controller::Step
     */
    Step(step: TimeStep): void;
    Draw(draw: Draw): void;
}
