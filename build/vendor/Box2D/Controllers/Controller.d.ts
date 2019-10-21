import { Draw } from '../Common/Draw';
import { Body } from '../Dynamics/Body';
import { TimeStep } from '../Dynamics/TimeStep';
/**
 * A controller edge is used to connect bodies and controllers
 * together in a bipartite graph.
 */
export declare class ControllerEdge {
    controller: Controller;
    body: Body;
    prevBody: ControllerEdge | null;
    nextBody: ControllerEdge | null;
    prevController: ControllerEdge | null;
    nextController: ControllerEdge | null;
    constructor(controller: Controller, body: Body);
}
/**
 * Base class for controllers. Controllers are a convience for
 * encapsulating common per-step functionality.
 */
export declare abstract class Controller {
    m_bodyList: ControllerEdge | null;
    m_bodyCount: number;
    m_prev: Controller | null;
    m_next: Controller | null;
    /**
     * Controllers override this to implement per-step functionality.
     */
    abstract Step(step: TimeStep): void;
    /**
     * Controllers override this to provide debug drawing.
     */
    abstract Draw(debugDraw: Draw): void;
    /**
     * Get the next controller in the world's body list.
     */
    GetNext(): Controller | null;
    /**
     * Get the previous controller in the world's body list.
     */
    GetPrev(): Controller | null;
    /**
     * Get the parent world of this body.
     */
    /**
     * Get the attached body list
     */
    GetBodyList(): ControllerEdge | null;
    /**
     * Adds a body to the controller list.
     */
    AddBody(body: Body): void;
    /**
     * Removes a body from the controller list.
     */
    RemoveBody(body: Body): void;
    /**
     * Removes all bodies from the controller list.
     */
    Clear(): void;
}
