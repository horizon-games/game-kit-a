import { Vec2 } from '../Common/Math';
/**
 * A field representing the nearest generator from each point.
 */
export declare class VoronoiDiagram {
    m_generatorBuffer: VoronoiDiagramGenerator[];
    m_generatorCapacity: number;
    m_generatorCount: number;
    m_countX: number;
    m_countY: number;
    m_diagram: VoronoiDiagramGenerator[];
    constructor(generatorCapacity: number);
    /**
     * Add a generator.
     *
     * @param center the position of the generator.
     * @param tag a tag used to identify the generator in callback functions.
     * @param necessary whether to callback for nodes associated with the generator.
     */
    AddGenerator(center: Vec2, tag: number, necessary: boolean): void;
    /**
     * Generate the Voronoi diagram. It is rasterized with a given
     * interval in the same range as the necessary generators exist.
     *
     * @param radius the interval of the diagram.
     * @param margin margin for which the range of the diagram is extended.
     */
    Generate(radius: number, margin: number): void;
    /**
     * Enumerate all nodes that contain at least one necessary
     * generator.
     */
    GetNodes(callback: VoronoiDiagramNodeCallback): void;
}
/**
 * Callback used by GetNodes().
 *
 * Receive tags for generators associated with a node.
 */
export declare type VoronoiDiagramNodeCallback = (a: number, b: number, c: number) => void;
export declare class VoronoiDiagramGenerator {
    center: Vec2;
    tag: number;
    necessary: boolean;
}
export declare class VoronoiDiagramTask {
    m_x: number;
    m_y: number;
    m_i: number;
    m_generator: VoronoiDiagramGenerator;
    constructor(x: number, y: number, i: number, g: VoronoiDiagramGenerator);
}
