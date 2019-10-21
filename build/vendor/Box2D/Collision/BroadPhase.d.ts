import { Vec2, XY } from '../Common/Math';
import { AABB, RayCastInput } from './Collision';
import { DynamicTree, TreeNode } from './DynamicTree';
export declare class Pair<T> {
    proxyA: TreeNode<T>;
    proxyB: TreeNode<T>;
    constructor(proxyA: TreeNode<T>, proxyB: TreeNode<T>);
}
export declare class BroadPhase<T> {
    readonly m_tree: DynamicTree<T>;
    m_proxyCount: number;
    m_moveCount: number;
    readonly m_moveBuffer: Array<TreeNode<T> | null>;
    m_pairCount: number;
    readonly m_pairBuffer: Array<Pair<T>>;
    CreateProxy(aabb: AABB, userData: T): TreeNode<T>;
    DestroyProxy(proxy: TreeNode<T>): void;
    MoveProxy(proxy: TreeNode<T>, aabb: AABB, displacement: Vec2): void;
    TouchProxy(proxy: TreeNode<T>): void;
    GetProxyCount(): number;
    UpdatePairs(callback: (a: T, b: T) => void): void;
    Query(aabb: AABB, callback: (node: TreeNode<T>) => boolean): void;
    QueryPoint(point: Vec2, callback: (node: TreeNode<T>) => boolean): void;
    RayCast(input: RayCastInput, callback: (input: RayCastInput, node: TreeNode<T>) => number): void;
    GetTreeHeight(): number;
    GetTreeBalance(): number;
    GetTreeQuality(): number;
    ShiftOrigin(newOrigin: XY): void;
    BufferMove(proxy: TreeNode<T>): void;
    UnBufferMove(proxy: TreeNode<T>): void;
}
export declare function PairLessThan<T>(pair1: Pair<T>, pair2: Pair<T>): number;
