import { GrowableStack } from '../Common/GrowableStack';
import { Vec2, XY } from '../Common/Math';
import { AABB, RayCastInput } from './Collision';
export declare class TreeNode<T> {
    m_id: number;
    readonly aabb: AABB;
    userData: T;
    parent: TreeNode<T> | null;
    child1: TreeNode<T> | null;
    child2: TreeNode<T> | null;
    height: number;
    constructor(id?: number);
    IsLeaf(): boolean;
}
export declare class DynamicTree<T> {
    static readonly s_r: Vec2;
    static readonly s_v: Vec2;
    static readonly s_abs_v: Vec2;
    static readonly s_segmentAABB: AABB;
    static readonly s_subInput: RayCastInput;
    static readonly s_combinedAABB: AABB;
    static readonly s_aabb: AABB;
    static s_node_id: number;
    private static GetAreaNode;
    private static GetMaxBalanceNode;
    private static ShiftOriginNode;
    m_root: TreeNode<T> | null;
    m_freeList: TreeNode<T> | null;
    m_path: number;
    m_insertionCount: number;
    readonly m_stack: GrowableStack<TreeNode<T>>;
    Query(aabb: AABB, callback: (node: TreeNode<T>) => boolean): void;
    QueryPoint(point: Vec2, callback: (node: TreeNode<T>) => boolean): void;
    RayCast(input: RayCastInput, callback: (input: RayCastInput, node: TreeNode<T>) => number): void;
    AllocateNode(): TreeNode<T>;
    FreeNode(node: TreeNode<T>): void;
    CreateProxy(aabb: AABB, userData: T): TreeNode<T>;
    DestroyProxy(proxy: TreeNode<T>): void;
    MoveProxy(proxy: TreeNode<T>, aabb: AABB, displacement: Vec2): boolean;
    InsertLeaf(leaf: TreeNode<T>): void;
    RemoveLeaf(leaf: TreeNode<T>): void;
    Balance(A: TreeNode<T>): TreeNode<T>;
    GetHeight(): number;
    GetAreaRatio(): number;
    ComputeHeightNode(node: TreeNode<T> | null): number;
    ComputeHeight(): number;
    ValidateStructure(index: TreeNode<T> | null): void;
    ValidateMetrics(index: TreeNode<T> | null): void;
    Validate(): void;
    GetMaxBalance(): number;
    RebuildBottomUp(): void;
    ShiftOrigin(newOrigin: XY): void;
}