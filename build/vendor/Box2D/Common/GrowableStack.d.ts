export declare class GrowableStack<T> {
    m_stack: Array<T | null>;
    m_count: number;
    constructor(N: number);
    Reset(): this;
    Push(element: T): void;
    Pop(): T;
    GetCount(): number;
}
