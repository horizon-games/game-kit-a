export declare class Timer {
    m_start: number;
    Reset(): Timer;
    GetMilliseconds(): number;
}
export declare class Counter {
    m_count: number;
    m_min_count: number;
    m_max_count: number;
    GetCount(): number;
    GetMinCount(): number;
    GetMaxCount(): number;
    ResetCount(): number;
    ResetMinCount(): void;
    ResetMaxCount(): void;
    Increment(): void;
    Decrement(): void;
}
