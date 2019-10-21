export declare class TimedTask {
    expireTime: number;
    private _task;
    constructor(expireTime: number, task: () => void);
    task(): void;
}
declare class Timer {
    time: number;
    tasks: TimedTask[];
    constructor();
    update(dt: number): void;
    add(task: () => void, delay: number, compensateTimeWarp?: boolean): TimedTask;
    runPrematurely(timedTask: TimedTask): void;
    cancel(timedTask: TimedTask): void;
}
export declare const taskTimer: Timer;
export {};
