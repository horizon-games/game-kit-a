import { simpleTweener } from '~/animation/tweeners';
import { removeFromArray } from '~/utils/arrayUtils';
export class TimedTask {
    constructor(expireTime, task) {
        this.expireTime = expireTime;
        this._task = task;
    }
    task() {
        if (this._task) {
            this._task();
            this._task = undefined;
        }
        else {
            console.warn('timed task asked to fire twice??');
        }
    }
}
class Timer {
    constructor() {
        this.time = 0;
        this.tasks = [];
    }
    update(dt) {
        this.time += dt;
        while (this.tasks.length > 0 && this.tasks[0].expireTime <= this.time) {
            this.tasks.shift().task();
        }
    }
    add(task, delay, compensateTimeWarp = false) {
        if (compensateTimeWarp) {
            delay *= simpleTweener.speed;
        }
        const timedTask = new TimedTask(this.time + delay, task);
        this.tasks.push(timedTask);
        this.tasks.sort((a, b) => a.expireTime - b.expireTime);
        return timedTask;
    }
    runPrematurely(timedTask) {
        removeFromArray(this.tasks, timedTask);
        timedTask.task();
    }
    cancel(timedTask) {
        removeFromArray(this.tasks, timedTask);
    }
}
export const taskTimer = new Timer();
//# sourceMappingURL=taskTimer.js.map