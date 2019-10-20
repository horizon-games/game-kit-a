import { copyDefaults } from '~/utils';
import { lerp } from '~/utils/math';
import { Easing } from './Easing';
const __defaultOptions = {
    delay: 0,
    duration: 1000,
    easing: Easing.Linear
};
class AnimatedProperty {
    constructor(key, valueStart, valueEnd) {
        this.key = key;
        this.valueStart = valueStart;
        this.valueEnd = valueEnd;
        //nothing
    }
}
class AnimatedObject {
    constructor(tweener, target, easing, startTime, endTime, animatedProperties, onUpdate, onComplete) {
        this.tweener = tweener;
        this.target = target;
        this.easing = easing;
        this.startTime = startTime;
        this.endTime = endTime;
        this.animatedProperties = animatedProperties;
        this.onUpdate = onUpdate;
        this.duration = endTime - startTime;
        this.finished = new Promise(resolve => {
            this.onComplete = () => {
                if (onComplete) {
                    onComplete();
                }
                resolve();
            };
        });
    }
    kill() {
        this.tweener.kill(this);
    }
}
export class RawTweener {
    constructor() {
        this._now = 0;
        this._processingTick = false;
        this._animations = [];
        this._animationsToComplete = [];
    }
    to(params) {
        this.killTweensOf(params.target);
        copyDefaults(params, __defaultOptions);
        if (typeof params.easing !== 'function') {
            throw new Error('ease must be an easing function that takes in a number (0..1) and returns a number (0..1)');
        }
        const target = params.target;
        const goals = params.propertyGoals;
        const animatedProperties = Object.keys(goals).map(key => {
            const numFrom = target[key];
            const numTo = goals[key];
            if (!isNaN(numFrom) && !isNaN(numTo)) {
                return new AnimatedProperty(key, numFrom, numTo);
            }
            else {
                throw new Error('values must be numbers');
            }
        });
        const startTime = this._now + params.delay;
        const endTime = startTime + params.duration;
        const animation = new AnimatedObject(this, target, params.easing, startTime, endTime, animatedProperties, params.onUpdate, params.onComplete);
        this._animations.push(animation);
        return animation;
    }
    tick(delta) {
        this._now += delta;
        const now = this._now;
        const animations = this._animations;
        this._processingTick = true;
        for (const animation of animations) {
            if (now > animation.startTime) {
                const target = animation.target;
                const progress = Math.min((now - animation.startTime) / animation.duration, 1);
                // if(isNaN(progress) || progress < 0 || progress > 1) throw new Error('Should not happen.');
                if (progress < 1) {
                    const mix = animation.easing(progress);
                    for (const ap of animation.animatedProperties) {
                        target[ap.key] = lerp(ap.valueStart, ap.valueEnd, mix);
                    }
                    if (animation.onUpdate) {
                        animation.onUpdate();
                    }
                }
                else {
                    this._animationsToComplete.push(animation);
                }
            }
        }
        this._processingTick = false;
        if (this._animationsToComplete.length > 0) {
            this._animationsToComplete.sort((a, b) => {
                return a.endTime - b.endTime;
            });
            for (const animation of this._animationsToComplete) {
                this.kill(animation);
            }
            this._animationsToComplete.length = 0;
        }
    }
    killTweensOf(target) {
        if (this._processingTick) {
            throw new Error('Not allowed during processing of tick');
        }
        const animations = this._animations;
        for (let i = animations.length - 1; i >= 0; i--) {
            const animation = animations[i];
            if (animation.target === target) {
                this.kill(animation, i);
            }
        }
    }
    kill(animation, index = -1) {
        if (this._processingTick) {
            throw new Error('Not allowed during processing of tick');
        }
        if (index === -1) {
            index = this._animations.indexOf(animation);
        }
        if (index !== -1) {
            for (const ap of animation.animatedProperties) {
                animation.target[ap.key] = ap.valueEnd;
            }
            if (animation.onUpdate) {
                animation.onUpdate();
            }
            if (animation.onComplete) {
                animation.onComplete();
            }
            this._animations.splice(index, 1);
        }
    }
}
//# sourceMappingURL=RawTweener.js.map