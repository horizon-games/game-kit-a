import { Easing } from '~/animation/Easing';
import { simpleTweener } from '~/animation/tweeners';
export class AnimatedBool {
    constructor(_onUpdate, initVal = false, _durationMS = 500, easing = Easing.Quartic.InOut, durationMSOut = -1) {
        this._onUpdate = _onUpdate;
        this._durationMS = _durationMS;
        this.easing = easing;
        this._value = initVal;
        this._durationMSOut = durationMSOut === -1 ? _durationMS : durationMSOut;
        this._animatedValue = initVal ? 1 : 0;
    }
    pulse() {
        this.value = true;
        this.value = false;
    }
    set animatedValue(val) {
        this._animatedValue = val;
        this._onUpdate(val);
    }
    get animatedValue() {
        return this._animatedValue;
    }
    set value(val) {
        if (val === this._value) {
            return;
        }
        simpleTweener.killTweensOf(this);
        const desiredVal = val ? 1 : 0;
        const duration = (desiredVal > this._animatedValue
            ? this._durationMS
            : this._durationMSOut) * Math.abs(this._animatedValue - desiredVal);
        if (duration === 0) {
            this.animatedValue = desiredVal;
        }
        else {
            simpleTweener.to({
                target: this,
                propertyGoals: {
                    animatedValue: desiredVal
                },
                duration,
                easing: this.easing
            });
        }
        this._value = val;
    }
    get value() {
        return this._value;
    }
}
//# sourceMappingURL=AnimatedBool.js.map