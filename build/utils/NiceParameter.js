import { removeFromArray } from './arrayUtils';
import { getLocalStorageFloat } from './localStorage';
import { getUrlFloat } from './location';
import { clamp } from './math';
export const niceParameters = [];
export class NiceParameter {
    constructor(name, label, defaultValue, minValue, maxValue, distribution, valueTextConverter, availableToUser, forceDefault = false, step = 0.01, sliderOrderPriority = 0) {
        this.name = name;
        this.label = label;
        this.defaultValue = defaultValue;
        this.minValue = minValue;
        this.maxValue = maxValue;
        this.step = step;
        this.distribution = distribution;
        this.valueTextConverter = valueTextConverter;
        this._availableToUserMethod =
            availableToUser === true || availableToUser === false
                ? () => availableToUser
                : availableToUser;
        this._listeners = [];
        if (forceDefault) {
            this.value = this.defaultValue;
        }
        this._value = getLocalStorageFloat('nice-param-' + name, defaultValue, minValue, maxValue);
        const urlVal = getUrlFloat(name, this._value, minValue, maxValue);
        if (urlVal !== this._value) {
            this.value = urlVal;
        }
        niceParameters.push(this);
        if (sliderOrderPriority === 0) {
            sliderOrderPriority = niceParameters.length;
        }
        this.sliderOrderPriority = sliderOrderPriority;
    }
    listen(callback) {
        this._listeners.push(callback);
        callback(this.value);
    }
    stopListening(callback) {
        removeFromArray(this._listeners, callback);
    }
    set value(val) {
        if (val === this._value) {
            return;
        }
        val = clamp(val, this.minValue, this.maxValue);
        if (val === this._value) {
            return;
        }
        this._value = val;
        localStorage.setItem('nice-param-' + this.name, String(val));
        this._listeners.forEach(cb => cb(val));
    }
    get value() {
        return this._value;
    }
    get availableToUser() {
        return this._availableToUserMethod();
    }
}
//# sourceMappingURL=NiceParameter.js.map