import { Color } from 'three';
import { hexColor } from './colors';
import { clamp } from './math';
export function getUrlParam(param) {
    __setReminder(param);
    return new URL(window.location.href).searchParams.get(param);
}
export function getUrlFlag(param) {
    const result = getUrlParam(param);
    return !!(result === '' || (result && result !== 'false'));
}
function __getUrlNumber(param, defaultVal, parser, min = -Infinity, max = Infinity) {
    return clamp(parser(getUrlParam(param) || defaultVal.toString()), min, max);
}
export function getUrlFloat(param, defaultVal, min = -Infinity, max = Infinity) {
    return __getUrlNumber(param, defaultVal, parseFloat, min, max);
}
export function getUrlInt(param, defaultVal, min = -Infinity, max = Infinity) {
    return __getUrlNumber(param, defaultVal, parseInt, min, max);
}
export function getUrlColor(param, defaultColor) {
    let str = getUrlParam(param);
    if (!str) {
        if (defaultColor instanceof Color) {
            return defaultColor;
        }
        else {
            str = defaultColor;
        }
    }
    return hexColor('#' + str);
}
const __keysToRemember = [];
let __reminderQueued = false;
function __setReminder(name) {
    if (!__keysToRemember.includes(name)) {
        __keysToRemember.push(name);
        if (!__reminderQueued) {
            __reminderQueued = true;
            setTimeout(() => {
                console.log('Nice Parameters to try: ' + __keysToRemember.join(', '));
                __reminderQueued = false;
            }, 2000);
        }
    }
}
//# sourceMappingURL=location.js.map