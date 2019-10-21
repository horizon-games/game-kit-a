import { hexColor } from './colors';
import { clamp } from './math';
export function getLocalStorageParam(key) {
    return localStorage.getItem(key);
}
export function setLocalStorageParam(key, val) {
    return localStorage.setItem(key, val);
}
export function getLocalStorageFlag(key) {
    const result = getLocalStorageParam(key);
    return !!(result === '' || (result && result !== 'false'));
}
export function setLocalStorageFlag(key, val) {
    setLocalStorageParam(key, val ? 'true' : 'false');
}
function __getLocalStorageNumber(key, defaultVal, parser, min = -Infinity, max = Infinity) {
    return clamp(parser(getLocalStorageParam(key) || defaultVal.toString()), min, max);
}
function __setLocalStorageNumber(key, val) {
    return setLocalStorageParam(key, val.toString());
}
export function getLocalStorageFloat(key, defaultVal, min = -Infinity, max = Infinity) {
    return __getLocalStorageNumber(key, defaultVal, parseFloat, min, max);
}
export function setLocalStorageFloat(key, val) {
    return __setLocalStorageNumber(key, val);
}
export function getLocalStorageInt(key, defaultVal, min = -Infinity, max = Infinity) {
    return __getLocalStorageNumber(key, defaultVal, parseInt, min, max);
}
export function setLocalStorageInt(key, val) {
    return __setLocalStorageNumber(key, val);
}
export function getLocalStorageColor(key, defaultColor) {
    let str = getLocalStorageParam(key);
    if (!str) {
        str = defaultColor;
    }
    return hexColor('#' + str);
}
export function setLocalStorageColor(key, color) {
    setLocalStorageParam(key, color.getHexString());
}
//# sourceMappingURL=localStorage.js.map