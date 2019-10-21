const happenedRegistry = [];
export function firstTime(key) {
    if (happenedRegistry.indexOf(key) === -1) {
        happenedRegistry.push(key);
        return true;
    }
    return false;
}
//# sourceMappingURL=firstTime.js.map