export function copyDefaults(onto, from) {
    for (const key of Object.keys(from)) {
        if (!onto.hasOwnProperty(key)) {
            onto[key] = from[key];
        }
    }
}
//# sourceMappingURL=index.js.map