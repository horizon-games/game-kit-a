const callbacks = [];
export function nextFrameUpdate() {
    if (callbacks.length > 0) {
        callbacks.forEach(cb => cb());
        callbacks.length = 0;
    }
}
export function onNextFrame(callback) {
    callbacks.push(callback);
}
//# sourceMappingURL=onNextFrame.js.map