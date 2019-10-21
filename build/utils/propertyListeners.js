import { removeFromArray } from '../utils/arrayUtils';
class LiveProperty {
    get listenerCount() {
        return this.listeners.length;
    }
    constructor(obj, propName) {
        this.propName = propName;
        this.listeners = [];
        this.setValue = this.setValue.bind(this);
        this.attach(obj);
    }
    attach(obj) {
        if (this.obj) {
            this.release();
        }
        this.obj = obj;
        const value = this.obj[this.propName];
        Object.defineProperty(obj, this.propName, {
            configurable: true,
            set: this.setValue,
            get: () => this.value
        });
        this.setValue(value);
    }
    release() {
        Object.defineProperty(this.obj, this.propName, {
            value: this.value,
            writable: true
        });
    }
    hasListener(listener) {
        return this.listeners.indexOf(listener) !== -1;
    }
    addListener(listener, firstOneForFree = true) {
        if (firstOneForFree) {
            listener(this.value, undefined);
        }
        this.listeners.push(listener);
    }
    removeListener(listener) {
        removeFromArray(this.listeners, listener);
    }
    setValue(value) {
        if (this.value === value) {
            return;
        }
        const oldValue = this.value;
        this.value = value;
        for (const listener of this.listeners) {
            listener(value, oldValue);
        }
    }
}
const propGroupLibrary = new Map();
function getObjectPropGroup(obj) {
    if (!propGroupLibrary.has(obj)) {
        propGroupLibrary.set(obj, new Map());
    }
    return propGroupLibrary.get(obj);
}
function getLiveProperty(obj, propName) {
    const objectPropGroup = getObjectPropGroup(obj);
    if (!objectPropGroup.has(propName)) {
        objectPropGroup.set(propName, new LiveProperty(obj, propName));
    }
    return objectPropGroup.get(propName);
}
export function listenToProperty(obj, propName, onChange, firstOneForFree = true) {
    getLiveProperty(obj, propName).addListener(onChange, firstOneForFree);
}
export function stopListeningToProperty(obj, propName, onChange) {
    const propGroup = propGroupLibrary.get(obj);
    if (propGroup) {
        const liveProp = propGroup.get(propName);
        if (liveProp) {
            liveProp.removeListener(onChange);
            if (liveProp.listenerCount === 0) {
                liveProp.release();
                propGroup.delete(propName);
            }
        }
        if (propGroup.size === 0) {
            propGroupLibrary.delete(obj);
        }
    }
}
export function migrateLiveProperty(oldObj, newObj, propName) {
    const oldPropGroup = propGroupLibrary.get(oldObj);
    if (oldPropGroup) {
        const liveProp = oldPropGroup.get(propName);
        if (liveProp) {
            liveProp.attach(newObj);
            oldPropGroup.delete(propName);
            const newPropGroup = getObjectPropGroup(newObj);
            newPropGroup.set(propName, liveProp);
            if (oldPropGroup.size === 0) {
                propGroupLibrary.delete(oldObj);
            }
        }
    }
}
//# sourceMappingURL=propertyListeners.js.map