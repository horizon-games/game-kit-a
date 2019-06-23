import { removeFromArray } from '~/utils/arrayUtils'

type ChangeCallback = (newVal: any, oldVal: any) => void

class LiveProperty {
  get listenerCount() {
    return this.listeners.length
  }
  private obj: any
  private propName: string
  private value: any
  private listeners: ChangeCallback[]
  constructor(obj: any, propName: string) {
    this.propName = propName
    this.listeners = []
    this.setValue = this.setValue.bind(this)
    this.attach(obj)
  }

  attach(obj: any) {
    if (this.obj) {
      this.release()
    }
    this.obj = obj
    const value = this.obj[this.propName]
    Object.defineProperty(obj, this.propName, {
      configurable: true,
      set: this.setValue,
      get: () => this.value
    })
    this.setValue(value)
  }
  release() {
    Object.defineProperty(this.obj, this.propName, {
      value: this.value,
      writable: true
    })
  }
  hasListener(listener: ChangeCallback) {
    return this.listeners.indexOf(listener) !== -1
  }
  addListener(listener: ChangeCallback, firstOneForFree: boolean = true) {
    if (firstOneForFree) {
      listener(this.value, undefined)
    }
    this.listeners.push(listener)
  }
  removeListener(listener: ChangeCallback) {
    removeFromArray(this.listeners, listener)
  }
  private setValue(value: any) {
    if (this.value === value) {
      return
    }
    const oldValue = this.value
    this.value = value
    for (const listener of this.listeners) {
      listener(value, oldValue)
    }
  }
}

const propGroupLibrary = new Map<any, Map<string, LiveProperty>>()

function getObjectPropGroup(obj: any) {
  if (!propGroupLibrary.has(obj)) {
    propGroupLibrary.set(obj, new Map<string, LiveProperty>())
  }
  return propGroupLibrary.get(obj)!
}

function getLiveProperty(obj: any, propName: string) {
  const objectPropGroup = getObjectPropGroup(obj)
  if (!objectPropGroup.has(propName)) {
    objectPropGroup.set(propName, new LiveProperty(obj, propName))
  }
  return objectPropGroup.get(propName)!
}

export function listenToProperty(
  obj: any,
  propName: string,
  onChange: ChangeCallback,
  firstOneForFree: boolean = true
) {
  getLiveProperty(obj, propName).addListener(onChange, firstOneForFree)
}

export function stopListeningToProperty(
  obj: any,
  propName: string,
  onChange: ChangeCallback
) {
  const propGroup = propGroupLibrary.get(obj)
  if (propGroup) {
    const liveProp = propGroup.get(propName)
    if (liveProp) {
      liveProp.removeListener(onChange)
      if (liveProp.listenerCount === 0) {
        liveProp.release()
        propGroup.delete(propName)
      }
    }
    if (propGroup.size === 0) {
      propGroupLibrary.delete(obj)
    }
  }
}

export function migrateLiveProperty(
  oldObj: any,
  newObj: any,
  propName: string
) {
  const oldPropGroup = propGroupLibrary.get(oldObj)
  if (oldPropGroup) {
    const liveProp = oldPropGroup.get(propName)
    if (liveProp) {
      liveProp.attach(newObj)
      oldPropGroup.delete(propName)
      const newPropGroup = getObjectPropGroup(newObj)
      newPropGroup.set(propName, liveProp)
      if (oldPropGroup.size === 0) {
        propGroupLibrary.delete(oldObj)
      }
    }
  }
}
