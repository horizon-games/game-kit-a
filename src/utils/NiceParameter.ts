import { removeFromArray } from './arrayUtils'
import { getLocalStorageFloat } from './localStorage'
import { getUrlFloat } from './location'
import { clamp } from './math'

export const niceParameters: NiceParameter[] = []

type Listener = (value: number) => void

export class NiceParameter {
  name: string
  label: string
  defaultValue: number
  minValue: number
  maxValue: number
  step: number
  sliderOrderPriority: number
  distribution: (value: number) => number
  valueTextConverter: (value: number) => string
  private _availableToUserMethod: () => boolean
  private _value: number
  private _listeners: Listener[]
  constructor(
    name: string,
    label: string,
    defaultValue: number,
    minValue: number,
    maxValue: number,
    distribution: (value: number) => number,
    valueTextConverter: (value: number) => string,
    availableToUser: (() => boolean) | boolean,
    forceDefault: boolean = false,
    step: number = 0.01,
    sliderOrderPriority: number = 0
  ) {
    this.name = name
    this.label = label
    this.defaultValue = defaultValue
    this.minValue = minValue
    this.maxValue = maxValue
    this.step = step
    this.distribution = distribution
    this.valueTextConverter = valueTextConverter
    this._availableToUserMethod =
      availableToUser === true || availableToUser === false
        ? () => availableToUser
        : availableToUser
    this._listeners = []
    if (forceDefault) {
      this.value = this.defaultValue
    }
    this._value = getLocalStorageFloat(
      'nice-param-' + name,
      defaultValue,
      minValue,
      maxValue
    )
    const urlVal = getUrlFloat(name, this._value, minValue, maxValue)
    if (urlVal !== this._value) {
      this.value = urlVal
    }
    niceParameters.push(this)
    if (sliderOrderPriority === 0) {
      sliderOrderPriority = niceParameters.length
    }
    this.sliderOrderPriority = sliderOrderPriority
  }

  listen(callback: Listener) {
    this._listeners.push(callback)
    callback(this.value)
  }

  stopListening(callback: Listener) {
    removeFromArray(this._listeners, callback)
  }

  set value(val: number) {
    if (val === this._value) {
      return
    }
    val = clamp(val, this.minValue, this.maxValue)
    if (val === this._value) {
      return
    }
    this._value = val
    localStorage.setItem('nice-param-' + this.name, String(val))
    this._listeners.forEach(cb => cb(val))
  }

  get value() {
    return this._value
  }

  get availableToUser() {
    return this._availableToUserMethod()
  }
}
