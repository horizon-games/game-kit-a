import { Easing } from '~/animation/Easing'
import { NumberEaser } from '~/animation/RawTweener'
import { simpleTweener } from '~/animation/tweeners'

type ProgressCallback = (val: number) => void

export class AnimatedBool {
  private _value: boolean
  private _durationMSOut: number
  private _animatedValue: number
  constructor(
    private _onUpdate: ProgressCallback,
    initVal: boolean = false,
    private _durationMS: number = 500,
    public easing: NumberEaser = Easing.Quartic.InOut,
    durationMSOut: number = -1
  ) {
    this._value = initVal
    this._durationMSOut = durationMSOut === -1 ? _durationMS : durationMSOut
    this._animatedValue = initVal ? 1 : 0
  }
  pulse() {
    this.value = true
    this.value = false
  }
  set animatedValue(val: number) {
    this._animatedValue = val
    this._onUpdate(val)
  }
  get animatedValue() {
    return this._animatedValue
  }
  set value(val: boolean) {
    if (val === this._value) {
      return
    }
    simpleTweener.killTweensOf(this)

    const desiredVal = val ? 1 : 0

    const duration =
      (desiredVal > this._animatedValue
        ? this._durationMS
        : this._durationMSOut) * Math.abs(this._animatedValue - desiredVal)

    if (duration === 0) {
      this.animatedValue = desiredVal
    } else {
      simpleTweener.to({
        target: this,
        propertyGoals: {
          animatedValue: desiredVal
        },
        duration,
        easing: this.easing
      })
    }
    this._value = val
  }
  get value() {
    return this._value
  }
}
