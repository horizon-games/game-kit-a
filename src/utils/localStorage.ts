import { Color } from 'three'

import { hexColor } from './colors'
import { clamp } from './math'

export function getLocalStorageParam(key: string) {
  return localStorage.getItem(key)
}

export function setLocalStorageParam(key: string, val: string) {
  return localStorage.setItem(key, val)
}

export function getLocalStorageFlag(key: string) {
  const result = getLocalStorageParam(key)
  return !!(result === '' || (result && result !== 'false'))
}

export function setLocalStorageFlag(key: string, val: boolean) {
  setLocalStorageParam(key, val ? 'true' : 'false')
}

function __getLocalStorageNumber(
  key: string,
  defaultVal: number,
  parser: (val: string) => number,
  min = -Infinity,
  max = Infinity
) {
  return clamp(
    parser(getLocalStorageParam(key) || defaultVal.toString()),
    min,
    max
  )
}

function __setLocalStorageNumber(key: string, val: number) {
  return setLocalStorageParam(key, val.toString())
}

export function getLocalStorageFloat(
  key: string,
  defaultVal: number,
  min = -Infinity,
  max = Infinity
) {
  return __getLocalStorageNumber(key, defaultVal, parseFloat, min, max)
}
export function setLocalStorageFloat(key: string, val: number) {
  return __setLocalStorageNumber(key, val)
}

export function getLocalStorageInt(
  key: string,
  defaultVal: number,
  min = -Infinity,
  max = Infinity
) {
  return __getLocalStorageNumber(key, defaultVal, parseInt, min, max)
}
export function setLocalStorageInt(key: string, val: number) {
  return __setLocalStorageNumber(key, val)
}

export function getLocalStorageColor(key: string, defaultColor: string) {
  let str = getLocalStorageParam(key)
  if (!str) {
    str = defaultColor
  }
  return hexColor('#' + str)
}

export function setLocalStorageColor(key: string, color: Color) {
  setLocalStorageParam(key, color.getHexString())
}
