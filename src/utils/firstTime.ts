const happenedRegistry: string[] = []

export function firstTime(key: string) {
  if (happenedRegistry.indexOf(key) === -1) {
    happenedRegistry.push(key)
    return true
  }
  return false
}
