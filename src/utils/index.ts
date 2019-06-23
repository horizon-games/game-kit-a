export function copyDefaults(onto: any, from: any) {
  for (const key of Object.keys(from)) {
    if (!onto.hasOwnProperty(key)) {
      onto[key] = from[key]
    }
  }
}
