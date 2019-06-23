const red = 0
const green = 1 / 3
const blue = 2 / 3

const yellow = red + 1 / 6
const cyan = green + 1 / 6
const majenta = blue + 1 / 6

const coolRed = red - 1 / 18
const warmRed = red + 1 / 18

const warmYellow = yellow - 1 / 18
const coolYellow = yellow + 1 / 18

const warmGreen = green - 1 / 18
const coolGreen = green + 1 / 18

const warmCyan = cyan - 1 / 18
const coolCyan = cyan + 1 / 18

const coolBlue = blue - 1 / 18
const warmBlue = blue + 1 / 18

const coolMajenta = majenta - 1 / 18
const warmMajenta = majenta + 1 / 18

export const hues = {
  _01_coolRed: coolRed,
  _02_red: red,
  _03_warmRed: warmRed,
  _04_warmYellow: warmYellow,
  _05_yellow: yellow,
  _06_coolYellow: coolYellow,
  _07_warmGreen: warmGreen,
  _08_green: green,
  _09_coolGreen: coolGreen,
  _10_warmCyan: warmCyan,
  _11_cyan: cyan,
  _12_coolCyan: coolCyan,
  _13_coolBlue: coolBlue,
  _14_blue: blue,
  _15_warmBlue: warmBlue,
  _16_coolMajenta: coolMajenta,
  _17_majenta: majenta,
  _18_warmMajenta: warmMajenta
}
