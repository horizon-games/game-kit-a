import { getUrlParam } from './utils/location'

declare global {
  interface Window {
    APP_CONFIG: any
  }
}

export interface Environment {
  NODE_ENV: string
  GITCOMMIT: string
  ASSETS_URL: string
}

const env: Environment = {
  NODE_ENV: String(process.env.NODE_ENV || ''),
  GITCOMMIT: String(process.env.GITCOMMIT || ''),
  ASSETS_URL: String(
    getUrlParam('assetsUrl') || window.APP_CONFIG.ASSETS_URL || ''
  )
}
;(window as any).env = env

export default env
