import { getUrlParam } from './utils/location';
const env = {
    NODE_ENV: String(process.env.NODE_ENV || ''),
    GITCOMMIT: String(process.env.GITCOMMIT || ''),
    ASSETS_URL: String(getUrlParam('assetsUrl') || window.APP_CONFIG.ASSETS_URL || '')
};
window.env = env;
export default env;
//# sourceMappingURL=env.js.map