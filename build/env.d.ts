declare global {
    interface Window {
        APP_CONFIG: any;
    }
}
export interface Environment {
    NODE_ENV: string;
    GITCOMMIT: string;
    ASSETS_URL: string;
}
declare const env: Environment;
export default env;
