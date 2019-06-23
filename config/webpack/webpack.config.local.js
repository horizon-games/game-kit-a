const path = require('path')
const webpack = require('webpack')
const ForkTsCheckerWebpackPlugin = require('fork-ts-checker-webpack-plugin')
// const TsconfigPathsPlugin = require('tsconfig-paths-webpack-plugin');
const HtmlWebpackPlugin = require('html-webpack-plugin')

let dist = process.env.DIST
if (!dist || dist === '') {
  dist = 'local'
}
const appConfig = require(`../game.${dist}.json`)

const main = [
  'webpack-dev-server/client?http://0.0.0.0:3001',
  'webpack/hot/only-dev-server',
  './src/index.ts'
]

module.exports = dist => ({
  context: process.cwd(), // to automatically find tsconfig.json
  entry: {
    main: main
  },
  output: {
    path: path.resolve(__dirname, 'dist'),
    filename: '[name].js',
    publicPath: '/',
    globalObject: 'this'
  },
  plugins: [
    new webpack.HotModuleReplacementPlugin(),
    new webpack.NamedModulesPlugin(),
    new webpack.NoEmitOnErrorsPlugin(),
    new ForkTsCheckerWebpackPlugin({
      tslint: true,
      checkSyntacticErrors: true,
      watch: ['./src']
    }),
    new webpack.DefinePlugin({
      'process.env': {
        GITCOMMIT: JSON.stringify('dev'),
        MODE_ENV: JSON.stringify(dist ? dist.MODE_ENV : ''),
        APP_CONFIG: `'${JSON.stringify(appConfig)}'`
      }
    }),
    new webpack.ProvidePlugin({
      THREE: 'three'
    }),
    new HtmlWebpackPlugin({
      inject: false,
      template: 'src/index.html',
      gitcommit: ''
    })
  ],
  module: {
    rules: [
      {
        test: /.ts$/,
        use: [
          {
            loader: 'ts-loader',
            options: {
              transpileOnly: true,
              projectReferences: true
            }
          }
        ],
        exclude: path.resolve(process.cwd(), 'node_modules'),
        include: [path.resolve(process.cwd(), 'src')]
      },
      {
        test: /\.css$/,
        use: ['style-loader', 'css-loader']
      },
      {
        test: /\.(glsl|frag|vert)$/,
        exclude: /node_modules/,
        loader: 'glslify-import-loader'
      },
      {
        test: /\.(glsl|frag|vert)$/,
        exclude: /node_modules/,
        loader: 'raw-loader'
      },
      {
        test: /\.(glsl|frag|vert)$/,
        exclude: /node_modules/,
        loader: 'glslify-loader'
      },
      {
        test: /\.worker\.js$/,
        use: { loader: 'worker-loader' }
      }
    ]
  },
  resolve: {
    extensions: ['.ts', '.js'],
    alias: {
      '~': path.join(process.cwd(), 'src')
    },
    plugins: [
      // new TsconfigPathsPlugin()
    ]
  },
  devtool: 'eval-source-map',
  devServer: {
    host: '0.0.0.0',
    port: 3001,
    open: false,
    hot: true,
    historyApiFallback: true,
    stats: 'errors-only',
    disableHostCheck: true,
    contentBase: path.resolve(process.cwd(), 'src/public'),
    publicPath: '/'
  }
})
