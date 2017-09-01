var path = require('path');

var TARGET_PATH = 'output';

module.exports = {
    context: path.resolve(__dirname, 'src'),

    entry: {
        index: './index.js',
        pid: './pid.js'
    },

    output: {
        filename: '[name].bundle.js',
        path: path.resolve(__dirname, TARGET_PATH)
    },

    module: {
        rules: [
            {
                test: /\.tag$/,
                enforce: 'pre',
                use: [
                    { loader: 'babel-loader', options: { presets: ['es2015'] } },
                    { loader: './riot-loader.js' }
                ]
            },

            {
                test: /\.js$/,
                loader: 'babel-loader',
                options: { presets: ['es2015'] }
            },

            {
                test: /\.css$/,
                use: [ 'style-loader', 'css-loader', 'postcss-loader' ]
            }
        ]
    },

    devServer: {
        contentBase: path.resolve(__dirname, TARGET_PATH)
    }
};

