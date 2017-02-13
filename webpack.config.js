var path = require('path');

module.exports = {
    context: path.resolve(__dirname, 'html-src'),

    entry: {
        index: './index.js',
        pid: './pid.js',
        cube: './cube.js'
    },

    output: {
        filename: '[name].bundle.js',
        path: path.resolve(__dirname, 'html')
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
    }
};

