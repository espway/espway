module.exports = {
    plugins: [
        require('postcss-cssnext')({ browsers: '> 1%' }),
        require('cssnano')({ autoprefixer: false })
    ]
};

