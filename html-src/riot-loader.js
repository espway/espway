var riot = require('riot-compiler');

module.exports = function(source) {
    this.cacheable();
    return riot.compile(source);
};

