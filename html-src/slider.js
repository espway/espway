
    function log10(x) {
        return Math.log(x) * Math.LOG10E;
    }

    this.setBounds = function(boundMin, boundMax) {
        this.boundMin = boundMin
        this.boundMax = boundMax
        this.step = (boundMax - boundMin) / 100
    }.bind(this)

    this.sliderChange = function(e) {
        let value = Math.pow(10, this.refs.slider.value)
        if (this.centered) {
            value *= this.centerValue
        }
        this.value = value
    }.bind(this)

    this.refreshValue = function() {
        let sliderValue = log10(this.value)
        if (this.centered) {
            sliderValue -= log10(this.centerValue)
        }
        this.refs.slider.value = sliderValue
        if (this.value === 0) {
            this.mantissa = 0
            this.exponent = 0
        } else {
            let exponent = Math.ceil(log10(this.value))
            let mantissa = Math.round(this.value * Math.pow(10, 2 - exponent))
            if (mantissa == 100) {
                exponent += 1
                mantissa = 10
            }
            this.mantissa = (mantissa / 10).toFixed(1)
            this.exponent = exponent - 1
        }
    }.bind(this)

    this.enable = function() {
        this.resetSlider()
        this.value = Math.pow(10, this.boundMin)
        this.enabled = true
    }.bind(this)

    this.disable = function() {
        this.value = 0
        this.enabled = false
    }.bind(this)

    this.resetSlider = function() {
        this.centered = false
        this.setBounds(this.opts.boundMin, this.opts.boundMax)
    }.bind(this)

    this.centerClick = function(e) {
        this.centered = true
        this.centerValue = this.value
        this.setBounds(-1, 1)
        this.refs.slider.value = 0
    }.bind(this)

    this.resetClick = function(e) {
        this.resetSlider()
    }.bind(this)

    this.toggle = function(e) {
        if (this.enabled) {
            this.disable()
        } else {
            this.enable()
        }
    }.bind(this)

    this.resetSlider()
    this.enabled = false

    this.on('mount', () => {
        this.value = this.opts.myValue
        if (this.value > 0) {
            this.enabled = true
        }
        this.update()
    })

    this.on('update', () => {
        this.refreshValue()
    })

