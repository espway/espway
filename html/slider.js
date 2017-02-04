<slider my-value={ value }>
    <div class='sliderInnerContainer'>
        <div>
            <span class='bold'>{ opts.label }</span> =
            <span>{ mantissa }</span> × 10<sup>{ exponent }</sup>
        </div>
        <div>
            <button onclick={ toggle }>{ enabled ? '☑ On' : '☐ Off' }</button>
            <button onclick={ centerClick } disabled={ !enabled }>Center</button>
            <button onclick={ resetClick } disabled={ !enabled }>Reset</button>
        </div>
    </div>

    <div class='sliderInnerContainer'>
        <div class='bound left'>
            <span class='times' show={ centered }>× </span>10<sup>{ boundMin }</sup>
        </div>
        <input type='range' ref='slider' oninput={ sliderChange } disabled={ !enabled }
            min={ boundMin } max={ boundMax } step={ step } />
        <div class='bound right'>
            <span class='times' show={ centered }>× </span>10<sup>{ boundMax }</sup>
        </div>
    </div>

    function log10(x) {
        return Math.log(x) * Math.LOG10E;
    }

    setBounds(boundMin, boundMax) {
        this.boundMin = boundMin
        this.boundMax = boundMax
        this.step = (boundMax - boundMin) / 100
    }

    sliderChange(e) {
        let value = Math.pow(10, this.refs.slider.value)
        if (this.centered) {
            value *= this.centerValue
        }
        this.value = value
    }

    refreshValue() {
        let sliderValue = log10(this.value)
        if (this.centered) {
            sliderValue -= log10(this.centerValue)
        }
        this.refs.slider.value = sliderValue
        if (this.value == 0) {
            this.mantissa = 0
            this.exponent = 0
        } else {
            let exponent = Math.floor(log10(this.value))
            let mantissa = this.value / Math.pow(10, exponent)
            this.mantissa = mantissa.toFixed(1)
            this.exponent = exponent
        }
    }

    enable() {
        this.resetSlider()
        this.value = Math.pow(10, this.boundMin)
        this.enabled = true
    }

    disable() {
        this.value = 0
        this.enabled = false
    }

    resetSlider() {
        this.centered = false
        this.setBounds(this.opts.boundMin, this.opts.boundMax)
    }

    centerClick(e) {
        this.centered = true
        this.centerValue = this.value
        this.setBounds(-1, 1)
        this.refs.slider.value = 0
    }

    resetClick(e) {
        this.resetSlider()
    }

    toggle(e) {
        if (this.enabled) {
            this.disable()
        } else {
            this.enable()
        }
    }

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

    <style>
        .sliderInnerContainer > input {
            display: block;
            width: 100%;
            padding: 0;
            margin: 0;
        }

        .sliderInnerContainer {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-top: 0.5em;
        }

        .bound {
            width: 5em;
        }

        .right {
            text-align: right;
        }

        button {
            border-radius: 0.5em;
            border-style: none;
            padding: 0.5em;
            background-color: #0bf;
            color: white;
            font-weight: bold;
            height: 2.5em;
            border: none;
        }

        button:disabled {
            background-color: #ddd;
            border: none;
        }

        slider {
            display: block;
            padding-bottom: 1rem;
            padding-top: 0.5rem;
        }

        slider:not(:last-child) {
            border-bottom: #ddd 1px solid;
        }

        .bold {
            font-weight: bold;
        }
    </style>
</slider>

