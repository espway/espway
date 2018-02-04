import './index.css'

window.addEventListener('load', () => {
    'use strict'

    let byId = id => document.getElementById(id)
    let ws = new WebSocket('ws://' + window.location.host + '/ws')
    ws.binaryType = 'arraybuffer'
    ws.addEventListener('open', () => {
        window.setInterval(() => {
            sendSteeringCommand(x, y)
        }, 50)

        window.setInterval(() => {
            requestGravity()
        }, 50)
    })

    let tiltBtn = byId('tiltBtn')
    let battery = byId('battery')
    let myCanvas = byId('joystick')
    let dpr = window.devicePixelRatio
    let canvasWidth = myCanvas.clientWidth,
        canvasHeight = myCanvas.clientHeight
    myCanvas.width = canvasWidth * dpr
    myCanvas.height = canvasHeight * dpr
    let ctx = myCanvas.getContext('2d')
    ctx.lineWidth = 10
    ctx.strokeStyle = 'black'
    ctx.fillStyle = 'red'
    ctx.scale(dpr, dpr)

    let mouseIsDown = false
    let hasCurrentTouch = false
    let currentTouchId = null
    let pageX = 0, pageY = 0
    let x = 0, y = 0
    let tiltX = 0, tiltY = 0
    let beta = 0, gamma = 0
    let hasDeviceOrientation = false
    let tiltControl = false
    let hasGravityData = false

    let touchById = (touches, id) => {
        for (let i = 0; i < touches.length; ++i) {
            if (touches[i].identifier === id) {
                return touches[i]
            }
        }
        return null
    }

    ws.addEventListener('message', e => {
        let dview = new DataView(e.data)
        let command = dview.getUint8(0)
        if (command === 3 && e.data.byteLength === 3) {
            let batteryValue = dview.getInt16(1, true) / 1000
            battery.innerText = batteryValue.toFixed(2)
        } else if (command === 2 && e.data.byteLength === 5) {
            hasGravityData = true
            tiltX = dview.getInt16(3, true) / 32768.0
            tiltY = dview.getInt16(1, true) / 32768.0
        }
    })

    let optionsOrUseCapture = true
    try {
        let options = Object.defineProperty({}, "passive", {
            get: function() {
                optionsOrUseCapture = {
                    capture: true,
                    passive: false
                }
            }
        });
        window.addEventListener("test", null, options)
    } catch (err) {}

    myCanvas.addEventListener('mousedown', e => {
        mouseIsDown = true
        pageX = e.pageX
        pageY = e.pageY
    }, optionsOrUseCapture)

    window.addEventListener('mousemove', e => {
        if (mouseIsDown) {
            pageX = e.pageX
            pageY = e.pageY
        }
    }, optionsOrUseCapture)

    window.addEventListener('mouseup', e => {
        mouseIsDown = false
    }, optionsOrUseCapture)

    myCanvas.addEventListener('touchstart', e => {
        currentTouchId = e.changedTouches[0].identifier
        pageX = e.changedTouches[0].pageX
        pageY = e.changedTouches[0].pageY
        hasCurrentTouch = true
    }, optionsOrUseCapture)

    window.addEventListener('touchend', e => {
        if (hasCurrentTouch &&
            touchById(e.changedTouches, currentTouchId) !== null) {
            hasCurrentTouch = false
        }
    }, optionsOrUseCapture)

    window.addEventListener('touchmove', e => {
        let item = touchById(e.changedTouches, currentTouchId)
        if (hasCurrentTouch && item !== null) {
            pageX = item.pageX
            pageY = item.pageY
        }
        e.preventDefault()
    }, optionsOrUseCapture)

    window.addEventListener('deviceorientation', e => {
        if (e.beta !== null && e.gamma !== null) {
            if (!hasDeviceOrientation) {
                hasDeviceOrientation = true
                tiltBtn.style.display = 'inline'
                updateTiltBtnText()
            }
            beta = e.beta
            gamma = e.gamma
        }
    })

    function updateTiltBtnText() {
        tiltBtn.innerText = tiltControl ? '☑ Tilt control' : '☐ Tilt control'
        tiltBtn.style.backgroundColor = tiltControl ? '#0bf' : '#ddd'
    }

    tiltBtn.addEventListener('click', () => {
        tiltControl = !tiltControl
        updateTiltBtnText()
    })

    function updateXY() {
        if (tiltControl) {
            x = gamma / 20
            y = -beta / 20
        } else {
            if (!hasCurrentTouch && !mouseIsDown) {
                x = 0
                y = 0
            } else {
                x = pageX - myCanvas.offsetLeft - canvasWidth/2
                y = (canvasHeight/2 - (pageY - myCanvas.offsetTop))
                x /= canvasWidth / 2
                y /= canvasHeight / 2
            }
        }
        x = Math.min(Math.max(x, -1), 1)
        y = Math.min(Math.max(y, -1), 1)
    }

    function sendSteeringCommand(turning, speed) {
        let dview = new DataView(new ArrayBuffer(3))
        dview.setUint8(0, 0)
        dview.setInt8(1, turning * 127)
        dview.setInt8(2, speed * 127)
        ws.send(dview.buffer)
    }

    function draw() {
        updateXY()

        ctx.lineWidth = 10
        ctx.clearRect(0, 0, canvasWidth, canvasHeight)
        ctx.beginPath()
        ctx.moveTo(canvasWidth / 2, canvasHeight / 2)
        ctx.lineTo(canvasX, canvasY)
        ctx.stroke()

        let canvasX = (tiltX + 1) / 2 * canvasWidth,
            canvasY = (-tiltY + 1) / 2 * canvasHeight

        if (hasGravityData) {
          ctx.fillStyle = '#aaa'
          ctx.beginPath()
          ctx.arc(canvasX, canvasY, 15, 0, 2*Math.PI)
          ctx.fill()
        }

        canvasX = (x + 1) / 2 * canvasWidth
        canvasY = (-y + 1) / 2 * canvasHeight
        ctx.fillStyle = 'red'
        ctx.lineWidth = 2
        ctx.beginPath()
        ctx.arc(canvasX, canvasY, 15, 0, 2*Math.PI)
        ctx.fill()
        ctx.stroke()

        window.requestAnimationFrame(draw)
    }
    window.requestAnimationFrame(draw)

    function requestGravity() {
        ws.send((new Uint8Array([1])).buffer)
    }
})

