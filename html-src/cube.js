import './cube.css'

(function() {
    'use strict'

    let byId = id => document.getElementById(id)
    let cube = byId('cube')
    let scale = 1 / 32768.0
    let gTransform = ''
    let ws = new WebSocket('ws://192.168.4.1/ws')
    ws.binaryType = 'arraybuffer'
    let sendBytes = bytes => ws.send((new Uint8Array(bytes)).buffer)
    ws.addEventListener('message', update)
    ws.addEventListener('open', raf)

    function update(e) {
        let dview = new DataView(e.data)
        let command = dview.getUint8(0)
        if (command === 2 && e.data.byteLength === 9) {
            let q0 = scale * dview.getInt16(1, true),
                q1 = -scale * dview.getInt16(5, true),
                q2 = scale * dview.getInt16(7, true),
                q3 = -scale * dview.getInt16(3, true)
            let a1 = q0*q0 + q1*q1 - q2*q2 - q3*q3,
                a2 = 2 * (q1*q2 - q0*q3),
                a3 = 2 * (q1*q3 + q0*q2),
                b1 = 2 * (q1*q2 + q0*q3),
                b2 = q0*q0 - q1*q1 + q2*q2 - q3*q3,
                b3 = 2 * (q2*q3 - q0*q1),
                c1 = 2 * (q1*q3 - q0*q2),
                c2 = 2 * (q2*q3 + q0*q1),
                c3 = q0*q0 - q1*q1 - q2*q2 + q3*q3
            gTransform = 'matrix3d(' +
                a1 + ',' + b1 + ',' + c1 + ',0,' +
                a2 + ',' + b2 + ',' + c2 + ',0,' +
                a3 + ',' + b3 + ',' + c3 + ',0,' +
                '0,0,0,1)'
        }

        requestAnimationFrame(raf)
    }

    function raf() {
        cube.style.transform = gTransform
        // request new data
        sendBytes([1])
    }
})()

