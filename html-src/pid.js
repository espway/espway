import riot from 'riot'
import './slider.tag'
import './pid.css'

window.addEventListener('load', () => {
    'use strict'

    const PID_SEND_INTERVAL = 200
    const PID_LIMITS = {
        'angle': {
            pMin: -3, pMax: 2,
            iMin: -3, iMax: 2,
            dMin: -3, dMax: 2
        },
        'angle_high': {
            pMin: -3, pMax: 2,
            iMin: -3, iMax: 2,
            dMin: -3, dMax: 2
        },
        'vel': {
            pMin: -3, pMax: 2,
            iMin: -3, iMax: 2,
            dMin: -4, dMax: 1
        }
    }
    const PID_INDICES = {
        'angle': 0, 'angle_high': 1, 'vel': 2
    }
    const FLOAT_TO_Q16 = (1 << 16)
    const Q16_TO_FLOAT = 1 / FLOAT_TO_Q16

    let ws = new WebSocket('ws://192.168.4.1/ws')
    ws.binaryType = 'arraybuffer'

    let byId = id => document.getElementById(id)
    let pidSelect = byId('pidSelect')

    function mountSliders(limits, values) {
        riot.mount('#pslider', {
            boundMin: limits.pMin,
            boundMax: limits.pMax,
            myValue: values.pValue
        })
        riot.mount('#islider', {
            boundMin: limits.iMin,
            boundMax: limits.iMax,
            myValue: values.iValue
        })
        riot.mount('#dslider', {
            boundMin: limits.dMin,
            boundMax: limits.dMax,
            myValue: values.dValue
        })
    }

    function sendPidValues() {
        let pidIndex = PID_INDICES[pidSelect.value]
        let buf = new ArrayBuffer(14)
        let dv = new DataView(buf)
        dv.setUint8(0, 5)
        dv.setUint8(1, pidIndex)
        let p = byId('pslider').getAttribute('my-value'),
            i = byId('islider').getAttribute('my-value'),
            d = byId('dslider').getAttribute('my-value')
        dv.setInt32(2, p * FLOAT_TO_Q16, true)
        dv.setInt32(6, i * FLOAT_TO_Q16, true)
        dv.setInt32(10, d * FLOAT_TO_Q16, true)
        ws.send(buf)
    }

    function showPidValues(dataView) {
        let pidName = pidSelect.value
        let pidIndex = dataView.getUint8(1)
        if (pidIndex !== PID_INDICES[pidName]) { return }
        let p = dataView.getInt32(2, true),
            i = dataView.getInt32(6, true),
            d = dataView.getInt32(10, true)
        mountSliders(PID_LIMITS[pidName], {
            pValue: p * Q16_TO_FLOAT,
            iValue: i * Q16_TO_FLOAT,
            dValue: d * Q16_TO_FLOAT
        })
    }

    function fetchPidValues() {
        let pidIndex = PID_INDICES[pidSelect.value]
        let buf = new ArrayBuffer(2)
        let dv = new DataView(buf)
        dv.setUint8(0, 7)
        dv.setUint8(1, pidIndex)
        ws.send(buf)
    }

    let pidSendIntervalId = 0

    function wsRecv(e) {
        let buf = e.data
        let dv = new DataView(buf)
        let command = dv.getUint8(0)
        switch (command) {
            case 8:
                if (buf.byteLength != 14) { break }
                showPidValues(dv)
                pidSendIntervalId =
                    setInterval(sendPidValues, PID_SEND_INTERVAL)
                break

            case 13:
                if (buf.byteLength != 1) { break }
                window.alert('Saving settings failed')
                break

            case 14:
                if (buf.byteLength != 1) { break }
                window.alert('Settings saved to flash')
                break

            case 16:
                if (buf.byteLength != 1) { break }
                window.alert('Clearing parameters failed')
                fetchValuesAndStartSending()
                break

            case 19:
                if (buf.byteLength != 1) { break }
                fetchPidValues()
                break
        }
    }

    pidSelect.addEventListener('change', fetchPidValues)
    ws.addEventListener('message', wsRecv)
    ws.addEventListener('open', () => {
        fetchPidValues()
    })

    byId('btnSaveConfig').addEventListener('click', () => {
        let buf = new ArrayBuffer(1)
        let dv = new DataView(buf)
        dv.setUint8(0, 12)
        ws.send(buf)
    })

    byId('btnLoadDefaults').addEventListener('click', () => {
        let buf = new ArrayBuffer(1)
        let dv = new DataView(buf)
        let pidIndex = PID_INDICES[pidSelect.value]
        if (pidSendIntervalId !== 0) {
            clearInterval(pidSendIntervalId)
            pidSendIntervalId = 0
        }
        dv.setUint8(0, 18)
        ws.send(buf)
    })
})

