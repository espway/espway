(function() {
    'use strict'

    let byId = id => document.getElementById(id)

    function main() {
        let ws = new WebSocket('ws://' + location.host + '/ws')
        ws.binaryType = 'arraybuffer'

        ws.addEventListener('message', e => {
            let arr = new Uint8Array(e.data)
            if (arr[0] == 1) {
                byId('ledStatus').textContent = 'off'
            } else {
                byId('ledStatus').textContent = 'on'
            }
        })

        let sendBytes = bytes => ws.send((new Uint8Array(bytes)).buffer)

        byId('btnOn').addEventListener('click', () => sendBytes([0]))
        byId('btnOff').addEventListener('click', () => sendBytes([1]))
    }

    window.addEventListener('load', main)
})()

