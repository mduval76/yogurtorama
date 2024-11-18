const ws = new WebSocket('ws://' + location.hostname + ':81');
let isOn = false;

function onPanic() {
    isOn = !isOn

    if (isOn) {
        ws.send("OFF");
        document.querySelector('.on_off').innerText = "OFF";
    }
    else {
        ws.send("ON");
        document.querySelector('.on_off').innerText = "ON";
    }
}


ws.onopen = function() {
    console.log('Websocket connection established...');
};

ws.onclose = function() {
    console.log('Websocket connection closed...')
}

ws.onerror = function(error) {
    console.error('Websocket error: ', error)
}

ws.onmessage = function(event) {
    console.log("Raw message data: ", event.data);
    try {
        const data = JSON.parse(event.data);
        const currentTemp = data.temperature;
        const heatPercent = data.heat_percent;
        const minTemp2min = data.min_temp2;
        const maxTemp2min = data.max_temp2;
        const minTemp5min = data.min_temp5;
        const maxTemp5min = data.max_temp5;
        const withinRangeTime = data.time_in_range;

        document.querySelector('.temperature').innerText = currentTemp;
        document.querySelector('.heat_percent').innerText = heatPercent;
        document.querySelector('.min_temp2').innerText = minTemp2min;
        document.querySelector('.max_temp2').innerText = maxTemp2min;
        document.querySelector('.min_temp5').innerText = minTemp5min;
        document.querySelector('.max_temp5').innerText = maxTemp5min;
        document.querySelector('.within_range_time').innerText = withinRangeTime;
    } 
    catch (error) {
        console.error("Error parsing WebSocket message data: ", error);
    }
};
