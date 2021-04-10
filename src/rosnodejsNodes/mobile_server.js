/*
This file describes a RESTful API that for communication between the ROS ecosystem and the "outside world", namely primarely our app.
*/

/*
The following structure reflects the REST API between the Comparing System and the Smtartphone App 
as specified under https://app.getguru.com/card/iGK7zMAT/Tech-Spec-REST-API-ComparingSystem-Smartphone-App
For questions, refer to Artur
This is not updated regularely and should only give you a rough idea

user state = {
  user_id: int16 (später UUID statt int16? USER_ID aus URI Pfad)
  repetitions: int16
  seconds_since_last_exercise_start: int16
  milliseconds_since_last_repetition: int32
  repetition_score: int8
  exercise_score: int8
  user_position:  {
    x: float32
    y: float32
   y: float32
  }
}

user correction = {
  user_id: int16 (später UUID statt int16? USER_ID aus URI Pfad)
  repetition: int16
  positive_correction: bool
  display_text: string
}
*/

// Imports:
const rosnodejs = require('rosnodejs');
const express = require('express');
const https = require('https');
const bodyParser = require('body-parser');
const WebSocket = require('ws');
const StringMsg = rosnodejs.require('std_msgs').msg.String;
const pose_estimation_messages = rosnodejs.require("backend");
const comparing_system_messages = rosnodejs.require("comparing_system");
const url = require('url');
const config = require('./config');

// Parameters and Constants:
const PORT = config.MOBILE_SERVER_PORT;
const app = express();
const server = app.listen(PORT, () => { console.log("Listening on port " + PORT) });
const wss = new WebSocket.Server({ server });

let SmartphoneAppClients = [];

rosnodejs.initNode('/mobile_server')

// TODO: Shutdown gracefully to app: rosnodejs.on('shutdown', function() {  });
const nh = rosnodejs.nh;

// We use this to advertise the Exercise name, read with the current QR Code
const pub_qr = nh.advertise('/qr_exercise', StringMsg);

// We use this user_state and deprecate the very first API structure "repetition"
const user_state = nh.subscribe('/user_state', StringMsg, (msg) => {
    const data = msg['data'];
    SmartphoneAppClients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(data);
        };
    });
});

// We use this user_correction and deprecate the very first API structure "corrections"
const user_correction = nh.subscribe('/user_correction', StringMsg, (msg) => {
    const data = msg['data'];
    SmartphoneAppClients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(data);
        };
    });
});

// Add new client connections
wss.on('connection', (ws, req) => {
    const location = url.parse(req.url, true);
    SmartphoneAppClients.push(ws);
    console.log("Orhan hat sich verbunden :)");
    ws.on('message', function incoming(message) {
        const qr = JSON.parse(message);
        const params = {
            stationID: qr['stationID'],
            isActive: qr['isActive'],
            exercise: qr['exercise']
        }
        pub_qr.publish({data: YAML.stringify(params)})  // Refine this
    });
    ws.send(JSON.stringify({ topic: 'start', data: { display_text: 'Übung wird gestartet. Viel Erfolg!', positive_correction: true, id: "StartMessage" } }));
});