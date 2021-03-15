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
            client.send(JSON.stringify({ topic: "user_state", data: data }));
        };
    });
});

// We use this user_correction and deprecate the very first API structure "corrections"
const user_correction = nh.subscribe('/user_correction', StringMsg, (msg) => {
    const data = msg['data'];

    SmartphoneAppClients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(JSON.stringify({ topic: "user_correction", data: data }));
        };
    });
});

// Add new client connections
wss.on('connection', (ws, req) => {
    const location = url.parse(req.url, true);
    SmartphoneAppClients.push(ws);
    console.log("Orhan hat sich verbunden :)");
    ws.on('message', function incoming(message) {
        pub_qr.publish({ data: message })  // Refine this
        console.log('received: %s', message); //json squats
    });
    ws.send(JSON.stringify({ topic: 'start', data: { display_text: 'Übung wird gestartet. Viel Erfolg!', positive_correction: true, id: "StartMessage" } }));
});