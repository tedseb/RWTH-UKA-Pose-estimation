"use strict";
/*
    SHOWROOM APPLICATION
    Node.js-Server
    @comcuoglu
*/
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    Object.defineProperty(o, k2, { enumerable: true, get: function() { return m[k]; } });
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || function (mod) {
    if (mod && mod.__esModule) return mod;
    var result = {};
    if (mod != null) for (var k in mod) if (k !== "default" && Object.prototype.hasOwnProperty.call(mod, k)) __createBinding(result, mod, k);
    __setModuleDefault(result, mod);
    return result;
};
Object.defineProperty(exports, "__esModule", { value: true });
const express = require('express');
const bodyParser = require('body-parser');
const config = require('./config.json');
var url = require('url');
const ws_1 = __importStar(require("ws"));
const args = require('minimist')(process.argv.slice(2));
let skeleton;
// Skeleton Usage
if (args['ai'] === 'spin') {
    skeleton = {
        used: config.metrabs.used,
        labels: config.metrabs.labels,
        connections: config.metrabs.connections
    };
}
else {
    skeleton = {
        used: config.ownpose.used,
        labels: config.ownpose.labels,
        connections: config.ownpose.connections
    };
}
// Parameters and Constants:
const PORT = config.PORT;
const app = express();
const server = app.listen(PORT, () => { console.log("Listening on port " + PORT); });
app.use(bodyParser.json());
app.use(express.static(process.cwd() + '/dist/'));
app.use(bodyParser.urlencoded({
    limit: '50mb',
    extended: true
}));
// WebSocket Handlers
const wss = new ws_1.WebSocketServer({ server });
var websockets = new Array();
wss.on('connection', (ws, req) => {
    let url_parts = url.parse(req.url, true);
    let query = url_parts.query;
    console.log(query);
    websockets.push(ws);
    ws.send("Connection established");
});
// WebServer
app.use(express.static(process.cwd() + '/webtarget'));
app.get('/', (req, res) => {
    res.sendFile(process.cwd() + '/webtarget/index.html');
});
app.get('/api/coordinates/dict', (req, res) => {
    console.log(skeleton);
    res.send(skeleton);
});
// rosnodejs
if (args['rosnodejs'] === 'on') {
    console.log("rosnodejs usage is on");
    const rosnodejs = require('rosnodejs');
    rosnodejs.initNode('/showroom');
    const nh = rosnodejs.nh;
    const std_msgs = rosnodejs.require('std_msgs');
    console.log(std_msgs);
    const int16 = std_msgs.msg.Int16;
    const StringMsg = std_msgs.msg.String;
    rosnodejs.initNode('/showroom');
    const skeleton_coordinates = nh.subscribe('/fused_skelleton', 'backend/Persons', (msg) => {
        let pose = {};
        let bodyParts = msg.persons[0]['bodyParts'];
        skeleton.used.forEach(index => {
            let point = {
                x: bodyParts[index].point.x,
                y: bodyParts[index].point.z,
                z: bodyParts[index].point.y
            };
            pose[skeleton.labels[index]] = point;
        });
        websockets.forEach(ws => {
            if (ws.readyState === ws_1.default.OPEN) {
                ws.send(JSON.stringify(pose));
            }
        });
    }); 
  const showroom_reference_progress = nh.subscribe('showroom_reference_progress', int16, (msg) => {
        //TODO: send message using websocket
        websockets.forEach(ws => {
            if (ws.readyState === ws_1.default.OPEN) {
                let res = {
                    usage: 'reference_progress',
                    data: msg
                };
                ws.send(JSON.stringify(res));
            }
        });
    });
    const showroom_reference_frame = nh.subscribe('showroom_reference_frame', int16, (msg) => {
        //TODO: send message using websocket
        websockets.forEach(ws => {
            if (ws.readyState === ws_1.default.OPEN) {
                let res = {
                    usage: 'reference_frame',
                    data: msg
                };
                ws.send(JSON.stringify(res));
            }
        });
    });
    const user_state = nh.subscribe('/user_state', StringMsg, (msg) => {
        const data = msg['data'];
        websockets.forEach(client => {
            if (client.readyState === ws_1.default.OPEN) {
                client.send(data);
            }
            ;
        });
    });
    const user_correction = nh.subscribe('/user_correction', StringMsg, (msg) => {
        const data = msg['data'];
        websockets.forEach(client => {
            if (client.readyState === ws_1.default.OPEN) {
                client.send(data);
            }
            ;
        });
    });
}
