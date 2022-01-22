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
let isRos = args['rosnodejs'] === 'on';
let ai = args['spin'];
let isIsolated = args['isolated'] === 'on';
let skeleton;
// Skeleton Usage
if (args['ai'] === 'spin') {
    skeleton = {
        used: config.metrabs.used,
        labels: config.metrabs.labels,
        connections: config.metrabs.connections
    };
    console.log(skeleton);
}
else {
    skeleton = {
        used: config.ownpose.used,
        labels: config.ownpose.labels,
        connections: config.ownpose.connections
    };
}
// local usage
let websocketCache = [];
let start = Date.now();
let getDelta = () => { return Date.now() - start; };
setInterval(() => {
    websocketCache = [];
}, 10000);
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
    if (isRos) {
    }
});
// WebServer
app.use(express.static(process.cwd() + '/webtarget'));
app.get('/', (req, res) => {
    res.sendFile(process.cwd() + '/webtarget/index.html');
});
app.get('/api/connections/dict', (req, res) => {
    console.log("WAS IST DAS HIER");
    console.log(skeleton, "SKELETON HIER");
    res.send(skeleton);
});
app.get('/api/websocket/cache', (req, res) => {
    res.send(JSON.stringify(websocketCache));
});
if (args['isolated'] === 'on') {
    console.log("running in isolated environment, reloading websocketCache");
    const cached = require('./websocket.json');
    let deployCacheOnWebSocket = () => {
        cached.forEach((delta, index) => {
            let todo = () => {
                websockets.forEach(ws => ws.send(JSON.stringify(JSON.parse(delta['message']))));
                if (index === cached.length - 1) {
                    deployCacheOnWebSocket();
                }
                ;
            };
            setTimeout(todo, delta['date']);
        });
    };
    deployCacheOnWebSocket();
}
let publishers = [];
let rosTypes = [];
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
    const StationUsage = rosnodejs.require("backend").msg.StationUsage;
    rosTypes.push(StationUsage);
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
        websocketCache.push({ 'date': getDelta(), 'message': JSON.stringify(pose) });
        websockets.forEach(ws => {
            ws.send(JSON.stringify(pose));
        });
    });
    const showroom_reference_progress = nh.subscribe('showroom_reference_progress', int16, (msg) => {
        websockets.forEach(ws => {
            if (ws.readyState === ws_1.default.OPEN) {
                let res = {
                    usage: 'reference_progress',
                    data: msg
                };
                websocketCache.push({ 'date': getDelta(), 'message': JSON.stringify(res) });
                ws.send(JSON.stringify(res));
            }
        });
    });
    const showroom_reference_frame = nh.subscribe('showroom_video_reference', StringMsg, (msg) => {
        let obj = JSON.parse(msg.data).data;
        console.log(obj);
        let res = {
            usage: 'reference_frame',
            data: obj.video_frame_idx
        };
        if (obj.video_file_name.includes("squats_1") || obj.video_file_name.includes("deadlift_1") || obj.video_file_name.includes("military_press_2"))
            return;
        console.log(res);
        websockets.forEach(ws => {
            if (ws.readyState === ws_1.default.OPEN) {
                let obj = JSON.parse(msg.data).data;
                let res = {
                    usage: 'reference_frame',
                    data: obj.video_frame_idx
                };
                console.log(res);
                websocketCache.push({ 'date': getDelta(), 'message': JSON.stringify(res) });
                ws.send(JSON.stringify(res));
            }
        });
    });
    const user_state = nh.subscribe('/user_state', StringMsg, (msg) => {
        const data = msg['data'];
        const parsed = JSON.parse(data).data;
        console.log("dis stirng", parsed);
        const reps = parsed.repetitions;
        const score = parsed.repetition_score;
        const exercise_score = parsed.exercise_score;
        websockets.forEach(client => {
            if (client.readyState === ws_1.default.OPEN) {
                let res = {
                    usage: 'user_state',
                    data: {
                        reps: reps,
                        score: score,
                        exercise_score: exercise_score
                    }
                };
                client.send(JSON.stringify(res));
            }
            ;
        });
    });
    const user_correction = nh.subscribe('/user_correction', StringMsg, (msg) => {
        const data = msg['data'];
        websockets.forEach(client => {
            if (client.readyState === ws_1.default.OPEN) {
                //client.send(data);
            }
            ;
        });
    });
    const signal_show = nh.subscribe('/signal/person', StringMsg, (msg) => {
        websockets.forEach(client => {
            if (client.readyState === ws_1.default.OPEN) {
                let signal = msg['data'];
                let res = {
                    usage: 'signal_person',
                    data: signal
                };
                websocketCache.push({ 'date': getDelta(), 'message': JSON.stringify(res) });
                client.send(JSON.stringify(res));
            }
        });
    });
}
else {
    console.log("rosnodejs is off");
}
