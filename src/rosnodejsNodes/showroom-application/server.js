"use strict";
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
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
const ws_1 = __importStar(require("ws"));
const body_parser_1 = __importDefault(require("body-parser"));
const express = require('express');
const args = require('minimist')(process.argv.slice(2));
const config = require('./config.json');
let skeleton;
if (args.ai === 'metrabs') {
    skeleton = {
        used: config.metrabs.used,
        labels: config.metrabs.labels,
        connections: config.metrabs.connections,
    };
}
else {
    skeleton = {
        used: config.ownpose.used,
        labels: config.ownpose.labels,
        connections: config.ownpose.connections,
    };
}
const app = express();
const server = app.listen(config.PORT, () => {
    const startMessage = String.raw `
                                                
    )                                      
    ( /(       (  (    (                 )    
  (   )\())  (   )\))(   )(    (    (     (     
  )\ ((_)\   )\ ((_)()\ (()\   )\   )\    )\  ' 
  ((_)| |(_) ((_)_(()((_) ((_) ((_) ((_) _((_))  
  (_-<| ' \ / _ \\ V  V /| '_|/ _ \/ _ \| '  \() 
  /__/|_||_|\___/ \_/\_/ |_|  \___/\___/|_|_|_|  
                                              
  `;
    console.log(startMessage);
    console.log(`Listening on http://localhost:${config.PORT}`);
});
app.use(body_parser_1.default.json());
app.use(express.static(`${process.cwd()}/webtarget/`));
app.use(body_parser_1.default.urlencoded({
    limit: '50mb',
    extended: true,
}));
const wss = new ws_1.WebSocketServer({ server });
const websockets = [];
wss.on('connection', (ws) => {
    websockets.push(ws);
});
app.get('/', (req, res) => {
    res.sendFile(`${process.cwd()}/webtarget/index.html`);
});
app.get('/api/connections/dict', (req, res) => {
    res.send(skeleton);
});
app.get('/api/mock/personsignal', (req, res) => {
    websockets.forEach((client) => {
        if (client.readyState === ws_1.default.OPEN) {
            const signal = 'mock';
            const msg = {
                usage: 'signal_person',
                data: signal,
            };
            client.send(JSON.stringify(msg));
        }
    });
    res.status(200);
});
// rosnodejs
if (args.rosnodejs === 'on') {
    console.log('rosnodejs usage is on');
    // eslint-disable-next-line global-require
    const rosnodejs = require('rosnodejs');
    rosnodejs.initNode('/showroom');
    const { nh } = rosnodejs;
    const stdMsgs = rosnodejs.require('std_msgs');
    // tscconsole.log(std_msgs);
    const int16 = stdMsgs.msg.Int16;
    const StringMsg = stdMsgs.msg.String;
    const StationUsage = rosnodejs.require("backend").msg.StationUsage;
    nh.subscribe('/fused_skelleton', 'backend/Persons', (msg) => {
        const pose = {};
        const { bodyParts } = msg.persons[0];
        skeleton.used.forEach((index) => {
            const point = {
                x: bodyParts[index].point.x,
                y: bodyParts[index].point.z,
                z: bodyParts[index].point.y,
            };
            pose[skeleton.labels[index]] = point;
        });
        // websocketCache.push({'date': getDelta(), 'message': JSON.stringify(pose)});
        websockets.forEach((ws) => {
            ws.send(JSON.stringify(pose));
        });
    });
    nh.subscribe('showroom_reference_progress', int16, (msg) => {
        websockets.forEach((ws) => {
            if (ws.readyState === ws_1.default.OPEN) {
                const res = {
                    usage: 'reference_progress',
                    data: msg,
                };
                ws.send(JSON.stringify(res));
            }
        });
    });
    nh.subscribe('showroom_video_reference', StringMsg, (msg) => {
        const obj = JSON.parse(msg.data).data;
        const res = {
            usage: 'reference_frame',
            data: obj.video_frame_idx,
        };
        if (obj.video_file_name.includes('squats_1') || obj.video_file_name.includes('deadlift_1') || obj.video_file_name.includes('military_press_2'))
            return;
        websockets.forEach((ws) => {
            if (ws.readyState === ws_1.default.OPEN) {
                // websocketCache.push({'date': getDelta(), 'message': JSON.stringify(res)});
                ws.send(JSON.stringify(res));
            }
        });
    });
    nh.subscribe('/user_state', StringMsg, (msg) => {
        const { data } = msg;
        const parsed = JSON.parse(data).data;
        const reps = parsed.repetitions;
        const score = parsed.repetition_score;
        const exerciseScore = parsed.exercise_score;
        websockets.forEach((client) => {
            if (client.readyState === ws_1.default.OPEN) {
                const res = {
                    usage: 'user_state',
                    data: {
                        reps,
                        score,
                        exerciseScore,
                    },
                };
                console.log(res);
                client.send(JSON.stringify(res));
            }
        });
    });
    nh.subscribe('/station_usage', StationUsage, (msg) => {
        console.log(msg);
        websockets.forEach((client) => {
            if (client.readyState === ws_1.default.OPEN) {
                const res = {
                    usage: 'station_usage',
                    data: {
                        'stationID': msg.stationID,
                        'isActive': msg.isActive,
                        'exerciseNumber': msg.exerciseName
                    },
                };
                client.send(JSON.stringify(res));
            }
        });
    });
    /* nh.subscribe('/user_correction', StringMsg, (msg: any) => {
      // const { data } = msg;
      websockets.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
          // client.send(data);
        }
      });
    }); */
    nh.subscribe('/signal/person', StringMsg, (msg) => {
        websockets.forEach((client) => {
            if (client.readyState === ws_1.default.OPEN) {
                const signal = msg.data;
                const res = {
                    usage: 'signal_person',
                    data: signal,
                };
                client.send(JSON.stringify(res));
            }
        });
    });
}
else {
    console.log('rosnodejs is off');
}
