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
const express = require('express');  // Express NodeJ Web App framework
const bodyParser = require('body-parser');
const WebSocket = require('ws');
const StringMsg = rosnodejs.require('std_msgs').msg.String;
const pose_estimation_messages = rosnodejs.require("pose_estimation");
const url = require('url');

// Parameters and Constants:

const PORT = 3000;
const ownpose_used = [0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 27, 28, 37, 39, 41, 42, 43];

const ownpose_labels = ['OP_Nose', 'OP_Neck', 'OP_R_Shoulder', 'OP_R_Elbow', 'OP_R_Wrist', 'OP_L_Shoulder', 'OP_L_Elbow', 'OP_L_Wrist', 'OP_Middle_Hip', 'OP_R_Hip', 'OP_R_Knee', 'OP_R_Ankle', 'OP_L_Hip', 'OP_L_Knee', 'OP_L_Ankle', 'OP_R_Eye', 'OP_L_Eye', 'OP_R_Ear', 'OP_L_Ear', 'OP_L_Big_Toe', 'OP_L_Small_Toe', 'OP_L_Heel', 'OP_R_Big_Toe', 'OP_R_Small_Toe', 'OP_R_Heel', 'R_Ankle', 'R_Knee', 'R_Hip', 'L_Hip', 'L_Knee', 'L_Ankle', 'R_Wrist', 'R_Elbow', 'R_Shoulder', 'L_Shoulder', 'L_Elbow', 'L_Wrist', 'Neck_LSP', 'Top_of_Head_LSP', 'Pelvis_MPII', 'Thorax_MPII', 'Spine_HM', 'Jaw_HM', 'Head_HM', 'Nose', 'L_Eye', 'R_Eye', 'L_Ear', 'R_Ear']

const ownpose = [
  [2, 3], [3, 4], [5, 6], [6, 7], [27, 9], [9, 12], [27, 28], [27, 10], [10, 11], [12, 13], [9, 10], [28, 12], [28, 13], [13, 14], [14, 21], [21, 20], [21, 19], [20, 19], [11, 24], [24, 22], [22, 23], [23, 24], [5, 28], [2, 27], [5, 2], [42, 17], [42, 18], [42, 0], [0, 15], [0, 16], [15, 16], [17, 43], [18, 43], [1, 37], [37, 43], [41, 37], [41, 39]
];

// Web App Code:

const app = express();
const server = app.listen(PORT, () => { console.log("Listening on port " + PORT) });
app.use(bodyParser.json());
app.use(express.static(process.cwd() + '/dist/'));

const wss = new WebSocket.Server({ server });

let SmartphoneAppClients = [];
let coordinateClients = [];


rosnodejs.initNode('/RESTApi')

// TODO: Shutdown gracefully to app: rosnodejs.on('shutdown', function() {  });

const nh = rosnodejs.nh;

// We use this to advertise the Exercise name, read with the current QR Code
const pub_qr = nh.advertise('/qr_exercise', StringMsg);

// We use this user_state and deprecate the very first API structure "repetition"
const user_state = nh.subscribe('/user_state', pose_estimation_messages.msg.user_state, (msg) => {
  SmartphoneAppClients.forEach(client => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify(msg));
    };
  });
});

// We use this user_correction and deprecate the very first API structure "corrections"
const user_correction = nh.subscribe('/user_correction', pose_estimation_messages.msg.user_correction, (msg) => {
  SmartphoneAppClients.forEach(client => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify(msg));
    };
  });
});


// Tamers Web-App Code
const wrong_coordinates = nh.subscribe('/wrongcoordinates', StringMsg, (msg) => {
  wrongcoordinates = msg.data;
});


const fused_skelleton = nh.subscribe('/fused_skelleton', 'backend/Persons', (msg) => {
  let pose = {};
  this.coordinates = msg;
  let bodyParts = msg.persons[0]['bodyParts'];
  let labels = ['nose', 'leftShoulder', 'rightShoulder', 'leftElbow', 'rightElbow', 'leftWrist', 'rightWrist', 'leftHip', 'rightHip', 'leftKnee', 'rightKnee', 'leftAnkle', 'rightAnkle'];

  ownpose_used.forEach(index => {
    let point = {};
    point.x = bodyParts[index].point.x;
    point.y = bodyParts[index].point.z;
    point.z = bodyParts[index].point.y;
    pose[ownpose_labels[index]] = point;
  });
  coordinateClients.forEach(client => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify(pose));
    }
  });
});

app.get('/', (req, res) => {
  res.sendFile(process.cwd() + './dist/index.html');
});

app.get('/api/coordinates', (req, res) => {
  res.json(this.coordinates);
});

app.post('/api/pose', (req, res) => {
  this.poses.push(req.body);
  res.json('saved');
});

app.get('/api/poses', (req, res) => {
  console.log(req.body);
  res.json(this.poses[0]);
});

app.get('/api/connections', (req, res) => {
  console.log(req.body);
  let dict = {};
  ownpose.forEach(element => {
    if (dict[ownpose_labels[element[0]]]) {
      dict[ownpose_labels[element[0]]].push(ownpose_labels[element[1]]);
    }
    else { dict[ownpose_labels[element[0]]] = [ownpose_labels[element[1]]] }
  });
  console.log(dict);
  res.json(dict);
});

app.get('/api/wrongCoordinates', (req, res) => {
  console.log(req.body);
  res.json(wrongcoordinates);
});




// Add new client connections
wss.on('connection', (ws, req) => {
  const my_url = URL(req.url); // TODO: @Tamer Add authorization and so on...
  const my_search_params = URLSearchParams(req.url)

  if((my_url.pathname == "/api/smartphone_app") || (my_url.path.includes('corrections'))){ // Deprecate "corrections" paths
    // Arturs connection
    SmartphoneAppClients.push(ws);
    ws.on('message', function incoming(message) {
      pub_qr.publish({ data: message })  // Refine this
      console.log('received: %s', message); //json squats
    });
  } else {
    // Tamers connection
    coordinateClients.push(ws);
  }

  ws.send(JSON.stringify({action: 'ubung', info: 'Übung wird gestartet. Viel Erfolg!', bool: true, id: "StartMessage"}));
});