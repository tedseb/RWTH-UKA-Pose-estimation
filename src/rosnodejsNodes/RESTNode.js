const rosnodejs = require('rosnodejs');
const express = require('express');
const bodyParser = require('body-parser');
const WebSocket = require('ws');
const port = 3000;
const app = express();
const server = app.listen(port, () => { console.log("Listening on " + port) });
//const hostname = '127.0.0.1';
const StringMsg = rosnodejs.require('std_msgs').msg.String;
//let coordinates = "";
let correctionClients = [];
let coordinateClients = [];
const url = require('url');
//let poses = [];
const wss = new WebSocket.Server({ server });
app.use(bodyParser.json());
app.use(express.static(process.cwd() + '/dist/'));
rosnodejs.initNode('/RESTApi')
  .then(() => {
    // do stuff
  });



const nh = rosnodejs.nh;
const reps = nh.subscribe('/repcounter', StringMsg, (msg) => {
  correctionClients.forEach(client => {
    if (client.readyState === WebSocket.OPEN) {
      let answer = {
        action: 'ubung',
        id: 1,
        info: msg['data'],
        bool: true
      };
      client.send(JSON.stringify(answer));
    }
  });
});
const errors = nh.subscribe('/corrections', StringMsg, (msg) => {
  correctionClients.forEach(client => {
    if (client.readyState === WebSocket.OPEN) {
      let answer = {
        action: 'ubung',
        id: 1,
        info: msg['data'],
        bool: false
      };
      client.send(JSON.stringify(answer));
    }
  });
})
const wrongc = nh.subscribe('/wrongcoordinates', StringMsg, (msg) => {
  wrongcoordinates = msg.data;
});

const ownpose_used = [0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 27, 28, 37, 39, 41, 42, 43];

const ownpose_labels = ['OP_Nose', 'OP_Neck', 'OP_R_Shoulder', 'OP_R_Elbow', 'OP_R_Wrist', 'OP_L_Shoulder', 'OP_L_Elbow', 'OP_L_Wrist', 'OP_Middle_Hip', 'OP_R_Hip', 'OP_R_Knee', 'OP_R_Ankle', 'OP_L_Hip', 'OP_L_Knee', 'OP_L_Ankle', 'OP_R_Eye', 'OP_L_Eye', 'OP_R_Ear', 'OP_L_Ear', 'OP_L_Big_Toe', 'OP_L_Small_Toe', 'OP_L_Heel', 'OP_R_Big_Toe', 'OP_R_Small_Toe', 'OP_R_Heel', 'R_Ankle', 'R_Knee', 'R_Hip', 'L_Hip', 'L_Knee', 'L_Ankle', 'R_Wrist', 'R_Elbow', 'R_Shoulder', 'L_Shoulder', 'L_Elbow', 'L_Wrist', 'Neck_LSP', 'Top_of_Head_LSP', 'Pelvis_MPII', 'Thorax_MPII', 'Spine_HM', 'Jaw_HM', 'Head_HM', 'Nose', 'L_Eye', 'R_Eye', 'L_Ear', 'R_Ear']

const ownpose = [
  [2, 3], [3, 4], [5, 6], [6, 7], [27, 9], [9, 12], [27, 28], [27, 10], [10, 11], [12, 13], [9, 10], [28, 12], [28, 13], [13, 14], [14, 21], [21, 20], [21, 19], [20, 19], [11, 24], [24, 22], [22, 23], [23, 24], [5, 28], [2, 27], [5, 2], [42, 17], [42, 18], [42, 0], [0, 15], [0, 16], [15, 16], [17, 43], [18, 43], [1, 37], [37, 43], [41, 37], [41, 39]
];


const sub = nh.subscribe('/personsJS', 'pose_estimation/Persons', (msg) => {
  let pose = {};
  this.coordinates = msg;
  let bodyParts = msg.persons[0]['bodyParts'];
  let labels = ['nose', 'leftShoulder', 'rightShoulder', 'leftElbow', 'rightElbow', 'leftWrist', 'rightWrist', 'leftHip', 'rightHip', 'leftKnee', 'rightKnee', 'leftAnkle', 'rightAnkle'];
  /* labels.forEach((label, index) => {
    let point = {};
    point.x = bodyParts[index].point.x;
    point.y = bodyParts[index].point.z;
    point.z = bodyParts[index].point.y;
    pose[label] = point;
  }); */
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
wss.on('connection', (ws, req) => {
  const location = url.parse(req.url, true);
  if(location.path.includes('corrections')){
    correctionClients.push(ws);
  } else {
    coordinateClients.push(ws);
  }
  ws.on('message', function incoming(message) {
    console.log('received: %s', message);
  });

  ws.send(JSON.stringify({action: 'ubung', info: 'Übung wird gestartet. Viel Erfolg!', bool: true, id: "StartMessage"}));
});