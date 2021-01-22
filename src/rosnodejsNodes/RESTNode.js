const http = require('http');
const rosnodejs = require('rosnodejs');
const express = require('express');
const bodyParser = require('body-parser');
const WebSocket = require('ws');
const port = 3000;
const app = express();
const server = app.listen(port, () => {console.log("Listening on " + port)});
const hostname = '127.0.0.1';
const StringMsg = rosnodejs.require('std_msgs').msg.String;
let coordinates = "";
let wrongcoordinates = [];
let poses = [];
const wss = new WebSocket.Server({server});
app.use(bodyParser.json());
app.use(express.static(process.cwd() + '/dist/'));
rosnodejs.initNode('/RESTApi')
.then(() => {
  // do stuff
});
const nh = rosnodejs.nh;
const reps = nh.subscribe('/repcounter', StringMsg, (msg) => {
  console.log(msg);
});
const errors = nh.subscribe('/corrections', StringMsg, (msg) => {
  console.log(msg);
})
const wrongc = nh.subscribe('/wrongcoordinates', StringMsg, (msg) => {
  wrongcoordinates = msg.data;
});



const sub = nh.subscribe('/personsJS', 'pose_estimation/Persons', (msg) => {
  console.log(msg);
 /*  let pose = {};
  this.coordinates = msg;
  let bodyParts = msg.persons[0]['bodyParts'];
  let labels = ['nose', 'leftShoulder', 'rightShoulder', 'leftElbow', 'rightElbow', 'leftWrist', 'rightWrist', 'leftHip', 'rightHip', 'leftKnee', 'rightKnee', 'leftAnkle', 'rightAnkle'];
  labels.forEach((label, index) => {
    let point = {};
    point.x = bodyParts[index].point.x;
    point.y = bodyParts[index].point.z;
    point.z = bodyParts[index].point.y;
    pose[label] = point;
  });
  wss.clients.forEach(client => {
    if(client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify(pose));
    }
  }); */
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
app.get('/api/wrongCoordinates', (req, res) => {
  console.log(req.body);
  res.json(wrongcoordinates);
});
wss.on('connection', ws => {
  ws.on('message', function incoming(message) {
    console.log('received: %s', message);
  });
  ws.send(JSON.stringify('connected'));
});