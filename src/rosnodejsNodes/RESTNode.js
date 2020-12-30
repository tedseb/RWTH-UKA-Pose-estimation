/* const http = require('http');
const rosnodejs = require('rosnodejs');

const hostname = '127.0.0.1';
const port = 3000;


rosnodejs.initNode('/DasTorZurUnterwelt')
.then(() => {
  // do stuff
});

const nh = rosnodejs.nh;

const server = http.createServer((req, res) => {
  res.statusCode = 200;
  res.setHeader('Content-Type', 'text/plain');
  //res.end('Hello World');
  //console.log('Got msg on chatter bevor');
  const sub = nh.subscribe('/personsJS', 'pose_estimation/Persons', (msg) => {
    //console.log('Got msg on chatter: %j', msg);
    res.end(JSON.stringify(msg));

  });
});

server.listen(port, hostname, () => {
  console.log(`Server running at http://${hostname}:${port}/`);
});
 */

const http = require('http');
const rosnodejs = require('rosnodejs');
const express = require('express');
const bodyParser = require('body-parser');
const WebSocket = require('ws');
const port = 3000;
const app = express();
const server = app.listen(port, () => {console.log("Listening on " + port)});
const hostname = '127.0.0.1';
let coordinates = "";
let poses = [];
const wss = new WebSocket.Server({server});
app.use(bodyParser.json());
app.use(express.static(process.cwd() + '/dist/'));
rosnodejs.initNode('/RESTApi')
.then(() => {
  // do stuff
});
const nh = rosnodejs.nh;
const sub = nh.subscribe('/personsJS', 'pose_estimation/Persons', (msg) => {
  
  let pose = {};
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
wss.on('connection', ws => {
  ws.on('message', function incoming(message) {
    console.log('received: %s', message);
  });
  ws.send(JSON.stringify('connected'));
});