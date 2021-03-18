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
const YAML = require('yaml');
var MongoClient = require('mongodb').MongoClient;


// Parameters and Constants:
const PORT = config.PORT;
const ownpose_labels = config.ownpose_labels;
const ownpose_used = config.ownpose_used;
const ownpose = config.ownpose;

// Web App Code:
const app = express();
const server = app.listen(PORT, () => { console.log("Listening on port " + PORT) });
app.use(bodyParser.json({ limit: '50mb' }));
app.use(bodyParser.urlencoded({
  limit: '50mb',
  extended: true
}));
app.use(express.static(process.cwd() + '/dist/'));
const wss = new WebSocket.Server({ server });
let SmartphoneAppClients = [];
let coordinateClients = [];

rosnodejs.initNode('/hmi')

// TODO: Shutdown gracefully to app: rosnodejs.on('shutdown', function() {  });
const nh = rosnodejs.nh;

const pubex = nh.advertise('/exercises', StringMsg);

MongoClient.connect(config.db_uri, { useUnifiedTopology: true }, (err, client) => {
  if (err) throw err;

  //get trainerai DB and exercises collection
  const db = client.db("trainerai");
  const exercises = db.collection("exercises");
  const recordings = db.collection("recordings");
  const hmiExercises = db.collection("hmiExercises");

  nh.subscribe('/qr_exercise', StringMsg, async (msg) => {
    exercises.findOne({ name: msg['data'] }, (err, result) => {
      if (err) throw err;
      if (result) {
        const stringified = YAML.stringify(result);
        nh.setParam('exercise', stringified);
        pubex.publish({ data: 'exercise' });
      } else {
        console.error(`No such exercise  ${msg['data']}`)
      }
    });
  });

  app.post('/expert/exercise/recordings', (req, res) => {
    console.log(req.body);
    res.status(200).send();
  });

  app.post('/api/expert/exercise/save', (req, res) => {
    if (req && req.body['exercise']) {
      const exercise = req.body['exercise'];
      console.log(exercise.name);
      exercises.insertOne(exercise);
      res.status(200).send();
    } else {
      res.status(500).send('Malformed Exercise');
    }
  });

  app.post('/api/expert/exercises/stages/save', (req, res) => {
    const stages = req.body;
    const toSave = {time: Date.now(),
    stages: stages};
    hmiExercises.insertOne(toSave);
    res.status(200).send();
  });

  //req.body == recording raw
  app.post('/api/expert/recording/save', (req, res) => {
    if (req) {
      const recording = req.body;
      const recObj = {
        recording: recording
      };
      recordings.insertOne(recObj);
      res.status(200).send();
    } else {
      res.status(500).send('Malformed Recording');
    }
  });

});

//datastructures for expertsystem
var isRecording = false;
var recordedPoses = [];
var recordStart = Date.now();

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

  if (isRecording) {
    recordedPoses.push([Date.now() - recordStart, pose]);
  }
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

app.post('/api/exercise/recording/start', (req, res) => {
  console.log('recording requested');
  if (isRecording) {
    console.error('a recording is already in process');
    res.status(500).send();
  } else {
    isRecording = true;
    recordStart = Date.now();
    res.status(200).send();
  }
});

app.post('/api/exercise/recording/stop', (req, res) => {
  console.log('saving record requested');
  isRecording = false;
  res.json(recordedPoses);
  recordedPoses = [];
});

// Add new client connections
wss.on('connection', (ws, req) => {
  const location = url.parse(req.url, true);
  coordinateClients.push(ws);
});