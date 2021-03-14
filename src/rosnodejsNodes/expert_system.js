var MongoClient = require('mongodb').MongoClient;
const express = require('express');
const rosnodejs = require('rosnodejs');
const StringMsg = rosnodejs.require('std_msgs').msg.String;
const YAML = require('yaml');
const config = require('./config');


rosnodejs.initNode('/expert_system')
  .then(() => { });

const nh = rosnodejs.nh;
const pubex = nh.advertise('/exercises', StringMsg);

//MongoDB path
const uri = config.db_uri;

//communication between REST Node and Expert
const express = require('express');
const app = express();
const PORT = config.PORT_exp;
const server = app.listen(PORT, () => { console.log("Listening on port " + PORT) });
app.use(express.json);

MongoClient.connect(uri, { useUnifiedTopology: true }, (err, client) => {
  if (err) throw err;

  //get trainerai DB and exercises collection
  const db = client.db("trainerai");
  const exercises = db.collection("exercises");

  nh.subscribe('/qr_exercise', StringMsg, async (msg) => {
    exercises.findOne({ name: msg['data'] }, (err, result) => {
      if (err) throw err;
      if(result) {
        const stringified = YAML.stringify(result);
        nh.setParam('exercise', stringified);
        pubex.publish({ data: 'exercise' });
      } else {
        console.error(`No such exercise  ${msg['data']}`)
      }
    });
  });

  app.post('/expert/exercises/recordings', (req, res) => {
      console.log(req);
      const recordings = db.collection('recordings');
      recordings.insertOne(req.body)
      res.status(200);
  });

});
