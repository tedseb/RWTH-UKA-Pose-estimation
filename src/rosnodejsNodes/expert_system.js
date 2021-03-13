var MongoClient = require('mongodb').MongoClient;
const rosnodejs = require('rosnodejs');
const StringMsg = rosnodejs.require('std_msgs').msg.String;
const YAML = require('yaml');


rosnodejs.initNode('/expert_system')
  .then(() => { });

const nh = rosnodejs.nh;
const pubex = nh.advertise('/exercises', StringMsg);

//MongoDB path
const uri = "mongodb://mongoadmin:secret@localhost:27888/?authSource=admin";

MongoClient.connect(uri, { useUnifiedTopology: true }, (err, client) => {
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
});
