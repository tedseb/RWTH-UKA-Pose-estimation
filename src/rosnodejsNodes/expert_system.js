var MongoClient = require('mongodb').MongoClient;
const rosnodejs = require('rosnodejs');
const StringMsg = rosnodejs.require('std_msgs').msg.String;
const YAML = require('yaml');

rosnodejs.initNode('/expert_system')
  .then(() => { });

const nh = rosnodejs.nh;
const pubex = nh.advertise('/exercises', StringMsg);

// Connect to the db
const uri = "mongodb://mongoadmin:secret@localhost:27888/?authSource=admin";

const client = new MongoClient(uri);

// MongoClient.connect(uri, (err, client) => {

// });


async function run() {
  try {
    await client.connect();
    const db = client.db("trainerai");
    const exercises = db.collection("exercises");

    nh.subscribe('/qr_exercise', StringMsg, async (msg) => {
      exercises.findOne({ name: msg }, (err, result) => {
        console.log(msg);
        console.log(result);
        const stringified = YAML.stringify(result);
        nh.setParam('exercise', stringified);
        pubex.publish({ data: 'exercise' });
      });
    });

  } finally {
    // await client.close();
  }
}
run().catch(console.dir);
