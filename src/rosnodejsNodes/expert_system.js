var MongoClient = require('mongodb').MongoClient;
const rosnodejs = require('rosnodejs');
const StringMsg = rosnodejs.require('std_msgs').msg.String;
const YAML = require('yaml')

rosnodejs.initNode('/expert_slim')
    .then(() => {});

const nh = rosnodejs.nh;
const pubex = nh.advertise('/exercises', StringMsg);

// Connect to the db
const uri = "mongodb://localhost:27017/";

const client = new MongoClient(uri);

async function run() {
  try {
    await client.connect();
    const db = client.db("trainerai");
    const exercises = db.collection("exercises");

    nh.subscribe('/qr_exercise', StringMsg, (msg) => {
        const query = { name: msg };
        const result = await exercises.findOne(query);
        const stringified = YAML.stringy(result);
        nh.setParam('exercise',stringified);
        pubex.publish({data: 'exercise'});
    });

    // Query for a movie that has the title 'The Room'
    const query = { name: "squats" };

    const result = await exercises.findOne(query);
    console.log(result);
  } finally {
    await client.close();
  }
}
run().catch(console.dir);