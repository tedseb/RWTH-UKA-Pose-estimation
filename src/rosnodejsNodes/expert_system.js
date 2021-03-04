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


async function run() {
  try {
    await client.connect();
    const db = client.db("trainerai");
    const exercises = db.collection("exercises");

    nh.subscribe('/qr_exercise', StringMsg, async (msg) => {
      try {
        const qr = { name: msg };
        var result = await exercises.findOne(qr); 
      } finally {
        const stringified = YAML.stringify(result);
        console.log(stringified);
       // nh.setParam('exercise', stringified);
        pubex.publish({ data: 'exercise' });
      }
    });

  } finally {
   // await client.close();
  }
}
run().catch(console.dir);
