var MongoClient = require('mongodb').MongoClient;
const rosnodejs = require('rosnodejs');
const StringMsg = rosnodejs.require('std_msgs').msg.String;
const YAML = require('yaml');

squats = {
  'name': 'squats',
  'stages': [
      {
          'angles': {
              'leftLeg': 165,
              'rightLeg': 165,
              'upperBody': 165
          },
          'rules': {
              'upperBody': ["min", 140, "Bitte achte darauf, dass dein Oberkoerper beim Stehen gerade ist"]
          },
          'name': "Anfangsposition"
      },
      {
          'angles': {
              'leftLeg': 125,
              'rightLeg': 125,
              'upperBody': 125
          },
          'rules': {},
          'name': "Transition"
      },
      {
          'angles': {
              'leftLeg': 90,
              'rightLeg': 90,
              'upperBody': 90
          },
          'rules': {
              'leftHipKneeToe': ["behind", 160, "Bitte achte darauf, in der Hocke dein linkes Knie nicht ueber deine Zehenspitzen zu lassen"],
              'rightHipKneeToe': ["behind", 160, "Bitte achte darauf, in der Hocke dein rechtes Knie nicht ueber deine Zehenspitzen zu lassen"],
          },
          'name': "Hocke"
      }
  ]
}

rosnodejs.initNode('/expert_system')
    .then(() => {});

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
        const query = { name: msg };
        const result = await exercises.findOne(query);
        const stringified = YAML.stringy(result);
        console.log(stringified);
        nh.setParam('exercise',stringified);
        pubex.publish({data: 'exercise'});
    });

  } finally {
    await client.close();
  }
}
run().catch(console.dir);
