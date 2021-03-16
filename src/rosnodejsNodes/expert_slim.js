const rosnodejs = require('rosnodejs');
const YAML = require('yaml')
const StringMsg = rosnodejs.require('std_msgs').msg.String;

var squats = {
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
                'rightHipKneeToe': ["behind", 160, "Bitte achte darauf, in der Hocke dein rechtes Knie nicht ueber deine Zehenspitzen zu lassen"]
            },
            'name': "Hocke"
        }
    ]
}


rosnodejs.initNode('/expert_slim')

const nh = rosnodejs.nh;

const errors = nh.subscribe('/qr_exercise', StringMsg, (msg) => {
    //Call Reader Function to load data from database and send it to server in ms is exercise name
    nh.setParam('exercise',s_exercises); //dirty
});

//Put into exercise the different exercises
var exercises = {
        '0.0': squats,
        '1.0': squats
}

console.log(exercises)

var s_exercises = YAML.stringify(exercises);
nh.setParam('exercise',s_exercises); //dirty
console.log(exercises);


