const rosnodejs = require('rosnodejs');
const StringMsg = rosnodejs.require('std_msgs').msg.String;

var squats = {
    exercise: 'squats',
    stages: {
        0: { angles: {}, rules: {} },
        1: {
            angles: {
                leftLeg: 165,
                rightLeg: 165,
                upperBody: 165
            },
            rules: {
                upperBody: ["min", 140, "Bitte achte darauf, dass dein Oberkoerper beim Stehen gerade ist"]
            },
            name: "Anfangsposition"
        },
        2: {
            angles: {
                leftLeg: 125,
                rightLeg: 125,
                upperBody: 125
            },
            rules: {},
            name: "Transition"
        },
        3: {
            angles: {
                leftLeg: 90,
                rightLeg: 90,
                upperBody: 90
            },
            rules: {
                leftHipKneeToe: ["behind", 160, "Bitte achte darauf, in der Hocke dein linkes Knie nicht ueber deine Zehenspitzen zu lassen"],
                rightHipKneeToe: ["behind", 160, "Bitte achte darauf, in der Hocke dein rechtes Knie nicht ueber deine Zehenspitzen zu lassen"],
            },
            name: "Hocke"
        }
    }
}



rosnodejs.initNode('/expert_slim')
    .then(() => {
        // do stuff
    });
const nh = rosnodejs.nh;
var s_squats = JSON.stringify(squats);

const errors = nh.subscribe('/qr_exercise', StringMsg, (msg) => {
    //Call Reader Function to load data from database and send it to server in ms is exercise name
    nh.setParam('exercise',s_squats); //dirty
    console.log(s_squats);
});




