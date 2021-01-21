const rosnodejs = require('rosnodejs');
const StringMsg = rosnodejs.require('std_msgs').msg.String;
const math = require('mathjs');


// detection in the wild labels in right order
const ditw = ['nose', 'leftShoulder', 'rightShoulder',
    'leftElbow', 'rightElbow', 'leftWrist', 'rightWrist',
    'leftHip', 'rightHip', 'leftKnee', 'rightKnee', 'leftAnkle',
    'rightAnkle'];

const angle_points = {
    leftLeg: ['leftHip', 'leftKnee', 'leftAngle'],
    leftArm: ['leftShoulder', 'leftElbow', 'leftWrist'],
    leftShin: ['leftKnee', 'leftAnkle'],
    rightLeg: ['rightHip', 'rightKnee', 'rightAngle'],
    rightArm: ['rightShoulder', 'rightElbow', 'rightWrist'],
    rightShin: ['rightKnee', 'rightAnkle'],
    upperBody: ['rightShoulder', 'leftShoulder', 'leftHip', 'rightHip']
}


var angles = {};
var alpha = 15;
// var squats = {
//     1: {
//         leftLeg: 165,
//         rightLeg: 165,
//         upperBody: 165
//     },
//     2: {
//         leftLeg: 90,
//         rightLeg: 90,
//         upperBody: 90
//     }
// }
var squats = {
    stages: {
        0: { angles: {}, rules: {} },
        1: {
            angles: {
                leftLeg: 165,
                rightLeg: 165,
                upperBody: 165
            },
            rules: {
                upperBody: ["min", 140, "Bitte achte darauf, dass dein Oberkörper beim Stehen gerade ist"]
            },
            name: "Anfangsposition"
        },
        2: {
            angles: {
                leftLeg: 130,
                rightLeg: 130,
                upperBody: 130
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
                leftShin: ["min", 160, "Bitte achte darauf, in der Hocke dein linkes Knie nicht über deine Zehenspitzen zu lassen"],
                rightShin: ["min", 160, "Bitte achte darauf, in der Hocke dein rechtes Knie nicht über deine Zehenspitzen zu lassen"],
            },
            name: "Hocke"
        }
    }
}
var state = 0;
var states = { squats: [] };
var corrections = {
    squats: {
        1: "",
        2: ""
    }
};
var reps = 0;


rosnodejs.initNode('/expert')
    .then(() => {
        // do stuff
    });
const nh = rosnodejs.nh;
const sub = nh.subscribe('/personsJS', 'pose_estimation/Persons', (msg) => {
    let pose = {};
    this.coordinates = msg;
    let bodyParts = msg.persons[0]['bodyParts'];
    let labels = ditw;
    labels.forEach((label, index) => {
        let point = {};
        point.x = bodyParts[index].point.x;
        point.y = bodyParts[index].point.z;
        point.z = bodyParts[index].point.y;
        pose[label] = point;
    });
    lastPose = pose;
    angles.leftLeg = threepointangle(pose.leftHip, pose.leftKnee, pose.leftAnkle);
    angles.rightLeg = threepointangle(pose.rightHip, pose.rightKnee, pose.rightAnkle);
    angles.leftArm = threepointangle(pose.leftShoulder, pose.leftElbow, pose.leftWrist);
    angles.rightArm = threepointangle(pose.rightShoulder, pose.rightElbow, pose.rightWrist);
    angles.upperBody = (threepointangle(pose.leftShoulder, pose.leftHip, pose.leftKnee) + threepointangle(pose.rightShoulder, pose.rightHip, pose.rightKnee)) / 2;
    const bottomLeft = { x: pose.leftAnkle.x, y: pose.leftAnkle.y - 1, z: pose.leftAnkle.z };
    const bottomRight = { x: pose.rightAnkle.x, y: pose.rightAnkle.y - 1, z: pose.rightAnkle.z };
    angles.leftShin = threepointangle(pose.leftKnee, pose.leftAnkle, bottomLeft);
    angles.rightShin = threepointangle(pose.rightKnee, pose.rightAnkle, bottomRight);
    count();
    correct();
});
const pub = nh.advertise('/repcounter', StringMsg);
const error_pub = nh.advertise('/corrections', StringMsg);
const coordinats_pub = nh.advertise('/wrongcoordinates', StringMsg);

correct = () => {
    let messages = checkForCorrection(angles, squats, state);
    if (messages === corrections["squats"][state]) {
    } else {
        console.log(messages);
        corrections["squats"][state] = messages;
        if(messages.length > 0) {error_pub.publish({ data: messages })};
    }

};

count = () => {
    if (state < 3) {
        if (checkforstate(angles, squats, state + 1)) {
            //states.squats.push(states + 1);
            state++;
        }
    }
    if (state === 3) {
        if (checkforstate(angles, squats, 1)) {
            states.squats.push(1);
            state = 1;
            reps++;
            console.log(reps);
            pub.publish({ data: reps.toString() })
        }
    }
    // if (state === 0) {
    //     if (checkforstate(angles, squats, 1)) {
    //         states.squats.push(1);
    //         state = 1;
    //     }
    // } else if (state === 1) {
    //     if (checkforstate(angles, squats, 2)) {
    //         states.squats.push(2);
    //         state = 2;
    //     }
    // } else if (state === 2) {
    //     if (checkforstate(angles, squats, 1)) {
    //         states.squats.push(1);
    //         state = 1;
    //         reps++;
    //         console.log(reps);
    //         pub.publish({ data: reps.toString() })
    //     }
    // }
    // if (checkforrep(states.squats)) {
    //     //reps = checkforrep;
    // }
};

checkForCorrection = (angles, exercise, state) => {
    const rules = exercise["stages"][state]["rules"];
    const keys = Object.keys(rules);
    let corrections = "";
    keys.forEach(k => {
        switch (rules[k][0]) {
            case "max":
                if (angles[k] >= rules[k][1] + alpha) {
                    corrections += rules[k][2] + ". ";
                    //console.log(JSON.stringify(angle_points[k]));
                    coordinats_pub.publish({data: JSON.stringify(angle_points[k])});
                }
                break;
            case "min":
                if (angles[k] <= rules[k][1] - alpha) {
                    corrections += rules[k][2] + ". ";
                    //console.log(JSON.stringify(angle_points[k]));
                    coordinats_pub.publish({data: JSON.stringify(angle_points[k])});
                }
                break;
        }
    });
    if(corrections.length === 0) {
        coordinats_pub.publish({data: "[]"});
    }
    return corrections;
};

checkforstate = (angles, exercise, state) => {
    const stage = exercise["stages"][state]["angles"];
    const keys = Object.keys(stage);
    let pass = true;
    keys.forEach(k => {
        pass = pass && (angles[k] >= stage[k] - alpha && angles[k] <= stage[k] + alpha);
    });
    return pass;
}

checkforrep = (arr) => {
    if (arr[0] !== 1 || arr[arr.length - 1] !== 1) {
        return false;
    }
    for (let i = 1; i < arr.length - 1; i++) {
        if (arr[i] - arr[i - 1] > 1) {
            return false;
        }
    }
    return true;
}

angle2d = (x, y) => {
    return math.atan2(y.y - x.y, y.x - x.x) * 180 / Math.PI
};
angle3d = (x, y) => {
    return math.acos(dot(x, y) / (length(x) * length(y))) * 180 / Math.PI
};
threepointangle = (a, b, c) => {
    const ba = vector(b, a);
    const bc = vector(b, c);
    return angle3d(ba, bc);
};
vector = (x, y) => {
    return { x: y.x - x.x, y: y.y - x.y, z: y.z - x.z }
};
dot = (x, y) => {
    return math.dot([x.x, x.y, x.z], [y.x, y.y, y.z]);
};
length = (x) => {
    return math.sqrt(math.pow(x.x, 2) + math.pow(x.y, 2) + math.pow(x.z, 2));
};

