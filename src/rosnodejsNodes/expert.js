const rosnodejs = require('rosnodejs');
const StringMsg = rosnodejs.require('std_msgs').msg.String;
const math = require('mathjs');



// detection in the wild labels in right order
const ditw = ['nose', 'leftShoulder', 'rightShoulder',
    'leftElbow', 'rightElbow', 'leftWrist', 'rightWrist',
    'leftHip', 'rightHip', 'leftKnee', 'rightKnee', 'leftAnkle',
    'rightAnkle'];
const ownpose_used = [0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 27, 28, 37, 39, 41, 42, 43];
const ownpose_labels = ['OP_Nose', 'OP_Neck', 'OP_R_Shoulder', 'OP_R_Elbow', 'OP_R_Wrist', 'OP_L_Shoulder', 'OP_L_Elbow', 'OP_L_Wrist', 'OP_Middle_Hip', 'OP_R_Hip', 'OP_R_Knee', 'OP_R_Ankle', 'OP_L_Hip', 'OP_L_Knee', 'OP_L_Ankle', 'OP_R_Eye', 'OP_L_Eye', 'OP_R_Ear', 'OP_L_Ear', 'OP_L_Big_Toe', 'OP_L_Small_Toe', 'OP_L_Heel', 'OP_R_Big_Toe', 'OP_R_Small_Toe', 'OP_R_Heel', 'R_Ankle', 'R_Knee', 'R_Hip', 'L_Hip', 'L_Knee', 'L_Ankle', 'R_Wrist', 'R_Elbow', 'R_Shoulder', 'L_Shoulder', 'L_Elbow', 'L_Wrist', 'Neck_LSP', 'Top_of_Head_LSP', 'Pelvis_MPII', 'Thorax_MPII', 'Spine_HM', 'Jaw_HM', 'Head_HM', 'Nose', 'L_Eye', 'R_Eye', 'L_Ear', 'R_Ear']
const ownpose = [
    [2, 3], [3, 4], [5, 6], [6, 7], [27, 9], [9, 12], [27, 28], [27, 10], [10, 11], [12, 13], [9, 10], [28, 12], [28, 13], [13, 14], [14, 21], [21, 20], [21, 19], [20, 19], [11, 24], [24, 22], [22, 23], [23, 24], [5, 28], [2, 27], [5, 2], [42, 17], [42, 18], [42, 0], [0, 15], [0, 16], [15, 16], [17, 43], [18, 43], [1, 37], [37, 43], [41, 37], [41, 39]
]
const angle_points = {
    leftHipKneeToe: ['OP_L_Hip', 'OP_L_Knee', 'OP_L_Big_Toe'],
    rightHipKneeToe: ['OP_R_Hip', 'OP_R_Knee', 'OP_R_Big_Toe'],
    leftLeg: ['OP_L_Hip', 'OP_L_Knee', 'OP_L_Ankle'],
    leftArm: ['OP_L_Shoulder', 'OP_L_Elbow', 'OP_L_Wrist'],
    leftShin: ['OP_L_Knee', 'OP_L_Ankle'],
    rightLeg: ['OP_R_Hip', 'OP_R_Knee', 'OP_R_Ankle'],
    rightArm: ['OP_R_Shoulder', 'OP_R_Elbow', 'OP_R_Wrist'],
    rightShin: ['OP_R_Knee', 'OP_R_Ankle'],
    upperBody: ['OP_R_Shoulder', 'OP_R_Hip', 'OP_R_Knee', 'OP_L_Shoulder', 'OP_L_Hip', 'OP_L_Knee']
}

var last30 = [];
var lastPose = {};
var angles = {};
var alpha = 20;
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
                leftHipKneeToe: ["behind", 160, "Bitte achte darauf, in der Hocke dein linkes Knie nicht über deine Zehenspitzen zu lassen"],
                rightHipKneeToe: ["behind", 160, "Bitte achte darauf, in der Hocke dein rechtes Knie nicht über deine Zehenspitzen zu lassen"],
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

nh.setParam('person', {'Bein_x': 102, 'Bein_y': 250, 'Bein_z': 170}) ;//Shawan
nh.setParam('station_skeleton', {'Station1': 1, 'Station2': 3, 'Station3': 2});//Shawan


const sub = nh.subscribe('/fused_skelleton', 'backend/Persons', (msg) => {
    let pose = {};
    this.coordinates = msg;
    let bodyParts = msg.persons[0]['bodyParts'];
    /* let labels = ditw;
    labels.forEach((label, index) => {
        let point = {};
        point.x = bodyParts[index].point.x;
        point.y = bodyParts[index].point.z;
        point.z = bodyParts[index].point.y;
        pose[label] = point;
    }); */

    ownpose_used.forEach(index => {
        let point = {};
        point.x = bodyParts[index].point.x;
        point.y = bodyParts[index].point.z;
        point.z = bodyParts[index].point.y;
        pose[ownpose_labels[index]] = point;
    });
    save(pose);
    let entries = Object.entries(angle_points);
    for (const [angle, points] of entries) {
        angles[angle] = calculateAngle(points);
    }
    
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
        if (messages.length > 0) { error_pub.publish({ data: messages }) };
    }

};

save = (obj) => {
    lastPose = obj;
    if (last30.length >= 30) {
        last30.shift();
        last30.push(obj);
    } else {
        last30.push(obj);
    }
}

calculateAngle = (arr) => {
    switch (arr.length) {
        case 2:
            let newPoint = {
                x: lastPose[arr[1]].x,
                y: lastPose[arr[1]].y - 1,
                z: lastPose[arr[1]].z
            };
            return threepointangle(lastPose[arr[0]], lastPose[arr[1]], newPoint);
            break;
        case 3:
            return threepointangle(lastPose[arr[0]], lastPose[arr[1]], lastPose[arr[2]]);
            break;
        case 6:
            return (threepointangle(lastPose[arr[0]], lastPose[arr[1]], lastPose[arr[2]]) + threepointangle(lastPose[arr[3]], lastPose[arr[4]], lastPose[arr[5]])) / 2;
            break;
        default:
            return 0;
    }
};

count = () => {
    if (state < 3) {
        if (checkforstate(angles, squats, state + 1)) {
            //states.squats.push(states + 1);
            state++;
        } else {

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
                    coordinats_pub.publish({ data: JSON.stringify(angle_points[k]) });
                }
                break;
            case "min":
                if (angles[k] <= rules[k][1] - alpha) {
                    corrections += rules[k][2] + ". ";
                    coordinats_pub.publish({ data: JSON.stringify(angle_points[k]) });
                }
                break;
            case "behind":
                //a: 0, b: 1, p: 1, x: 2
                const a = lastPose[angle_points[k][0]]
                const b = lastPose[angle_points[k][1]]
                const p = lastPose[angle_points[k][1]]
                const x = lastPose[angle_points[k][2]]
                const b_ = {x: b.x, y: a.y, z: b.z};
                const val_x = plane3d(a, b_, p, x);
                const val_a = plane3d(a, b_, p, a);
                if(math.sign(val_x) === math.sign(val_a)) {
                    corrections += rules[k][2] + ". ";
                    coordinats_pub.publish({ data: JSON.stringify(angle_points[k]) });
                }
                break;
        }
    });
    if (corrections.length === 0) {
        coordinats_pub.publish({ data: "[]" });
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


const vectorUp = {x: 0, y: 1, z: 0}; 
const vectorRight = {x: 1, y: 0, z: 0};

//a and b are the start and end point of a given vector, which is perpendicular to the plane
//p is a point of that plane
//x will be checked against that plane
// const b_ = {x: b.x, y: a.y, z: b.z}; for a plane that is 
plane3d = (a, b, p, x) => {
    //const b_ = {x: b.x, y: a.y, z: b.z};
    const direction = vector(a, b);
    //console.log(direction);
    //console.log({x: x.x - p.x, y: x.y - p.y, z: x.z - p.z});
    const val = dot({x: x.x - p.x, y: x.y - p.y, z: x.z - p.z}, direction);
    //console.log(val);
    return val;
}

cross = (x, y) => {
    const cross =  math.cross([x.x, x.y, x.z], [y.x, y.y, y.z]);
    return {x: cross[0], y: cross[1], z: cross[2]};
}

checkForStretch = (arr) => {
    let lastAngles;
    let currentAngles = {};
    let differences = {};
    let keys = Object.keys(angle_points);
    for (const key of keys) {
        differences[key] = [];
    }
    arr.forEach(el => {
        let entries = Object.entries(angle_points);
        for (const [angle, points] of entries) {
            currentAngles[angle] = calculateAngle(points);
        }
        if (lastAngles) {
            let keys = Object.keys(lastAngles);
            for (const key of keys) {
                console.log(currentAngles[key] - lastAngles[key]);
                differences[key].push(currentAngles[key] - lastAngles[key]);
            }
        } else {
            lastAngles = currentAngles;
        }
    });
    let entries = Object.entries(differences);
    let sums = {};
    for (const [angle, diff] of entries) {
        console.log(diff);
        let sum = 0;
        diff.forEach(el => { sum += el });
        sums[angle] = sum;
    }
    console.log(sums);
    console.log(Object.values(sums));
    if (sums[leftLeg] >= 0 || sums[rightLeg] >= 0) {
        return true;
    } else return false;
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
    console.log(angle3d(ba, bc));
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

