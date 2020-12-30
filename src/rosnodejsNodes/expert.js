const rosnodejs = require('rosnodejs');
const math = require('mathjs');

// detection in the wild labels in right order
const ditw = ['nose', 'leftShoulder', 'rightShoulder',
    'leftElbow', 'rightElbow', 'leftWrist', 'rightWrist',
    'leftHip', 'rightHip', 'leftKnee', 'rightKnee', 'leftAnkle',
    'rightAnkle'];

var angles = {};
var lastPose;

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
    console.log(pose.leftShoulder, pose.leftElbow, pose.leftWrist);
    console.log(threepointangle(pose.leftShoulder, pose.leftElbow, pose.leftWrist));
});

angle2d = (x, y) => { 
    return math.atan2(y.y - x.y, y.x - x.x) * 180 / Math.PI 
};
angle3d = (x, y) => { 
    return math.acos(dot(x,y) / (length(x)*length(y))) * 180 / Math.PI
};
threepointangle = (a, b, c) => {
    const ba = vector(b, a);
    const bc = vector(b, c);
    return angle3d(ba, bc);
}
vector = (x, y) => {
    return {x: y.x - x.x, y: y.y - x.y, z: y.z - x.z}
}
dot = (x, y) => {
    return math.dot([x.x, x.y, x.z], [y.x, y.y, y.z]);
};
length = (x) => {
    return math.sqrt(math.pow(x.x, 2) + math.pow(x.y, 2) + math.pow(x.z, 2));
};

