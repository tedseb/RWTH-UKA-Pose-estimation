const rosnodejs = require('rosnodejs');
const pose_estimation = rosnodejs.require("pose_estimation");

const { createCanvas, loadImage } = require('canvas')
const fs = require('fs')
const tf = require('@tensorflow/tfjs-node');
const posenet = require('@tensorflow-models/posenet');
const cv = require('opencv4nodejs');
const { drawKeyPoints, ellipse } = require('opencv4nodejs');
const { array } = require('@tensorflow/tfjs-data');

const imageScaleFactor = 0.5;
const outputStride = 16;
const flipHorizontal = false;

// get up to 5 poses
const maxPoseDetections = 5;
// minimum confidence of the root part of a pose
const scoreThreshold = 0.0001;
// minimum distance in pixels between the root parts of poses
const nmsRadius = 20;
  

const loadModel = async() => {
    console.log('start');

    global.net = await posenet.load({
      architecture: 'MobileNetV1',
      outputStride: 16,
      inputResolution: { width: 224, height: 224 },
      multiplier: 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
    });
}

tryModelIN = async(msg) => {
  //Check if net-variable has been initialized or defined. Take care is aysnc!
  if(typeof net !== 'undefined'){
    //convert from buffer to CV-mat)
    const matFromArray = new cv.Mat(new Buffer.from(msg.data), msg.height, msg.width, cv.CV_8UC3);
    let src = matFromArray.resize(224, 224);
    //convert from BGR to RGB
    let mat = src.cvtColor(cv.COLOR_BGR2RGB);
  
    // convert to 3d tensor
    let buffer = new Uint8Array(mat.getData().buffer);
    let tFrame = tf.tensor3d(buffer, [224, 224, 3]);

    // net is your poseNet instance
    let pose = await net.estimateSinglePose(tFrame,flipHorizontal);
    //let pose = await net.estimateMultiplePoses(tFrame, imageScaleFactor, flipHorizontal, outputStride, maxPoseDetections, scoreThreshold, nmsRadius);
    for(const keypoint of pose.keypoints) {
      console.log(`${keypoint.part}: (${keypoint.position.x},${keypoint.position.y},${keypoint.score})`);
    }

    const msgPersons = new pose_estimation.msg.Persons;
    msgPersons.header = msg.header
    msgPersons.persons = new Array();
    person_msg = new pose_estimation.msg.Person;
  
    for(idx = 0; idx < 25; idx++) {
      person_msg.bodyParts[idx] = new pose_estimation.msg.Bodypart;
    }
    var len = pose.keypoints.length;
    for(idx = 0; idx < len; idx++) {/*
      person_msg.bodyParts[idx].score = pose.keypoints[idx].score
      person_msg.bodyParts[idx].pixel.x = pose.keypoints[idx].position.x
      person_msg.bodyParts[idx].pixel.y = pose.keypoints[idx].position.y
      person_msg.bodyParts[idx].point.x =pose.keypoints[idx].position.x/100
      person_msg.bodyParts[idx].point.y =pose.keypoints[idx].position.y/100
      person_msg.bodyParts[idx].point.z = 0*/
      
      person_msg.bodyParts[idx].score = pose.keypoints[idx].score
      person_msg.bodyParts[idx].pixel.x = pose.keypoints[idx].position.x
      person_msg.bodyParts[idx].pixel.y = pose.keypoints[idx].position.y
      person_msg.bodyParts[idx].point.x =2*(pose.keypoints[idx].position.x/50-2)* msg.width/msg.height///500-0.5
      person_msg.bodyParts[idx].point.y =0.0
      person_msg.bodyParts[idx].point.z = -2*(pose.keypoints[idx].position.y/50-4)  ///500-1.5
      //mat.drawCircle(new cv.Point2(pose.keypoints[idx].position.x, pose.keypoints[idx].position.y)  , 5 , new cv.Vec(255,255,255) ,1 , 2 ,  0 ); 
      
    }
   
    //cv.imwrite('./src/rosnodejsNodes/img.png', mat);

    msgPersons.persons.push(person_msg)
    pub.publish(msgPersons)
 
  }
}
rosnodejs.initNode('/DasTorZurUnterwelt')
.then(() => {loadModel()
});
const nh = rosnodejs.nh;
const sub = nh.subscribe('/image', 'sensor_msgs/Image', tryModelIN);
const pub = nh.advertise('/personsJS', 'pose_estimation/Persons');

 /*  
    person_msg.bodyParts[0].score = pose.keypoints[0].score;
    person_msg.bodyParts[0].pixel.x = pose.keypoints[0].position.x/1000;
    person_msg.bodyParts[0].pixel.y = pose.keypoints[0].position.y/1000;
    person_msg.bodyParts[0].point.x = 2*(pose.keypoints[0].position.x/1000-0.5)*msg.width/msg.height;
    person_msg.bodyParts[0].point.y = 0.0;
    person_msg.bodyParts[0].point.z = -2*(pose.keypoints[0].position.y/1000-1.5);

    person_msg.bodyParts[16].score = pose.keypoints[1].score;
    person_msg.bodyParts[16].pixel.x = pose.keypoints[1].position.x/1000;
    person_msg.bodyParts[16].pixel.y = pose.keypoints[1].position.y/1000;
    person_msg.bodyParts[16].point.x = 2*(pose.keypoints[1].position.x/1000-0.5)*msg.width/msg.height;
    person_msg.bodyParts[16].point.y = 0.0;
    person_msg.bodyParts[16].point.z = -2*(pose.keypoints[1].position.y/1000-1.5);

    person_msg.bodyParts[15].score = pose.keypoints[2].score;
    person_msg.bodyParts[15].pixel.x = pose.keypoints[2].position.x/1000;
    person_msg.bodyParts[15].pixel.y = pose.keypoints[2].position.y/1000;
    person_msg.bodyParts[15].point.x = 2*(pose.keypoints[2].position.x/1000-0.5)*msg.width/msg.height;
    person_msg.bodyParts[15].point.y = 0.0;
    person_msg.bodyParts[15].point.z = -2*(pose.keypoints[2].position.y/1000-1.5);

    person_msg.bodyParts[18].score = pose.keypoints[3].score;
    person_msg.bodyParts[18].pixel.x = pose.keypoints[3].position.x/1000;
    person_msg.bodyParts[18].pixel.y = pose.keypoints[3].position.y/1000;
    person_msg.bodyParts[18].point.x = 2*(pose.keypoints[3].position.x/1000-0.5)*msg.width/msg.height;
    person_msg.bodyParts[18].point.y = 0.0;
    person_msg.bodyParts[18].point.z = -2*(pose.keypoints[3].position.y/1000-1.5);

    person_msg.bodyParts[17].score = pose.keypoints[4].score;
    person_msg.bodyParts[17].pixel.x = pose.keypoints[4].position.x/1000;
    person_msg.bodyParts[17].pixel.y = pose.keypoints[4].position.y/1000;
    person_msg.bodyParts[17].point.x = 2*(pose.keypoints[4].position.x/1000-0.5)*msg.width/msg.height;
    person_msg.bodyParts[17].point.y = 0.0;
    person_msg.bodyParts[17].point.z = -2*(pose.keypoints[7].position.y/1000-1.5);

    person_msg.bodyParts[5].score = pose.keypoints[5].score;
    person_msg.bodyParts[5].pixel.x = pose.keypoints[5].position.x/1000;
    person_msg.bodyParts[5].pixel.y = pose.keypoints[5].position.y/1000;
    person_msg.bodyParts[5].point.x = 2*(pose.keypoints[5].position.x/1000-0.5)*msg.width/msg.height;
    person_msg.bodyParts[5].point.y = 0.0;
    person_msg.bodyParts[5].point.z = -2*(pose.keypoints[5].position.y/1000-1.5);

    person_msg.bodyParts[2].score = pose.keypoints[6].score;
    person_msg.bodyParts[2].pixel.x = pose.keypoints[6].position.x/1000;
    person_msg.bodyParts[2].pixel.y = pose.keypoints[6].position.y/1000;
    person_msg.bodyParts[2].point.x = 2*(pose.keypoints[6].position.x/1000-0.5)*msg.width/msg.height;
    person_msg.bodyParts[2].point.y = 0.0;
    person_msg.bodyParts[2].point.z = -2*(pose.keypoints[6].position.y/1000-1.5);

    person_msg.bodyParts[6].score = pose.keypoints[7].score;
    person_msg.bodyParts[6].pixel.x = pose.keypoints[7].position.x/1000;
    person_msg.bodyParts[6].pixel.y = pose.keypoints[7].position.y/1000;
    person_msg.bodyParts[6].point.x = 2*(pose.keypoints[7].position.x/1000-0.5)*msg.width/msg.height;
    person_msg.bodyParts[6].point.y = 0.0;
    person_msg.bodyParts[6].point.z = -2*(pose.keypoints[7].position.y/1000-1.5);

    person_msg.bodyParts[3].score = pose.keypoints[8].score;
    person_msg.bodyParts[3].pixel.x = pose.keypoints[8].position.x/1000;
    person_msg.bodyParts[3].pixel.y = pose.keypoints[8].position.y/1000;
    person_msg.bodyParts[3].point.x = 2*(pose.keypoints[8].position.x/1000-0.5)*msg.width/msg.height;
    person_msg.bodyParts[3].point.y = 0.0;
    person_msg.bodyParts[3].point.z = -2*(pose.keypoints[8].position.y/1000-1.5);

    person_msg.bodyParts[7].score = pose.keypoints[9].score;
    person_msg.bodyParts[7].pixel.x = pose.keypoints[9].position.x/1000;
    person_msg.bodyParts[7].pixel.y = pose.keypoints[9].position.y/1000;
    person_msg.bodyParts[7].point.x = 2*(pose.keypoints[9].position.x/1000-0.5)*msg.width/msg.height;
    person_msg.bodyParts[7].point.y = 0.0;
    person_msg.bodyParts[7].point.z = -2*(pose.keypoints[9].position.y/1000-1.5);

    person_msg.bodyParts[4].score = pose.keypoints[10].score;
    person_msg.bodyParts[4].pixel.x = pose.keypoints[10].position.x/1000;
    person_msg.bodyParts[4].pixel.y = pose.keypoints[10].position.y/1000;
    person_msg.bodyParts[4].point.x = 2*(pose.keypoints[10].position.x/1000-0.5)*msg.width/msg.height;
    person_msg.bodyParts[4].point.y = 0.0;
    person_msg.bodyParts[4].point.z = -2*(pose.keypoints[10].position.y/1000-1.5);

    person_msg.bodyParts[12].score = pose.keypoints[11].score;
    person_msg.bodyParts[12].pixel.x = pose.keypoints[11].position.x/1000;
    person_msg.bodyParts[12].pixel.y = pose.keypoints[11].position.y/1000;
    person_msg.bodyParts[12].point.x = 2*(pose.keypoints[11].position.x/100-0.5)*msg.width/msg.height;
    person_msg.bodyParts[12].point.y = 0.0;
    person_msg.bodyParts[12].point.z = -2*(pose.keypoints[11].position.y/1000-1.5);

    person_msg.bodyParts[9].score = pose.keypoints[12].score;
    person_msg.bodyParts[9].pixel.x = pose.keypoints[12].position.x/1000;
    person_msg.bodyParts[9].pixel.y = pose.keypoints[12].position.y/1000;
    person_msg.bodyParts[9].point.x = 2*(pose.keypoints[12].position.x/1000-0.5)*msg.width/msg.height;
    person_msg.bodyParts[9].point.y = 0.0;
    person_msg.bodyParts[9].point.z = -2*(pose.keypoints[12].position.y/1000-1.5);

    person_msg.bodyParts[13].score = pose.keypoints[13].score;
    person_msg.bodyParts[13].pixel.x = pose.keypoints[13].position.x/1000;
    person_msg.bodyParts[13].pixel.y = pose.keypoints[13].position.y/1000;
    person_msg.bodyParts[13].point.x = 2*(pose.keypoints[13].position.x/1000-0.5)*msg.width/msg.height;
    person_msg.bodyParts[13].point.y = 0.0;
    person_msg.bodyParts[13].point.z = -2*(pose.keypoints[13].position.y/1000-1.5);

    person_msg.bodyParts[10].score = pose.keypoints[14].score;
    person_msg.bodyParts[10].pixel.x = pose.keypoints[14].position.x/1000;
    person_msg.bodyParts[10].pixel.y = pose.keypoints[14].position.y/1000;
    person_msg.bodyParts[10].point.x = 2*(pose.keypoints[14].position.x/1000-0.5)*msg.width/msg.height;
    person_msg.bodyParts[10].point.y = 0.0;
    person_msg.bodyParts[10].point.z = -2*(pose.keypoints[14].position.y/1000-1.5);

    person_msg.bodyParts[14].score = pose.keypoints[15].score;
    person_msg.bodyParts[14].pixel.x = pose.keypoints[15].position.x/1000;
    person_msg.bodyParts[14].pixel.y = pose.keypoints[15].position.y/1000;
    person_msg.bodyParts[14].point.x = 2*(pose.keypoints[15].position.x/1000-0.5)*msg.width/msg.height;
    person_msg.bodyParts[14].point.y = 0.0;
    person_msg.bodyParts[14].point.z = -2*(pose.keypoints[15].position.y/1000-1.5);

    person_msg.bodyParts[11].score = pose.keypoints[16].score;
    person_msg.bodyParts[11].pixel.x = pose.keypoints[16].position.x/1000;
    person_msg.bodyParts[11].pixel.y = pose.keypoints[16].position.y/1000;
    person_msg.bodyParts[11].point.x = 2*(pose.keypoints[16].position.x/1000-0.5)*msg.width/msg.height;
    person_msg.bodyParts[11].point.y = 0.0;
    person_msg.bodyParts[11].point.z = -2*(pose.keypoints[16].position.y/1000-1.5);

  */