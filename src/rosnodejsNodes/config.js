const { SSL_OP_EPHEMERAL_RSA } = require("constants");

var config = {};


config.MOBILE_SERVER_PORT = 3030;

// webapp constants
config.PORT = 3000;

// db constans
config.db_uri = "mongodb://mongoadmin:secret@localhost:27888/?authSource=admin&readPreference=primary&appname=MongoDB%20Compass&ssl=false";
config.PORT_exp = 3001;
config.exp_api_options = {
  secure: false,
  hostname: 'localhost',
  port: 3001,
  path: '/expert/exercises/recordings',
  method: 'POST',
  headers: {
    'Content-Type': 'application/json'
  }
};

// ownpose configuration for the Metrabs Paper style pose defenition (the new default since our beta)
config.matrabs_used = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23];
config.matrabs_labels = ['M_Hip', 'L_Hip', 'R_Hip', 'L_Back', 'L_Knee', 'R_Knee', 'M_Back', 'L_Ankle', 'R_Ankle', 'U_Back', 'L_Toes', 'R_Toes', 'Neck', 'L_Collarbone', 'R_Collarbone', 'Head', 'L_Shoulder', 'R_Shoulder', 'L_Elbow', 'R_Elbow', 'L_Wrist', 'R_Wrist', 'L_Fingers', 'R_Fingers'];
config.matrabs = [[1,4], [1,0], [2,5], [2,0], [3,6], [3,0], [4,7], [5,8], [6,9], [7,10], [8,11], [9,12], [12,13], [12,14], [12,15], [13,16], [14,17], [16,18], [17,19], [18,20], [19,21], [20,22], [21,23]];

// ownpose configuration for the SPIN Paper style pose defenition (this is legacy)
config.ownpose_used_spin = [0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 27, 28, 37, 39, 41, 42, 43];
config.ownpose_labels_spin = ['OP_Nose', 'OP_Neck', 'OP_R_Shoulder', 'OP_R_Elbow', 'OP_R_Wrist', 'OP_L_Shoulder', 'OP_L_Elbow', 'OP_L_Wrist', 'OP_Middle_Hip', 'OP_R_Hip', 'OP_R_Knee', 'OP_R_Ankle', 'OP_L_Hip', 'OP_L_Knee', 'OP_L_Ankle', 'OP_R_Eye', 'OP_L_Eye', 'OP_R_Ear', 'OP_L_Ear', 'OP_L_Big_Toe', 'OP_L_Small_Toe', 'OP_L_Heel', 'OP_R_Big_Toe', 'OP_R_Small_Toe', 'OP_R_Heel', 'R_Ankle', 'R_Knee', 'R_Hip', 'L_Hip', 'L_Knee', 'L_Ankle', 'R_Wrist', 'R_Elbow', 'R_Shoulder', 'L_Shoulder', 'L_Elbow', 'L_Wrist', 'Neck_LSP', 'Top_of_Head_LSP', 'Pelvis_MPII', 'Thorax_MPII', 'Spine_HM', 'Jaw_HM', 'Head_HM', 'Nose', 'L_Eye', 'R_Eye', 'L_Ear', 'R_Ear'];
config.ownpose_spin = [[2, 3], [3, 4], [5, 6], [6, 7], [27, 9], [9, 12], [27, 28], [27, 10], [10, 11], [12, 13], [9, 10], [28, 12], [28, 13], [13, 14], [14, 21], [21, 20], [21, 19], [20, 19], [11, 24], [24, 22], [22, 23], [23, 24], [5, 28], [2, 27], [5, 2], [42, 17], [42, 18], [42, 0], [0, 15], [0, 16], [15, 16], [17, 43], [18, 43], [1, 37], [37, 43], [41, 37], [41, 39]];

module.exports = config;