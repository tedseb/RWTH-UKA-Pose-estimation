const rosnodejs = require('rosnodejs');
const StationUsage = rosnodejs.require("backend").msg.StationUsage;
var db = require('../postgres_sequelize/database');
const StringMsg = rosnodejs.require('std_msgs').msg.String;



rosnodejs.initNode('/crawler')
const nh = rosnodejs.nh;

nh.subscribe('/station_usage', StationUsage, async (msg) => {
    db.StationUsages.create({
        stationId: msg['stationID'],
        status: msg['isActive'],
        exercise: msg['exerciseName']
    });
});

nh.subscribe('/emergency', StringMsg, async (msg) => {
    db.Emergency.create({
        message: msg['data']
    });
});

