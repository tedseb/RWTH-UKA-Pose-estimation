const { ClientBase } = require('pg');
var db = require('../../postgres_sequelize/database');

exports.allAccess = (req, res) => {
  res.status(200).send("Public Content.");
};

exports.userBoard = (req, res) => {
  res.status(200).send("User Content.");
};

exports.adminBoard = (req, res) => {
  res.status(200).send("Admin Content.");
};

exports.moderatorBoard = (req, res) => {
  res.status(200).send("Moderator Content.");
};

exports.framesAll = async (req, res) => {
  const frames = await db.Frames.findAll({
    include: [
      { model: db.CameraStationMappings, include: [db.Stations, db.Cameras] }
    ]
  });
  let parsedFrames = frames.map(x => parseFrameBox(x));
  res.status(200).send(
    JSON.stringify(parsedFrames)
  );
}

exports.stationsAll = async(req, res) => {
  const stations = await db.Stations.findAll({include: db.Cameras});

  res.status(200).send(JSON.stringify(stations));
}

exports.statisticsAll = async(req, res) => {
  const station_usage = await db.StationUsages.findAll({where: {status: true}});
  const stationIds = [...new Set(station_usage.map(item => item.stationId))]
  objs = [];
  for(id of stationIds) {
    
    let stations_today = station_usage.filter(x => {
      console.log(x.createdAt, new Date(x.createdAt))
      if(x.stationId == id &&new Date(x.createdAt).getDay() == new Date().getDay()) return true; else return false;
    });
    let stations_yesterday = station_usage.filter(x => x.stationId == id && Date(x.createdAt).getDay == (new Date().getDay() - 1));
    let obj = {
      name: "Station " + id,
      series: [
        {name: "Today",
         value:  stations_today.length},
         {name: "Yesterday",
         value:  stations_yesterday.length}
      ]
    }
    objs.push(obj);
  }
  res.status(200).send(JSON.stringify(objs));
}

function parseFrameBox(value) {
  console.log(value);
  value.dataValues.frame_box = parse(value.dataValues.frame_box);
  return value;
}

function parse(val) {
  const regexp = /\d+/g;
  const match = val.match(regexp);
  console.log(match);
  return [[match[0], match[1]], [match[2], match[3]]];
}

