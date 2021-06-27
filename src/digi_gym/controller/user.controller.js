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

  exports.stationsAll = async (req, res) => {
    const stations = await db.Stations.findAll();
    const frames = await db.Frames.findAll();

    const obj = {
      stations: stations,
      frames: frames
    }
    console.log("called");

    res.status(200).send(
      JSON.stringify(obj)
    );
  }