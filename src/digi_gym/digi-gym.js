const express = require('express');
const rosnodejs = require('rosnodejs');
const StationUsage = rosnodejs.require("backend").msg.StationUsage;
const config = require('./config.json');
const bodyParser = require('body-parser');
var db = require('./database');
const WebSocket = require('ws');

// Parameters and Constants:
const PORT = config.PORT;
const app = express();
const server = app.listen(PORT, () => { console.log("Listening on port " + PORT) })
app.use(bodyParser.json());
app.use(express.static(process.cwd() + '/dist/'));
const wss = new WebSocket.Server({ server });
var websockets = [];


app.get('/db/Spots/all', (req, res) => {
    db.Spot.findAll()
        .then(spots => {
            res.status(200).send(spots);
        })
        .catch(err => {
            res.status(500).send(err);
        });
});

app.post('/db/Spots/save', (req, res) => {
    db.Spot.create({
        id: req.body.id,
        exercise: req.body.exercise,
        active: req.body.active
    }).then(spot => {
        res.status(200).send(spot);
    }).catch(err => {
        res.status(500).send(err);
    });
});

app.post('/db/Spots/:id/update', (req, res) => {
    db.Spot.update({ exercise: req.body.exercise, active: req.body.active },
        {
            where: {
                id: req.params.id
            }
        }).then(spot => {
            res.status(200).send(spot);
        }).catch(err => {
            res.status(500).send(err);
        });
});

rosnodejs.initNode('/digi_gym')
const nh = rosnodejs.nh;

nh.subscribe('/station_usage', StationUsage, async (msg) => {
    const obj = {
        stationID: msg['stationID'],
        isActive: msg['isActive'],
        exerciseName: msg['exerciseName']
    };
    db.Spot.findOne({
        where: {
            id: msg['stationID']
        }
    }).then(spot => {
        if (!spot) {
            db.Spot.create({
                id: msg['stationID'],
                exercise: msg['exerciseName'],
                active: msg['isActive']
            }).then(spot => {
                refreshSpots();
            }).catch(err => {
                console.log(err);
            });
        } else {
            db.Spot.update({ exercise: msg['exerciseName'], active: msg['isActive'] },
                {
                    where: {
                        id: msg['stationID']
                    }
                }).then(spot => {
                    refreshSpots();
                }).catch(err => {
                    console.log(err);
                });
        }
    });
});

// Add new client connections
wss.on('connection', ws => {
    websockets.push(ws);
    ws.on('message', function incoming(message) {
        console.log('received: %s', message);
    });
    refreshSpots();
});

function refreshSpots() {
    db.Spot.findAll().then(spots => {
        websockets.forEach(ws => {
            console.log(spots);
            ws.send(JSON.stringify(spots));
        });
    }).catch(err => {
        console.log(err);
    });

}