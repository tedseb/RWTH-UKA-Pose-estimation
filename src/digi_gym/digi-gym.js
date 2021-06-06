const express = require('express');
const cors = require("cors");
const config = require('./config/config.json');
const bodyParser = require('body-parser');
var db = require('../postgres_sequelize/database');
const WebSocket = require('ws');





// Parameters and Constants:
const PORT = config.PORT;
const app = express();


const server = app.listen(PORT, () => { console.log("Listening on port " + PORT) })


var corsOptions = {
    origin: "http://localhost:8081"
};

app.use(cors(corsOptions));

app.use(bodyParser.urlencoded({ extended: false }));
app.use(bodyParser.json());

require('./routes/auth.routes')(app);
require('./routes/user.routes')(app);


app.use(express.static(process.cwd() + '/dist/'));
const wss = new WebSocket.Server({ server });
var websockets = [];