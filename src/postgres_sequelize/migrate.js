var db = require('./database.js');
db.sequelize.sync({force: false});