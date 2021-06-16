const config = require('./config.json');
const Sequelize = require('sequelize');
const {associate} = require('./associations');


const sequelize = new Sequelize('trainerai_db',
    process.env.DB_USER || config.POSTGRES_USER,
    process.env.DB_PASSWORD || config.POSTGRES_PW,
    {
        host: process.env.DB_HOST || config.POSTGRES_HOST,
        port: process.env.DB_PORT || config.POSTGRES_PORT,
        dialect: 'postgres',
        dialectOptions: {
            ssl: process.env.DB_SSL == "true"
        }
    });

const modelDefiners = [
    require('./models/camera.model'),
    require('./models/frame.model'),
    require('./models/station.model'),
    require('./models/camera_station_mapping.model'),
    require('./models/stations_usage.model'),
    require('./models/digigym/user.model'),
    require('./models/digigym/role.model')
    //require (./models/item.model)
]

for (const modelDefiner of modelDefiners) {
    modelDefiner(sequelize);
}

associate(sequelize);

module.exports = {
    sequelize: sequelize,
    Stations: sequelize.models.station,
    Cameras: sequelize.models.camera,
    Frames: sequelize.models.frame,
    StationUsages: sequelize.models.station_usage,
    DigiGymUsers: sequelize.models.dguser,
    DigiGymRoles: sequelize.models.dgrole
};