const config = require('./config.json');


const Sequelize = require('sequelize');
const sequelize = new Sequelize(process.env.DB_SCHEMA || 'postgres',
    process.env.DB_USER || 'postgres',
    process.env.DB_PASSWORD || config.POSTGRES_PW,
    {
        host: process.env.DB_HOST || 'localhost',
        port: process.env.DB_PORT || config.POSTGRES_PORT,
        dialect: 'postgres',
        dialectOptions: {
            ssl: process.env.DB_SSL == "true"
        }
    });

const Spot = sequelize.define('Spot', {
    id: {
        type: Sequelize.INTEGER,
        allowNull: false,
        primaryKey: true
    },
    exercise: {
        type: Sequelize.STRING,
        allowNull: true
    },
    active: {
        type: Sequelize.BOOLEAN,
        allowNull: false
    }
});

function addSpot(id, exercise, active) {
    Spot.create({
        id: id,
        exercise: exercise,
        active: active
    })
    .then( spot => {
       return spot;
    })
    .catch( err => {
       return err;
    });
}

module.exports = {
    sequelize: sequelize,
    Spot: Spot,
    addSpot: addSpot
};

