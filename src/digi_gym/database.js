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


const Camera = sequelize.define('Camera', {
    id: {
        type: Sequelize.INTEGER,
        allowNull: false,
        primaeryKey: true 
    }
})

module.exports = {
    sequelize: sequelize,
    Spot: Spot
};

