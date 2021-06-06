const { DataTypes, Utils } = require('sequelize');

module.exports = (sequelize) => {
    sequelize.define('station_usage', {
        id: {
            allowNull: false,
            autoIncrement: true,
            primaryKey: true,
            type: DataTypes.INTEGER
        },
        status: {
            allowNull: false,
            type: DataTypes.BOOLEAN
        },
        exercise: {
            allowNull: false,
            type: DataTypes.STRING
        }
    });
};
