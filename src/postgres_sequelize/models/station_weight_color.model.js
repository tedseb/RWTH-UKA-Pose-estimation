const sequelize = require('sequelize');
const { DataTypes } = require('sequelize');

module.exports = (sequelize) => {
    sequelize.define('station_weight_color', {
        id: {
            allowNull: false,
            autoIncrement: true,
            primaryKey: true,
            type: DataTypes.INTEGER
        },
        name: {
            allowNull: false,
            type: DataTypes.STRING(40),
            unique: true,
        },
        weight: {
            type: DataTypes.FLOAT,
            allowNull: false
        },
        hsv_low: {
            type: DataTypes.ARRAY(DataTypes.INTEGER),
            allowNull: false
        },
        hsv_high: {
            type: DataTypes.ARRAY(DataTypes.INTEGER),
            allowNull: false
        }
    });
};