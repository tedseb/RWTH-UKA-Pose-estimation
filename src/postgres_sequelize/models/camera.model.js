const sequelize = require('sequelize');
const { DataTypes } = require('sequelize');

module.exports = (sequelize) => {
    sequelize.define('camera', {
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
        type: {
            type: DataTypes.INTEGER,
            allowNull: false
        },
        typeInfo: {
            type: DataTypes.STRING(16),
            allowNull: true
        }
    });
};