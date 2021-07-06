const { DataTypes } = require('sequelize');

module.exports = (sequelize) => {
    sequelize.define('emergency', {
        id: {
            allowNull: false,
            autoIncrement: true,
            primaryKey: true,
            type: DataTypes.INTEGER
        },
        message: {
            allowNull: false,
            type: DataTypes.STRING(500),
            unique: true
        },
        done: {
            allowNull: false,
            type: DataTypes.BOOLEAN,
            defaultValue: false
        }
    });
};
