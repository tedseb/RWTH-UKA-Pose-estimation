const { DataTypes } = require('sequelize');

module.exports = (sequelize) => {
    sequelize.define('dguser', {
        id: {
            type: DataTypes.INTEGER,
            primaryKey: true,
            autoIncrement: true
        },
        username: {
            allowNull: false,
            type: DataTypes.STRING(40),
            unique: true
        },
        email: {
            allowNull: false,
            type: DataTypes.STRING(),
            unique: true
        },
        password: {
            allowNull: false,
            type: DataTypes.STRING()
        }
    });
};