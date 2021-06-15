const { DataTypes, Utils } = require('sequelize');

module.exports = (sequelize) => {
    sequelize.define('station', {
        id: {
            allowNull: false,
            autoIncrement: true,
            primaryKey: true,
            type: DataTypes.INTEGER
        },
        name: {
            allowNull: false,
            type: DataTypes.STRING(40),
            unique: true
        }
    });
};
