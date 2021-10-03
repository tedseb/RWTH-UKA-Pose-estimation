const { DataTypes, Utils } = require('sequelize');

module.exports = (sequelize) => {
    sequelize.define('exercise', {
        id: {
            allowNull: false,
            autoIncrement: true,
            primaryKey: true,
            type: DataTypes.INTEGER
        },
        exerciseId: {
            allowNull: false,
            type: DataTypes.INTEGER
        },
        
    },
    { timestamps: false });
};