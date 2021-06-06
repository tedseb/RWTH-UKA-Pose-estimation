const { DataTypes } = require('sequelize');

module.exports = (sequelize) => {
    sequelize.define('dgrole', {
        id: {
            type: DataTypes.INTEGER,
            primaryKey: true
        }, 
        role: {
            type: DataTypes.STRING,
            allowNull: false
        }
    });
};