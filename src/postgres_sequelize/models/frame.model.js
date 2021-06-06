const { values } = require('lodash');
const { DataTypes, Utils } = require('sequelize');

createBoxDataType();

module.exports = (sequelize) => {
    sequelize.define('frame', {
        id: {
            allowNull: false,
            autoIncrement: true,
            primaryKey: true,
            type: DataTypes.INTEGER
        },
        frame_box: {
            allowNull: false,
            type: DataTypes.BOX
        }
    });
};


function createBoxDataType() {

    const ABSTRACT = DataTypes.ABSTRACT.prototype.constructor;

    class BOX extends ABSTRACT {
        toSql() {
            return 'box not null'
        }

        validate(value, options) {
            if (!Array.isArray(value)) return false;
            if (value.length != 2) return false;
            for (el of value) {
                if (!Array.isArray(value)) return false;
                if(value.length != 2) return false;
            }
            return true;
        }

        _stringify(value) {
            return `(${value[0][0]}, ${value[0][1]}),(${value[1][0]}, ${value[1][1]})`
        }

        static parse(value) {
            const match = value.match('/\d+/');
            return [[match[0], match[1]], [match[2], match[3]]];
        }
    }
    BOX.prototype.key = BOX.key = 'BOX';
    DataTypes.BOX = Utils.classToInvokable(BOX);
}

