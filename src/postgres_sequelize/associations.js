function associate(sequelize) {
    frame_station_onetomany(sequelize);
    station_camera_mapping(sequelize);
    station_usage_station_onetomany(sequelize);
    digigym_user_digigym_role_manytomany(sequelize);
}

function frame_station_onetomany(sequelize) {
    const {frame, station} = sequelize.models;
    station.hasMany(frame);
    frame.belongsTo(station);
}

function station_camera_mapping(sequelize) {
    const {camera, station} = sequelize.models;
    station.belongsToMany(camera, {through: "camera_station_mapping"});
    camera.belongsToMany(station, {through: "camera_station_mapping"});
}

function station_usage_station_onetomany(sequelize) {
    const {station_usage, station} = sequelize.models;
    station.hasMany(station_usage);
    station_usage.belongsTo(station);
}

function digigym_user_digigym_role_manytomany(sequelize) {
    const {dguser, dgrole} = sequelize.models;
    dgrole.belongsToMany(dguser, {through: "user_roles"});
    dguser.belongsToMany(dgrole, {through: "user_roles"});
}
module.exports = {associate};