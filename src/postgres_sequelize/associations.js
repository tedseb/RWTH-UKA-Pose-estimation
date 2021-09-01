function associate(sequelize) {
    frame_station_onetomany(sequelize);
    frame_station_onetomany(sequelize)
/*     station_camera_mapping_camera(sequelize);
    station_camera_mapping_station(sequelize); */
    station_usage_station_onetomany(sequelize);
    digigym_user_digigym_role_manytomany(sequelize);
    camera_station_mapping(sequelize);
}

function frame_station_onetomany(sequelize) {
    const {frame, camera_station_mapping} = sequelize.models;
    camera_station_mapping.hasMany(frame);
    frame.belongsTo(camera_station_mapping);
}

function frame_station_onetomany(sequelize) {
    const {station_weight_color, camera_station_mapping} = sequelize.models;
    camera_station_mapping.hasMany(station_weight_color);
    station_weight_color.belongsTo(camera_station_mapping);
}

/* function station_camera_mapping_camera(sequelize) {
    const {camera_station_mapping, camera} = sequelize.models;
    camera.hasMany(camera_station_mapping);
    camera_station_mapping.belongsTo(camera);
}

function station_camera_mapping_station(sequelize) {
    const {camera_station_mapping, station} = sequelize.models;
    station.hasMany(camera_station_mapping);
    camera_station_mapping.belongsTo(station);
} */

function camera_station_mapping(sequelize) {
    const {camera, station, camera_station_mapping} = sequelize.models;
    station.belongsToMany(camera, {through: {model: camera_station_mapping}});
    camera.belongsToMany(station, {through: {model: camera_station_mapping}});
    station.hasMany(camera_station_mapping);
    camera_station_mapping.belongsTo(station);
    camera.hasMany(camera_station_mapping);
    camera_station_mapping.belongsTo(camera);
}

function digigym_user_digigym_role_manytomany(sequelize) {
    const {dguser, dgrole} = sequelize.models;
    dgrole.belongsToMany(dguser, {through: "user_roles"});
    dguser.belongsToMany(dgrole, {through: "user_roles"});
}

function station_usage_station_onetomany(sequelize) {
    const {station_usage, station} = sequelize.models;
    station.hasMany(station_usage);
    station_usage.belongsTo(station);
}

function station_usage_station_onetomany(sequelize) {
    const {station_usage, station} = sequelize.models;
    station.hasMany(station_usage);
    station_usage.belongsTo(station);
}


module.exports = {associate};