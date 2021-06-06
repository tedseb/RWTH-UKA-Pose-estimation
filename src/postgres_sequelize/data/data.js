const { sequelize } = require('../database');

// const webcam0 = sequelize.models.camera.create({ name: 'webcam0' });
// const youtube0 = sequelize.models.camera.create({ name: 'youtube0' });


// const station1 = sequelize.models.station.create({ cameraId: 1, name: 'Power Rack' });
// const station2 = sequelize.models.station.create({ cameraId: 2, name: 'office_ted' });
// const station3 = sequelize.models.station.create({ cameraId: 2, name: 'office_orhan' });
// const station4 = sequelize.models.station.create({ cameraId: 2, name: 'ted_gym_test1' });

// const frame1 = sequelize.models.frame.create({ stationId: 1, frame_box: [[1158, 640], [493, 100]] });
// const frame2 = sequelize.models.frame.create({ stationId: 2, frame_box: [[456, 470], [107, 34]] });
// const frame3 = sequelize.models.frame.create({ stationId: 2, frame_box: [[1126, 691], [681, 57]] });
// const frame4 = sequelize.models.frame.create({ stationId: 2, frame_box: [[1158, 670], [549, 100]] });

(async () => {

    const webcam0 = await sequelize.models.camera.create({ name: 'webcam0' });
    const youtube0 = await sequelize.models.camera.create({ name: 'youtube0' });


    const station1 = await sequelize.models.station.create({ cameraId: 1, name: 'Power Rack' });
    const station2 = await sequelize.models.station.create({ cameraId: 2, name: 'office_ted' });
    const station3 = await sequelize.models.station.create({ cameraId: 2, name: 'office_orhan' });
    const station4 = await sequelize.models.station.create({ cameraId: 2, name: 'ted_gym_test1' });

    const frame1 = await sequelize.models.frame.create({ stationId: 1, frame_box: [[1158, 640], [493, 100]] });
    const frame2 = await sequelize.models.frame.create({ stationId: 2, frame_box: [[456, 470], [107, 34]] });
    const frame3 = await sequelize.models.frame.create({ stationId: 2, frame_box: [[1126, 691], [681, 57]] });
    const frame4 = await sequelize.models.frame.create({ stationId: 2, frame_box: [[1158, 670], [549, 100]] });

    const role1 = await sequelize.models.dgrole.create({id: 1, role: "user"});
    const role2 = await sequelize.models.dgrole.create({id: 2, role: "moderator"});
    const role3 = await sequelize.models.dgrole.create({id: 3, role: "admin"});


})();