const db = require('../../postgres_sequelize/database');
const ROLES = db.DigiGymRoles;
const User = db.DigiGymUsers;

checkDuplicateUsernameOrEmail = (req, res, next) => {
    console.log(req.body);
    // Username
    User.findOne({
      where: {
        username: req.body.username
      }
    }).then(user => {
      if (user) {
        res.status(400).send({
          message: "Failed! Username is already in use!"
        });
        return;
      }
  
      // Email
      User.findOne({
        where: {
          email: req.body.email
        }
      }).then(user => {
        if (user) {
          res.status(400).send({
            message: "Failed! Email is already in use!"
          });
          return;
        }
  
        next();
      });
    });
  };
  
  checkRolesExisted = async (req, res, next) => {
    if (req.body.roles) {
      const role_entities = await ROLES.findAll();
      const roles = role_entities.map(x => x.dataValues.role);
      console.log(roles);
      for (let role of req.body.roles) {
        if(!roles.includes(role)) {
          res.status(400).send({
            message: "Failed! Role does not exist = " + role
          });
          return;
        }
      }
    }
    
    next();
  };
  
  const verifySignUp = {
    checkDuplicateUsernameOrEmail: checkDuplicateUsernameOrEmail,
    checkRolesExisted: checkRolesExisted
  };
  
  module.exports = verifySignUp;