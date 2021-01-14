const http = require('http');
const rosnodejs = require('rosnodejs');

const hostname = '127.0.0.1';
const port = 3000;


rosnodejs.initNode('/DasTorZurUnterwelt')
.then(() => {
  // do stuff
});

const nh = rosnodejs.nh;

const server = http.createServer((req, res) => {
  res.statusCode = 200;
  res.setHeader('Content-Type', 'text/plain');
  //res.end('Hello World');
  //console.log('Got msg on chatter bevor');
  const sub = nh.subscribe('/personsJS', 'pose_estimation/Persons', (msg) => {
    //console.log('Got msg on chatter: %j', msg);
    res.end(JSON.stringify(msg));

  });
});

server.listen(port, hostname, () => {
  console.log(`Server running at http://${hostname}:${port}/`);
});
