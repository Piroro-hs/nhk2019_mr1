import rosnodejs from 'rosnodejs';
import SerialPort from 'serialport';

import {wait} from './lib/utils';

const SQRT_2 = 2 ** 0.5;

const wheelDiameter = 127;
const wheelbaseDiameter = 670.33318806;
const pulse = 500 * 4;
const velToPulseScale = (1 / (wheelDiameter * Math.PI)) * pulse;

const {Twist} = rosnodejs.require('geometry_msgs').msg;
const {Bool} = rosnodejs.require('std_msgs').msg;

const state = {
  enabled: false,
  vx: 0,
  vy: 0,
  omega: 0,
};

const openSerialPort = path =>
  new Promise((resolve, reject) => {
    const port = new SerialPort(path, {
      autoOpen: false,
      baudRate: 1000000,
      highWaterMark: 1024,
    });
    port.open(err => {
      if (err) {
        reject(err);
      } else {
        resolve(port);
      }
    });
  });

(async () => {
  const nodeHandle = await rosnodejs.initNode('/omni');
  await wait(500); // necessary
  const frPort = await nodeHandle.getParam('/omni/fr_port').then(openSerialPort);
  const brPort = await nodeHandle.getParam('/omni/br_port').then(openSerialPort);
  const flPort = await nodeHandle.getParam('/omni/fl_port').then(openSerialPort);
  const blPort = await nodeHandle.getParam('/omni/bl_port').then(openSerialPort);
  frPort.once('data', () => {
    rosnodejs.log.info('FR connected');
  });
  brPort.once('data', () => {
    rosnodejs.log.info('BR connected');
  });
  flPort.once('data', () => {
    rosnodejs.log.info('FL connected');
  });
  blPort.once('data', () => {
    rosnodejs.log.info('BL connected');
  });
  const kp = await nodeHandle.getParam('/omni/kp');
  const ki = await nodeHandle.getParam('/omni/ki');
  const kd = await nodeHandle.getParam('/omni/kd');
  [frPort, brPort, flPort, blPort].forEach(port => {
    port.write(`S kp ${kp.toFixed(9)}\n`);
    port.write(`S ki ${ki.toFixed(9)}\n`);
    port.write(`S kd ${kd.toFixed(9)}\n`);
  });
  await wait(500);
  nodeHandle.subscribe('robot_power', Bool, ({data}) => {
    state.enabled = data; // eslint-disable-line fp/no-mutation
  });
  nodeHandle.subscribe('cmd_vel', Twist, ({linear, angular}) => {
    state.vx = linear.x * 1000; // eslint-disable-line fp/no-mutation
    state.vy = linear.y * 1000; // eslint-disable-line fp/no-mutation
    state.omega = angular.z; // eslint-disable-line fp/no-mutation
  });
  setInterval(() => {
    if (state.enabled) {
      const {vx, vy, omega} = state;
      const fr = ((-vx - vy) / SQRT_2 - wheelbaseDiameter * omega) * velToPulseScale;
      const br = ((-vx + vy) / SQRT_2 - wheelbaseDiameter * omega) * velToPulseScale;
      const fl = ((vx - vy) / SQRT_2 - wheelbaseDiameter * omega) * velToPulseScale;
      const bl = ((vx + vy) / SQRT_2 - wheelbaseDiameter * omega) * velToPulseScale;
      frPort.write(`V ${fr.toFixed(5)}\n`);
      brPort.write(`V ${br.toFixed(5)}\n`);
      flPort.write(`V ${fl.toFixed(5)}\n`);
      blPort.write(`V ${bl.toFixed(5)}\n`);
    } else {
      [frPort, brPort, flPort, blPort].forEach(port => {
        port.write('R 0\n');
      });
    }
  }, 16);
})();
