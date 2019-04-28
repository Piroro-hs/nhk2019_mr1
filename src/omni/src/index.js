import rosnodejs from 'rosnodejs';
import SerialPort from 'serialport';
import qte from 'quaternion-to-euler';

import pid from './lib/pid';
import {wait} from './lib/utils';

const SQRT_2 = 2 ** 0.5;

const wheelDiameter = 127;
const wheelbaseRadius = 670.33318806 / 2;
const pulse = 500 * 4;
const velToPulseScale = (1 / (wheelDiameter * Math.PI)) * pulse;

const {Twist, Quaternion} = rosnodejs.require('geometry_msgs').msg;
// const {Imu} = rosnodejs.require('sensor_msgs').msg;
const {Bool} = rosnodejs.require('std_msgs').msg;

const state = {
  enabled: false,
  vx: 0,
  vy: 0,
  omega: 0,
  targetAngle: 0,
  currentAngle: 0,
  prevAngle: 0,
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
  // const anglePid = pid(20, 0.001, 150, 0.5);
  const anglePid = pid(20, 0.2, 200, 0.5);
  nodeHandle.subscribe('robot_power', Bool, ({data}) => {
    state.enabled = data; // eslint-disable-line fp/no-mutation
    state.targetAngle = state.currentAngle; // eslint-disable-line fp/no-mutation
    state.prevAngle = state.currentAngle; // eslint-disable-line fp/no-mutation
    anglePid.reset();
  });
  nodeHandle.subscribe('cmd_vel', Twist, ({linear, angular}) => {
    state.vx = linear.x * 1000; // eslint-disable-line fp/no-mutation
    state.vy = linear.y * 1000; // eslint-disable-line fp/no-mutation
    state.omega = angular.z; // eslint-disable-line fp/no-mutation
  });
  nodeHandle.subscribe('imu', Quaternion, ({x, y, z, w}) => {
    [state.currentAngle] = qte([x, y, z, w]); // eslint-disable-line fp/no-mutation
  });
  await wait(500);
  setInterval(() => {
    if (state.enabled) {
      const {vx, vy, omega} = state;
      state.targetAngle += omega * 0.02; // eslint-disable-line fp/no-mutation
      const correctedOmega =
        state.prevAngle === state.currentAngle &&
        Math.abs(state.targetAngle - state.currentAngle) < 0.02
          ? omega
          : anglePid.run(state.targetAngle - state.currentAngle);
      // if (
      //   state.prevAngle === state.currentAngle &&
      //   Math.abs(state.targetAngle - state.currentAngle) < 0.02
      // ) {
      //   state.targetAngle = state.currentAngle; // eslint-disable-line fp/no-mutation
      //   anglePid.reset();
      // }
      // const correctedOmega = anglePid.run(state.targetAngle - state.currentAngle);
      const fr = ((-vx - vy) / SQRT_2 - wheelbaseRadius * correctedOmega) * velToPulseScale;
      const br = ((-vx + vy) / SQRT_2 - wheelbaseRadius * correctedOmega) * velToPulseScale;
      const fl = ((vx - vy) / SQRT_2 - wheelbaseRadius * correctedOmega) * velToPulseScale;
      const bl = ((vx + vy) / SQRT_2 - wheelbaseRadius * correctedOmega) * velToPulseScale;
      frPort.write(`V ${fr.toFixed(5)}\n`);
      brPort.write(`V ${br.toFixed(5)}\n`);
      flPort.write(`V ${fl.toFixed(5)}\n`);
      blPort.write(`V ${bl.toFixed(5)}\n`);
      state.prevAngle = state.currentAngle; // eslint-disable-line fp/no-mutation
      console.log({state, correctedOmega});
    } else {
      [frPort, brPort, flPort, blPort].forEach(port => {
        port.write('R 0\n');
      });
    }
  }, 20);
})();
