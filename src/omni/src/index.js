import rosnodejs from 'rosnodejs';
import SerialPort from 'serialport';

import pid from './lib/pid';
import {wait} from './lib/utils';

const DIV_BY_SQRT_2 = 1 / 2 ** 0.5;

const wheelDiameter = 127;
const wheelbaseRadius = 670.33318806 / 2;
const pulse = 500 * 4;
const velToPulseScale = (1 / (wheelDiameter * Math.PI)) * pulse;

const {Twist} = rosnodejs.require('geometry_msgs').msg;
const {Bool, Float32} = rosnodejs.require('std_msgs').msg;

const state = {
  enable: false,
  vx: 0,
  vy: 0,
  omega: 0,
  angleCorrection: {
    enable: true,
    target: 0,
    current: 0,
    prev: 0,
  },
};

// const openSerialPort = path => ({write: console.log, once: console.log});
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
  const kp = await nodeHandle.getParam('/omni/motor_kp');
  const ki = await nodeHandle.getParam('/omni/motor_ki');
  const kd = await nodeHandle.getParam('/omni/motor_kd');
  [frPort, brPort, flPort, blPort].forEach(port => {
    port.write(`S kp ${kp.toFixed(9)}\n`);
    port.write(`S ki ${ki.toFixed(9)}\n`);
    port.write(`S kd ${kd.toFixed(9)}\n`);
  });
  state.angleCorrection.enable = await nodeHandle // eslint-disable-line fp/no-mutation
    .getParam('/omni/angle_correction')
    .catch(() => state.angleCorrection.enable);
  // const anglePid = pid(20, 0.001, 150, 0.5);
  const anglePid = pid(
    await nodeHandle.getParam('/omni/angle_kp').catch(() => 0),
    await nodeHandle.getParam('/omni/angle_ki').catch(() => 0),
    await nodeHandle.getParam('/omni/angle_kd').catch(() => 0),
    await nodeHandle.getParam('/omni/max_rot_vel').catch(() => 0),
  );
  nodeHandle.subscribe('robot_power', Bool, ({data}) => {
    state.enable = data; // eslint-disable-line fp/no-mutation
    state.angleCorrection.target = state.angleCorrection.current; // eslint-disable-line fp/no-mutation
    state.angleCorrection.prev = state.angleCorrection.current; // eslint-disable-line fp/no-mutation
    anglePid.reset();
  });
  nodeHandle.subscribe('cmd_vel', Twist, ({linear, angular}) => {
    state.vx = linear.x * 1000; // eslint-disable-line fp/no-mutation
    state.vy = linear.y * 1000; // eslint-disable-line fp/no-mutation
    state.omega = angular.z; // eslint-disable-line fp/no-mutation
  });
  nodeHandle.subscribe('angle', Float32, ({data}) => {
    state.angleCorrection.current = data; // eslint-disable-line fp/no-mutation
  });
  await wait(500);
  setInterval(() => {
    if (state.enable) {
      const {vx, vy, omega} = state;
      state.angleCorrection.target += omega * 0.016; // eslint-disable-line fp/no-mutation
      const correctedOmega =
        !state.angleCorrection.enable ||
        (state.angleCorrection.prev === state.angleCorrection.current &&
          Math.abs(state.angleCorrection.target - state.angleCorrection.current) < 0.05)
          ? omega
          : anglePid.run(state.angleCorrection.target - state.angleCorrection.current);
      // if (
      //   state.prevAngle === state.currentAngle &&
      //   Math.abs(state.targetAngle - state.currentAngle) < 0.02
      // ) {
      //   state.targetAngle = state.currentAngle; // eslint-disable-line fp/no-mutation
      //   anglePid.reset();
      // }
      // const correctedOmega = anglePid.run(state.targetAngle - state.currentAngle);
      const fr = ((-vx - vy) * DIV_BY_SQRT_2 - wheelbaseRadius * correctedOmega) * velToPulseScale;
      const br = ((-vx + vy) * DIV_BY_SQRT_2 - wheelbaseRadius * correctedOmega) * velToPulseScale;
      const fl = ((vx - vy) * DIV_BY_SQRT_2 - wheelbaseRadius * correctedOmega) * velToPulseScale;
      const bl = ((vx + vy) * DIV_BY_SQRT_2 - wheelbaseRadius * correctedOmega) * velToPulseScale;
      frPort.write(`V ${fr.toFixed(5)}\n`);
      brPort.write(`V ${br.toFixed(5)}\n`);
      flPort.write(`V ${fl.toFixed(5)}\n`);
      blPort.write(`V ${bl.toFixed(5)}\n`);
      console.log({fr, br, fl, bl, state, correctedOmega});
      state.angleCorrection.prev = state.angleCorrection.current; // eslint-disable-line fp/no-mutation
    } else {
      [frPort, brPort, flPort, blPort].forEach(port => {
        port.write('R 0\n');
      });
    }
  }, 16);
})();
