import rosnodejs from 'rosnodejs';

import smoother from './lib/smoother';
import {wait} from './lib/utils';

const {Twist} = rosnodejs.require('geometry_msgs').msg;
const {Joy} = rosnodejs.require('sensor_msgs').msg;
const {Bool, Int8, UInt8} = rosnodejs.require('std_msgs').msg;

const state = {
  power: false,
  axes: [],
  buttons: {prev: Array(12).fill(0), cur: Array(12).fill(0)},
  geregePhase: 0, // FXXK!!!!
};

const checkButtonChanged = num => state.buttons.prev[num] !== state.buttons.cur[num];

(async () => {
  const nodeHandle = await rosnodejs.initNode('/main');
  const robotPower = nodeHandle.advertise('robot_power', Bool);
  const cmdVel = nodeHandle.advertise('cmd_vel', Twist);
  const shagaiPick = nodeHandle.advertise('shagai_pick', Int8);
  const shagaiArm = nodeHandle.advertise('shagai_arm', Int8);
  const shagaiGrab = nodeHandle.advertise('shagai_grab', Bool);
  const shagaiThrow = nodeHandle.advertise('shagai_throw', Int8);
  const geregeActuator = nodeHandle.advertise('gerege_actuator', UInt8);
  await wait(500); // necessary
  const accLimV = await nodeHandle.getParam('/main/acc_lim_v');
  const accLimW = await nodeHandle.getParam('/main/acc_lim_w');
  const velLimV = await nodeHandle.getParam('/main/vel_lim_v');
  const velLimW = await nodeHandle.getParam('/main/vel_lim_w');
  const decelFactor = await nodeHandle.getParam('/main/decel_factor');
  const linearXSmoother = smoother(accLimV, decelFactor);
  const linearYSmoother = smoother(accLimV, decelFactor);
  const angularSmoother = smoother(accLimW);
  nodeHandle.subscribe('joy', Joy, ({axes, buttons}) => {
    state.axes = axes; // eslint-disable-line fp/no-mutation
    state.buttons.cur = buttons; // eslint-disable-line fp/no-mutation
  });
  await wait(500);
  setInterval(() => {
    const buttons = state.buttons.cur;
    if (checkButtonChanged(9) && buttons[9]) {
      state.power = !state.power; // eslint-disable-line fp/no-mutation
      robotPower.publish({data: state.power});
    }
    if (checkButtonChanged(2) || checkButtonChanged(1)) {
      shagaiPick.publish({data: buttons[2] ? 1 : buttons[1] ? -1 : 0});
    }
    if (checkButtonChanged(3) || checkButtonChanged(0)) {
      shagaiArm.publish({data: buttons[3] ? 1 : buttons[0] ? -1 : 0});
    }
    if (checkButtonChanged(5)) {
      shagaiGrab.publish({data: buttons[5]});
    }
    if (checkButtonChanged(6) || checkButtonChanged(7)) {
      shagaiThrow.publish({data: buttons[7] ? 1 : buttons[6] ? -1 : 0});
    }
    if (checkButtonChanged(4)) {
      switch (state.geregePhase) {
        case 0:
          if (buttons[4]) {
            geregeActuator.publish({data: 0});
          }
          break;
        case 1:
          if (buttons[4]) {
            geregeActuator.publish({data: 0b1000});
          }
          break;
        case 2:
          if (buttons[4]) {
            geregeActuator.publish({data: 0b1010});
          }
          break;
        case 3:
          if (buttons[4]) {
            geregeActuator.publish({data: 0b1001});
          }
          break;
        case 4:
          if (buttons[4]) {
            geregeActuator.publish({data: 0b0101});
          }
          break;
        case 5:
          if (buttons[4]) {
            geregeActuator.publish({data: 0b0110});
          }
          break;
        default:
          break;
      }
      if (!buttons[4]) {
        state.geregePhase = (state.geregePhase + 1) % 6; // eslint-disable-line fp/no-mutation
      }
    }
    state.buttons.prev = state.buttons.cur; // eslint-disable-line fp/no-mutation
    if (state.power) {
      const [lx, ly, rx, , px, py] = state.axes;
      // const lx2 = lx ** 2;
      // const ly2 = ly ** 2;
      // const r2 = lx2 + ly2;
      const x = py
        ? linearXSmoother(velLimV / 2) * py
        : linearXSmoother(velLimV * Math.abs(ly)) * (ly > 0 ? 1 : -1);
      // : linearXSmoother(r2 ? velLimV * (ly2 / r2) : 0) * (ly > 0 ? 1 : -1);
      const y = px
        ? linearYSmoother(velLimV / 2) * px
        : linearYSmoother(velLimV * Math.abs(lx)) * (lx > 0 ? 1 : -1);
      // : linearYSmoother(r2 ? velLimV * (lx2 / r2) : 0) * (lx > 0 ? 1 : -1);
      const w = angularSmoother(velLimW * rx);
      cmdVel.publish(
        new Twist({
          linear: {x, y, z: 0},
          angular: {x: 0, y: 0, z: w},
        }),
      );
    }
  }, 16);
})();
