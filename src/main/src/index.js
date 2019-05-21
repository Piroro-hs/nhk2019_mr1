import rosnodejs from 'rosnodejs';

import smoother from './lib/smoother';
import {wait} from './lib/utils';

const {Twist} = rosnodejs.require('geometry_msgs').msg;
const {Joy} = rosnodejs.require('sensor_msgs').msg;
const {Bool, UInt16} = rosnodejs.require('std_msgs').msg;

const DIV_BY_SQRT_2 = 1 / 2 ** 0.5;

const MASK_SP_F = 0x01 << 0;
const MASK_SP_B = 0x01 << 1;
const MASK_SA_F = 0x01 << 2;
const MASK_SA_B = 0x01 << 3;
const MASK_SG = 0x01 << 4;
const MASK_ST_F = 0x01 << 5;
const MASK_ST_B = 0x01 << 6;
const MASK_GA_F = 0x01 << 7;
const MASK_GA_B = 0x01 << 8;
const MASK_GG_F = 0x01 << 9;
const MASK_GG_B = 0x01 << 10;

const state = {
  power: false,
  axes: [],
  buttons: {prev: Array(12).fill(0), cur: Array(12).fill(0)},
  geregePhase: 0, // FXXK!!!!
};

const checkButtonChanged = num => state.buttons.prev[num] !== state.buttons.cur[num];
const checkButtonsChanged = nums => nums.some(checkButtonChanged);

(async () => {
  const nodeHandle = await rosnodejs.initNode('/main');
  const robotPower = nodeHandle.advertise('robot_power', Bool);
  const cmdVel = nodeHandle.advertise('cmd_vel', Twist);
  const actuator = nodeHandle.advertise('actuator', UInt16);
  await wait(500); // necessary
  const accLimV = await nodeHandle.getParam('/main/acc_lim_v');
  const accLimW = await nodeHandle.getParam('/main/acc_lim_w');
  const velLimV = await nodeHandle.getParam('/main/vel_lim_v');
  const velLimW = await nodeHandle.getParam('/main/vel_lim_w');
  const decelFactor = await nodeHandle.getParam('/main/decel_factor');
  const linearSmootherR = smoother(accLimV, decelFactor);
  const linearSmootherL = smoother(accLimV, decelFactor);
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
    if (checkButtonsChanged([0, 1, 2, 3, 4, 5, 6, 7])) {
      actuator.publish({
        data:
          (buttons[2] ? MASK_SP_F : buttons[1] ? MASK_SP_B : 0) |
          (buttons[3] ? MASK_SA_F : buttons[0] ? MASK_SA_B : 0) |
          (buttons[5] ? MASK_SG : 0) |
          (buttons[6] ? MASK_ST_F : buttons[7] ? MASK_ST_B : 0) |
          (checkButtonChanged(4) && buttons[4]
            ? state.geregePhase === 1
              ? MASK_GA_F
              : state.geregePhase === 2
              ? MASK_GA_F | MASK_GG_B
              : state.geregePhase === 3
              ? MASK_GA_F | MASK_GG_F
              : state.geregePhase === 4
              ? MASK_GA_B | MASK_GG_F
              : state.geregePhase === 5
              ? MASK_GA_B | MASK_GG_B
              : 0
            : 0),
      });
      if (checkButtonChanged(4) && !buttons[4]) {
        state.geregePhase = (state.geregePhase + 1) % 6; // eslint-disable-line fp/no-mutation
      }
    }
    state.buttons.prev = state.buttons.cur; // eslint-disable-line fp/no-mutation
    if (state.power) {
      const [lx, ly, rx, , px, py] = state.axes;
      const rawX = py ? velLimV * py : (velLimV / 2) * ly;
      const rawY = px ? velLimV * px : (velLimV / 2) * lx;
      const vr = linearSmootherR((rawX + rawY) * DIV_BY_SQRT_2);
      const vl = linearSmootherL((-rawX + rawY) * DIV_BY_SQRT_2);
      const x = (vr - vl) * DIV_BY_SQRT_2;
      const y = (vr + vl) * DIV_BY_SQRT_2;
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
