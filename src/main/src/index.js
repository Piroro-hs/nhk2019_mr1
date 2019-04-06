import rosnodejs from 'rosnodejs';

import {wait} from './lib/utils';

const {Twist} = rosnodejs.require('geometry_msgs').msg;
const {Joy} = rosnodejs.require('sensor_msgs').msg;
const {Bool, Int8} = rosnodejs.require('std_msgs').msg;

const state = {
  power: false,
  axes: [],
  buttons: {prev: [], cur: []},
  phase: 0, // FXXK!!!!
};

const checkButtonChanged = num => state.buttons.prev[num] !== state.buttons.cur[num];

(async () => {
  const nodeHandle = await rosnodejs.initNode('/main');
  const robotPower = nodeHandle.advertise('robot_power', Bool);
  const cmdVel = nodeHandle.advertise('cmd_vel', Twist);
  const shagaiPickUp = nodeHandle.advertise('shagai_pick_up', Bool);
  const shagaiPickDown = nodeHandle.advertise('shagai_pick_down', Bool);
  const shagaiArmExtend = nodeHandle.advertise('shagai_arm_extend', Bool);
  const shagaiArmCollapse = nodeHandle.advertise('shagai_arm_collapse', Bool);
  const shagaiPick = nodeHandle.advertise('shagai_pick', Bool);
  const shagaiThrow = nodeHandle.advertise('shagai_throw', Int8);
  const geregeArmMove = nodeHandle.advertise('gerege_arm_move', Bool);
  const geregeGrab = nodeHandle.advertise('gerege_grab', Int8);
  await wait(500); // necessary
  nodeHandle.subscribe('joy', Joy, ({axes, buttons}) => {
    state.axes = axes; // eslint-disable-line fp/no-mutation
    state.buttons.cur = buttons; // eslint-disable-line fp/no-mutation
  });
  setInterval(() => {
    const buttons = state.buttons.cur;
    if (checkButtonChanged(9) && buttons[9]) {
      state.power = !state.power; // eslint-disable-line fp/no-mutation
      robotPower.publish({data: state.power});
    }
    if (checkButtonChanged(2)) {
      shagaiPickUp.publish({data: buttons[2]});
    }
    if (checkButtonChanged(1)) {
      shagaiPickDown.publish({data: buttons[1]});
    }
    if (checkButtonChanged(3)) {
      shagaiArmExtend.publish({data: buttons[3]});
    }
    if (checkButtonChanged(0)) {
      shagaiArmCollapse.publish({data: buttons[0]});
    }
    if (checkButtonChanged(5)) {
      shagaiPick.publish({data: buttons[5]});
    }
    if (checkButtonChanged(6) || checkButtonChanged(7)) {
      shagaiThrow.publish({data: buttons[7] ? 1 : buttons[6] ? -1 : 0});
    }
    if (checkButtonChanged(4)) {
      switch (state.phase) {
        case 0:
          geregeGrab.publish({data: buttons[4] ? 1 : 0});
          break;
        case 1:
          if (buttons[4]) {
            geregeArmMove.publish({data: true});
          }
          break;
        case 2:
          geregeGrab.publish({data: buttons[4] ? -1 : 0});
          break;
        default:
          break;
      }
      if (!buttons[4]) {
        state.phase = (state.phase + 1) % 3; // eslint-disable-line fp/no-mutation
      }
    }
    // if (state.power) {
    // } else {
    // }
    state.buttons.prev = state.buttons.cur; // eslint-disable-line fp/no-mutation
    if (state.power) {
      const [lx, ly, rx] = state.axes;
      cmdVel.publish(
        new Twist({
          linear: {x: ly * 2, y: lx * 2, z: 0},
          angular: {x: 0, y: 0, z: (rx * Math.PI) / 2},
        }),
      );
    }
  }, 16);
})();
