export default function smoother(accLim, decelFactor = 1) {
  const state = {prevVal: 0, prevTime: process.hrtime.bigint()};
  return target => {
    const curTime = process.hrtime.bigint();
    const dt = Number(curTime - state.prevTime) / 1000000;
    state.prevTime = curTime; // eslint-disable-line fp/no-mutation
    if (target > state.prevVal) {
      const dt1 = Math.min(Math.max(-state.prevVal, 0) / (accLim * decelFactor), dt);
      const dt2 = dt - dt1;
      state.prevVal = Math.min(target, state.prevVal + accLim * decelFactor * dt1 + accLim * dt2); // eslint-disable-line fp/no-mutation
    } else {
      const dt1 = Math.min(Math.max(state.prevVal, 0) / (accLim * decelFactor), dt);
      const dt2 = dt - dt1;
      state.prevVal = Math.max(target, state.prevVal - accLim * decelFactor * dt1 - accLim * dt2); // eslint-disable-line fp/no-mutation
    }
    return state.prevVal;
  };
}
