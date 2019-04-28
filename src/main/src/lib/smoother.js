export default function smoother(accLim, decelFactor = 1) {
  const state = {prevVal: 0, prevTime: process.hrtime.bigint()};
  return target => {
    const curTime = process.hrtime.bigint();
    const dt = Number(curTime - state.prevTime) / 1000000;
    state.prevVal = // eslint-disable-line fp/no-mutation
      target >= state.prevVal
        ? Math.min(target, state.prevVal + accLim * dt)
        : Math.max(target, state.prevVal - accLim * decelFactor * dt);
    state.prevTime = curTime; // eslint-disable-line fp/no-mutation
    return state.prevVal;
  };
}
