export default function pid(kp, ki, kd, velLim) {
  const gains = [kp + ki + kd, -kp - 2 * kd, kd];
  const state = {
    prevInput: 0,
    prevPrevInput: 0,
    prevOutput: 0,
  };
  return {
    run(input) {
      const output =
        gains[0] * input +
        gains[1] * state.prevInput +
        gains[2] * state.prevPrevInput +
        state.prevOutput;
      state.prevPrevInput = state.prevInput; // eslint-disable-line fp/no-mutation
      state.prevInput = input; // eslint-disable-line fp/no-mutation
      state.prevOutput = Math.min(Math.max(output, -velLim), velLim); // eslint-disable-line fp/no-mutation
      return state.prevOutput;
    },
    reset() {
      state.prevInput = 0; // eslint-disable-line fp/no-mutation
      state.prevPrevInput = 0; // eslint-disable-line fp/no-mutation
      state.prevOutput = 0; // eslint-disable-line fp/no-mutation
    },
  };
}
