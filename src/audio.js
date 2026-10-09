// Motor sound and race cues, synthesised with Web Audio (no sound files to load).
// Browser only; the numbers that drive it come from motorTone() in fx.js.

export function makeAudio() {
  const AC = globalThis.AudioContext ?? globalThis.webkitAudioContext;
  let ctx = null, master, motor, noise;

  function build() {
    ctx = new AC();
    master = ctx.createGain();
    master.gain.value = 0;
    const comp = ctx.createDynamicsCompressor();
    master.connect(comp).connect(ctx.destination);

    // Four motors, slightly detuned against each other so they beat like a real quad.
    // Sawtooths through a low-pass give the buzzy whine; the cutoff opens with rotor speed.
    const filter = ctx.createBiquadFilter();
    filter.type = 'lowpass';
    filter.Q.value = 4;
    const mGain = ctx.createGain();
    mGain.gain.value = 0;
    filter.connect(mGain).connect(master);
    const oscs = [1, 1.013, 0.991, 1.021].map((k, i) => {
      const o = ctx.createOscillator();
      o.type = i % 2 ? 'sawtooth' : 'square';
      const g = ctx.createGain();
      g.gain.value = i % 2 ? 0.22 : 0.08;
      o.connect(g).connect(filter);
      o.start();
      return { o, k };
    });
    // blade-pass harmonic: two blades per prop
    const blade = ctx.createOscillator();
    blade.type = 'triangle';
    const bg = ctx.createGain();
    bg.gain.value = 0.12;
    blade.connect(bg).connect(mGain);
    blade.start();
    motor = { filter, gain: mGain, oscs, blade };

    // rushing air: looped noise through a band-pass, louder and brighter with speed
    const buf = ctx.createBuffer(1, ctx.sampleRate * 2, ctx.sampleRate);
    const data = buf.getChannelData(0);
    for (let i = 0; i < data.length; i++) data[i] = Math.random() * 2 - 1;
    const src = ctx.createBufferSource();
    src.buffer = buf;
    src.loop = true;
    const bp = ctx.createBiquadFilter();
    bp.type = 'bandpass';
    bp.Q.value = 0.7;
    const nGain = ctx.createGain();
    nGain.gain.value = 0;
    src.connect(bp).connect(nGain).connect(master);
    src.start();
    noise = { buf, bp, gain: nGain };
  }

  const api = {
    on: false,
    supported: !!AC,
    // Must be called from a user gesture (browsers keep audio locked until one).
    setOn(on) {
      if (!AC) return false;
      api.on = on;
      if (on && !ctx) build();
      if (!ctx) return on;
      if (on) ctx.resume();
      master.gain.setTargetAtTime(on ? 0.5 : 0, ctx.currentTime, 0.05);
      if (!on) setTimeout(() => { if (!api.on) ctx.suspend(); }, 300);
      return on;
    },
    // tone = { freq, gain, air } from motorTone(); called every frame
    update(tone) {
      if (!api.on || !ctx) return;
      const t = ctx.currentTime, tc = 0.04;
      for (const { o, k } of motor.oscs) o.frequency.setTargetAtTime(tone.freq * k, t, tc);
      motor.blade.frequency.setTargetAtTime(tone.freq * 2, t, tc);
      motor.filter.frequency.setTargetAtTime(600 + tone.freq * 6, t, tc);
      motor.gain.gain.setTargetAtTime(0.32 * tone.gain, t, tone.gain ? tc : 0.15);
      noise.gain.gain.setTargetAtTime(0.02 + 0.3 * tone.air, t, 0.1);
      noise.bp.frequency.setTargetAtTime(500 + 2500 * tone.air, t, 0.1);
    },
    // short cues: 'gate', 'lap', 'crash'
    cue(kind) {
      if (!api.on || !ctx) return;
      const t = ctx.currentTime;
      if (kind === 'crash') {
        const src = ctx.createBufferSource();
        src.buffer = noise.buf;
        const lp = ctx.createBiquadFilter();
        lp.type = 'lowpass';
        lp.frequency.setValueAtTime(2400, t);
        lp.frequency.exponentialRampToValueAtTime(120, t + 0.6);
        const g = ctx.createGain();
        g.gain.setValueAtTime(0.9, t);
        g.gain.exponentialRampToValueAtTime(0.001, t + 0.7);
        src.connect(lp).connect(g).connect(master);
        src.start(t, Math.random());
        src.stop(t + 0.75);
        return;
      }
      const notes = kind === 'lap' ? [880, 1320] : [1175];
      notes.forEach((f, i) => {
        const o = ctx.createOscillator(), g = ctx.createGain(), t0 = t + i * 0.09;
        o.type = 'sine';
        o.frequency.value = f;
        g.gain.setValueAtTime(0, t0);
        g.gain.linearRampToValueAtTime(0.18, t0 + 0.008);
        g.gain.exponentialRampToValueAtTime(0.001, t0 + 0.18);
        o.connect(g).connect(master);
        o.start(t0);
        o.stop(t0 + 0.2);
      });
    },
  };
  // go quiet when the tab is hidden
  document.addEventListener('visibilitychange', () => {
    if (!ctx || !api.on) return;
    if (document.hidden) ctx.suspend(); else ctx.resume();
  });
  return api;
}
