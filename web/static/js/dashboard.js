/**
 * ==============================================================================
 * Painel Web de Controle e Calibração (Ball and Plate 3-DoF)
 * JavaScript do Cliente: Sincronização Bidirecional, Gráfico Osciloscópio e REST
 * Autor: Miguel Carlos Costa Sousa — UFG
 * ==============================================================================
 */

// Funções Utilitárias Unificadas de Comunicação REST
function updateConfig(patchObj) {
  return fetch('/api/config', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(patchObj)
  });
}

function sendAction(action, params = {}) {
  return fetch('/api/action', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ action, ...params })
  });
}

// Navegação entre Abas
function switchTab(tabId) {
  document.querySelectorAll('.tab-nav-btn').forEach(b => b.classList.remove('active'));
  document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));

  const btn = document.getElementById('btn-' + tabId);
  if (btn) btn.classList.add('active');

  const content = document.getElementById(tabId);
  if (content) content.classList.add('active');
}

// 1. Alternância de Modo Operacional de Hardware (STANDALONE_RPI vs HYBRID_STM32)
function setOpMode(opMode) {
  fetch('/api/op_mode', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ op_mode: opMode })
  }).then(res => res.json())
    .then(data => {
      if (data.status === 'ok') {
        const btnSt = document.getElementById('btn-op-standalone');
        const btnHy = document.getElementById('btn-op-hybrid');
        if (opMode === 'STANDALONE_RPI') {
          if (btnSt) btnSt.classList.add('active');
          if (btnHy) btnHy.classList.remove('active');
        } else {
          if (btnSt) btnSt.classList.remove('active');
          if (btnHy) btnHy.classList.add('active');
        }
      }
    });
}

// 2. Alternância de Modo de Controle (AUTO vs MANUAL_TILT vs MANUAL_MOTOR)
function setControlMode(mode) {
  fetch('/api/mode', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ mode: mode })
  }).then(res => res.json())
    .then(data => {
      const badge = document.getElementById('badge-ctrl-mode-txt');
      if (badge) badge.innerText = 'MODO: ' + mode;
      const btnAuto = document.getElementById('btn-mode-auto');
      if (btnAuto) {
        if (mode === 'AUTO') {
          btnAuto.classList.remove('btn-emerald');
          btnAuto.classList.add('btn-cyan');
          btnAuto.innerText = '⏸ PAUSAR BALANCEAMENTO AUTO';
          btnAuto.setAttribute('onclick', "setControlMode('MANUAL_TILT')");
        } else {
          btnAuto.classList.add('btn-emerald');
          btnAuto.classList.remove('btn-cyan');
          btnAuto.innerText = '▶ INICIAR BALANCEAMENTO AUTO';
          btnAuto.setAttribute('onclick', "setControlMode('AUTO')");
        }
      }
    });
}

// Parada de Emergência e Nivelamento
function emergencyStop() {
  setControlMode('MANUAL_TILT');
  document.getElementById('rng-pitch').value = 0;
  document.getElementById('rng-roll').value = 0;
  document.getElementById('lbl-manual-pitch').innerText = '0.0°';
  document.getElementById('lbl-manual-roll').innerText = '0.0°';
  fetch('/api/level', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({})
  });
}

function levelPlatform() {
  emergencyStop();
}

function centerSetpoint() {
  document.getElementById('rng-spx').value = 0;
  document.getElementById('num-spx').value = 0;
  document.getElementById('rng-spy').value = 0;
  document.getElementById('num-spy').value = 0;
  dispatchPIDUpdate();
}

function tareZeroPosition() {
  fetch('/api/zero', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({})
  });
}

// ==============================================================================
// SINCRONIZAÇÃO BIDIRECIONAL DE GANHOS PID (RANGE <-> NUMBER INPUT)
// ==============================================================================
let pidDebounceTimer = null;

function syncFromRange(param) {
  const range = document.getElementById('rng-' + param);
  const num = document.getElementById('num-' + param);
  if (range && num) {
    num.value = range.value;
  }
  schedulePIDUpdate();
}

function syncFromNum(param) {
  const range = document.getElementById('rng-' + param);
  const num = document.getElementById('num-' + param);
  if (range && num) {
    range.value = num.value;
  }
  schedulePIDUpdate();
}

function schedulePIDUpdate() {
  clearTimeout(pidDebounceTimer);
  pidDebounceTimer = setTimeout(dispatchPIDUpdate, 60);
}

function dispatchPIDUpdate() {
  const kp = parseFloat(document.getElementById('num-kp').value);
  const ki = parseFloat(document.getElementById('num-ki').value);
  const kd = parseFloat(document.getElementById('num-kd').value);
  const n = parseFloat(document.getElementById('num-n').value);
  const spx = parseFloat(document.getElementById('num-spx').value);
  const spy = parseFloat(document.getElementById('num-spy').value);

  const payload = {};
  if (Number.isFinite(kp)) payload.kp = kp;
  if (Number.isFinite(ki)) payload.ki = ki;
  if (Number.isFinite(kd)) payload.kd = kd;
  if (Number.isFinite(n)) payload.n = n;
  if (Number.isFinite(spx)) payload.setpoint_x = spx;
  if (Number.isFinite(spy)) payload.setpoint_y = spy;

  if (Object.keys(payload).length === 0) return;

  fetch('/api/pid', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload)
  });
}

// Presets Rápidos de Ganhos PID
function applyGainPreset(presetName, btnEl) {
  document.querySelectorAll('.preset-pill-row .chip-btn').forEach(b => b.classList.remove('active'));
  if (btnEl) {
    btnEl.classList.add('active');
  }

  fetch('/api/preset', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name: presetName })
  }).then(() => {
    if (presetName === 'conservative') {
      setPIDInputs(1.05, 0.02, 0.65, 18.0);
    } else if (presetName === 'balanced') {
      setPIDInputs(1.35, 0.05, 0.85, 20.0);
    } else if (presetName === 'sport') {
      setPIDInputs(1.80, 0.08, 1.15, 25.0);
    }
  });
}

function setPIDInputs(kp, ki, kd, n) {
  document.getElementById('num-kp').value = kp;
  document.getElementById('rng-kp').value = kp;
  document.getElementById('num-ki').value = ki;
  document.getElementById('rng-ki').value = ki;
  document.getElementById('num-kd').value = kd;
  document.getElementById('rng-kd').value = kd;
  document.getElementById('num-n').value = n;
  document.getElementById('rng-n').value = n;
}

// ==============================================================================
// FILTRO DE KALMAN E LATÊNCIA
// ==============================================================================
function syncKalmanLatencyRange() {
  const rng = document.getElementById('rng-latency');
  const num = document.getElementById('num-latency');
  if (rng && num) num.value = rng.value;
  updateKalmanSettings();
}

function syncKalmanLatencyNum() {
  const rng = document.getElementById('rng-latency');
  const num = document.getElementById('num-latency');
  if (rng && num) rng.value = num.value;
  updateKalmanSettings();
}

function updateKalmanSettings() {
  const latency = parseFloat(document.getElementById('num-latency').value);
  const qa = parseFloat(document.getElementById('num-qa').value);
  const rmeasInput = parseFloat(document.getElementById('num-rmeas').value);
  const useK = document.getElementById('chk-use-kalman').checked;

  const payload = { use_kalman: useK };
  if (Number.isFinite(latency)) payload.latency_ms = latency;
  if (Number.isFinite(qa)) payload.q_accel = qa;
  if (Number.isFinite(rmeasInput)) payload.r_meas = rmeasInput * 1e-6; // mm² para m²

  fetch('/api/kalman', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload)
  });
}

// ==============================================================================
// VISÃO COMPUTACIONAL E FILTROS DE COR
// ==============================================================================
function setDetectorMode(det) {
  fetch('/api/detector', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ detector: det })
  }).then(() => {
    document.getElementById('btn-det-hsv').classList.toggle('active', det === 'hsv');
    document.getElementById('btn-det-yolo').classList.toggle('active', det === 'yolo');
    const badge = document.getElementById('badge-detector-txt');
    if (badge) badge.innerText = 'DETECTOR: ' + det.toUpperCase();
  });
}

function toggleBinaryMask(checked) {
  fetch('/api/toggle_mask', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ show_mask: checked })
  });
}

function updateVisionFilters() {
  const h_min = parseInt(document.getElementById('rng-h-min').value);
  const h_max = parseInt(document.getElementById('rng-h-max').value);
  const s_min = parseInt(document.getElementById('rng-s-min').value);
  const s_max = parseInt(document.getElementById('rng-s-max').value);
  const v_min = parseInt(document.getElementById('rng-v-min').value);
  const v_max = parseInt(document.getElementById('rng-v-max').value);
  const blur = parseInt(document.getElementById('rng-blur').value);
  const max_r = parseInt(document.getElementById('rng-max-radius').value);

  document.getElementById('lbl-h-min').innerText = h_min;
  document.getElementById('lbl-h-max').innerText = h_max;
  document.getElementById('lbl-s-min').innerText = s_min;
  document.getElementById('lbl-s-max').innerText = s_max;
  document.getElementById('lbl-v-min').innerText = v_min;
  document.getElementById('lbl-v-max').innerText = v_max;
  document.getElementById('lbl-blur').innerText = blur + ' px';
  document.getElementById('lbl-radius').innerText = '4 - ' + max_r;

  fetch('/api/hsv', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      lower: [h_min, s_min, v_min],
      upper: [h_max, s_max, v_max],
      blur_size: blur,
      max_radius: max_r
    })
  });
}

function applyColorPreset(preset) {
  fetch('/api/preset', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name: preset })
  }).then(() => {
    if (preset === 'orange_ball') {
      document.getElementById('rng-h-min').value = 5;
      document.getElementById('rng-h-max').value = 25;
      document.getElementById('rng-s-min').value = 120;
      document.getElementById('rng-s-max').value = 255;
      document.getElementById('rng-v-min').value = 100;
      document.getElementById('rng-v-max').value = 255;
    } else if (preset === 'white_ball') {
      document.getElementById('rng-h-min').value = 0;
      document.getElementById('rng-h-max').value = 179;
      document.getElementById('rng-s-min').value = 0;
      document.getElementById('rng-s-max').value = 60;
      document.getElementById('rng-v-min').value = 180;
      document.getElementById('rng-v-max').value = 255;
    }
    updateVisionFilters();
  });
}

// ==============================================================================
// ORIENTAÇÃO E ALINHAMENTO CÂMERA <-> MOTORES
// ==============================================================================
function updateOrientationSettings() {
  const rot = parseInt(document.getElementById('sel-cam-rot').value);
  const front = parseInt(document.getElementById('sel-front-motor').value);
  const invX = document.getElementById('chk-inv-x').checked;
  const invY = document.getElementById('chk-inv-y').checked;
  const swXy = document.getElementById('chk-swap-xy').checked;

  fetch('/api/orientation', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      cam_rotation: rot,
      front_motor: front,
      invert_x: invX,
      invert_y: invY,
      swap_xy: swXy
    })
  });
}

// Trava de Torque (Holding Current)
function setTorqueHold(hold) {
  fetch('/api/torque', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ hold: hold })
  });
}

// ==============================================================================
// CONTROLE MANUAL DE MOTORES
// ==============================================================================
let tiltPending = false;
let nextTilt = null;

function sendManualTilt() {
  const pitch = parseFloat(document.getElementById('rng-pitch').value);
  const roll = parseFloat(document.getElementById('rng-roll').value);
  document.getElementById('lbl-manual-pitch').innerText = pitch.toFixed(1) + '°';
  document.getElementById('lbl-manual-roll').innerText = roll.toFixed(1) + '°';

  nextTilt = { pitch_deg: pitch, roll_deg: roll };
  if (!tiltPending) {
    dispatchTilt();
  }
}

function dispatchTilt() {
  if (!nextTilt) return;
  tiltPending = true;
  const payload = nextTilt;
  nextTilt = null;

  fetch('/api/manual_tilt', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload)
  }).finally(() => {
    tiltPending = false;
    if (nextTilt) dispatchTilt();
  });
}

function jogMotor(motorId, steps) {
  fetch('/api/jog_motor', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ motor: motorId, steps: steps })
  });
}

function updateMotorSpeed() {
  const delay = parseInt(document.getElementById('rng-delay').value);
  const freq = Math.round(1000000 / (2 * delay));
  document.getElementById('lbl-delay').innerText = delay + ' µs (~' + freq + ' Hz)';
  fetch('/api/motor_speed', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ delay_us: delay })
  });
}

function toggleInvertMotor(motorId, invert) {
  fetch('/api/invert_motor', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ motor: motorId, invert: invert })
  });
}

// Recorte ROI
function updateCropSettings() {
  const x = parseInt(document.getElementById('rng-crop-x').value);
  const y = parseInt(document.getElementById('rng-crop-y').value);
  const w = parseInt(document.getElementById('rng-crop-w').value);
  const h = parseInt(document.getElementById('rng-crop-h').value);

  document.getElementById('lbl-crop-x').innerText = x;
  document.getElementById('lbl-crop-y').innerText = y;
  document.getElementById('lbl-crop-w').innerText = w;
  document.getElementById('lbl-crop-h').innerText = h;

  fetch('/api/crop', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ crop_x: x, crop_y: y, crop_w: w, crop_h: h })
  });
}

function resetCropFull() {
  document.getElementById('rng-crop-x').value = 0;
  document.getElementById('rng-crop-y').value = 0;
  document.getElementById('rng-crop-w').value = 320;
  document.getElementById('rng-crop-h').value = 240;
  updateCropSettings();
}

// ==============================================================================
// OSCILOSCÓPIO EM TEMPO REAL NO CANVAS HTML5
// ==============================================================================
const canvas = document.getElementById('scopeCanvas');
const ctx = canvas ? canvas.getContext('2d') : null;

function renderOscilloscope(history) {
  if (!ctx || !history || history.length === 0) return;

  const w = canvas.width;
  const h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  // Fundo e Linhas de Grade
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let y = 20; y < h; y += 30) {
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
  }
  ctx.stroke();

  // Linha Central (Zero mm)
  const midY = h / 2.0;
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(0, midY);
  ctx.lineTo(w, midY);
  ctx.stroke();
  ctx.setLineDash([]);

  // Escala vertical: +/- 70 mm mapeados em [0, h]
  const scaleY = (h * 0.42) / 60.0;
  const stepX = w / Math.max(1, history.length - 1);

  function drawSeries(key, strokeStyle, dashed = false) {
    ctx.strokeStyle = strokeStyle;
    ctx.lineWidth = dashed ? 1.5 : 2;
    if (dashed) ctx.setLineDash([3, 3]); else ctx.setLineDash([]);
    ctx.beginPath();

    history.forEach((pt, idx) => {
      const px = idx * stepX;
      const py = midY - (pt[key] * scaleY);
      if (idx === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Traços: X Real (Cyan), X Setpoint (Amber), Y Real (Emerald), Y Setpoint (Rose)
  drawSeries('sp_x', '#ffab00', true);
  drawSeries('x', '#00f0ff', false);
  drawSeries('sp_y', '#ff3d71', true);
  drawSeries('y', '#00e676', false);
}

// ==============================================================================
// LOOP DE TELEMETRIA CONTÍNUA (10 HZ VIA HTTP POLLING)
// ==============================================================================
let isHydrated = false;

function hydrateInitialControls(data) {
  if (isHydrated) return;
  isHydrated = true;

  if (Number.isFinite(data.kp) && Number.isFinite(data.ki) && Number.isFinite(data.kd) && Number.isFinite(data.n)) {
    setPIDInputs(data.kp, data.ki, data.kd, data.n);
  }
  const elNumSpx = document.getElementById('num-spx');
  const elRngSpx = document.getElementById('rng-spx');
  if (elNumSpx && Number.isFinite(data.sp_x)) elNumSpx.value = data.sp_x;
  if (elRngSpx && Number.isFinite(data.sp_x)) elRngSpx.value = data.sp_x;

  const elNumSpy = document.getElementById('num-spy');
  const elRngSpy = document.getElementById('rng-spy');
  if (elNumSpy && Number.isFinite(data.sp_y)) elNumSpy.value = data.sp_y;
  if (elRngSpy && Number.isFinite(data.sp_y)) elRngSpy.value = data.sp_y;

  const elNumLat = document.getElementById('num-latency');
  const elRngLat = document.getElementById('rng-latency');
  if (elNumLat && Number.isFinite(data.latency_ms)) elNumLat.value = data.latency_ms;
  if (elRngLat && Number.isFinite(data.latency_ms)) elRngLat.value = data.latency_ms;

  const elNumQa = document.getElementById('num-qa');
  if (elNumQa && Number.isFinite(data.q_accel)) elNumQa.value = data.q_accel;

  const elNumRmeas = document.getElementById('num-rmeas');
  if (elNumRmeas && Number.isFinite(data.r_meas)) elNumRmeas.value = (data.r_meas * 1e6).toFixed(2);

  const elChkKalman = document.getElementById('chk-use-kalman');
  if (elChkKalman && typeof data.use_kalman !== 'undefined') elChkKalman.checked = Boolean(data.use_kalman);

  const elSelCamRot = document.getElementById('sel-cam-rot');
  if (elSelCamRot && typeof data.cam_rotation !== 'undefined') elSelCamRot.value = String(data.cam_rotation);

  const elSelFrontMotor = document.getElementById('sel-front-motor');
  if (elSelFrontMotor && typeof data.front_motor !== 'undefined') elSelFrontMotor.value = String(data.front_motor);

  const elInvX = document.getElementById('chk-inv-x');
  if (elInvX && typeof data.invert_x !== 'undefined') elInvX.checked = Boolean(data.invert_x);

  const elInvY = document.getElementById('chk-inv-y');
  if (elInvY && typeof data.invert_y !== 'undefined') elInvY.checked = Boolean(data.invert_y);

  const elSwapXy = document.getElementById('chk-swap-xy');
  if (elSwapXy && typeof data.swap_xy !== 'undefined') elSwapXy.checked = Boolean(data.swap_xy);

  if (Array.isArray(data.invert) && data.invert.length === 3) {
    const elInv1 = document.getElementById('chk-inv-1');
    const elInv2 = document.getElementById('chk-inv-2');
    const elInv3 = document.getElementById('chk-inv-3');
    if (elInv1) elInv1.checked = Boolean(data.invert[0]);
    if (elInv2) elInv2.checked = Boolean(data.invert[1]);
    if (elInv3) elInv3.checked = Boolean(data.invert[2]);
  }

  const btnOpStandalone = document.getElementById('btn-op-standalone');
  const btnOpHybrid = document.getElementById('btn-op-hybrid');
  if (btnOpStandalone && btnOpHybrid && data.op_mode) {
    btnOpStandalone.classList.toggle('active', data.op_mode === 'STANDALONE_RPI');
    btnOpHybrid.classList.toggle('active', data.op_mode === 'HYBRID_STM32');
  }

  const btnDetHsv = document.getElementById('btn-det-hsv');
  const btnDetYolo = document.getElementById('btn-det-yolo');
  if (btnDetHsv && btnDetYolo && data.detector) {
    btnDetHsv.classList.toggle('active', data.detector === 'hsv');
    btnDetYolo.classList.toggle('active', data.detector === 'yolo');
  }

  // Hidratação dos Filtros HSV e Visão
  if (Array.isArray(data.hsv_lower) && data.hsv_lower.length === 3 &&
      Array.isArray(data.hsv_upper) && data.hsv_upper.length === 3) {
    const elHmin = document.getElementById('rng-h-min');
    const elSmin = document.getElementById('rng-s-min');
    const elVmin = document.getElementById('rng-v-min');
    const elHmax = document.getElementById('rng-h-max');
    const elSmax = document.getElementById('rng-s-max');
    const elVmax = document.getElementById('rng-v-max');
    if (elHmin) elHmin.value = data.hsv_lower[0];
    if (elSmin) elSmin.value = data.hsv_lower[1];
    if (elVmin) elVmin.value = data.hsv_lower[2];
    if (elHmax) elHmax.value = data.hsv_upper[0];
    if (elSmax) elSmax.value = data.hsv_upper[1];
    if (elVmax) elVmax.value = data.hsv_upper[2];

    const lblHmin = document.getElementById('lbl-h-min');
    const lblSmin = document.getElementById('lbl-s-min');
    const lblVmin = document.getElementById('lbl-v-min');
    const lblHmax = document.getElementById('lbl-h-max');
    const lblSmax = document.getElementById('lbl-s-max');
    const lblVmax = document.getElementById('lbl-v-max');
    if (lblHmin) lblHmin.innerText = data.hsv_lower[0];
    if (lblSmin) lblSmin.innerText = data.hsv_lower[1];
    if (lblVmin) lblVmin.innerText = data.hsv_lower[2];
    if (lblHmax) lblHmax.innerText = data.hsv_upper[0];
    if (lblSmax) lblSmax.innerText = data.hsv_upper[1];
    if (lblVmax) lblVmax.innerText = data.hsv_upper[2];
  }

  if (Number.isFinite(data.blur_size)) {
    const elBlur = document.getElementById('rng-blur');
    const lblBlur = document.getElementById('lbl-blur');
    if (elBlur) elBlur.value = data.blur_size;
    if (lblBlur) lblBlur.innerText = data.blur_size + ' px';
  }

  if (Number.isFinite(data.max_radius)) {
    const elRadius = document.getElementById('rng-max-radius');
    const lblRadius = document.getElementById('lbl-radius');
    if (elRadius) elRadius.value = data.max_radius;
    if (lblRadius) lblRadius.innerText = (data.min_radius || 4) + ' - ' + data.max_radius;
  }

  if (typeof data.show_mask !== 'undefined') {
    const elMask = document.getElementById('chk-mask-view');
    if (elMask) elMask.checked = Boolean(data.show_mask);
  }

  if (Number.isFinite(data.crop_x)) {
    const elCropX = document.getElementById('rng-crop-x');
    const lblCropX = document.getElementById('lbl-crop-x');
    if (elCropX) elCropX.value = data.crop_x;
    if (lblCropX) lblCropX.innerText = data.crop_x;
  }
  if (Number.isFinite(data.crop_y)) {
    const elCropY = document.getElementById('rng-crop-y');
    const lblCropY = document.getElementById('lbl-crop-y');
    if (elCropY) elCropY.value = data.crop_y;
    if (lblCropY) lblCropY.innerText = data.crop_y;
  }
  if (Number.isFinite(data.crop_w)) {
    const elCropW = document.getElementById('rng-crop-w');
    const lblCropW = document.getElementById('lbl-crop-w');
    if (elCropW) elCropW.value = data.crop_w;
    if (lblCropW) lblCropW.innerText = data.crop_w;
  }
  if (Number.isFinite(data.crop_h)) {
    const elCropH = document.getElementById('rng-crop-h');
    const lblCropH = document.getElementById('lbl-crop-h');
    if (elCropH) elCropH.value = data.crop_h;
    if (lblCropH) lblCropH.innerText = data.crop_h;
  }

  if (Number.isFinite(data.step_delay_us)) {
    const elDelay = document.getElementById('rng-delay');
    const lblDelay = document.getElementById('lbl-delay');
    if (elDelay) elDelay.value = data.step_delay_us;
    if (lblDelay) {
      const freq = Math.round(1000000 / (2 * data.step_delay_us));
      lblDelay.innerText = data.step_delay_us + ' µs (~' + freq + ' Hz)';
    }
  }

  updateAutoButton(data.mode);
}

function updateAutoButton(mode) {
  const btnAuto = document.getElementById('btn-mode-auto');
  if (!btnAuto || !mode) return;
  if (mode === 'AUTO') {
    btnAuto.classList.remove('btn-emerald');
    btnAuto.classList.add('btn-cyan');
    btnAuto.innerText = '⏸ PAUSAR BALANCEAMENTO AUTO';
    btnAuto.setAttribute('onclick', "setControlMode('MANUAL_TILT')");
  } else {
    btnAuto.classList.add('btn-emerald');
    btnAuto.classList.remove('btn-cyan');
    btnAuto.innerText = '▶ INICIAR BALANCEAMENTO AUTO';
    btnAuto.setAttribute('onclick', "setControlMode('AUTO')");
  }
}

setInterval(() => {
  fetch('/api/telemetry')
    .then(res => res.json())
    .then(data => {
      hydrateInitialControls(data);
      // 1. Atualização dos Cards Métricos
      const telX = document.getElementById('tel-x');
      const telY = document.getElementById('tel-y');
      const telVx = document.getElementById('tel-vx');
      const telVy = document.getElementById('tel-vy');
      const telPitch = document.getElementById('tel-pitch');
      const telRoll = document.getElementById('tel-roll');
      const telM1 = document.getElementById('tel-m1');
      const telM2M3 = document.getElementById('tel-m2-m3');

      if (telX) telX.innerText = (data.x_mm >= 0 ? '+' : '') + data.x_mm.toFixed(1) + ' mm';
      if (telY) telY.innerText = (data.y_mm >= 0 ? '+' : '') + data.y_mm.toFixed(1) + ' mm';
      if (telVx) telVx.innerText = 'Vx: ' + (data.vel_x_mms >= 0 ? '+' : '') + data.vel_x_mms.toFixed(0) + ' mm/s';
      if (telVy) telVy.innerText = 'Vy: ' + (data.vel_y_mms >= 0 ? '+' : '') + data.vel_y_mms.toFixed(0) + ' mm/s';
      if (telPitch) telPitch.innerText = (data.pitch_deg >= 0 ? '+' : '') + data.pitch_deg.toFixed(2) + '°';
      if (telRoll) telRoll.innerText = (data.roll_deg >= 0 ? '+' : '') + data.roll_deg.toFixed(2) + '°';
      if (telM1) telM1.innerText = 'M1: ' + data.motor_steps[0] + ' passos';
      if (telM2M3) telM2M3.innerText = 'M2: ' + data.motor_steps[1] + ' | M3: ' + data.motor_steps[2];

      // 2. HUD Overlay no Vídeo
      const hudFps = document.getElementById('hud-fps-pill');
      const hudLatency = document.getElementById('hud-latency-pill');
      const hudPos = document.getElementById('hud-pos-display');
      const hudVel = document.getElementById('hud-vel-display');
      const hudStatus = document.getElementById('hud-status-display');

      if (hudFps) hudFps.innerText = data.fps.toFixed(1) + ' FPS';
      if (hudLatency) hudLatency.innerText = 'Latência: ' + data.latency_ms.toFixed(0) + ' ms';
      if (hudPos) hudPos.innerText = 'X: ' + data.x_mm.toFixed(1) + ' mm | Y: ' + data.y_mm.toFixed(1) + ' mm';
      if (hudVel) hudVel.innerText = 'V: ' + Math.hypot(data.vel_x_mms, data.vel_y_mms).toFixed(0) + ' mm/s';
      if (hudStatus) {
        hudStatus.innerText = data.detected
          ? 'DETECTADO (' + (data.conf * 100).toFixed(0) + '%)'
          : 'PROCURANDO ESFERA';
      }

      // 3. Badges de Status do Header
      const badgePig = document.getElementById('badge-pigpio');
      const badgePigTxt = document.getElementById('badge-pigpio-txt');
      if (badgePig && badgePigTxt) {
        if (data.op_mode === 'HYBRID_STM32') {
          badgePig.className = 'status-pill status-cyan';
          badgePigTxt.innerText = 'STM32: ATUANDO';
        } else if (data.pigpio) {
          badgePig.className = 'status-pill status-live';
          badgePigTxt.innerText = 'PIGPIO: CONECTADO';
        } else {
          badgePig.className = 'status-pill status-warn';
          badgePigTxt.innerText = 'PIGPIO: DESCONECTADO';
        }
      }

      const badgeDetTxt = document.getElementById('badge-detector-txt');
      if (badgeDetTxt && data.detector) {
        badgeDetTxt.innerText = 'DETECTOR: ' + data.detector.toUpperCase();
      }

      const badgeCtrlTxt = document.getElementById('badge-ctrl-mode-txt');
      if (badgeCtrlTxt) {
        badgeCtrlTxt.innerText = 'MODO: ' + data.mode;
      }
      updateAutoButton(data.mode);

      // 4. Status de Hardware / UART
      const hwTx = document.getElementById('hw-packets-tx');
      const hwFail = document.getElementById('hw-packets-fail');
      if (hwTx) hwTx.innerText = data.uart_packets_sent;
      if (hwFail) hwFail.innerText = data.uart_packets_failed;

      // 5. Renderização do Osciloscópio
      if (data.history) {
        renderOscilloscope(data.history);
      }
    })
    .catch(() => {});
}, 100);
