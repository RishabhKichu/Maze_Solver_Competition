#ifndef HTML_H
#define HTML_H

#include <Arduino.h>
const char html[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MazeBot</title>
  <style>
    body { font-family: sans-serif; background: #f0f0f0; padding: 10px; font-size: 14px; }
    .wrap { display: flex; flex-wrap: wrap; gap: 10px; }
    .card { background: #fff; padding: 10px; border-radius: 4px; min-width: 250px; flex: 1; }
    .sect { margin: 5px 0; padding: 5px; border-left: 3px solid #4CAF50; }
    .row { display: flex; justify-content: space-between; margin: 3px 0; }
    input[type=number] { width: 60px; }
    .btn { width: 100%; background: #4CAF50; color: #fff; border: 0; padding: 5px; cursor: pointer; }
    /* Ultra-lightweight raw text log */
    #log { width: 100%; height: 60px; font-family: monospace; font-size: 11px; margin-top: 10px; display: block; box-sizing: border-box; }
  </style>
</head>
<body>
  <h3>MazeBot</h3>
  <div class="wrap">
    <div class="card">
      <h4>Sensors</h4>
      <p>L: <strong id="s_l">0</strong> | C: <strong id="s_c">0</strong> | R: <strong id="s_r">0</strong></p>
      <p>Enc Err: <strong id="s_enc_e">0</strong></p>
      <p>PWM Err: <strong id="s_e1">0</strong></p>
      <p>ToF Err: <strong id="s_e2">0</strong></p>
      <p>Motor L PWM: <strong id="s_pwml">0</strong></p>
      <p>Motor R PWM: <strong id="s_pwmr">0</strong></p>
      <p>State: <strong id="s_state">0</strong></p>
    </div>
    <div class="card">
      <form id="f">
        
        <div class="sect">
          <strong>Enc PID</strong>
          <div class="row">P:<input type="number" name="enc_kp" step="0.01" value="8.00"></div>
          <div class="row">D:<input type="number" name="enc_kd" step="0.01" value="0.00"></div>
          <div class="row">I:<input type="number" name="enc_ki" step="0.01" value="0.00"></div>
        </div>
        <div class="sect">
          <strong>ToF PID</strong>
          <div class="row">P:<input type="number" name="tof_kp" step="0.01" value="0.8"></div>
          <div class="row">D:<input type="number" name="tof_kd" step="0.01" value="4"></div>
          <div class="row">I:<input type="number" name="tof_ki" step="0.01" value="0.00"></div>
        </div>
        <div class="sect">
    <strong>Motion</strong>
    <div class="row">Base Spd:<input type="number" name="base_speed" min="0" max="255" value="230"></div>
    <div class="row">Min Spd:<input type="number" name="min_speed" min="0" max="255" value="120"></div>
    <div class="row">Turn Spd:<input type="number" name="turn_speed" min="0" max="255" value="230"></div>
    <div class="row">Turn Thresh:<input type="number" name="turning_threshold" step="1" value="100"></div>
    <div class="row">Braking:<input type="number" name="braking_threshold" step="1" value="40"></div>
    <div class="row">Turn Pulses:<input type="number" name="turn_pulses" step="1" value="800"></div>
</div>
        <input type="submit" class="btn" value="Update">
        <span id="msg" style="color:green; font-size:12px;"></span>
        <button type="button" class="btn" style="background:#e53935; margin-top:5px;" onclick="cmd('stop')">STOP</button>
        <button type="button" class="btn" style="background:#2E7D32; margin-top:5px;" onclick="cmd('follow')">FOLLOW</button>
        <button type="button" class="btn" style="background:#1565C0; margin-top:5px;" onclick="cmd('turn')">TURN</button>

      </form>
    </div>
  </div>

  <textarea id="log" readonly>Awaiting logs...</textarea>

  <script>
    let lastLog = "";
    setInterval(() => {
      fetch('/d')
        .then(r => r.text())
        .then(t => {
          const d = t.split(',');
          if(d.length >= 6) {
            document.getElementById('s_l').innerText = d[0];
            document.getElementById('s_c').innerText = d[1];
            document.getElementById('s_r').innerText = d[2];
            document.getElementById('s_enc_e').innerText = d[3];
            document.getElementById('s_e1').innerText = d[4];
            document.getElementById('s_e2').innerText = d[5];
            
            // Appends 7th value if it exists and changed
            if(d.length >= 7 && d[6].trim() !== "" && d[6] !== lastLog) {
              lastLog = d[6];
              const l = document.getElementById('log');
              l.value += "\n" + lastLog;
              l.scrollTop = l.scrollHeight; 
            }
            if(d.length >= 10) {
                document.getElementById('s_pwml').innerText = d[7];
                document.getElementById('s_pwmr').innerText = d[8];
                document.getElementByID('s_state').innerText = d[9];
            }
          }
        }).catch(()=>{});
    }, 100);

    document.getElementById('f').addEventListener('submit', function(e) {
      e.preventDefault();
      fetch('/updatePID', { method: 'POST', body: new URLSearchParams(new FormData(this)) })
        .then(r => r.text())
        .then(t => {
          const m = document.getElementById('msg');
          m.innerText = " OK";
          setTimeout(() => m.innerText = "", 1500);
        });
    });
    function cmd(action) {
    fetch('/' + action)
        .then(r => r.text())
        .then(t => {
            const m = document.getElementById('msg');
            m.innerText = t;
            setTimeout(() => m.innerText = '', 1500);
        });
    }

    // fetch current config on page load
fetch('/config')
    .then(r => r.text())
    .then(t => {
        const c = t.split(',');
        document.querySelector('[name=base_speed]').value      = c[0];
        document.querySelector('[name=min_speed]').value       = c[1];
        document.querySelector('[name=turn_speed]').value      = c[2];
        document.querySelector('[name=turning_threshold]').value = c[3];
        document.querySelector('[name=braking_threshold]').value = c[4];
        document.querySelector('[name=turn_pulses]').value = c[5];
    });
    
  </script>
</body>
</html>
)=====";

#endif