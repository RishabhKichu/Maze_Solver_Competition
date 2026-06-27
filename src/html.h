#ifndef HTML_H
#define HTML_H

#include <Arduino.h>
const char html[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MazeBot </title>
  <style>
    body { font-family: sans-serif; text-align: center; margin: 0; padding: 15px; background: #f0f0f0; color: #333; }
    .wrap { display: flex; flex-wrap: wrap; justify-content: center; gap: 15px; max-width: 800px; margin: 0 auto; }
    .card { flex: 1; min-width: 280px; background: #fff; padding: 15px; border-radius: 6px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
    .sect { background: #f9f9f9; margin: 10px 0; padding: 10px; border-radius: 4px; border-left: 4px solid #4CAF50; text-align: left; }
    h2, h4 { margin: 5px 0; }
    .row { margin: 6px 0; display: flex; justify-content: space-between; align-items: center; }
    input[type=number] { padding: 4px; width: 70px; border: 1px solid #ccc; border-radius: 4px; }
    .btn { padding: 8px; background: #4CAF50; color: #fff; border: none; border-radius: 4px; cursor: pointer; width: 100%; font-weight: bold; }
    .btn:hover { background: #45a049; }
    #msg { margin-top: 5px; font-size: 0.9em; color: #4CAF50; font-weight: bold; }
  </style>
</head>
<body>

  <h2>MazeBot Dashboard</h2>
  <div class="wrap">
    
    <div class="card">
      <h2>Sensors</h2>
      <div style="text-align: left; padding: 5px; font-size: 1.1em;">
        <p>Left ToF: <strong id="s_l">0</strong> mm</p>
        <p>Center ToF: <strong id="s_c">0</strong> mm</p>
        <p>Right ToF: <strong id="s_r">0</strong> mm</p>
        <p>Encoder Error: <strong id="s_enc_e">0</strong></p>
        <p>PWM Error: <strong id="s_e1">0</strong></p>
        <p>Tof Error: <strong id="s_e2">0</strong></p>
      </div>
    </div>
    
    <div class="card">
      <h2>Control</h2>
      <form id="f">
        
        <div class="sect">
          <h4>Configuration</h4>
          <div class="row"><label>Base Speed:</label><input type="number" name="base_speed" min="0" max="255" step="1" value="100"></div>
        </div>

        <div class="sect">
          <h4>Encoder PID</h4>
          <div class="row"><label>Kp:</label><input type="number" name="enc_kp" step="0.01" value="8.00"></div>
          <div class="row"><label>Kd:</label><input type="number" name="enc_kd" step="0.01" value="0.00"></div>
          <div class="row"><label>Ki:</label><input type="number" name="enc_ki" step="0.01" value="0.00"></div>
        </div>
        
        <div class="sect">
          <h4>ToF PID</h4>
          <div class="row"><label>Kp:</label><input type="number" name="tof_kp" step="0.01" value="0.00"></div>
          <div class="row"><label>Kd:</label><input type="number" name="tof_kd" step="0.01" value="0.00"></div>
          <div class="row"><label>Ki:</label><input type="number" name="tof_ki" step="0.01" value="0.00"></div>
        </div>
        
        <input type="submit" class="btn" value="Update Parameters">
        <div id="msg"></div>
      </form>
    </div>

  </div>

  <script>
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
          }
        }).catch(()=>{});
    }, 100);

    document.getElementById('f').addEventListener('submit', function(e) {
      e.preventDefault();
      fetch('/updatePID', { method: 'POST', body: new URLSearchParams(new FormData(this)) })
        .then(r => r.text())
        .then(t => {
          const m = document.getElementById('msg');
          m.innerText = t;
          setTimeout(() => m.innerText = "", 2000);
        });
    });
  </script>

</body>
</html>
)=====";

#endif
