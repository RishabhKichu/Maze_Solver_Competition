const MAX_DATA_POINTS = 1000;
let stored_data = [];
let enc_error = 0;
let tof_error = 0;
let pwm_error = 0;

let start_time = Date.now();
let chart_paused = false;

setInterval(() => {
  fetch("/d")
    .then((r) => r.text())
    .then((t) => {
      const d = t.split(",");
      if (d.length >= 6) {
        document.getElementById("s_l").innerText = d[0] + 1;
        document.getElementById("s_c").innerText = d[1] + 10;
        document.getElementById("s_r").innerText = d[2];
        document.getElementById("s_enc_e").innerText = d[3];
        document.getElementById("s_pwm_e").innerText = d[4];
        document.getElementById("s_tof_e").innerText = d[5];

        enc_error = parseFloat(d[3]);
        tof_error = parseFloat(d[5]);
      }
    })
    .catch(() => {});
}, 100);

document.getElementById("f").addEventListener("submit", function (e) {
  e.preventDefault();
  fetch("/updatePID", {
    method: "POST",
    body: new URLSearchParams(new FormData(this)),
  })
    .then((r) => r.text())
    .then((t) => {
      const m = document.getElementById("msg");
      m.innerText = t;
      setTimeout(() => (m.innerText = ""), 2000);
    });
});

const ctx = document.getElementById("myChart");

const chart_ = new Chart(ctx, {
  data: {
    datasets: [
      {
        type: "line",
        label: "Encoder PID",
        data: [],
      },
      {
        type: "line",
        label: "ToF PID",
        data: [],
      },
      {
        type: "line",
        label: "PWM PID",
        data: [],
      },
    ],
  },
  options: {
    scales: {
      x: {
        type: "linear",
        position: "bottom",
      },
      y: {
        beginAtZero: true,
      },
      display: true,
      position: "top",
    },
  },
});

setInterval(() => {
  if (chart_paused) return;

  let enc_data = [(Date.now() - start_time) / 1000, enc_error];
  let tof_data = [(Date.now() - start_time) / 1000, tof_error];
  let pwm_data = [(Date.now() - start_time) / 1000, pwm_error];

  chart_.data.datasets[0].data.push(enc_data);
  chart_.data.datasets[1].data.push(tof_data);
  chart_.data.datasets[2].data.push(pwm_data);

  stored_data.push(enc_data);
  if (chart_.data.datasets[0].data.length > MAX_DATA_POINTS) {
    chart_.data.datasets[0].data.shift();
    chart_.data.datasets[1].data.shift();
    chart_.data.datasets[2].data.shift();
  }
  chart_.update("none");
}, 20);

document.getElementById("ENCBtn").addEventListener("click", function () {
  chart_.data.datasets[0].hidden = !chart_.data.datasets[0].hidden;
  document.getElementById("ENCBtn").innerText = chart_.data.datasets[0].hidden
    ? "Show Encoder Data"
    : "Hide Encoder Data";
});
document.getElementById("TOFBtn").addEventListener("click", function () {
  chart_.data.datasets[1].hidden = !chart_.data.datasets[1].hidden;
  document.getElementById("TOFBtn").innerText = chart_.data.datasets[1].hidden
    ? "Show TOF Data"
    : "Hide TOF Data";
});
document.getElementById("PWMBtn").addEventListener("click", function () {
  chart_.data.datasets[2].hidden = !chart_.data.datasets[2].hidden;
  document.getElementById("PWMBtn").innerText = chart_.data.datasets[2].hidden
    ? "Show PWM Data"
    : "Hide PWM Data";
});
document.getElementById("ClearBtn").addEventListener("click", function () {
  chart_.data.datasets[0].data = [];
  chart_.data.datasets[1].data = [];
  chart_.data.datasets[2].data = [];
  stored_data = [];
  chart_.update("none");

  start_time = Date.now();
});
document.getElementById("PauseBtn").addEventListener("click", function () {
  chart_paused = !chart_paused;
  document.getElementById("PauseBtn").innerText = chart_paused
    ? "Resume Data"
    : "Pause Data";
});
