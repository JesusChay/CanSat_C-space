let lastPrimaryDataTime = null;
let lastSecondaryDataTime = null;

function createDualLineChart(ctx, label, primaryColor = '#4bc0c0', secondaryColor = '#ff9f40') {
  return new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        {
          label: `${label} Alternativa (Primaria)`,
          data: [],
          borderColor: primaryColor,
          backgroundColor: 'transparent',
          borderWidth: 2,
          pointRadius: 2,
          fill: false
        },
        {
          label: `${label} Alternativa (Secundaria)`,
          data: [],
          borderColor: secondaryColor,
          backgroundColor: 'transparent',
          borderWidth: 2,
          pointRadius: 2,
          fill: false
        },
        {
          label: `${label} Absoluta (Primaria)`,
          data: [],
          borderColor: '#2196f3',
          backgroundColor: 'transparent',
          borderWidth: 2,
          pointRadius: 2,
          borderDash: [5, 5],
          fill: false
        },
        {
          label: `${label} Absoluta (Secundaria)`,
          data: [],
          borderColor: '#ff5722',
          backgroundColor: 'transparent',
          borderWidth: 2,
          pointRadius: 2,
          borderDash: [5, 5],
          fill: false
        }
      ]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      scales: {
        x: {
          type: 'category',
          title: { display: true, text: 'Hora', font: { size: 12 } },
          ticks: { color: '#ffffff', font: { size: 10 } }
        },
        y: {
          beginAtZero: true,
          title: { display: true, text: 'Altitud (m)', font: { size: 12 } },
          ticks: { color: '#ffffff', font: { size: 10 } }
        }
      },
      plugins: {
        legend: { labels: { color: '#ffffff' } }
      },
      elements: {
        line: {
          tension: 0,
          spanGaps: true
        }
      }
    }
  });
}

function createSecondaryOnlyChart(ctx, label, yAxisLabel, color = '#ff9f40') {
  return new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        {
          label: `${label} (Secundaria)`,
          data: [],
          borderColor: color,
          backgroundColor: 'transparent',
          borderWidth: 2,
          pointRadius: 2,
          fill: false
        }
      ]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      scales: {
        x: {
          type: 'category',
          title: { display: true, text: 'Hora', font: { size: 12 } },
          ticks: { color: '#ffffff', font: { size: 10 } }
        },
        y: {
          beginAtZero: true,
          title: { display: true, text: yAxisLabel, font: { size: 12 } },
          ticks: { color: '#ffffff', font: { size: 10 } }
        }
      },
      plugins: {
        legend: { labels: { color: '#ffffff' } }
      },
      elements: {
        line: {
          tension: 0,
          spanGaps: true
        }
      }
    }
  });
}

function createSingleLineChart(ctx, label, color = '#4bc0c0') {
  return new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [{
        label: label,
        data: [],
        borderColor: color,
        backgroundColor: 'transparent',
        borderWidth: 2,
        pointRadius: 2,
        fill: false
      }]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      scales: {
        x: {
          type: 'category',
          title: { display: true, text: 'Hora', font: { size: 12 } },
          ticks: { color: '#ffffff', font: { size: 10 } }
        },
        y: {
          beginAtZero: true,
          title: { display: true, text: 'Viento (m/s)', font: { size: 12 } },
          ticks: { color: '#ffffff', font: { size: 10 } }
        }
      },
      plugins: {
        legend: { labels: { color: '#ffffff' } }
      },
      elements: {
        line: {
          tension: 0,
          spanGaps: true
        }
      }
    }
  });
}

function createDistanceChart(ctx) {
  return new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [{
        label: 'Distancia',
        data: [],
        borderColor: '#ffeb3b',
        backgroundColor: 'transparent',
        borderWidth: 2,
        pointRadius: 2,
        fill: false
      }]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      scales: {
        x: {
          type: 'category',
          title: { display: true, text: 'Hora', font: { size: 12 } },
          ticks: { color: '#ffffff', font: { size: 10 } }
        },
        y: {
          beginAtZero: true,
          title: { display: true, text: 'Distancia (m)', font: { size: 12 } },
          ticks: { color: '#ffffff', font: { size: 10 } }
        }
      },
      plugins: {
        legend: { labels: { color: '#ffffff' } }
      },
      elements: {
        line: {
          tension: 0,
          spanGaps: true
        }
      }
    }
  });
}

function createAccelChart() {
  return new Chart(document.getElementById('accelChart').getContext('2d'), {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: 'Ax (Primaria)', data: [], borderColor: '#ff4444', backgroundColor: 'transparent', borderWidth: 2, pointRadius: 2, fill: false },
        { label: 'Ay (Primaria)', data: [], borderColor: '#44ff44', backgroundColor: 'transparent', borderWidth: 2, pointRadius: 2, fill: false },
        { label: 'Az (Primaria)', data: [], borderColor: '#4444ff', backgroundColor: 'transparent', borderWidth: 2, pointRadius: 2, fill: false },
        { label: 'Atotal (Primaria)', data: [], borderColor: '#ffcc00', backgroundColor: 'transparent', borderWidth: 2, pointRadius: 2, fill: false },
        { label: 'Ax (Secundaria)', data: [], borderColor: '#ff4444', backgroundColor: 'transparent', borderWidth: 2, borderDash: [5, 5], pointRadius: 2, fill: false },
        { label: 'Ay (Secundaria)', data: [], borderColor: '#44ff44', backgroundColor: 'transparent', borderWidth: 2, borderDash: [5, 5], pointRadius: 2, fill: false },
        { label: 'Az (Secundaria)', data: [], borderColor: '#4444ff', backgroundColor: 'transparent', borderWidth: 2, borderDash: [5, 5], pointRadius: 2, fill: false },
        { label: 'Atotal (Secundaria)', data: [], borderColor: '#ffcc00', backgroundColor: 'transparent', borderWidth: 2, borderDash: [5, 5], pointRadius: 2, fill: false }
      ]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      scales: {
        x: {
          type: 'category',
          title: { display: true, text: 'Hora', font: { size: 12 } },
          ticks: { color: '#ffffff', font: { size: 10 } }
        },
        y: {
          beginAtZero: true,
          title: { display: true, text: 'gravedad (g)', font: { size: 12 } },
          ticks: { color: '#ffffff', font: { size: 10 } }
        }
      },
      plugins: {
        legend: { labels: { color: '#ffffff' } }
      },
      elements: {
        line: {
          tension: 0,
          spanGaps: true
        }
      }
    }
  });
}

function updateAltitudeChart(chart, primaryRelative, secondaryRelative, primaryAbsolute, secondaryAbsolute, time) {
  if (!chart) return;
  if (chart.data.labels.length > 50) {
    chart.data.labels.splice(0, 1);
    chart.data.datasets.forEach(ds => ds.data.splice(0, 1));
  }
  chart.data.labels.push(time);
  chart.data.datasets[0].data.push(primaryRelative !== undefined ? parseFloat(primaryRelative) : null);
  chart.data.datasets[1].data.push(secondaryRelative !== undefined ? parseFloat(secondaryRelative) : null);
  chart.data.datasets[2].data.push(primaryAbsolute !== undefined ? parseFloat(primaryAbsolute) : null);
  chart.data.datasets[3].data.push(secondaryAbsolute !== undefined ? parseFloat(secondaryAbsolute) : null);
  chart.update();
}

function updateDualLine(chart, primaryValue, secondaryValue, time) {
  if (!chart) return;
  if (chart.data.labels.length > 50) {
    chart.data.labels.splice(0, 1);
    chart.data.datasets.forEach(ds => ds.data.splice(0, 1));
  }
  chart.data.labels.push(time);
  if (chart === temperatureChart || chart === humidityChart || chart === pressureChart) {
    chart.data.datasets[0].data.push(secondaryValue !== undefined ? parseFloat(secondaryValue) : null);
  } else {
    chart.data.datasets[0].data.push(primaryValue !== undefined ? parseFloat(primaryValue) : null);
    chart.data.datasets[1].data.push(secondaryValue !== undefined ? parseFloat(secondaryValue) : null);
  }
  chart.update();
}

function updateSingleLine(chart, value, time) {
  if (!chart) return;
  if (chart.data.labels.length > 50) {
    chart.data.labels.splice(0, 1);
    chart.data.datasets[0].data.splice(0, 1);
  }
  chart.data.labels.push(time);
  chart.data.datasets[0].data.push(value !== undefined ? parseFloat(value) : null);
  chart.update();
}

function updateAccelChart(chart, axP, ayP, azP, axS, ayS, azS, time) {
  if (!chart) return;
  if (chart.data.labels.length > 50) {
    chart.data.labels.splice(0, 1);
    chart.data.datasets.forEach(ds => ds.data.splice(0, 1));
  }
  chart.data.labels.push(time);
  chart.data.datasets[0].data.push(axP !== undefined ? parseFloat(axP) : null);
  chart.data.datasets[1].data.push(ayP !== undefined ? parseFloat(ayP) : null);
  chart.data.datasets[2].data.push(azP !== undefined ? parseFloat(azP) : null);
  chart.data.datasets[3].data.push(axP !== undefined && ayP !== undefined && azP !== undefined ? Math.sqrt(parseFloat(axP) * parseFloat(axP) + parseFloat(ayP) * parseFloat(ayP) + parseFloat(azP) * parseFloat(azP)) : null);
  chart.data.datasets[4].data.push(axS !== undefined ? parseFloat(axS) : null);
  chart.data.datasets[5].data.push(ayS !== undefined ? parseFloat(ayS) : null);
  chart.data.datasets[6].data.push(azS !== undefined ? parseFloat(azS) : null);
  chart.data.datasets[7].data.push(axS !== undefined && ayS !== undefined && azS !== undefined ? Math.sqrt(parseFloat(axS) * parseFloat(axS) + parseFloat(ayS) * parseFloat(ayS) + parseFloat(azS) * parseFloat(azS)) : null);
  chart.update();
}

function showNotification(message) {
  const notification = document.createElement('div');
  notification.className = 'notification';
  notification.textContent = message;
  document.body.appendChild(notification);

  setTimeout(() => {
    notification.classList.add('show');
  }, 100);

  setTimeout(() => {
    notification.classList.remove('show');
    setTimeout(() => {
      notification.remove();
    }, 500);
  }, 3000);
}

const temperatureChart = createSecondaryOnlyChart(
  document.getElementById('temperatureChart').getContext('2d'),
  'Temperatura',
  'Temperatura (°C)',
  '#ff9800'
);
const humidityChart = createSecondaryOnlyChart(
  document.getElementById('humidityChart').getContext('2d'),
  'Humedad',
  'Humedad (%)',
  '#4caf50'
);
const pressureChart = createSecondaryOnlyChart(
  document.getElementById('pressureChart').getContext('2d'),
  'Presión',
  'Presión (hPa)',
  '#673ab7'
);
const altitudeChart = createDualLineChart(
  document.getElementById('altitudeChart').getContext('2d'),
  'Altitud',
  '#4caf50',
  '#f44336'
);
const accelChart = createAccelChart();
const windChart = createSingleLineChart(
  document.getElementById('windChart').getContext('2d'),
  'Viento',
  '#00bcd4'
);

const velocityChart = new Chart(document.getElementById('velocityChart').getContext('2d'), {
  type: 'line',
  data: {
    labels: [],
    datasets: [
      {
        label: 'Velocidad (Primaria)',
        data: [],
        borderColor: '#9c27b0', // Morado
        backgroundColor: 'transparent',
        borderWidth: 2,
        pointRadius: 2,
        fill: false
      },
      {
        label: 'Velocidad (Secundaria)',
        data: [],
        borderColor: '#ffeb3b',
        backgroundColor: 'transparent',
        borderWidth: 2,
        pointRadius: 2,
        fill: false
      }
    ]
  },
  options: {
    responsive: true,
    maintainAspectRatio: false,
    scales: {
      x: {
        type: 'category',
        title: { display: true, text: 'Hora', font: { size: 12 } },
        ticks: { color: '#ffffff', font: { size: 10 } }
      },
      y: {
        beginAtZero: true,
        title: { display: true, text: 'Velocidad (m/s)', font: { size: 12 } },
        ticks: { color: '#ffffff', font: { size: 10 } }
      }
    },
    plugins: {
      legend: { labels: { color: '#ffffff' } }
    },
    elements: {
      line: {
        tension: 0,
        spanGaps: true
      }
    }
  }
});

const distanceChart = createDistanceChart(
  document.getElementById('distanceChart').getContext('2d')
);

window.api.onPrimaryData((data) => {
  lastPrimaryDataTime = Date.now();

  updateSingleLine(windChart, data.speed, data.time);
  updateAccelChart(accelChart, data.accelx, data.accely, data.accelz, null, null, null, data.time);
  updateAltitudeChart(altitudeChart, data.relativeAltitude, null, data.altitude, null, data.time);
  updateDualLine(velocityChart, data.velocity, null, data.time);

  document.getElementById('windValue').textContent = `${data.speed} m/s`;
  document.getElementById('accelValuePrimary').textContent = `${data.atotal} g`;
  document.getElementById('altitudeValuePrimary').textContent = `${data.relativeAltitude} m`;
  document.getElementById('absoluteAltitudeValuePrimary').textContent = `${data.altitude} m`;
  document.getElementById('velocityValuePrimary').textContent = `${data.velocity} m/s`;

  if (data.decouplingStatus) {
    showNotification('Rele activado con éxito');
  }
});

window.api.onSecondaryData((data) => {
  lastSecondaryDataTime = Date.now();

  updateDualLine(temperatureChart, null, data.temperature, data.time);
  updateDualLine(humidityChart, null, data.humidity, data.time);
  updateDualLine(pressureChart, null, data.pressure, data.time);
  updateAltitudeChart(altitudeChart, null, data.relativeAltitude, null, data.altitude, data.time);
  updateAccelChart(accelChart, null, null, null, data.accelx, data.accely, data.accelz, data.time);
  updateDualLine(velocityChart, null, data.velocity, data.time);

  document.getElementById('tempValueSecondary').textContent = `${data.temperature}°C`;
  document.getElementById('humidityValueSecondary').textContent = `${data.humidity}%`;
  document.getElementById('pressureValueSecondary').textContent = `${data.pressure} hPa`;
  document.getElementById('accelValueSecondary').textContent = `${data.atotal} g`;
  document.getElementById('altitudeValueSecondary').textContent = `${data.relativeAltitude} m`;
  document.getElementById('absoluteAltitudeValueSecondary').textContent = `${data.altitude} m`;
  document.getElementById('velocityValueSecondary').textContent = `${data.velocity} m/s`;
});

window.api.onDistanceData((data) => {
  updateSingleLine(distanceChart, data.distance, data.time);
  document.getElementById('distanceValue').textContent = `${data.distance} m`;
});

window.api.onError((message) => {
  showNotification(`Error: ${message}`);
});

window.api.onReportGenerated((data) => {
  showNotification(data.message);
});

window.api.onSimulationStatus((data) => {
  showNotification(data.message);
});

window.onload = async () => {
  const select = document.getElementById('serialPortSelect');
  try {
    const ports = await window.api.listSerialPorts();
    ports.forEach(port => {
      const option = document.createElement('option');
      option.value = port.path;
      option.text = `${port.path} (${port.manufacturer || 'Desconocido'})`;
      select.appendChild(option);
    });

    select.addEventListener('change', async (e) => {
      if (e.target.value) {
        const result = await window.api.setSerialPort(e.target.value);
        if (!result.success) {
          showNotification(`Error: ${result.message}`);
        }
      }
    });
  } catch (err) {
    showNotification('Error al cargar los puertos seriales: ' + err.message);
  }
};

document.getElementById('generateReportBtn').addEventListener('click', () => {
  window.api.generateReport();
});

setInterval(() => {
  const now = Date.now();
  const threshold = 5000;

  const primaryStatus = document.getElementById('primaryStatus');
  const secondaryStatus = document.getElementById('secondaryStatus');

  if (lastPrimaryDataTime && (now - lastPrimaryDataTime) < threshold) {
    primaryStatus.textContent = '✅ Recibiendo datos de carga primaria';
    primaryStatus.style.color = '#00ff00';
  } else {
    primaryStatus.textContent = '⚠️ No se reciben datos de carga primaria';
    primaryStatus.style.color = '#ff0000';
  }

  if (lastSecondaryDataTime && (now - lastSecondaryDataTime) < threshold) {
    secondaryStatus.textContent = '✅ Recibiendo datos de carga secundaria';
    secondaryStatus.style.color = '#00ff00';
  } else {
    secondaryStatus.textContent = '⚠️ No se reciben datos de carga secundaria';
    secondaryStatus.style.color = '#ff0000';
  }
}, 1000);
