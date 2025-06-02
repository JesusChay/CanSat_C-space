console.log('Chart cargado:', Chart);
console.log('THREE cargado:', THREE);

window.onload = () => {
  console.log('window.onload ejecutado');

  // Configurar las pestañas
  let map = null;

  function initializeMap() {
    if (!map) {
      map = L.map('map').setView([19.4326, -99.1332], 13);
      L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
      }).addTo(map);
      path = L.polyline([], { color: 'red' }).addTo(map);
    }
  }

  function openTab(tabName) {
    const tabContents = document.getElementsByClassName("tab-content");
    for (let i = 0; i < tabContents.length; i++) {
      tabContents[i].classList.remove("active");
    }
    const tabButtons = document.getElementsByClassName("tab-button");
    for (let i = 0; i < tabButtons.length; i++) {
      tabButtons[i].classList.remove("active");
    }
    document.getElementById(tabName).classList.add("active");
    tabButtons[Array.from(tabButtons).findIndex(btn => btn.dataset.tab === tabName)].classList.add("active");

    if (tabName === 'mapTab') {
      initializeMap();
    }
  }

  const tabButtons = document.getElementsByClassName("tab-button");
  for (let i = 0; i < tabButtons.length; i++) {
    tabButtons[i].addEventListener("click", () => {
      const tabName = tabButtons[i].dataset.tab;
      openTab(tabName);
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
          borderWidth: 2
        }]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        scales: {
          x: {
            type: 'linear',
            title: { display: true, text: 'Tiempo (s)', font: { size: 12 } },
            ticks: { color: '#ffffff', font: { size: 10 } }
          },
          y: {
            beginAtZero: true,
            title: { display: true, text: 'Magnitud', font: { size: 12 } },
            ticks: { color: '#ffffff', font: { size: 10 } }
          }
        },
        plugins: {
          legend: { labels: { color: '#ffffff' } }
        }
      }
    });
  }

  function createAccelChart(ctx) {
    return new Chart(ctx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          { label: 'Ax', data: [], borderColor: '#ff6384', backgroundColor: 'transparent', borderWidth: 2 },
          { label: 'Ay', data: [], borderColor: '#36a2eb', backgroundColor: 'transparent', borderWidth: 2 },
          { label: 'Az', data: [], borderColor: '#cc65fe', backgroundColor: 'transparent', borderWidth: 2 },
          { label: 'Atotal', data: [], borderColor: '#4bc0c0', backgroundColor: 'transparent', borderWidth: 2 }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        scales: {
          x: {
            type: 'linear',
            title: { display: true, text: 'Tiempo (s)', font: { size: 12 } },
            ticks: { color: '#ffffff', font: { size: 10 } }
          },
          y: {
            beginAtZero: true,
            title: { display: true, text: 'Magnitud', font: { size: 12 } },
            ticks: { color: '#ffffff', font: { size: 10 } }
          }
        },
        plugins: {
          legend: { labels: { color: '#ffffff' } }
        }
      }
    });
  }

  function updateSingleLine(chart, value) {
    if (chart.data.labels.length > 50) {
      chart.data.labels.shift();
      chart.data.datasets[0].data.shift();
    }
    let last = chart.data.labels.length ? chart.data.labels[chart.data.labels.length - 1] : 0;
    chart.data.labels.push(last + 0.5);
    chart.data.datasets[0].data.push(value);
    chart.update();
  }

  function updateAccelChart(chart, ax, ay, az) {
    if (chart.data.labels.length > 50) {
      chart.data.labels.shift();
      chart.data.datasets.forEach(ds => ds.data.shift());
    }
    let last = chart.data.labels.length ? chart.data.labels[chart.data.labels.length - 1] : 0;
    chart.data.labels.push(last + 0.5);
    chart.data.datasets[0].data.push(ax);
    chart.data.datasets[1].data.push(ay);
    chart.data.datasets[2].data.push(az);
    let atotal = Math.sqrt(ax * ax + ay * ay + az * az);
    chart.data.datasets[3].data.push(atotal);
    chart.update();
  }

  const temperatureChart = createSingleLineChart(
    document.getElementById('temperatureChart').getContext('2d'),
    'Temperatura',
    '#f56954'
  );
  const humidityChart = createSingleLineChart(
    document.getElementById('humidityChart').getContext('2d'),
    'Humedad',
    '#36a2eb'
  );
  const pressureChart = createSingleLineChart(
    document.getElementById('pressureChart').getContext('2d'),
    'Presión',
    '#ffce56'
  );
  const altitudeChart = createSingleLineChart(
    document.getElementById('altitudeChart').getContext('2d'),
    'Altura',
    '#4bc0c0'
  );
  const accelChart = createAccelChart(
    document.getElementById('accelChart').getContext('2d')
  );

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(75, 1, 0.1, 1000);
  const renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
  renderer.setSize(280, 180);
  document.getElementById('model3d').appendChild(renderer.domElement);

  const geometry = new THREE.BoxGeometry(1, 1, 1);
  const material = new THREE.MeshPhongMaterial({ color: 0x00ff00 });
  const cube = new THREE.Mesh(geometry, material);
  scene.add(cube);

  cube.position.set(0, 0, 0);

  const light = new THREE.DirectionalLight(0xffffff, 1);
  light.position.set(1, 1, 2).normalize();
  scene.add(light);

  camera.position.z = 2;

  let gyroX = 0, gyroY = 0, gyroZ = 0;

  let marker = null;
  let path = L.polyline([], { color: 'red' });
  let pathCoordinates = [];

  let time = 0;

  function generateSimulatedData() {
    time += 0.5;

    const temp = 25 + 5 * Math.sin(time * 0.1) + (Math.random() - 0.5) * 1;
    const ax = Math.sin(time * 0.2) + (Math.random() - 0.5) * 0.2;
    const ay = Math.cos(time * 0.2) + (Math.random() - 0.5) * 0.2;
    const az = 1 + Math.sin(time * 0.15) * 0.5 + (Math.random() - 0.5) * 0.2;
    const gx = 0.3 * Math.sin(time * 0.3) + (Math.random() - 0.5) * 0.05;
    const gy = 0.3 * Math.cos(time * 0.3) + (Math.random() - 0.5) * 0.05;
    const gz = 0.2 * Math.sin(time * 0.25) + (Math.random() - 0.5) * 0.05;
    const humidity = 50 + 20 * Math.sin(time * 0.08) + (Math.random() - 0.5) * 5;
    const pressure = 1000 + 10 * Math.cos(time * 0.1) + (Math.random() - 0.5) * 2;
    const altitude = 250 + 250 * Math.sin(time * 0.05) + (Math.random() - 0.5) * 10;
    const baseLat = 19.4326;
    const baseLon = -99.1332;
    const lat = baseLat + (Math.sin(time * 0.1) * 0.01) + (Math.random() - 0.5) * 0.001;
    const lon = baseLon + (Math.cos(time * 0.1) * 0.01) + (Math.random() - 0.5) * 0.001;

    const dataLine = `${temp.toFixed(2)},${ax.toFixed(2)},${ay.toFixed(2)},${az.toFixed(2)},${gx.toFixed(2)},${gy.toFixed(2)},${gz.toFixed(2)},${humidity.toFixed(2)},${pressure.toFixed(2)},${altitude.toFixed(2)},${lat.toFixed(6)},${lon.toFixed(6)}`;

    return dataLine;
  }

  function processSimulatedData() {
    const line = generateSimulatedData();
    console.log('Datos simulados:', line);

    try {
      const values = line.split(',').map(parseFloat);
      if (values.length !== 12 || values.some(v => isNaN(v))) {
        console.error('Datos simulados incompletos o inválidos:', values);
        return;
      }

      const [temp, ax, ay, az, gx, gy, gz, humidity, pressure, altitude, lat, lon] = values;

      updateSingleLine(temperatureChart, temp);
      updateSingleLine(humidityChart, humidity);
      updateSingleLine(pressureChart, pressure);
      updateSingleLine(altitudeChart, altitude);
      updateAccelChart(accelChart, ax, ay, az);

      document.getElementById('tempValue').textContent = `${temp.toFixed(2)}°C`;
      document.getElementById('humidityValue').textContent = `${humidity.toFixed(2)}%`;
      document.getElementById('pressureValue').textContent = `${pressure.toFixed(2)} hPa`;
      document.getElementById('accelValue').textContent = `${Math.sqrt(ax * ax + ay * ay + az * az).toFixed(2)} g`;
      document.getElementById('altitudeValue').textContent = `${altitude.toFixed(2)} m`;

      document.getElementById('gyroX').textContent = gx.toFixed(2);
      document.getElementById('gyroY').textContent = gy.toFixed(2);
      document.getElementById('gyroZ').textContent = gz.toFixed(2);

      gyroX = gx * 0.5;
      gyroY = gy * 0.5;
      gyroZ = gz * 0.5;

      const coords = [lat, lon];
      pathCoordinates.push(coords);
      if (map) {
        path.setLatLngs(pathCoordinates);
        if (marker) {
          marker.setLatLng(coords);
        } else {
          marker = L.marker(coords).addTo(map);
        }
        map.setView(coords, 13);
      }
    } catch (error) {
      console.error('Error al parsear datos simulados:', error.message);
    }
  }

  setInterval(processSimulatedData, 500);

  function animate() {
    requestAnimationFrame(animate);
    cube.rotation.x = gyroX;
    cube.rotation.y = gyroY;
    cube.rotation.z = gyroZ;
    renderer.render(scene, camera);
  }

  animate();
};