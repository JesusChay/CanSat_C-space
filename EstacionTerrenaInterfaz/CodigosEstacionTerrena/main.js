const { app, BrowserWindow, ipcMain } = require('electron');
const path = require('path');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const fs = require('fs');
const XLSX = require('xlsx');

let dashboardWindow, mapWindow, model3dWindow;
let serialPort, parser;
let primaryDataLog = [];
let secondaryDataLog = [];
let distanceDataLog = [];
let simulationInterval = null;
let lastPrimaryTime = null;
let lastSecondaryTime = null;

class KalmanFilter {
    constructor() {
        this.x = [[0], [0]];
        this.P = [[1000, 0], [0, 1000]];
        this.A = [[1, 0], [0, 1]];
        this.B = [[0], [0]];
        this.H = [[1, 0]];
        this.Q = [[0.01, 0], [0, 0.01]];
        this.R = [[10]];
    }

    multiplyMatrix(A, B) {
        if (!A[0] || !B[0] || A[0].length !== B.length) {
            console.error('Error de multiplicación de matrices: Dimensiones incompatibles');
            console.error('A dimensiones:', A.length, A[0] ? A[0].length : 0);
            console.error('B dimensiones:', B.length, B[0] ? B[0].length : 0);
            console.error('A:', A);
            console.error('B:', B);
            throw new Error('Dimensiones de matrices no compatibles');
        }
        const rowsA = A.length, colsA = A[0].length, colsB = B[0].length;
        const result = Array(rowsA).fill().map(() => Array(colsB).fill(0));
        for (let i = 0; i < rowsA; i++) {
            for (let j = 0; j < colsB; j++) {
                for (let k = 0; k < colsA; k++) {
                    result[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return result;
    }

    transpose(A) {
        return A[0].map((_, colIndex) => A.map(row => row[colIndex]));
    }

    inverseMatrix1x1(A) {
        if (A[0][0] === 0) {
            throw new Error('Matriz no invertible (división por cero)');
        }
        return [[1 / A[0][0]]];
    }

    predict(u, dt) {
        if (dt <= 0 || isNaN(dt)) {
            console.warn('Dato invalido:', dt);
            return;
        }
        this.A = [[1, dt], [0, 1]];
        this.B = [[dt * dt / 2], [dt]];

        const Ax = this.multiplyMatrix(this.A, this.x);
        const Bu = this.multiplyMatrix(this.B, [[u]]);
        this.x = [[Ax[0][0] + Bu[0][0]], [Ax[1][0] + Bu[1][0]]];

        const P_A = this.multiplyMatrix(this.P, this.transpose(this.A));
        this.P = this.multiplyMatrix(this.A, P_A);
        for (let i = 0; i < 2; i++) {
            for (let j = 0; j < 2; j++) {
                this.P[i][j] += this.Q[i][j];
            }
        }
    }

    update(z) {
        if (isNaN(z) || z < -1000 || z > 10000) {
            console.warn('Medición de altura inválida:', z);
            return;
        }

        const Ht = this.transpose(this.H);
        const H_P = this.multiplyMatrix(this.H, this.P);
        const H_P_Ht = this.multiplyMatrix(H_P, Ht);
        const H_P_Ht_R = [[H_P_Ht[0][0] + this.R[0][0]]];
        const K_num = this.multiplyMatrix(this.P, Ht);
        const K_den = this.inverseMatrix1x1(H_P_Ht_R);
        const K = this.multiplyMatrix(K_num, K_den);

        const Hx = this.multiplyMatrix(this.H, this.x);
        const innovation = z - Hx[0][0];
        this.x[0][0] += K[0][0] * innovation;
        this.x[1][0] += K[1][0] * innovation;

        const KH = this.multiplyMatrix(K, this.H);
        const I_KH = [[1 - KH[0][0], -KH[0][1]], [-KH[1][0], 1 - KH[1][1]]];
        this.P = this.multiplyMatrix(I_KH, this.P);
    }

    getState() {
        return { relativeAltitude: this.x[0][0], velocityZ: this.x[1][0] };
    }
}

let primaryKalman = null;
let secondaryKalman = null;

function createWindows() {
    dashboardWindow = new BrowserWindow({
        width: 1200,
        height: 700,
        webPreferences: {
            nodeIntegration: false,
            contextIsolation: true,
            enableRemoteModule: false,
            preload: path.join(__dirname, 'preload.js')
        }
    });
    dashboardWindow.loadFile('dashboard.html');

    dashboardWindow.on('close', () => {
        if (mapWindow) {
            mapWindow.close();
        }
        if (model3dWindow) {
            model3dWindow.close();
        }
    });

    mapWindow = new BrowserWindow({
        width: 800,
        height: 600,
        webPreferences: {
            nodeIntegration: false,
            contextIsolation: true,
            enableRemoteModule: false,
            preload: path.join(__dirname, 'preload.js')
        }
    });
    mapWindow.loadFile('map.html');

    model3dWindow = new BrowserWindow({
        width: 800,
        height: 600,
        webPreferences: {
            nodeIntegration: false,
            contextIsolation: true,
            enableRemoteModule: false,
            preload: path.join(__dirname, 'preload.js')
        }
    });
    model3dWindow.loadFile('model3d.html');
}

app.whenReady().then(() => {
    createWindows();
    app.on('activate', () => {
        if (BrowserWindow.getAllWindows().length === 0) createWindows();
    });
});

app.on('window-all-closed', () => {
    if (process.platform !== 'darwin') app.quit();
});

function generateRandom(min, max) {
    return (Math.random() * (max - min) + min).toFixed(2);
}

function simulateData() {
    const primaryData = [
        generateRandom(0, 20), // speed (km/h)
        generateRandom(-2, 2), // accelx (g)
        generateRandom(-2, 2), // accely
        generateRandom(-2, 2), // accelz
        generateRandom(-180, 180), // gyrox (°/s)
        generateRandom(-180, 180), // gyroy
        generateRandom(-180, 180), // gyroz
        generateRandom(-100, 100), // magx (μT)
        generateRandom(-100, 100), // magy
        generateRandom(-100, 100), // magz
        generateRandom(0, 1000), // altitude (m)
        generateRandom(19.5, 19.7), // latitude
        generateRandom(-99.2, -99.0), // longitude
        Math.random() > 0.9 ? 'true' : 'false' // decoupling_status
    ].join(',');

    const secondaryData = [
        generateRandom(15, 35), // temperature (°C)
        generateRandom(-2, 2), // accelx (g)
        generateRandom(-2, 2), // accely
        generateRandom(-2, 2), // accelz
        generateRandom(-180, 180), // gyrox (°/s)
        generateRandom(-180, 180), // gyroy
        generateRandom(-180, 180), // gyroz
        generateRandom(20, 80), // humidity (%)
        generateRandom(95000, 105000), // pressure (Pa)
        generateRandom(0, 1000), // altitude (m)
        generateRandom(19.5, 19.7), // latitude
        generateRandom(-99.2, -99.0) // longitude
    ].join(',');

    processPrimaryData(primaryData);
    processSecondaryData(secondaryData);
}

function initializeSerialPort(portName, baudRate = 115200) {
    if (serialPort) {
        serialPort.close(() => {
            console.log('Puerto serial anterior cerrado');
        });
    }

    try {
        serialPort = new SerialPort({ path: portName, baudRate: parseInt(baudRate) });
        parser = serialPort.pipe(new ReadlineParser({ delimiter: '\n' }));

        parser.on('data', (line) => {
            const trimmed = line.trim();
            if (trimmed.startsWith('[PRIMARY]')) {
                const data = trimmed.replace('[PRIMARY]', '').trim();
                processPrimaryData(data);
            } else if (trimmed.startsWith('[SECONDARY]')) {
                const data = trimmed.replace('[SECONDARY]', '').trim();
                processSecondaryData(data);
            } else {
                console.warn('⚠️ Línea no reconocida:', trimmed);
            }
        });

        serialPort.on('error', (err) => {
            console.error('❌ Error en el puerto serial:', err.message);
            dashboardWindow.webContents.send('error', 'Error en el puerto serial: ' + err.message);
        });

        serialPort.on('open', () => {
            console.log('✅ Puerto serial abierto:', portName);
            dashboardWindow.webContents.send('simulation-status', { message: `Conectado al puerto ${portName}` });
        });
    } catch (err) {
        console.error('❌ Error al inicializar el puerto:', err.message);
        dashboardWindow.webContents.send('error', 'No se pudo inicializar el puerto serial: ' + portName);
    }
}

ipcMain.handle('list-serial-ports', async () => {
    try {
        const ports = await SerialPort.list();
        return ports.map(port => ({ path: port.path, manufacturer: port.manufacturer || 'Desconocido' }));
    } catch (err) {
        console.error('❌ Error al listar puertos:', err.message);
        dashboardWindow.webContents.send('error', 'No se pudieron listar los puertos seriales.');
        return [];
    }
});

ipcMain.handle('set-serial-port', async (event, portName) => {
    try {
        if (portName === 'simulation') {
            if (serialPort) {
                serialPort.close(() => {
                    console.log('Puerto serial cerrado para iniciar simulación');
                });
                serialPort = null;
                parser = null;
            }
            if (simulationInterval) {
                clearInterval(simulationInterval);
            }
            simulationInterval = setInterval(simulateData, 500);
            console.log('✅ Modo simulación activado');
            return { success: true, message: 'Modo simulación activado' };
        } else {
            if (simulationInterval) {
                clearInterval(simulationInterval);
                simulationInterval = null;
                console.log('Modo simulación desactivado');
            }
            initializeSerialPort(portName);
            return { success: true, message: `Puerto ${portName} seleccionado correctamente.` };
        }
    } catch (err) {
        console.error('❌ Error al establecer el puerto:', err.message);
        dashboardWindow.webContents.send('error', `No se pudo establecer el puerto ${portName}.`);
        return { success: false, message: `Error al establecer el puerto ${portName}.` };
    }
});

function calculateDistance(lat1, lon1, lat2, lon2) {
    if (!lat1 || !lon1 || !lat2 || !lon2) return 0;
    const toRad = x => x * Math.PI / 180;
    const R = 6371e3;
    const φ1 = toRad(lat1);
    const φ2 = toRad(lat2);
    const Δφ = toRad(lat2 - lat1);
    const Δλ = toRad(lon2 - lon1);
    const a = Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
              Math.cos(φ1) * Math.cos(φ2) *
              Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
}

let lastPrimaryPosition = null;
let lastSecondaryPosition = null;

function processPrimaryData(data) {
    const parts = data.split(',');
    if (parts.length !== 14) {
        console.error(`❌ Primaria: Cantidad incorrecta de valores (${parts.length})`);
        return;
    }

    const numericValues = parts.slice(0, 13).map(v => parseFloat(v));
    if (numericValues.some(v => isNaN(v))) {
        console.error('❌ Primaria: Valor no numérico en datos numéricos:', parts.slice(0, 13));
        return;
    }

    const decoupling_status = parts[13].trim();
    const currentTime = new Date();
    const timeString = currentTime.toLocaleTimeString('fr-FR', { timeZone: 'Europe/Paris', hour12: false });
    const [speed, accelx, accely, accelz, gyrox, gyroy, gyroz, magx, magy, magz, altitude, latitude, longitude] = numericValues;
    const decouplingStatus = decoupling_status === 'true';

    let velocity = 0;
    if (lastPrimaryPosition) {
        const dist = calculateDistance(
            lastPrimaryPosition.latitude,
            lastPrimaryPosition.longitude,
            latitude,
            longitude
        );
        velocity = dist / 0.5;
    }

    const GRAVITY = 9.81;
    let relativeAltitude = 0;
    let velocityZ = 0;
    if (!isNaN(altitude)) {
        if (!primaryKalman) {
            primaryKalman = new KalmanFilter();
        }
        const deltaTime = lastPrimaryTime ? (currentTime - lastPrimaryTime) / 1000 : 0.5;
        if (deltaTime > 0 && deltaTime < 1) {
            const accelZNet = accelz * GRAVITY - GRAVITY;
            primaryKalman.predict(accelZNet, deltaTime);
            primaryKalman.update(altitude);
            const state = primaryKalman.getState();
            relativeAltitude = state.relativeAltitude;
            velocityZ = state.velocityZ;
            const accelTotal = Math.sqrt(accelx * accelx + accely * accely + accelz * accelz);
            if (Math.abs(accelTotal - 1) < 0.05 && Math.abs(velocityZ) < 0.1 && Math.abs(altitude) < 10) {
                relativeAltitude = 0;
                velocityZ = 0;
                primaryKalman.x = [0, 0];
                primaryKalman.P = [[1000, 0], [0, 1000]];
            }
        }
    } else {
        console.warn('Altura absoluta inválida, omitiendo filtro de Kalman');
    }

    lastPrimaryTime = currentTime;
    lastPrimaryPosition = { latitude, longitude };

    const primaryData = {
        time: timeString,
        speed: (speed / 3.6).toFixed(2), //km/h a m/s
        accelx: accelx.toFixed(2),
        accely: accely.toFixed(2),
        accelz: accelz.toFixed(2),
        atotal: Math.sqrt(accelx * accelx + accely * accely + accelz * accelz).toFixed(2),
        gyrox: (gyrox * 0.0174533).toFixed(2), //°/s a rad/s
        gyroy: (gyroy * 0.0174533).toFixed(2),
        gyroz: (gyroz * 0.0174533).toFixed(2),
        magx: magx.toFixed(2),
        magy: magy.toFixed(2),
        magz: magz.toFixed(2),
        altitude: altitude.toFixed(2),
        latitude: latitude.toFixed(6),
        longitude: longitude.toFixed(6),
        velocity: velocity.toFixed(2),
        velocityZ: velocityZ.toFixed(2),
        relativeAltitude: relativeAltitude.toFixed(2),
        decouplingStatus: decouplingStatus
    };

    primaryDataLog.push(primaryData);
    dashboardWindow.webContents.send('primary-data', primaryData);
    mapWindow.webContents.send('primary-data', { latitude, longitude });
    model3dWindow.webContents.send('primary-data', { gyrox: gyrox * 0.0174533, gyroy: gyroy * 0.0174533, gyroz: gyroz * 0.0174533 });
    updateDistance();
}

function processSecondaryData(data) {
    const parts = data.split(',').map(part => part.trim());
    if (parts.length !== 12) {
        console.error(`❌ Secundaria: Cantidad incorrecta de valores (${parts.length}). Datos recibidos: ${parts}`);
        return;
    }

    const values = parts.map(v => parseFloat(v));
    if (values.some(v => isNaN(v))) {
        console.error('❌ Secundaria: Valor no numérico:', parts);
        return;
    }

    const currentTime = new Date();
    const timeString = currentTime.toLocaleTimeString('fr-FR', { timeZone: 'Europe/Paris', hour12: false });
    const [temperature, accelx, accely, accelz, gyrox, gyroy, gyroz, humidity, pressure, altitude, latitude, longitude] = values;

    let velocity = 0;
    if (lastSecondaryPosition) {
        const dist = calculateDistance(
            lastSecondaryPosition.latitude,
            lastSecondaryPosition.longitude,
            latitude,
            longitude
        );
        velocity = dist / 0.5;
    }

    const GRAVITY = 9.81;
    let relativeAltitude = 0;
    let velocityZ = 0;
    if (!isNaN(altitude)) {
        if (!secondaryKalman) {
            secondaryKalman = new KalmanFilter();
        }
        const deltaTime = lastSecondaryTime ? (currentTime - lastSecondaryTime) / 1000 : 0.5;
        if (deltaTime > 0 && deltaTime < 1) {
            const accelZNet = accelz * GRAVITY - GRAVITY;
            secondaryKalman.predict(accelZNet, deltaTime);
            secondaryKalman.update(altitude);
            const state = secondaryKalman.getState();
            relativeAltitude = state.relativeAltitude;
            velocityZ = state.velocityZ;

            const accelTotal = Math.sqrt(accelx * accelx + accely * accely + accelz * accelz);
            if (Math.abs(accelTotal - 1) < 0.05 && Math.abs(velocityZ) < 0.1 && Math.abs(altitude) < 10) {
                relativeAltitude = 0;
                velocityZ = 0;
                secondaryKalman.x = [0, 0];
                secondaryKalman.P = [[1000, 0], [0, 1000]];
            }
        }
    } else {
        console.warn('Altura absoluta inválida, omitiendo filtro de Kalman');
    }

    lastSecondaryTime = currentTime;
    lastSecondaryPosition = { latitude, longitude };

    const secondaryData = {
        time: timeString,
        temperature: temperature.toFixed(2),
        accelx: accelx.toFixed(2),
        accely: accely.toFixed(2),
        accelz: accelz.toFixed(2),
        atotal: Math.sqrt(accelx * accelx + accely * accely + accelz * accelz).toFixed(2),
        gyrox: (gyrox * 0.0174533).toFixed(2),
        gyroy: (gyroy * 0.0174533).toFixed(2),
        gyroz: (gyroz * 0.0174533).toFixed(2),
        humidity: humidity.toFixed(2),
        pressure: (pressure / 100).toFixed(2),
        altitude: altitude.toFixed(2),
        latitude: latitude.toFixed(6),
        longitude: longitude.toFixed(6),
        velocity: velocity.toFixed(2),
        velocityZ: velocityZ.toFixed(2),
        relativeAltitude: relativeAltitude.toFixed(2)
    };

    secondaryDataLog.push(secondaryData);
    dashboardWindow.webContents.send('secondary-data', secondaryData);
    mapWindow.webContents.send('secondary-data', { latitude, longitude });
    model3dWindow.webContents.send('secondary-data', { gyrox: gyrox * 0.0174533, gyroy: gyroy * 0.0174533, gyroz: gyroz * 0.0174533 });
    updateDistance();
}

function updateDistance() {
  if (lastPrimaryPosition && lastSecondaryPosition) {
    const currentTime = new Date().toLocaleTimeString('fr-FR', { timeZone: 'Europe/Paris', hour12: false });
    const distance = calculateDistance(
      lastPrimaryPosition.latitude,
      lastPrimaryPosition.longitude,
      lastSecondaryPosition.latitude,
      lastSecondaryPosition.longitude
    );
    dashboardWindow.webContents.send('distance-data', { time: currentTime, distance: distance.toFixed(2) });
    distanceDataLog.push({ time: currentTime, distance: distance.toFixed(2) });
  }
}

ipcMain.on('generate-report', () => {
    if (primaryDataLog.length === 0 && secondaryDataLog.length === 0) {
        console.log('No hay datos para generar el reporte');
        dashboardWindow.webContents.send('error', 'No hay datos para generar el reporte');
        return;
    }

    const reportsDir = path.join(app.getPath('documents'), 'KAAN_ASTRA_Reportes');
    if (!fs.existsSync(reportsDir)) {
        fs.mkdirSync(reportsDir, { recursive: true });
    }

    const headers = [
        'Tiempo',
        'Velocidad del viento (m/s) Primaria',
        'Aceleración X (g) Primaria', 'Aceleración Y (g) Primaria', 'Aceleración Z (g) Primaria', 'Aceleración Total (g) Primaria',
        'Giroscopio X (°/s) Primaria', 'Giroscopio Y (°/s) Primaria', 'Giroscopio Z (°/s) Primaria',
        'Magnetómetro X (μT) Primaria', 'Magnetómetro Y (μT) Primaria', 'Magnetómetro Z (μT) Primaria',
        'Altitud (m) Primaria', 'Altitud Alternativa (m) Primaria', 'Latitud Primaria', 'Longitud Primaria', 'Velocidad (m/s) Primaria',
        'Temperatura (°C) Secundaria', 'Aceleración X (g) Secundaria', 'Aceleración Y (g) Secundaria', 'Aceleración Z (g) Secundaria', 'Aceleración Total (g) Secundaria',
        'Giroscopio X (°/s) Secundaria', 'Giroscopio Y (°/s) Secundaria', 'Giroscopio Z (°/s) Secundaria',
        'Humedad (%) Secundaria', 'Presión (hPa) Secundaria', 'Altitud (m) Secundaria', 'Altitud Alternativa (m) Secundaria', 'Latitud Secundaria', 'Longitud Secundaria', 'Velocidad (m/s) Secundaria',
        'Distancia entre Cargas (m)'
    ];

    const maxLength = Math.max(primaryDataLog.length, secondaryDataLog.length, distanceDataLog.length);
    const data = [];
    for (let i = 0; i < maxLength; i++) {
        const primary = primaryDataLog[i] || {
            time: '',
            speed: '0.00',
            accelx: '0.00',
            accely: '0.00',
            accelz: '0.00',
            atotal: '0.00',
            gyrox: '0.00',
            gyroy: '0.00',
            gyroz: '0.00',
            magx: '0.00',
            magy: '0.00',
            magz: '0.00',
            altitude: '0.00',
            relativeAltitude: '0.00',
            latitude: '0.000000',
            longitude: '0.000000',
            velocity: '0.00'
        };
        const secondary = secondaryDataLog[i] || {
            time: '',
            temperature: '0.00',
            accelx: '0.00',
            accely: '0.00',
            accelz: '0.00',
            atotal: '0.00',
            gyrox: '0.00',
            gyroy: '0.00',
            gyroz: '0.00',
            humidity: '0.00',
            pressure: '0.00',
            altitude: '0.00',
            relativeAltitude: '0.00',
            latitude: '0.000000',
            longitude: '0.000000',
            velocity: '0.00'
        };
        const distanceEntry = distanceDataLog[i] || { time: '', distance: '0.00' };
        data.push([
            primary.time || secondary.time || distanceEntry.time || '',
            primary.speed,
            primary.accelx, primary.accely, primary.accelz, primary.atotal,
            primary.gyrox, primary.gyroy, primary.gyroz,
            primary.magx, primary.magy, primary.magz,
            primary.altitude, primary.relativeAltitude, primary.latitude, primary.longitude, primary.velocity,
            secondary.temperature,
            secondary.accelx, secondary.accely, secondary.accelz, secondary.atotal,
            secondary.gyrox, secondary.gyroy, secondary.gyroz,
            secondary.humidity, secondary.pressure,
            secondary.altitude, secondary.relativeAltitude, secondary.latitude, secondary.longitude, secondary.velocity,
            distanceEntry.distance
        ]);
    }

    const ws = XLSX.utils.aoa_to_sheet([headers, ...data]);
    ws['!cols'] = headers.map((_, index) => ({ wch: Math.max(headers[index].length, ...data.map(row => (row[index] ? row[index].toString().length : 0))) + 2 }));
    const wb = XLSX.utils.book_new();
    XLSX.utils.book_append_sheet(wb, ws, 'Reporte CanSat');

    const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
    const excelFilePath = path.join(reportsDir, `reporte-cansat-${timestamp}.xlsx`);
    XLSX.writeFile(wb, excelFilePath);
    console.log(`Archivo Excel guardado en: ${excelFilePath}`);

    const textFilePath = path.join(reportsDir, `reporte-cansat-analisis-${timestamp}.txt`);
    const duration = maxLength * 0.5;
    const samplesPrimary = primaryDataLog.length;
    const samplesSecondary = secondaryDataLog.length;
    const samplesDistance = distanceDataLog.length;

    const calculateStats = (data, key) => {
        const values = data.map(d => parseFloat(d[key])).filter(v => !isNaN(v));
        return {
            avg: values.length > 0 ? values.reduce((a, b) => a + b, 0) / values.length : 0,
            min: values.length > 0 ? Math.min(...values) : 0,
            max: values.length > 0 ? Math.max(...values) : 0
        };
    };

    const primaryStats = {
        speed: calculateStats(primaryDataLog, 'speed'),
        accelx: calculateStats(primaryDataLog, 'accelx'),
        accely: calculateStats(primaryDataLog, 'accely'),
        accelz: calculateStats(primaryDataLog, 'accelz'),
        atotal: calculateStats(primaryDataLog, 'atotal'),
        gyrox: calculateStats(primaryDataLog, 'gyrox'),
        gyroy: calculateStats(primaryDataLog, 'gyroy'),
        gyroz: calculateStats(primaryDataLog, 'gyroz'),
        magx: calculateStats(primaryDataLog, 'magx'),
        magy: calculateStats(primaryDataLog, 'magy'),
        magz: calculateStats(primaryDataLog, 'magz'),
        altitude: calculateStats(primaryDataLog, 'altitude'),
        relativeAltitude: calculateStats(primaryDataLog, 'relativeAltitude'),
        velocity: calculateStats(primaryDataLog, 'velocity')
    };

    const secondaryStats = {
        temperature: calculateStats(secondaryDataLog, 'temperature'),
        accelx: calculateStats(secondaryDataLog, 'accelx'),
        accely: calculateStats(secondaryDataLog, 'accely'),
        accelz: calculateStats(secondaryDataLog, 'accelz'),
        atotal: calculateStats(secondaryDataLog, 'atotal'),
        gyrox: calculateStats(secondaryDataLog, 'gyrox'),
        gyroy: calculateStats(secondaryDataLog, 'gyroy'),
        gyroz: calculateStats(secondaryDataLog, 'gyroz'),
        humidity: calculateStats(secondaryDataLog, 'humidity'),
        pressure: calculateStats(secondaryDataLog, 'pressure'),
        altitude: calculateStats(secondaryDataLog, 'altitude'),
        relativeAltitude: calculateStats(secondaryDataLog, 'relativeAltitude'),
        velocity: calculateStats(secondaryDataLog, 'velocity')
    };

    const distanceStats = calculateStats(distanceDataLog, 'distance');

    let txtContent = 'Reporte de Análisis CanSat\n';
    txtContent += `Generado el: ${new Date().toLocaleString('fr-FR', { timeZone: 'Europe/Paris' })}\n`;
    txtContent += `Modo: ${simulationInterval ? 'Simulación' : 'Datos Reales'}\n\n`;

    txtContent += 'Resumen General:\n';
    txtContent += `- Duración total estimada: ${duration.toFixed(2)} segundos\n`;
    txtContent += `- Número de muestras (Primaria): ${samplesPrimary}\n`;
    txtContent += `- Número de muestras (Secundaria): ${samplesSecondary}\n`;
    txtContent += `- Número de muestras (Distancia): ${samplesDistance}\n\n`;

    txtContent += 'Estadísticas de Magnitudes:\n\n';

    txtContent += 'Carga Primaria:\n';
    txtContent += '1. Velocidad del Viento (m/s):\n';
    txtContent += `   - Promedio: ${primaryStats.speed.avg.toFixed(2)}\n`;
    txtContent += `   - Mínimo: ${primaryStats.speed.min.toFixed(2)}\n`;
    txtContent += `   - Máximo: ${primaryStats.speed.max.toFixed(2)}\n\n`;

    txtContent += '2. Aceleración (g):\n';
    txtContent += `   - X Promedio: ${primaryStats.accelx.avg.toFixed(2)}\n`;
    txtContent += `   - Y Promedio: ${primaryStats.accely.avg.toFixed(2)}\n`;
    txtContent += `   - Z Promedio: ${primaryStats.accelz.avg.toFixed(2)}\n`;
    txtContent += `   - Total Promedio: ${primaryStats.atotal.avg.toFixed(2)}\n`;
    txtContent += `   - Total Mínimo: ${primaryStats.atotal.min.toFixed(2)}\n`;
    txtContent += `   - Total Máximo: ${primaryStats.atotal.max.toFixed(2)}\n\n`;

    txtContent += '3. Giroscopio (°/s):\n';
    txtContent += `   - X Promedio: ${primaryStats.gyrox.avg.toFixed(2)}\n`;
    txtContent += `   - Y Promedio: ${primaryStats.gyroy.avg.toFixed(2)}\n`;
    txtContent += `   - Z Promedio: ${primaryStats.gyroz.avg.toFixed(2)}\n\n`;

    txtContent += '4. Magnetómetro (μT):\n';
    txtContent += `   - X Promedio: ${primaryStats.magx.avg.toFixed(2)}\n`;
    txtContent += `   - Y Promedio: ${primaryStats.magy.avg.toFixed(2)}\n`;
    txtContent += `   - Z Promedio: ${primaryStats.magz.avg.toFixed(2)}\n\n`;

    txtContent += '5. Altitud (m):\n';
    txtContent += `   - Promedio: ${primaryStats.altitude.avg.toFixed(2)}\n`;
    txtContent += `   - Mínimo: ${primaryStats.altitude.min.toFixed(2)}\n`;
    txtContent += `   - Máximo: ${primaryStats.altitude.max.toFixed(2)}\n\n`;

    txtContent += '6. Altitud Alternativa (m):\n';
    txtContent += `   - Promedio: ${primaryStats.relativeAltitude.avg.toFixed(2)}\n`;
    txtContent += `   - Mínimo: ${primaryStats.relativeAltitude.min.toFixed(2)}\n`;
    txtContent += `   - Máximo: ${primaryStats.relativeAltitude.max.toFixed(2)}\n\n`;

    txtContent += '7. Velocidad de Desplazamiento (m/s):\n';
    txtContent += `   - Promedio: ${primaryStats.velocity.avg.toFixed(2)}\n`;
    txtContent += `   - Mínimo: ${primaryStats.velocity.min.toFixed(2)}\n`;
    txtContent += `   - Máximo: ${primaryStats.velocity.max.toFixed(2)}\n\n`;

    txtContent += 'Carga Secundaria:\n';
    txtContent += '1. Temperatura (°C):\n';
    txtContent += `   - Promedio: ${secondaryStats.temperature.avg.toFixed(2)}\n`;
    txtContent += `   - Mínimo: ${secondaryStats.temperature.min.toFixed(2)}\n`;
    txtContent += `   - Máximo: ${secondaryStats.temperature.max.toFixed(2)}\n\n`;

    txtContent += '2. Aceleración (g):\n';
    txtContent += `   - X Promedio: ${secondaryStats.accelx.avg.toFixed(2)}\n`;
    txtContent += `   - Y Promedio: ${secondaryStats.accely.avg.toFixed(2)}\n`;
    txtContent += `   - Z Promedio: ${secondaryStats.accelz.avg.toFixed(2)}\n`;
    txtContent += `   - Total Promedio: ${secondaryStats.atotal.avg.toFixed(2)}\n`;
    txtContent += `   - Total Mínimo: ${secondaryStats.atotal.min.toFixed(2)}\n`;
    txtContent += `   - Total Máximo: ${secondaryStats.atotal.max.toFixed(2)}\n\n`;

    txtContent += '3. Giroscopio (°/s):\n';
    txtContent += `   - X Promedio: ${secondaryStats.gyrox.avg.toFixed(2)}\n`;
    txtContent += `   - Y Promedio: ${secondaryStats.gyroy.avg.toFixed(2)}\n`;
    txtContent += `   - Z Promedio: ${secondaryStats.gyroz.avg.toFixed(2)}\n\n`;

    txtContent += '4. Humedad (%):\n';
    txtContent += `   - Promedio: ${secondaryStats.humidity.avg.toFixed(2)}\n`;
    txtContent += `   - Mínimo: ${secondaryStats.humidity.min.toFixed(2)}\n`;
    txtContent += `   - Máximo: ${secondaryStats.humidity.max.toFixed(2)}\n\n`;

    txtContent += '5. Presión (hPa):\n';
    txtContent += `   - Promedio: ${secondaryStats.pressure.avg.toFixed(2)}\n`;
    txtContent += `   - Mínimo: ${secondaryStats.pressure.min.toFixed(2)}\n`;
    txtContent += `   - Máximo: ${secondaryStats.pressure.max.toFixed(2)}\n\n`;

    txtContent += '6. Altitud (m):\n';
    txtContent += `   - Promedio: ${secondaryStats.altitude.avg.toFixed(2)}\n`;
    txtContent += `   - Mínimo: ${secondaryStats.altitude.min.toFixed(2)}\n`;
    txtContent += `   - Máximo: ${secondaryStats.altitude.max.toFixed(2)}\n\n`;

    txtContent += '7. Altitud Alternativa (m):\n';
    txtContent += `   - Promedio: ${secondaryStats.relativeAltitude.avg.toFixed(2)}\n`;
    txtContent += `   - Mínimo: ${secondaryStats.relativeAltitude.min.toFixed(2)}\n`;
    txtContent += `   - Máximo: ${secondaryStats.relativeAltitude.max.toFixed(2)}\n\n`;

    txtContent += '8. Velocidad de Desplazamiento (m/s):\n';
    txtContent += `   - Promedio: ${secondaryStats.velocity.avg.toFixed(2)}\n`;
    txtContent += `   - Mínimo: ${secondaryStats.velocity.min.toFixed(2)}\n`;
    txtContent += `   - Máximo: ${secondaryStats.velocity.max.toFixed(2)}\n\n`;

    txtContent += 'Distancia entre Cargas (m):\n';
    txtContent += `   - Promedio: ${distanceStats.avg.toFixed(2)}\n`;
    txtContent += `   - Mínimo: ${distanceStats.min.toFixed(2)}\n`;
    txtContent += `   - Máximo: ${distanceStats.max.toFixed(2)}\n`;

    fs.writeFileSync(textFilePath, txtContent);
    console.log(`Archivo de texto guardado en: ${textFilePath}`);

    dashboardWindow.webContents.send('report-generated', {
        message: `Reportes generados con éxito: ${path.basename(excelFilePath)} y ${path.basename(textFilePath)}`
    });
});