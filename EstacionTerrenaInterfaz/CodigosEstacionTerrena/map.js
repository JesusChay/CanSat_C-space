let map = null;
let primaryMarker = null;
let secondaryMarker = null;
let primaryPath = L.polyline([], { color: 'red' });
let secondaryPath = L.polyline([], { color: 'blue' });
let primaryPathCoordinates = [];
let secondaryPathCoordinates = [];
let firstValidPrimaryCoord = false;
let firstValidSecondaryCoord = false;

function initializeMap() {
  if (!map) {
    map = L.map('map').setView([19.4326, -99.1332], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(map);
    primaryPath.addTo(map);
    secondaryPath.addTo(map);
  }
}

window.onload = () => {
  initializeMap();
};

window.api.onPrimaryData((data) => {
  const coords = [parseFloat(data.latitude), parseFloat(data.longitude)];
  if (!isNaN(coords[0]) && !isNaN(coords[1]) && !(coords[0] === 0 && coords[1] === 0)) {
    primaryPathCoordinates.push(coords);
    primaryPath.setLatLngs(primaryPathCoordinates);
    if (primaryMarker) {
      primaryMarker.setLatLng(coords);
    } else {
      primaryMarker = L.marker(coords, {
        icon: L.icon({
          iconUrl: 'assets/Marcador_Primaria.png',
          iconSize: [25, 41],
          iconAnchor: [12, 41]
        })
      }).addTo(map);
    }
    if (!firstValidPrimaryCoord) {
      map.setView(coords, 13);
      firstValidPrimaryCoord = true;
    }
  } else {
    console.warn('Coordenadas inválidas para carga primaria:', coords);
  }
});

window.api.onSecondaryData((data) => {
  const coords = [parseFloat(data.latitude), parseFloat(data.longitude)];
  if (!isNaN(coords[0]) && !isNaN(coords[1]) && !(coords[0] === 0 && coords[1] === 0)) {
    secondaryPathCoordinates.push(coords);
    secondaryPath.setLatLngs(secondaryPathCoordinates);
    if (secondaryMarker) {
      secondaryMarker.setLatLng(coords);
    } else {
      secondaryMarker = L.marker(coords, {
        icon: L.icon({
          iconUrl: 'assets/Marcador_Secundaria.png',
          iconSize: [25, 41],
          iconAnchor: [12, 41]
        })
      }).addTo(map);
    }
    if (!firstValidSecondaryCoord && !firstValidPrimaryCoord) {
      map.setView(coords, 13);
      firstValidSecondaryCoord = true;
    }
  } else {
    console.warn('Coordenadas inválidas para carga secundaria:', coords);
  }
});

window.api.onError((message) => {
  alert(message);
});
