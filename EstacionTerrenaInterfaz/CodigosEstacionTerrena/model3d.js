let primaryScene, primaryCamera, primaryRenderer, primaryCylinder, primaryLight;
let secondaryScene, secondaryCamera, secondaryRenderer, secondaryCylinder, secondaryLight;
let primaryModel3dInitialized = false;
let secondaryModel3dInitialized = false;

function initializePrimaryModel3D() {
  if (!primaryModel3dInitialized) {
    primaryScene = new THREE.Scene();
    primaryCamera = new THREE.PerspectiveCamera(75, document.getElementById('model3dPrimary').offsetWidth / document.getElementById('model3dPrimary').offsetHeight, 0.1, 1000);
    primaryRenderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
    primaryRenderer.setSize(document.getElementById('model3dPrimary').offsetWidth, document.getElementById('model3dPrimary').offsetHeight);
    document.getElementById('model3dPrimary').appendChild(primaryRenderer.domElement);

    const primaryGeometry = new THREE.CylinderGeometry(1.0, 1.0, 1.0, 32);
    const primaryMaterial = new THREE.MeshPhongMaterial({ color: 0x555b5a });
    primaryCylinder = new THREE.Mesh(primaryGeometry, primaryMaterial);
    primaryCylinder.position.set(0, 0, 0);
    primaryScene.add(primaryCylinder);

    primaryLight = new THREE.DirectionalLight(0xffffff, 1);
    primaryLight.position.set(1, 1, 2).normalize();
    primaryScene.add(primaryLight);

    primaryCamera.position.z = 5;
    primaryCamera.lookAt(0, 0, 0);
    primaryModel3dInitialized = true;
  }
}

function initializeSecondaryModel3D() {
  if (!secondaryModel3dInitialized) {
    secondaryScene = new THREE.Scene();
    secondaryCamera = new THREE.PerspectiveCamera(75, document.getElementById('model3dSecondary').offsetWidth / document.getElementById('model3dSecondary').offsetHeight, 0.1, 1000);
    secondaryRenderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
    secondaryRenderer.setSize(document.getElementById('model3dSecondary').offsetWidth, document.getElementById('model3dSecondary').offsetHeight);
    document.getElementById('model3dSecondary').appendChild(secondaryRenderer.domElement);

    const secondaryGeometry = new THREE.CylinderGeometry(1.0, 1.0, 1.0, 32);
    const secondaryMaterial = new THREE.MeshPhongMaterial({ color: 0xffffff });
    secondaryCylinder = new THREE.Mesh(secondaryGeometry, secondaryMaterial);
    secondaryCylinder.position.set(0, 0, 0);
    secondaryScene.add(secondaryCylinder);

    secondaryLight = new THREE.DirectionalLight(0xffffff, 1);
    secondaryLight.position.set(1, 1, 2).normalize();
    secondaryScene.add(secondaryLight);

    secondaryCamera.position.z = 5;
    secondaryCamera.lookAt(0, 0, 0);
    secondaryModel3dInitialized = true;
  }
}

function animate() {
  requestAnimationFrame(animate);
  if (primaryModel3dInitialized && primaryCylinder) {
    primaryRenderer.render(primaryScene, primaryCamera);
  }
  if (secondaryModel3dInitialized && secondaryCylinder) {
    secondaryRenderer.render(secondaryScene, secondaryCamera);
  }
}

window.onload = () => {
  initializePrimaryModel3D();
  initializeSecondaryModel3D();
  animate();
};

window.api.onPrimaryData((data) => {
  if (primaryCylinder) {
    primaryCylinder.rotation.x = data.gyrox;
    primaryCylinder.rotation.y = data.gyroy;
    primaryCylinder.rotation.z = data.gyroz;
  }
  document.getElementById('gyroXPrimary').textContent = `X: ${data.gyrox} rad/s`;
  document.getElementById('gyroYPrimary').textContent = `Y: ${data.gyroy} rad/s`;
  document.getElementById('gyroZPrimary').textContent = `Z: ${data.gyroz} rad/s`;
});

window.api.onSecondaryData((data) => {
  if (secondaryCylinder) {
    secondaryCylinder.rotation.x = data.gyrox;
    secondaryCylinder.rotation.y = data.gyroy;
    secondaryCylinder.rotation.z = data.gyroz;
  }
  document.getElementById('gyroXSecondary').textContent = `X: ${data.gyrox} rad/s`;
  document.getElementById('gyroYSecondary').textContent = `Y: ${data.gyroy} rad/s`;
  document.getElementById('gyroZSecondary').textContent = `Z: ${data.gyroz} rad/s`;
});

window.api.onError((message) => {
  alert(message);
});