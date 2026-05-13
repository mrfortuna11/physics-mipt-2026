class RigidBody {
  constructor(mass, size) {
    this.mass = mass;
    this.invMass = mass > 0 ? 1 / mass : 0;
    this.size = size.clone();
    this.I_body     = boxInertia(mass > 0 ? mass : 1, size.x, size.y, size.z);
    this.I_body_inv = mat3Inverse(this.I_body);
    if (mass === 0) {
      this.I_body_inv.set(0,0,0, 0,0,0, 0,0,0);
    }

    this.x = new THREE.Vector3();
    this.v = new THREE.Vector3();
    this.q = new THREE.Quaternion();
    this.omega = new THREE.Vector3();

    this.L0 = new THREE.Vector3();
  }

  getL() {
    const { I } = inertiaWorld(this);
    return mat3MulVec(I, this.omega);
  }

  getEnergy(gravity) {
    const kinT = 0.5 * this.mass * this.v.lengthSq();
    const kinR = 0.5 * this.omega.dot(this.getL());
    const pot  = -this.mass * gravity.dot(this.x);
    return kinT + kinR + pot;
  }

  worldPoint(rLocal) {
    const R = quatToMat3(this.q);
    return mat3MulVec(R, rLocal).add(this.x);
  }

  worldR(rLocal) {
    const R = quatToMat3(this.q);
    return mat3MulVec(R, rLocal);
  }

  pointVelocity(rWorld) {
    return this.v.clone().add(new THREE.Vector3().crossVectors(this.omega, rWorld));
  }

  resetFree(q0, omega0) {
    this.q.copy(q0);
    this.omega.copy(omega0);
    this.x.set(0,0,0);
    this.v.set(0,0,0);
    this.L0.copy(this.getL());
  }
}
