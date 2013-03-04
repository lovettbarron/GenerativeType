class Boid {
 
  Vec3D loc;
  Vec3D vel;
  Vec3D acc;
  Matrix4x4 mat;
  float maxforce;
  float maxspeed;
 
  float neighborDist;
  float desiredSeparation;
  float mutationRate;

  DNA dna;
  int pool;
  int finishTime, recordDist;
  boolean hitTarget;
  float fitness;
  int geneCounter = 0;

  Boid(DNA _dna, int _pool, Vec3D l, float ms, float mf, float nd, float sep) {
    loc=l;
    acc = new Vec3D();
    vel = Vec3D.randomVector();
    mat = new Matrix4x4();
    maxspeed = ms;
    maxforce = mf;
    neighborDist=nd*nd;
    desiredSeparation=sep;
    pool = _pool;
    dna = _dna;
    finishTime = 0;
    recordDist = 1000;

  }

    // FITNESS FUNCTION 
  // distance = distance from target
  // finish = what order did i finish (first, second, etc. . .)
  // f(distance,finish) =   (1.0f / finish^1.5) * (1.0f / distance^6);
  // a lower finish is rewarded (exponentially) and/or shorter distance to target (exponetially)
	void fitness() {
		if (recordDist < 1) recordDist = 1;
		//fitness = (1/(finishTime*recordDist));
		fitness = loc.distanceTo( container.getExtent() );
		fitness = pow(fitness, 4);
		if (insideTarget()) fitness *= 2;
		//fitness *= 1/NUM;
	}

	boolean insideTarget() {
		return container.containsPoint(loc);
	}

	float getFitness() {
	return fitness;
	}

	DNA getDNA() {
	return dna;
	}

  	void run(ArrayList boids) {
    flock(boids);

	applyForce(dna.genes[geneCounter]);
	geneCounter = (geneCounter + 1) % dna.genes.length;

    update();
    borders();
    render();
  }

	void applyForce(Vec3D f) {
		acc.addSelf(f);
	}
 
  // We accumulate a new acceleration each time based on three rules
  void flock(ArrayList boids) {
    Vec3D sep = separate(boids);   // Separation
    Vec3D ali = align(boids);      // Alignment
    Vec3D coh = cohesion(boids);   // Cohesion
    // Arbitrarily weight these forces
    sep.scaleSelf(1.5);
    ali.scaleSelf(1.0);
    coh.scaleSelf(1.0);
    // Add the force vectors to acceleration
    acc.addSelf(sep);
    acc.addSelf(ali);
    acc.addSelf(coh);
  }
 
  // Method to update location
  void update() {
    // Update velocity
    vel.addSelf(acc);
    // Limit speed
    vel.limit(maxspeed);
    loc.addSelf(vel);
    // Reset accelertion to 0 each cycle
    acc.clear();
  }
 
  void seek(Vec3D target) {
    acc.addSelf(steer(target,false));
  }
 
  void arrive(Vec3D target) {
    acc.addSelf(steer(target,true));
  }
 
 
  // A method that calculates a steering vector towards a target
  // Takes a second argument, if true, it slows down as it approaches the target
  Vec3D steer(Vec3D target, boolean slowdown) {
    Vec3D steer;  // The steering vector
    Vec3D desired = target.sub(loc);  // A vector pointing from the location to the target
    float d = desired.magnitude(); // Distance from the target is the magnitude of the vector
    // If the distance is greater than 0, calc steering (otherwise return zero vector)
    if (d > 0) {
      // Normalize desired
      desired.normalize();
      // Two options for desired vector magnitude (1 -- based on distance, 2 -- maxspeed)
      if ((slowdown) && (d < 100.0f)) desired.scaleSelf(maxspeed*(d/100.0f)); // This damping is somewhat arbitrary
      else desired.scaleSelf(maxspeed);
      // Steering = Desired minus Velocity
      steer = desired.sub(vel).limit(maxforce);  // Limit to maximum steering force
    }
    else {
      steer = new Vec3D();
    }
    return steer;
  }
 
  void render() {
    // use the color matrix to transform position into RGB values
    if(!drawMesh) {
	    Vec3D col=colorMatrix.applyTo(loc);
	    fill(col.x,col.y,col.z);
	    gfx.cone(new Cone(loc,vel,0,BOID_SIZE,BOID_SIZE*4),5,false);
	}
  }

  Mesh3D getMesh() {
  	return new Sphere(loc, BOID_SIZE).toMesh(SPHERE_RES);
  }
 
  // Wraparound
  void borders() {
    if (loc.x < -DIM) loc.x = DIM;
    if (loc.y < -DIM) loc.y = DIM;
    if (loc.z < -DIM) loc.z = DIM;
    if (loc.x > DIM) loc.x = -DIM;
    if (loc.y > DIM) loc.y = -DIM;
    if (loc.z > DIM) loc.z = -DIM;
  }
 
  // Separation
  // Method checks for nearby boids and steers away
  Vec3D separate (ArrayList boids) {
    Vec3D steer = new Vec3D();
    int count = 0;
    // For every boid in the system, check if it's too close
    for (int i = boids.size()-1 ; i >= 0 ; i--) {
      Boid other = (Boid) boids.get(i);
      if (this != other) {
        float d = loc.distanceTo(other.loc);
        // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
        if (d < desiredSeparation) {
          // Calculate vector pointing away from neighbor
          Vec3D diff = loc.sub(other.loc);
          diff.normalizeTo(1.0/d);
          steer.addSelf(diff);
          count++;
        }
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.scaleSelf(1.0/count);
    }
 
    // As long as the vector is greater than 0
    if (steer.magSquared() > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalizeTo(maxspeed);
      steer.subSelf(vel);
      steer.limit(maxforce);
    }
    return steer;
  }
 
  // Alignment
  // For every nearby boid in the system, calculate the average velocity
  Vec3D align (ArrayList boids) {
    Vec3D steer = new Vec3D();
    int count = 0;
    for (int i = boids.size()-1 ; i >= 0 ; i--) {
      Boid other = (Boid) boids.get(i);
      if (this != other) {
        if (loc.distanceToSquared(other.loc) < neighborDist) {
          steer.addSelf(other.vel);
          count++;
        }
      }
    }
    if (count > 0) {
      steer.scaleSelf(1.0/count);
    }
 
    // As long as the vector is greater than 0
    if (steer.magSquared() > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalizeTo(maxspeed);
      steer.subSelf(vel);
      steer.limit(maxforce);
    }
    return steer;
  }
 
  // Cohesion
  // For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
  Vec3D cohesion (ArrayList boids) {
    Vec3D sum = new Vec3D();   // Start with empty vector to accumulate all locations
    int count = 0;
    for (int i = boids.size()-1 ; i >= 0 ; i--) {
      Boid other = (Boid) boids.get(i);
      if (this != other) {
        if (loc.distanceToSquared(other.loc) < neighborDist) {
          sum.addSelf(other.loc); // Add location
          if(insideTarget()) sum.scaleSelf(2);
          count++;
        }
      }
    }
    if (count > 0) {
      sum.scaleSelf(1.0/count);
      return steer(sum,false);  // Steer towards the location
    }
    return sum;
  }
}
