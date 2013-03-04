class Flock {
	ArrayList<Boid> boids; // An arraylist for all the boids
	ArrayList<Boid> matingPool;
	float mutationRate;
	int generations;

	Flock() {
		boids = new ArrayList<Boid>(); // Initialize the arraylist

		mutationRate = 0.01;
		matingPool = new ArrayList<Boid>();
		generations = 0;

		for (int i = 0; i < NUM; i++) {
		  PVector location = new PVector(width/2,height+20);
		  boids.add( new Boid( 
		  		new DNA(), 
		  		NUM, 
		  		new Vec3D(random(0,DIM),random(0,DIM),random(0,DIM)),
		  		3, 
		  		0.05, 
		  		NEIGHBOR_DIST, 
		  		SEPARATION )
		  );
		}


	}

	void run() {
		for (int i = boids.size()-1 ; i >= 0 ; i--) {
			Boid b = (Boid) boids.get(i); 
			b.run(boids);  // Passing the entire list of boids to each boid individually
		}
	}

	void addBoid() {
		boids.add( new Boid( 
		  		new DNA(), 
		  		NUM, 
		  		new Vec3D(),
		  		3, 
		  		0.05, 
		  		NEIGHBOR_DIST, 
		  		SEPARATION ) );
	}

	Boid getBoid(int i) {
		return (Boid)boids.get(i);
	}


//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////

  // Did anything finish?
  // boolean targetReached() {
  //   for (int i = 0; i < NUM; i++) {
  //     if (boids[i].hitTarget) return true;
  //   }
  //   return false;
  // }

  // Calculate fitness for each creature
  void fitness() {
    for (int i = 0; i < NUM; i++) {
      boids.get(i).fitness();
    }
  }

  // Generate a mating pool
  void selection() {
    // Clear the ArrayList
    matingPool.clear();

    // Calculate total fitness of whole population
    float maxFitness = getMaxFitness();

    // Calculate fitness for each member of the population (scaled to value between 0 and 1)
    // Based on fitness, each member will get added to the mating pool a certain number of times
    // A higher fitness = more entries to mating pool = more likely to be picked as a parent
    // A lower fitness = fewer entries to mating pool = less likely to be picked as a parent
    for (int i = 0; i < NUM; i++) {
      float fitnessNormal = map(boids.get(i).getFitness(),0,maxFitness,0,1);
      int n = (int) (fitnessNormal * 100);  // Arbitrary multiplier
      for (int j = 0; j < n; j++) {
        matingPool.add(boids.get(i));
      }
    }
  }

  // Making the next generation
  void reproduction() {

	genPool.add(getNormalFitness());

    // Refill the population with children from the mating pool
    for (int i = 0; i < NUM; i++) {
      // Sping the wheel of fortune to pick two parents
      int m = int(random(matingPool.size()));
      int d = int(random(matingPool.size()));
      // Pick two parents
      Boid mom = matingPool.get(m);
      Boid dad = matingPool.get(d);
      // Get their genes
      DNA momgenes = mom.getDNA();
      DNA dadgenes = dad.getDNA();
      // Mate their genes
      DNA child = momgenes.crossover(dadgenes);
      // Mutate their genes
      child.mutate(mutationRate);
      // Fill the new population with the new child
      //Vec3D location = new Vec3D(width/2,height+20);
      boids.set(i, new Boid( 
		  		new DNA(), 
		  		NUM, 
		  		new Vec3D(random(0,DIM),random(0,DIM),random(0,DIM)),
		  		3, 
		  		0.05, 
		  		NEIGHBOR_DIST, 
		  		SEPARATION ) );
    }
    generations++;
  }

  int getGenerations() {
    return generations;
  }

  // Find highest fintess for the population
  float getMaxFitness() {
    float record = 0;
    for (int i = 0; i < NUM; i++) {
       if(boids.get(i).getFitness() > record) {
         record += boids.get(i).getFitness();
       }
    }
    return record;
  }

  float getNormalFitness() {
  	float n=0.0f;
    float maxFitness = getMaxFitness();
	for (int i = 0; i < NUM; i++) {
		float fitnessNormal = map(boids.get(i).getFitness(),0,maxFitness,0,1);
		n += fitnessNormal * 1;  // Arbitrary multiplier
	}
  	return n/NUM;
  }

}