class DNA {
  Vec3D[] genes;
  float maxforce = 0.1;

  DNA() {
    genes = new Vec3D[LIFETIME];
    for (int i = 0; i < genes.length; i++) {
      float angle = random(TWO_PI);
      genes[i] = new Vec3D(
          cos(angle) * random(0, maxforce), 
          sin(angle) * random(0, maxforce), 
          cos(angle) * random(0, maxforce));
      //genes[i] *= (random(0, maxforce));
    }

    genes[0].normalize();
  }

  DNA(Vec3D[] newgenes) {
    genes = newgenes;
  }

  DNA crossover(DNA partner) {
    Vec3D[] child = new Vec3D[genes.length];
    // Pick a midpoint
    int crossover = int(random(genes.length));
    // Take "half" from one and "half" from the other
    for (int i = 0; i < genes.length; i++) {
      if (i > crossover) child[i] = genes[i];
      else               child[i] = partner.genes[i];
    }    
    DNA newgenes = new DNA(child);
    return newgenes;
  }

  void mutate(float m) {
    for (int i = 0; i < genes.length; i++) {
      if (random(1) < m) {
        float angle = random(TWO_PI);
        genes[i] = new Vec3D(
          cos(angle) * random(0, maxforce), 
          sin(angle) * random(0, maxforce), 
          cos(angle) * random(0, maxforce));

        if (i ==0) genes[i].normalize();
      }
    }
  }
}

