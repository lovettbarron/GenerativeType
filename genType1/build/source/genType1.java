import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.opengl.*; 
import toxi.processing.*; 
import toxi.geom.*; 
import toxi.geom.mesh.*; 
import toxi.math.*; 
import toxi.volume.*; 
import toxi.color.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class genType1 extends PApplet {








 
int DIM = 200;
int NUM = 100;
int NEIGHBOR_DIST = 70;
int SEPARATION = 40;
float BOID_SIZE = 5;

int VOXEL_RES=32;
int VOXEL_STROKE_WEIGHT=7;
int SPHERE_RES=4;
int SMOOTH_ITER=1;
int LIFETIME=200;
boolean drawMesh=false;
int lifecycle, recordtime;

Flock flock;

ToneMap toneMap;
PImage img;

ArrayList<Float>genPool; 

// color transformation matrix used to map XYZ position into RGB values
Matrix4x4 colorMatrix=new Matrix4x4().scale(255f/(DIM*2)).translate(DIM,DIM,DIM);
WETriangleMesh mesh;
AABB container;
ToxiclibsSupport gfx;
 
public void setup() {
	size(1024,960,P3D);
	gfx = new ToxiclibsSupport(this);
	flock = new Flock();
	genPool = new ArrayList<Float>();
	container = new AABB(
		new Vec3D(),
		new Vec3D(random(DIM/4,DIM/2),random(DIM/4,DIM/2),random(DIM/4,DIM/2)));

	img=loadImage("a.png");
	toneMap = new ToneMap(0,0.33f,NamedColor.BLACK,NamedColor.WHITE,256);

	lifecycle = 0;
	recordtime = LIFETIME;
	// Add an initial set of boids into the system
	
	smooth();
}
 
public void draw() {
	// Iterate generation
	pushMatrix();


	// Draw debug box
	background(0); lights();
	translate(width/2,height/2,0); rotateY(mouseX*0.01f);
	noFill(); stroke(255,50);
	box(DIM*2);


	if (lifecycle < LIFETIME) {
		flock.run();
		lifecycle++;
	} else {
		lifecycle = 0;
		flock.fitness();
		flock.selection();
		flock.reproduction();
	}
	
	noFill();
	stroke(180,0,0);
	strokeWeight(4);
	gfx.box(container);
	strokeWeight(1);
	fill(255); noStroke();
	// Build mesh rep
	if(drawMesh) {
		buildMesh();
		gfx.mesh(mesh);
		mesh.clear();
	}
	popMatrix();
	// Write debug
	fill(255);
	text("Generation #: " + flock.getGenerations(), 10, 18);
	text("Cycles left: " + (LIFETIME-lifecycle), 10, 36);
	text("Record cycles: " + recordtime, 10, 54);
	for(int i=genPool.size()-1;i>0;i--) {
		float gen = genPool.get(i);

		int count;
		if(i > 50) count = 50-i;
		else count=i;
		
		if(genPool.size()>0) {
			text("Generation " + i + " at " + gen + " fitness", 10, 960-(18*count));
			strokeWeight(2);stroke(255);
			line(1024 - ( gen * 100 ), i, 1024, i);
		}
	}

}

public void buildMesh() {
	//Boid[] boids = flock.getBoids();
	
	WETriangleMesh tmp = new WETriangleMesh();
	for(int i=0;i<NUM;i++) {

		//println(i);
		tmp.addMesh( flock.getBoid(i).getMesh() );
	}
	mesh = MeshLatticeBuilder.build( tmp, VOXEL_RES, VOXEL_STROKE_WEIGHT);
	for(int i=0; i<SMOOTH_ITER; i++) {
	  new LaplacianSmooth().filter(mesh,1);
	}
}

// Add a new boid into the System
public void mousePressed() {
	flock.addBoid();
}

public void keyPressed() {
	drawMesh = !drawMesh;
}
class DNA {
  Vec3D[] genes;
  float maxforce = 0.1f;

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

  public DNA crossover(DNA partner) {
    Vec3D[] child = new Vec3D[genes.length];
    // Pick a midpoint
    int crossover = PApplet.parseInt(random(genes.length));
    // Take "half" from one and "half" from the other
    for (int i = 0; i < genes.length; i++) {
      if (i > crossover) child[i] = genes[i];
      else               child[i] = partner.genes[i];
    }    
    DNA newgenes = new DNA(child);
    return newgenes;
  }

  public void mutate(float m) {
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
	public void fitness() {
		if (recordDist < 1) recordDist = 1;
		//fitness = (1/(finishTime*recordDist));
		fitness = loc.distanceTo( container.getExtent() );
		fitness = pow(fitness, 4);
		if (insideTarget()) fitness *= 2;
		//fitness *= 1/NUM;
	}

	public boolean insideTarget() {
		return container.containsPoint(loc);
	}

	public float getFitness() {
	return fitness;
	}

	public DNA getDNA() {
	return dna;
	}

  	public void run(ArrayList boids) {
    flock(boids);

	applyForce(dna.genes[geneCounter]);
	geneCounter = (geneCounter + 1) % dna.genes.length;

    update();
    borders();
    render();
  }

	public void applyForce(Vec3D f) {
		acc.addSelf(f);
	}
 
  // We accumulate a new acceleration each time based on three rules
  public void flock(ArrayList boids) {
    Vec3D sep = separate(boids);   // Separation
    Vec3D ali = align(boids);      // Alignment
    Vec3D coh = cohesion(boids);   // Cohesion
    // Arbitrarily weight these forces
    sep.scaleSelf(1.5f);
    ali.scaleSelf(1.0f);
    coh.scaleSelf(1.0f);
    // Add the force vectors to acceleration
    acc.addSelf(sep);
    acc.addSelf(ali);
    acc.addSelf(coh);
  }
 
  // Method to update location
  public void update() {
    // Update velocity
    vel.addSelf(acc);
    // Limit speed
    vel.limit(maxspeed);
    loc.addSelf(vel);
    // Reset accelertion to 0 each cycle
    acc.clear();
  }
 
  public void seek(Vec3D target) {
    acc.addSelf(steer(target,false));
  }
 
  public void arrive(Vec3D target) {
    acc.addSelf(steer(target,true));
  }
 
 
  // A method that calculates a steering vector towards a target
  // Takes a second argument, if true, it slows down as it approaches the target
  public Vec3D steer(Vec3D target, boolean slowdown) {
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
 
  public void render() {
    // use the color matrix to transform position into RGB values
    if(!drawMesh) {
	    Vec3D col=colorMatrix.applyTo(loc);
	    fill(col.x,col.y,col.z);
	    gfx.cone(new Cone(loc,vel,0,BOID_SIZE,BOID_SIZE*4),5,false);
	}
  }

  public Mesh3D getMesh() {
  	return new Sphere(loc, BOID_SIZE).toMesh(SPHERE_RES);
  }
 
  // Wraparound
  public void borders() {
    if (loc.x < -DIM) loc.x = DIM;
    if (loc.y < -DIM) loc.y = DIM;
    if (loc.z < -DIM) loc.z = DIM;
    if (loc.x > DIM) loc.x = -DIM;
    if (loc.y > DIM) loc.y = -DIM;
    if (loc.z > DIM) loc.z = -DIM;
  }
 
  // Separation
  // Method checks for nearby boids and steers away
  public Vec3D separate (ArrayList boids) {
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
          diff.normalizeTo(1.0f/d);
          steer.addSelf(diff);
          count++;
        }
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.scaleSelf(1.0f/count);
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
  public Vec3D align (ArrayList boids) {
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
      steer.scaleSelf(1.0f/count);
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
  public Vec3D cohesion (ArrayList boids) {
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
      sum.scaleSelf(1.0f/count);
      return steer(sum,false);  // Steer towards the location
    }
    return sum;
  }
}
class Flock {
	ArrayList<Boid> boids; // An arraylist for all the boids
	ArrayList<Boid> matingPool;
	float mutationRate;
	int generations;

	Flock() {
		boids = new ArrayList<Boid>(); // Initialize the arraylist

		mutationRate = 0.01f;
		matingPool = new ArrayList<Boid>();
		generations = 0;

		for (int i = 0; i < NUM; i++) {
		  PVector location = new PVector(width/2,height+20);
		  boids.add( new Boid( 
		  		new DNA(), 
		  		NUM, 
		  		new Vec3D(random(0,DIM),random(0,DIM),random(0,DIM)),
		  		3, 
		  		0.05f, 
		  		NEIGHBOR_DIST, 
		  		SEPARATION )
		  );
		}


	}

	public void run() {
		for (int i = boids.size()-1 ; i >= 0 ; i--) {
			Boid b = (Boid) boids.get(i); 
			b.run(boids);  // Passing the entire list of boids to each boid individually
		}
	}

	public void addBoid() {
		boids.add( new Boid( 
		  		new DNA(), 
		  		NUM, 
		  		new Vec3D(),
		  		3, 
		  		0.05f, 
		  		NEIGHBOR_DIST, 
		  		SEPARATION ) );
	}

	public Boid getBoid(int i) {
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
  public void fitness() {
    for (int i = 0; i < NUM; i++) {
      boids.get(i).fitness();
    }
  }

  // Generate a mating pool
  public void selection() {
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
  public void reproduction() {

	genPool.add(getNormalFitness());

    // Refill the population with children from the mating pool
    for (int i = 0; i < NUM; i++) {
      // Sping the wheel of fortune to pick two parents
      int m = PApplet.parseInt(random(matingPool.size()));
      int d = PApplet.parseInt(random(matingPool.size()));
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
		  		0.05f, 
		  		NEIGHBOR_DIST, 
		  		SEPARATION ) );
    }
    generations++;
  }

  public int getGenerations() {
    return generations;
  }

  // Find highest fintess for the population
  public float getMaxFitness() {
    float record = 0;
    for (int i = 0; i < NUM; i++) {
       if(boids.get(i).getFitness() > record) {
         record += boids.get(i).getFitness();
       }
    }
    return record;
  }

  public float getNormalFitness() {
  	float n=0.0f;
    float maxFitness = getMaxFitness();
	for (int i = 0; i < NUM; i++) {
		float fitnessNormal = map(boids.get(i).getFitness(),0,maxFitness,0,1);
		n += fitnessNormal * 1;  // Arbitrary multiplier
	}
  	return n/NUM;
  }

}
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "genType1" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
