import processing.opengl.*;
import toxi.processing.*;
import toxi.geom.*;
import toxi.geom.mesh.*;
import toxi.math.*;
import toxi.volume.*;
import toxi.color.*;
 
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
 
void setup() {
	size(1024,960,P3D);
	gfx = new ToxiclibsSupport(this);
	flock = new Flock();
	genPool = new ArrayList<Float>();
	container = new AABB(
		new Vec3D(),
		new Vec3D(random(DIM/4,DIM/2),random(DIM/4,DIM/2),random(DIM/4,DIM/2)));

	img=loadImage("a.png");
	toneMap = new ToneMap(0,0.33,NamedColor.BLACK,NamedColor.WHITE,256);

	lifecycle = 0;
	recordtime = LIFETIME;
	// Add an initial set of boids into the system
	
	smooth();
}
 
void draw() {
	// Iterate generation
	pushMatrix();


	// Draw debug box
	background(0); lights();
	translate(width/2,height/2,0); rotateY(mouseX*0.01);
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

void buildMesh() {
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
void mousePressed() {
	flock.addBoid();
}

void keyPressed() {
	drawMesh = !drawMesh;
}