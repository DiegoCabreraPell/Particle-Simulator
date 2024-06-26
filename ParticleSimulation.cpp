#include"ParticleSimulation.h"
#include"Particle.h"
#include"SimTypes.h"

void DEFAULTCOLLISIONRESOLVER(Particle &p1, Particle &p2, float distance, Grid &grid)
{
	
}

ParticleSimulation::ParticleSimulation(int pHeight, int pWidth, int mParticles, int nTypes, float maxCmptDist, float gSize, forceFunc dForceFunc)
{
	int numX = (int) round((float)pWidth/gridSize);
	int numY = (int) round((float)pHeight/gridSize);
	grid = new Grid(numX, numY, gSize, maxCmptDist, mParticles);

	aspectRatio = (float)pHeight / (float)pWidth;
	resolver = DEFAULTCOLLISIONRESOLVER;
	gridSize = gSize;

	pixelWidth = pWidth;
	pixelHeight = pHeight;
	numTypes = nTypes;
	maxParticles = mParticles;
	numParticles = 0;
	speedLimit = 100.0f / 60.0f;

	for (int i = 0; i < nTypes; i++)
		typeMat[i] = new float[nTypes] {0.0f};

	defaultForceFunc = dForceFunc;

	typeForceFuncs = new forceFunc[nTypes]{defaultForceFunc};
	typeSizes = new float[nTypes] {gSize / 25};
	typeWeights = new float[nTypes] {1.0f};

	//Initialising free particle id vector and particle address array
	particles = new Particle * [maxParticles]();
	freeIDs = vector<int>(maxParticles);
	for (int i = 0; i < maxParticles; i++)
	{
		particles[i] = NULL;
		freeIDs.push_back(i);
	}
}

void ParticleSimulation::setSpeedLimit(float limit) {
	speedLimit = limit;
}

int ParticleSimulation::getFreeID() 
{
	if (freeIDs.empty())
		return -1;
	else
	{
		int id = freeIDs.back();
		freeIDs.pop_back();
		return id;
	}
}

void ParticleSimulation::step(float timeStep) 
{
	//update velocity
	grid->updateVelocities(timeStep, typeMat, typeForceFuncs);
	
	//update position
	grid->step(timeStep);
	
	//detect and resolve collisions
	grid->handleCollsions(resolver, typeSizes);

}

void ParticleSimulation::setCollisionResolver(collisionResolver res) {
	resolver = res;
}

Particle* ParticleSimulation::getParticle(int pID) 
{
	if (pID >= 0 || pID < maxParticles)
	{
		if (particles[pID] != NULL)
			return particles[pID];
	}

	return NULL;
}

int ParticleSimulation::removeParticle(int pID)
{
	if (pID >= 0 || pID < maxParticles)
	{
		if (particles[pID] == NULL)
			return -1;

		Particle p = *particles[pID];

		grid->deleteParticle(pID, p.x,p.y);
		delete particles[pID];
		particles[pID] = NULL;

		numParticles -= 1;
	}
	return -1;
}

int ParticleSimulation::addParticle(float x, float y, int type, float dx, float dy) 
{
	int id = getFreeID();
	if (id != -1)
	{
		particles[id] = new Particle(id, type, x, y, dx, dy);
		grid->insertParticle(*(particles[id]));
		numParticles += 1;
		return 0;
	}
	return -1;
}

int ParticleSimulation::addParticle(float x, float y, int type) 
{
	return addParticle(x, y, type, 0.0f, 0.0f);
}

int ParticleSimulation::addParticle(int type) 
{
	return addParticle(0.0f, 0.0f, type, 0.0f, 0.0f);
}

int ParticleSimulation::setPWeight(int type, float weight)
{
	if (type >= numTypes || type < 0)
		return 1;

	typeWeights[type] = weight;
	return 0;
}

int ParticleSimulation::setPSize(int type, float radius)
{
	if (type >= numTypes || type < 0)
		return 1;

	typeSizes[type] = radius;
	return 0;
}

const float* ParticleSimulation::getPWeights()
{
	return typeWeights;
}

const float* ParticleSimulation::getPSizes()
{
	return typeSizes;
}

int ParticleSimulation::getNumParticles()
{
	return numParticles;
}

bool ParticleSimulation::isFull() const 
{
	if (numParticles == maxParticles)
	{
		return true;
	}
	else {
		return false;
	}
}

Particle** ParticleSimulation::particleList()
{
	return particles;
}

