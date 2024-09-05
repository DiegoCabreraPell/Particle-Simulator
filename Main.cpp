#include<iostream>
#include<glad/glad.h>
#include<GLFW/glfw3.h>

#include<array>

#include"shaderClass.h"
#include"VAO.h"
#include"VBO.h"
#include"EBO.h"
#include"fillBuffer.h"

#include"ParticleSimulation.h"
#include"Particle.h"

#define NUM_PARTICLES_MAIN 200
#define NUM_TYPES 4

float defaultForceFunc(float x) 
{
	if (x >= 80.0f)
		return 0;
	float ans = (80.0f-x)/(10*x + 80.0f);
	return ans;
}

void collisonHandler (Particle &p1, Particle &p2, float dist){}

int main()
{
	const int num_particles = NUM_PARTICLES_MAIN * NUM_TYPES;

	std::srand(num_particles);

	glfwInit();

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(800, 800, "Particle Simulation", NULL, NULL);

	// Error check if the window fails to create
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(window);

	//openGL environment
	gladLoadGL();

	glViewport(0, 0, 800, 800);

	glPrimitiveRestartIndex(0xffff);
	glEnable(GL_PRIMITIVE_RESTART);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	//creating type data
	GLfloat typeData[] = {
		0.008f, 1.0f, 0.0f, 0.0f,
		0.008f, 0.0f, 1.0f, 0.0f,
		0.008f, 0.0f, 0.0f, 1.0f,
		0.008f, 1.0f, 1.0f, 1.0f
	};

	GLfloat typeMatrix[] = {
		40.0f, 40.0f, 40.0f, 40.0f,
		40.0f, 40.0f, -40.0f, -40.0f,
		40.0f, -40.0f, 40.0f, -40.0f,
		40.0f, -40.0f, -40.0f, 40.0f
	};

	float fps = 60.0f;
	float ts = 1.0f / fps;
	int substeps = 1;
	float ss = ts / (float) substeps;

	ParticleSimulation simulator = ParticleSimulation(800, 800, num_particles, NUM_TYPES, 88, 40, defaultForceFunc);
	//simulator.setCollisionResolver(collisonHandler);
	GLfloat randX, randY;

	for (int i = 0; i < NUM_PARTICLES_MAIN; i++)
	{
		for (int j = 0; j < NUM_TYPES; j++)
		{
			randX = (GLfloat)(rand() % 800);
			randY = (GLfloat)(rand() % 800);
			simulator.addParticle(randX, randY, j);
		}
	}
	for (int i = 0; i < NUM_TYPES; i++)
	{
		simulator.setPSize(i, 400 * typeData[4*i]);
		for (int j = 0; j < NUM_TYPES; j++)
		{
			simulator.setTypeInteractionCoefficient(i, j, typeMatrix[i * NUM_TYPES + j]);
		}
	}

	simulator.setSpeedLimit(90.0f);

	// Initilising vertex and index buffers
	GLfloat* vertices = new GLfloat[13 * 6 * num_particles]{};

	GLuint* indices = new GLuint[21 * num_particles]{};
	fillIndices(indices, num_particles);

	Shader shaderProgram("particle2d.vert", "default.frag");

	// Generates Vertex Array Object and binds it
	VAO VAO1;

	// Generates Element Buffer Object and links it to indices
	EBO EBO1(indices, 21 * num_particles);

	GLuint uniID = glGetUniformLocation(shaderProgram.ID, "scale");

	double crntTime, timeDiff, prevTime = -0.02;

	// Main while loop
	while (!glfwWindowShouldClose(window))
	{	
		crntTime = glfwGetTime();
		timeDiff = crntTime - prevTime;
		if (timeDiff >= ts)
		{
			prevTime = crntTime;
			// Specify the color of the background
			glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			// Clean the back buffer and assign the new color to it
			glClear(GL_COLOR_BUFFER_BIT);

			shaderProgram.Activate();
			glUniform1f(uniID, 0.0f);

			//Simulation
			for (int i = 0; i < substeps; i++)
				simulator.step(ss);
			
			fillVertices(simulator.particleList(), vertices, num_particles, typeData, 4);

			VAO1.Bind();
			VBO VBO1(vertices, 13 * 6 * num_particles);
			EBO1.Bind();

			//Linking layouts
			VAO1.LinkAttrib(VBO1, 0, 2, GL_FLOAT, 6 * sizeof(float), (void*)0);
			VAO1.LinkAttrib(VBO1, 1, 4, GL_FLOAT, 6 * sizeof(float), (void*)(2 * sizeof(float)));

			VAO1.Unbind();
			VBO1.Unbind();
			EBO1.Unbind();

			//Draw call
			VAO1.Bind();
			glDrawElements(GL_TRIANGLE_STRIP, 21 * simulator.getNumParticles(), GL_UNSIGNED_INT, 0);
			VAO1.Unbind();

			//deleting old VBO
			VBO1.Delete();

			glfwSwapBuffers(window);
		}


		glfwPollEvents();
	}



	// Delete all the objects we've created
	VAO1.Delete();
	EBO1.Delete();
	shaderProgram.Delete();

	// Delete window before ending the program
	glfwDestroyWindow(window);
	// Terminate GLFW before ending the program
	glfwTerminate();
	return 0;
}