#include <Windows.h>

#define _USE_MATH_DEFINES
#include <cmath>

#ifdef __APPLE__
#include <GLUT/gl.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#endif

#include <iostream>
#include <vector>
#include <cassert>

// donwload glm at https://github.com/g-truc/glm
#include <glm/vec3.hpp> 
#include <glm/gtc/quaternion.hpp> 
#include <glm/gtx/quaternion.hpp>

using namespace glm;

// This class is used internally by the CalcIK_3D_CCD function to represent a bone in
// world space.
class Bone_3D_CCD_World {
public:
	vec3 position; // end position in world coordinates
	quat orientation; // orientation in world coordinates
};

// This class is used to supply the CalcIK_3D_CCD function with a bone's representation
// relative to its parent in the kinematic chain.
class Bone_3D_CCD {
public:
	float l; // length of this arm
	quat angle; // rotation relative to parent arm
	// or relative to (1,0,0) axis for the first arm
	Bone_3D_CCD(float _l, quat _angle) {
		angle = _angle;
		l = _l;
	}
};



class robotnR {
private:

	float arrivalDist;
	std::vector<Bone_3D_CCD> bones;

public:

	robotnR (std::vector<float> L, // vecor of bone lengths
		std::vector<bool> axis, // 1 if rotation is along y axis, 0 if it is along z 
		float _arrivalDist) // determines if the calculation is complete
		{
		assert(L.size() == axis.size());
		for (int i = 1; i < L.size(); i++) {
			// initialize angle at non zero values
			Bone_3D_CCD bone(L[i], angleAxis((float)0.35, axis[i] ? vec3(0, 1, 0) : vec3(0, 0, 1)));
			bones.push_back(bone);
		}
		arrivalDist = _arrivalDist;
	}
	
	// move the robot arm to target _x, _y in robot coordinates
	void move(vec3 target) {
		while (CalcIK_3D_CCD(target) == "processing") {
		}
	}

	void draw() {
		// draw arm using direct kinematics
		vec3 position(0, 0, 0);
		quat direction = angleAxis((float)0,vec3(1,0,0)); // direction of nth arm to draw
		glBegin(GL_LINES);
		for (unsigned int i = 0; i < bones.size(); i++) {
			if (i % 2)
				glColor3f(0.0, 1.0, 0.0);
			else
				glColor3f(0.0, 0.0, 1.0);
			glVertex3f(position.x, position.y, position.z);
			direction = bones[i].angle * direction;
			position += rotate(direction , vec3(bones[i].l, 0, 0));
			glVertex3f(position.x, position.y, position.z);
		}
		glEnd();
	}
	// Given a bone chain located at the origin, this function will perform a single cyclic
	// coordinate descent (CCD) iteration. This finds a solution of bone angles that places
	// the final bone in the given chain at a target position. The supplied bone angles are
	// used to prime the CCD iteration. If a valid solution does not exist, the angles will
	// move as close to the target as possible. The user should resupply the updated angles 
	// until a valid solution is found (or until an iteration limit is met).
	//  
	// returns: Success when a valid solution was found.
	//          Processing when still searching for a valid solution.
	//          Failure when it can get no closer to the target.
	std::string CalcIK_3D_CCD(vec3 target) {
		// Set an epsilon value to prevent division by small numbers.
		const float epsilon = 0.0001;

		// Set max arc length a bone can move the end effector an be considered no motion
		// so that we can detect a failure state.
		const float trivialArcLength = 0.00001;


		int numBones = bones.size();
		assert(numBones > 0);

		//===
		// Generate the world space bone data.
		std::vector<Bone_3D_CCD_World> worldBones;

		// Start with the root bone.
		Bone_3D_CCD_World rootWorldBone;
		rootWorldBone.orientation = bones[0].angle;
		rootWorldBone.position = rootWorldBone.orientation * vec3(bones[0].l, 0, 0);
		worldBones.push_back(rootWorldBone);

		// Convert child bones to world space.
		for (int boneIdx = 1; boneIdx < numBones; boneIdx++)
		{
			Bone_3D_CCD_World prevWorldBone = worldBones[boneIdx - 1];
			Bone_3D_CCD curLocalBone = bones[boneIdx];

			Bone_3D_CCD_World newWorldBone;
			newWorldBone.orientation =  curLocalBone.angle * prevWorldBone.orientation;
			newWorldBone.position = prevWorldBone.position + rotate(newWorldBone.orientation, vec3(curLocalBone.l, 0, 0));
			worldBones.push_back(newWorldBone);
		}

		//===
		// Track the end effector position (the final bone)
		vec3 end = worldBones[numBones - 1].position;

		//===
		// Perform CCD on the bones by optimizing each bone in a loop 
		// from the final bone to the root bone
		bool modifiedBones = false;
		for (int boneIdx = numBones - 1; boneIdx > 0; boneIdx--)
		{
			// normal vector to current bone's plane  
			vec3 nPlan = axis(bones[boneIdx].angle);
			nPlan /= length(nPlan);
			// project end effector and target in this plane
			vec3 endProj = end - dot(end, nPlan)*nPlan;
			vec3 targetProj = target - dot(target, nPlan)*nPlan;

			// Get the vector from the current bone to the end effector position projection
			vec3 curToEnd = endProj - worldBones[boneIdx - 1].position;

			// Get the vector from the current bone to the target position projection
			vec3 curToTarget = targetProj - worldBones[boneIdx - 1].position;
			
			// Get rotation to place the end effector projection on the line from the current
			// joint position to the target postion projection
			double cosRotAng;
			if (length(curToEnd) * length(curToTarget) <= epsilon) {
				cosRotAng = 1;
			}
			else {
				cosRotAng = dot(curToTarget, curToEnd) / length(curToEnd) / length(curToTarget);
			}

			// Clamp the cosine into range when computing the angle (might be out of range
			// due to floating point error).
			float rotAng = acos(max(-1.0, min(1.0, cosRotAng)));

			// Rotate the current bone in local space
			quat rotation = angleAxis(rotAng, cross(curToTarget, curToEnd)/length(cross(curToTarget, curToEnd)));
			bones[boneIdx].angle = rotation * bones[boneIdx].angle;

			// Rotate the end effector position
			end = worldBones[boneIdx - 1].position + rotate(rotation, end - worldBones[boneIdx - 1].position);

			// Check for termination
			vec3 endToTarget = (target - end);

			if (length(endToTarget) <= arrivalDist) {
				// We found a valid solution.
				return "success";
			}
			

			// Track if the arc length that we moved the end effector was
			// a nontrivial distance.
			if (!modifiedBones && abs(rotAng) * length(curToEnd) > trivialArcLength) {
				modifiedBones = true;
			}
		}
		
		// We have to handle the first bone (index 0) separately 
		// because the algorithm needs value for the previous bone

		// normal vector to current bone's plane  
		vec3 nPlan = axis(bones[0].angle);
		nPlan /= length(nPlan);
		// project end effector and target in this plane
		vec3 endProj = end - dot(end, nPlan)*nPlan;
		vec3 targetProj = target - dot(target, nPlan)*nPlan;

		// Get the vector from the current bone to the end effector position.
		vec3 curToEnd = endProj;

		// Get the vector from the current bone to the target position.
		vec3 curToTarget = targetProj;

		// Get rotation to place the end effector on the line from the current
		// joint position to the target postion.
		double cosRotAng;
		//double sinRotAng;
		if (length(curToEnd) * length(curToTarget) <= epsilon) {
			cosRotAng = 1;
		}
		else {
			cosRotAng = dot(curToTarget, curToEnd) / length(curToEnd) / length(curToTarget);
		}

		// Clamp the cosine into range when computing the angle (might be out of range
		// due to floating point error).
		float rotAng = acos(max(-1.0, min(1.0, cosRotAng)));

		// Rotate the current bone in local space
		quat rotation = angleAxis(rotAng, cross(curToTarget, curToEnd)/length(cross(curToTarget, curToEnd)));
		bones[0].angle = rotation * bones[0].angle;

		// Rotate the end effector position.
		end = rotate(rotation, end);

		// Check for termination
		vec3 endToTarget = (target - end);

		if (length(endToTarget) <= arrivalDist) {
			// We found a valid solution.
			return "success";
		}

		// Track if the arc length that we moved the end effector was
		// a nontrivial distance.
		if (!modifiedBones && abs(rotAng) * length(curToEnd) > trivialArcLength) {
			modifiedBones = true;
		}
		std::cout << end.x << " " << end.y << " " << end.z << std::endl;
		// We failed to find a valid solution during this iteration.
		if (modifiedBones)
			return "processing";
		else
			return "failure";
	}
};

// vector of arm lengths
std::vector<float> L{ 0.7,0.6,0.5,0.4,0.3,0.2,0.1 };
// vector describing the axis of each bone
std::vector<bool> A{ 0,1,0,1,0,1,0 };
// arm without rotation limits 
robotnR robot(L, A, 0.0005);

// window size
int viewWidth = 720;
int viewHeight = 720;

// target position in pixel coordinates
int xOrigin = 3 * viewWidth / 4;
int yOrigin = 2 * viewHeight / 4;
int zOrigin = viewHeight / 2;

float pasZ = 35; // in pixel 

// target position in openGL coordinates 
// conveniently also in robot coordinates
float xTarget;
float yTarget;
float zTarget;


void drawPoint(float x, float y, float z, float r, int segments, float R, float G, float B) {
	// draw point of radius r at x,y in openGL coordinates
	// color R,G,B
	// segments is the number of triangle that forms the point
	glBegin(GL_TRIANGLE_FAN);
	glColor3f(R, G, B);
	glVertex3f(x, y, z);
	for (int n = 0; n <= segments; ++n) {
		float const t = 2 * M_PI * (float)n / (float)segments;
		glVertex3f(x + sin(t) * r, y + cos(t) * r, z);
	}
	glEnd();
}

void drawTarget() {
	// draw target at cursor position
	int segments = 10;
	float offset = 0.03;
	float r = 0.004;
	xTarget = 2 * (float)xOrigin / (float)viewWidth - 1;
	yTarget = -2 * (float)yOrigin / (float)viewHeight + 1;
	zTarget = -2 * (float)zOrigin / (float)viewWidth + 1;
	float x, y, z;
	z = zTarget;
	// modify offset to fake perspective
	offset += z / 50;
	z = 0;
	for (int i = -1; i <= 1; i++) {
		for (int j = -1; j <= 1; j++) {
			if ((i + j) % 2 == 0) {
				x = xTarget + i * offset;
				y = yTarget + j * offset;
				drawPoint(x, y, z, r, segments, 1, 0, 0);
			}
		}
	}
}

void drawOrigin() {
	// draw point at (0,0), the origin of the arm
	int segments = 10;
	float r = 0.005;
	float x = 0;
	float y = 0;
	float z = 0;
	drawPoint(x, y, z, r, segments, 1, 0, 0);
}

// Définition de la fonction d'affichage
GLvoid affichage() {
	// Effacement du frame buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// On passe en matice modelview
	glMatrixMode(GL_MODELVIEW);
	// on charge la matrice identite
	glLoadIdentity();
	drawOrigin();
	//drawWorkspace();
	drawTarget();
	robot.move(vec3(xTarget, yTarget, zTarget));
	robot.draw();

	glutSwapBuffers();
}

GLvoid clavier(unsigned char touche, int x, int y) {
	switch (touche) {
	case 'q': // quitter
	case 27:
		exit(0);
		break;
	}

	// Demande a GLUT de reafficher la scene
	glutPostRedisplay();
}

GLvoid clavierSpecial(int touche, int x, int y) {
	switch (touche) {
	case GLUT_KEY_DOWN:
		zOrigin -= pasZ;
		break;
	case GLUT_KEY_UP:
		zOrigin += pasZ;
		break;
	}

	// Demande a GLUT de reafficher la scene
	glutPostRedisplay();
}

// Fonction de gestion du deplacement de la souris
void deplacementSouris(int x, int y) {

	// On ne fait quelque chose que si l'utilisateur
	// a deja clique quelque part avec le bouton gauche
	if (xOrigin >= 0 || yOrigin >= 0) {
		xOrigin = x;
		yOrigin = y;
	}
	glutPostRedisplay();
}

// Fonction de gestion des clics souris
void clicSouris(int button, int state, int x, int y) {

	// On ne fait quelque chose que sur le bouton gauche de la souris
	if (state == GLUT_DOWN) {
		// si l'on a clique sur le bouton gauche
		// on garde les positions de la souris au moment du clic gauche
		if (button == GLUT_LEFT_BUTTON) {
			xOrigin = x;
			yOrigin = y;
		}
		if (button == 3) {
			zOrigin += pasZ;
		}
		if (button == 4) {
			zOrigin -= pasZ;
		}
	}
	glutPostRedisplay();
}

// Fonction de redimensionnement de la fenetre
void redimensionner(int w, int h) {

	viewWidth = w;
	viewHeight = h;

	// On evite une division par 0
	// la fenetre ne peut avoir une largeur de 0
	if (h == 0)
		h = 1;

	// Calcul du ratio
	float ratio = (w * 1.0) / h;

	// On passe en mode "matrice de projection"
	glMatrixMode(GL_PROJECTION);

	// on charge la matrice identite
	glLoadIdentity();

	// on definit le viewport pour prendre toute la fenetre
	glViewport(0, 0, w, h);

	// on repasse en mode "matrice modelview"
	glMatrixMode(GL_MODELVIEW);

	// on definit la projection orthographique
	glOrtho(-1, 1, -1, 1, -1, 1);
}

int main(int argc, char** argv) {

	// Initialisation de GLUT
	glutInit(&argc, argv);

	// Choix du mode d'affichage
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	// Position initiale de la fenêtre GLUT
	glutInitWindowPosition(200, 200);

	// Taille initiale de la fenêtre GLUT
	glutInitWindowSize(viewWidth, viewHeight);

	// Création de la fenêtre GLUT
	glutCreateWindow("X arm robot - 3D");

	// Définition de la couleur d'effacement du framebuffer OpenGL
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	// Définition des fonctionsde callbacks
	glutDisplayFunc(affichage);
	glutKeyboardFunc(clavier);
	glutSpecialFunc(clavierSpecial);
	glutMouseFunc(clicSouris);
	glutMotionFunc(deplacementSouris);
	glutReshapeFunc(redimensionner);

	// Lancement de la boucle infinie GLUT
	glutMainLoop();
}
