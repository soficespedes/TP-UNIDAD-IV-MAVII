#pragma once
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include "SFMLRenderer.h"
#include <list>
#include <vector>
using namespace sf;

class Ragdoll {
public:
	Ragdoll(b2World* world, b2Vec2 position);
	~Ragdoll();
public:
	b2Body* torso;
	b2Body* GetTorso() const { return torso; }
	
private:
	//Piezas del cuerpo
	b2Body* upperArmLeft;
	b2Body* upperArmRight;
	b2Body* upperLegLeft;
	b2Body* upperLegRight;
	b2Body* head;

	//Joints (articulaciones)
	std::vector<b2RevoluteJoint*> joints;

	void CreateBodyParts(b2World* world, b2Vec2 position);
	void CreateJoints(b2World* world);
};

class Game
{
private:
	// Propiedades de la ventana
	int alto;
	int ancho;
	RenderWindow *wnd;
	Color clearColor;
	//Objetos de box2d
	b2World *phyWorld;
	SFMLRenderer *debugRender;
	b2Body* obstacleBody;
	b2Body* controlBody;
	//tiempo de frame
	float frameTime; 
	int fps;


	float SCALE;
	float MAX_DISTANCE;
	std::vector<Ragdoll*> ragdolls;
	
public:

	//Constructores, destructores e inicializadores
	Game(int ancho, int alto,std::string titulo);
	void CheckCollitions();
	~Game(void);
	void InitPhysics();
	//Main game loop
	void Disparar();
	void Loop();
	void DrawGame();
	void UpdatePhysics();
	void DoEvents();
	void SetZoom();
};

