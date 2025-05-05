#include "Game.h"
#include "Box2DHelper.h"
#include <iostream>
#include<cmath>


Game::Game(int ancho, int alto, std::string titulo) : SCALE(30.0f)
{
    //Inicializacion de la ventana
    wnd = new RenderWindow(VideoMode(ancho, alto), titulo);
    wnd->setVisible(true);
    //Calculamos la diagonal de la ventana en pixeles:
    float w_px = static_cast<float>(wnd->getSize().x);
    float h_px = static_cast<float>(wnd->getSize().y);
    MAX_DISTANCE = std::sqrt(w_px * w_px + h_px * h_px) / SCALE;
    //FPS y tiempo por frame
    fps = 60;
    wnd->setFramerateLimit(fps);
    frameTime = 1.0f / fps;
    SetZoom(); //Configuración de la vista del juego
    InitPhysics(); //Inicialización del motor de física
}

//Bucle principal del juego
void Game::Loop()
{
    while (wnd->isOpen())
    {
        wnd->clear(clearColor); //Limpiar la ventana
        DoEvents(); //Procesar eventos de entrada
        CheckCollitions(); //Comprobar colisiones
        UpdatePhysics(); //Actualizar la simulación física
        DrawGame(); //Dibujar el juego
        wnd->display(); //Mostrar la ventana
    }
}

//Actualización de la simulación física
void Game::UpdatePhysics()
{
    phyWorld->Step(frameTime, 8, 8); //Simular el mundo físico
    phyWorld->ClearForces(); //Limpiar las fuerzas aplicadas a los cuerpos
    phyWorld->DebugDraw(); //Dibujar el mundo físico para depuración
}

//Constructor para crear el Ragdoll
Ragdoll::Ragdoll(b2World* world, b2Vec2 position) {
    CreateBodyParts(world, position);
    CreateJoints(world);
}

Ragdoll::~Ragdoll() 
{}

void Ragdoll::CreateBodyParts(b2World* world, b2Vec2 pos) { //Definimos al Ragdoll

    //Cabeza
    head = Box2DHelper::CreateRectangularDynamicBody(world, 3.5, 3.5, 1.0f, 1.0f, 0.1f);
    head->SetTransform(pos + b2Vec2(0, -5), 0.0f);
    auto fixHead = head->GetFixtureList();
    fixHead->SetFriction(1.0f);
    fixHead->SetRestitution(0.1f);

    //Torso
    torso = Box2DHelper::CreateRectangularDynamicBody(world, 5, 8, 1.0f, 0.3f, 0.1f);
    torso->SetTransform(pos, 0.0f);
    auto fixTorso = torso->GetFixtureList();
    fixTorso->SetFriction(1.0f);
    fixTorso->SetRestitution(0.1f);

    //Brazos
    upperArmLeft = Box2DHelper::CreateRectangularDynamicBody(world, 2, 6, 1.0f, 0.3f, 0.1f);
    upperArmLeft->SetTransform(pos + b2Vec2(-4.5f, -2.0f), 0.0f);
    auto fixupperArmLeft = upperArmLeft->GetFixtureList();
    fixupperArmLeft->SetFriction(1.0f);
    fixupperArmLeft->SetRestitution(0.1f);

    upperArmRight = Box2DHelper::CreateRectangularDynamicBody(world, 2, 6, 1.0f, 0.3f, 0.1f);
    upperArmRight->SetTransform(pos + b2Vec2(4.5f, -2.0f), 0.0f);
    auto fixupperArmRight = upperArmRight->GetFixtureList();
    fixupperArmRight->SetFriction(1.0f);
    fixupperArmRight->SetRestitution(0.1f);

    //Piernas
    upperLegLeft = Box2DHelper::CreateRectangularDynamicBody(world, 2.5f, 10, 1.0f, 0.3f, 0.1f);
    upperLegLeft->SetTransform(pos + b2Vec2(-1.7f, 6.5f), 0.0f);
    auto fixupperLegLeft = upperLegLeft->GetFixtureList();
    fixupperLegLeft->SetFriction(1.0f);
    fixupperLegLeft->SetRestitution(0.1f);

    upperLegRight = Box2DHelper::CreateRectangularDynamicBody(world, 2.5f, 10.0f, 1.0f, 0.3f, 0.1f);
    upperLegRight->SetTransform(pos + b2Vec2(1.7f, 6.5f), 0.0f);
    auto fixupperLegRight = upperLegRight->GetFixtureList();
    fixupperLegRight->SetFriction(1.0f);
    fixupperLegRight->SetRestitution(0.1f);
}

void Ragdoll::CreateJoints(b2World* world) { //unimos las partes
    b2RevoluteJointDef jointDef;
    jointDef.collideConnected = false;

    // Cabeza al torso, no supe como corregir para que la cabeza no se fuese a un lado al caer
    jointDef.bodyA = head;
    jointDef.bodyB = torso;
    jointDef.localAnchorA.Set(0, 1.5f);
    jointDef.localAnchorB.Set(0, -4.0f);
    joints.push_back(static_cast<b2RevoluteJoint*>(world->CreateJoint(&jointDef)));

    //Brazo izquierdo al torso
    jointDef.bodyA = upperArmLeft;
    jointDef.bodyB = torso;
    jointDef.localAnchorA.Set(0, 3.0f);
    jointDef.localAnchorB.Set(-2.5f, -2.5f);
    joints.push_back(static_cast<b2RevoluteJoint*>(world->CreateJoint(&jointDef)));

    //Brazo derecho al torso
    jointDef.bodyA = upperArmRight;
    jointDef.bodyB = torso;
    jointDef.localAnchorA.Set(0, 3.0f);
    jointDef.localAnchorB.Set(2.5f, -2.5f);
    joints.push_back(static_cast<b2RevoluteJoint*>(world->CreateJoint(&jointDef)));

    //Pierna izquierda al torso
    jointDef.bodyA = upperLegLeft;
    jointDef.bodyB = torso;
    jointDef.localAnchorA.Set(0, -3.0f);
    jointDef.localAnchorB.Set(-1.5f, 4.0f);
    joints.push_back(static_cast<b2RevoluteJoint*>(world->CreateJoint(&jointDef)));

    //Pierna derecha al torso
    jointDef.bodyA = upperLegRight;
    jointDef.bodyB = torso;
    jointDef.localAnchorA.Set(0, -3.0f);
    jointDef.localAnchorB.Set(1.5f, 4.0f);
    joints.push_back(static_cast<b2RevoluteJoint*>(world->CreateJoint(&jointDef)));
}

//Dibujo de los elementos del juego
void Game::DrawGame()
{
    //Dibujar las paredes
    sf::RectangleShape leftWallShape(sf::Vector2f(10, alto)); //Alto de la ventana
    leftWallShape.setFillColor(sf::Color::Blue);
    leftWallShape.setPosition(0, 0);
    wnd->draw(leftWallShape);

    sf::RectangleShape rightWallShape(sf::Vector2f(10, alto));
    rightWallShape.setFillColor(sf::Color::Blue);
    rightWallShape.setPosition(95, 0);
    wnd->draw(rightWallShape);


    //Dibujar el cuerpo de control (cañon)
    sf::RectangleShape controlShape(sf::Vector2f(20, 10));
    controlShape.setFillColor(sf::Color::Yellow);
    //Obtener posición del mouse en coordenadas del mundo
    sf::Vector2i mousePixelPos = sf::Mouse::getPosition(*wnd);
    sf::Vector2f mouseWorldPos = wnd->mapPixelToCoords(mousePixelPos);

    //Posición del centro del rectángulo
    sf::Vector2f cannonPos(controlBody->GetPosition().x, controlBody->GetPosition().y);

    //Calculamos el ángulo entre el rectángulo y el mouse
    float dx = mouseWorldPos.x - cannonPos.x;
    float dy = mouseWorldPos.y - cannonPos.y;
    float angleRad = std::atan2(dy, dx);
    float angleDeg = angleRad * 180.0f / b2_pi;
    controlBody->SetTransform(controlBody->GetPosition(), angleRad);
    // Configuramos rotación y origen del rectángulo para que gire como un cañón
    controlShape.setOrigin(10.0f, 5.0f); //centro del rectángulo (20x10)
    controlShape.setRotation(angleDeg); //aplicar rotación
    controlShape.setPosition(cannonPos);

    wnd->draw(controlShape);

    //Dibujar el suelo
    sf::RectangleShape groundShape(sf::Vector2f(500, 5));
    groundShape.setFillColor(sf::Color::Red);
    groundShape.setPosition(0, 95);
    
    wnd->draw(groundShape);
    //Dibujar el techo
    sf::RectangleShape upWallShape(sf::Vector2f(500, 5));
    upWallShape.setFillColor(sf::Color::Red);
    upWallShape.setPosition(0, 0);
    wnd->draw(upWallShape);

    //Dibujamos los obstaculos que estarán en pantalla
    sf::RectangleShape obstacleShape1(sf::Vector2f(10, 10));
    obstacleShape1.setFillColor(sf::Color::Blue);
    obstacleShape1.setPosition(60, 30);
    wnd->draw(obstacleShape1);

    sf::RectangleShape obstacleShape2(sf::Vector2f(10, 10));
    obstacleShape2.setFillColor(sf::Color::Blue);
    obstacleShape2.setPosition(40, 60);
    wnd->draw(obstacleShape2);

    //obstaculo dinámico
    if (obstacleBody != nullptr) {
        b2Vec2 pos = obstacleBody->GetPosition();
        float angle = obstacleBody->GetAngle();

        sf::RectangleShape obstacleShape(sf::Vector2f(10, 10));
        obstacleShape.setFillColor(sf::Color::Cyan);
        obstacleShape.setOrigin(5, 5); // Centrar para rotación
        obstacleShape.setPosition(pos.x, pos.y);
        obstacleShape.setRotation(angle * 180.0f / b2_pi); // Radianes a grados

        wnd->draw(obstacleShape);
    }
}

void Game::Disparar()
{
    //Creamos ragdoll
    Ragdoll* ragdoll = new Ragdoll(phyWorld, controlBody->GetPosition());
    ragdolls.push_back(ragdoll);
    b2Body* body = ragdoll->GetTorso();

    //Calculamos la distancia en pixeles entre el mouse y el cañon 
    sf::Vector2i mousePixel = sf::Mouse::getPosition(*wnd);
    sf::Vector2f cannonWorld{
        controlBody->GetPosition().x,
        controlBody->GetPosition().y
    };
    sf::Vector2i cannonPixel = wnd->mapCoordsToPixel(cannonWorld);

    sf::Vector2f deltaPixel{
        float(mousePixel.x - cannonPixel.x),
        float(mousePixel.y - cannonPixel.y)
    };
    float distPixel = std::sqrt(
        deltaPixel.x * deltaPixel.x +
        deltaPixel.y * deltaPixel.y
    );

    //Mapeo de pixel
    const float PIXEL_RANGE = 800.0f;
    float rawT = distPixel / PIXEL_RANGE;
    if (rawT > 1.0f) rawT = 1.0f;

    //Curvatura para realce
    float curvedT = std::sqrt(rawT);

    //Calculamos la velocidad minima y maxima que deberá tener nuestro ragdoll
    const float MIN_VEL = 40.0f;
    const float MAX_VEL = 250.0f;
    float vel = MIN_VEL + curvedT * (MAX_VEL - MIN_VEL);

    //Colocamos un debug para chequear que se cumpla la condicion de: puntero + alejado del cañón = más lejor irá el ragdoll
    std::cout
        << "mousePx=(" << mousePixel.x << "," << mousePixel.y << ")  "
        << "cannonPx=(" << cannonPixel.x << "," << cannonPixel.y << ")  "
        << "distPx=" << distPixel
        << "  rawT=" << rawT
        << "  curvedT=" << curvedT
        << "  vel=" << vel
        << std::endl;

    //Dirección en Box2D
    sf::Vector2f worldPos = wnd->mapPixelToCoords(mousePixel);
    b2Vec2 cannonPos = controlBody->GetPosition();
    b2Vec2 mousePos(worldPos.x / SCALE, worldPos.y / SCALE);

    b2Vec2 dir = mousePos - cannonPos;
    float dist = dir.Length();
    if (dist > 0.0f) dir *= 1.0f / dist;

    //Aplicamos la velocidad
    b2Vec2 velocity = vel * dir;
    body->SetLinearVelocity(velocity);
}

//Procesamiento de eventos de entrada
void Game::DoEvents()
{
    Event evt;
    while (wnd->pollEvent(evt))
    {
        switch (evt.type)
        {
        case Event::Closed:
            wnd->close(); //Cerrar la ventana si se presiona el botón de cerrar
            break;
            
        case Event::MouseButtonPressed: //Disparar si se presiona Clic Izquierdo
            if (evt.mouseButton.button == sf::Mouse::Left)
                Disparar();
            break;
        }  
    }
    controlBody->SetAwake(true);
}

//Comprobación de colisiones (a implementar más adelante)
void Game::CheckCollitions()
{
}

//Configuración de la vista del juego
void Game::SetZoom()
{
    View camara;
    //Posicionamiento y tamaño de la vista
    camara.setSize(100.0f, 100.0f);
    camara.setCenter(50.0f, 50.0f);
    wnd->setView(camara); //Asignar la vista a la ventana
}

//Inicialización del motor de física y los cuerpos del mundo físico
void Game::InitPhysics()
{
    //Inicializar el mundo físico con la gravedad por defecto
    phyWorld = new b2World(b2Vec2(0.0f, 9.8f));

    //Crear un renderer de debug para visualizar el mundo físico
    debugRender = new SFMLRenderer(wnd);
    debugRender->SetFlags(UINT_MAX);
    phyWorld->SetDebugDraw(debugRender);

    //Crear el suelo y las paredes estáticas del mundo físico
    b2Body* groundBody = Box2DHelper::CreateRectangularStaticBody(phyWorld, 200, 10);
    groundBody->SetTransform(b2Vec2(0.0f, 100.0f), 0.0f);
    auto fixedgroundBody = groundBody->GetFixtureList();
    fixedgroundBody->SetFriction(1.0f);

    b2Body* upWallBody = Box2DHelper::CreateRectangularStaticBody(phyWorld, 200, 10);
    upWallBody->SetTransform(b2Vec2(0.0f, 0.0f), 0.0f);

    b2Body* leftWallBody = Box2DHelper::CreateRectangularStaticBody(phyWorld, 10, 100);
    leftWallBody->SetTransform(b2Vec2(0.0f, 50.0f), 0.0f);

    b2Body* rightWallBody = Box2DHelper::CreateRectangularStaticBody(phyWorld, 10, 100);
    rightWallBody->SetTransform(b2Vec2(100.0f, 50.0f), 0.0f);

    //creamos tambien los obstaculos
    
    b2Body* obstacleBody1 = Box2DHelper::CreateRectangularStaticBody(phyWorld, 10, 10);
    obstacleBody1->SetTransform(b2Vec2(65.0f, 35.0f), 0.0f);
    b2Body* obstacleBody2 = Box2DHelper::CreateRectangularStaticBody(phyWorld, 10, 10);
    obstacleBody2->SetTransform(b2Vec2(45.0f, 65.0f), 0.0f);
    
    obstacleBody = Box2DHelper::CreateRectangularDynamicBody(phyWorld, 10, 10, 0.7f, 0.1f, 0.1f);
    obstacleBody->SetTransform(b2Vec2(25, 25), 0.0f);
    obstacleBody->SetGravityScale(0.0f);
    obstacleBody->SetAngularDamping(2.0f);

    // Crear cuerpo estático como ancla en la misma posición
    b2BodyDef anchorDef;
    anchorDef.position = obstacleBody->GetPosition(); //Mismo punto que el cuerpo dinámico
    b2Body* anchor = phyWorld->CreateBody(&anchorDef);

    //Crear el revolute joint entre el cuerpo y el ancla
    b2RevoluteJointDef jointDef;
    jointDef.bodyA = anchor;
    jointDef.bodyB = obstacleBody;
    jointDef.localAnchorA.Set(0, 0); //Ancla en el centro del estático
    jointDef.localAnchorB.Set(0, 0); //Ancla en el centro del dinámico
    jointDef.collideConnected = false;

    phyWorld->CreateJoint(&jointDef);

    //Crear el cuerpo de control (cañón) como kinemático para que no se mueva por colisiones
    controlBody = Box2DHelper::CreateRectangularKinematicBody(phyWorld, 20, 10);
    controlBody->SetTransform(b2Vec2(5.0f, 90.0f), 0.0f);
    controlBody->SetFixedRotation(false);

}


//Destructor de la clase
Game::~Game(void)
{ }