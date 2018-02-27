#include "phys.h"
#include "renderer.h"
#include "bookKeeping.h"
#include "../../SGV3D/src/logger.h"

#include <ostream>
#include <chrono>
#include <iostream>

#define VERT(v) mesh->positions.push_back(std::move(v)) 

Entity MakeSphere (float const& mass, 
                   float const& damping=1.0, 
                   float const& rad=1.0, 
                   unsigned const& nInc=16, 
                   glm::vec4 const& color1={1.0f,1.0f,1.0f,1.0f}, 
                   glm::vec4 const& color2={1.0f,1.0f,1.0f,1.0f},
                   arma::fvec3 const& pos={0.0f,0.0f,0.0f,0.0f}, 
                   arma::fvec3 const& vel={0.0f,0.0f,0.0f,0.0f}) 
{
    Collidable* col{new SphereCollidable(rad)};


    arma::fmat33 inertia;
    inertia.eye();
    inertia *= 2.0f/5.0f * mass * rad * rad;

    RigidBody* rb{new RigidBody{mass, inertia, pos, col, vel, damping, 0.5f}}; 
    Mesh* mesh{new Mesh};

    for(unsigned t = 0; t < (nInc+1)/2; ++t) //Note: we use (n+1)/2 to round UP
    {
        float theta1 = ((t  )/(float)((nInc+1)/2))*M_PI;
        float theta2 = ((t+1)/(float)((nInc+1)/2))*M_PI;

        for(unsigned p = 0; p < nInc; ++p) 
        {
            float phi1 = ((p  )/(float)nInc)*2*M_PI;
            float phi2 = ((p+1)/(float)nInc)*2*M_PI;

            //phi2   phi1
            // |      |
            // 2------1 -- theta1
            // |\ _   |
            // |    \ |
            // 3------4 -- theta2
            //

            glm::vec3 v1{rad*std::cos(theta1)*std::sin(phi1), rad*std::sin(theta1)*std::sin(phi1), rad*std::cos(phi1)}; 
            glm::vec3 v2{rad*std::cos(theta1)*std::sin(phi2), rad*std::sin(theta1)*std::sin(phi2), rad*std::cos(phi2)}; 
            glm::vec3 v3{rad*std::cos(theta2)*std::sin(phi2), rad*std::sin(theta2)*std::sin(phi2), rad*std::cos(phi2)}; 
            glm::vec3 v4{rad*std::cos(theta2)*std::sin(phi1), rad*std::sin(theta2)*std::sin(phi1), rad*std::cos(phi1)}; 
            float thetaAvg{(theta1+theta2)/2}, phiAvg{(phi1+phi2)/2};

            glm::vec3 nm{std::cos(thetaAvg)*std::sin(phiAvg), std::sin(thetaAvg)*std::sin(phiAvg), std::cos(phiAvg)};
            glm::vec4 color{glm::mix(color1, color2, t/(float)nInc * p/(float)nInc)};

            mesh->normals.insert(mesh->normals.end(), 6, nm);
            mesh->colors.insert(mesh->colors.end(), 6, color); 

            VERT(v1); VERT(v2); VERT(v4);
            VERT(v2); VERT(v3); VERT(v4);
        }
    }
    return Entity(std::move(rb), std::move(mesh));
}

#undef VERT

void KeyCallback(GLFWwindow* window, int key, int, int action, int)
{   
    Renderer* windowPtr{reinterpret_cast<Renderer*>(glfwGetWindowUserPointer(window))};

    if(!windowPtr)
        return;

    if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        windowPtr->Done();

    else if(key == GLFW_KEY_UP && action == GLFW_PRESS)
    {
        Engine* engine{windowPtr->GetEnginePtr()};
        GLProgram* curProgram{windowPtr->GetCurrentProgram()};

        if(engine && curProgram)
        {
            FreeRoamCamera const& cam{windowPtr->GetCamera()};
            arma::fvec3 pos{BookKeeping::gta3(cam.GetPosition())};
            arma::fvec3 dir{BookKeeping::gta3(cam.GetDirection())};

            arma::fvec3 shootDir{arma::normalise(dir)};
            shootDir *= 40.0f;

            Entity* ent{new Entity{MakeSphere(30.0f, 1.0f, 1.0f, 30, {1.0f, 1.0f, 1.0f, 1.0f}, {1.0f, 0.0f, 0.0f, 1.0f}, pos, shootDir)}};

            windowPtr->AddEntity(*curProgram, ent);
            engine->AddEntity(ent);
            windowPtr->SetModelUniforms(engine->GetEntities());

            IForce* force{new GravForce(0.5f, ent->GetRigidBody())};
            engine->AddForceToSystem(force);

    	    ent->GetRigidBody()->ApplyTorque({10000.0f, 60.0f, 0.0f});
        }
    }

    else if(key == GLFW_KEY_LEFT_BRACKET && action == GLFW_PRESS)
    {
        Engine* engine{windowPtr->GetEnginePtr()};
        engine->SetGravity(engine->GetGravity() - 5.0f);
	}

    else if(key == GLFW_KEY_RIGHT_BRACKET && action == GLFW_PRESS)
    {
        Engine* engine{windowPtr->GetEnginePtr()};
        engine->SetGravity(engine->GetGravity() + 5000.0f);
	}

    else if(key == GLFW_KEY_BACKSLASH && action == GLFW_PRESS)
    {
        Engine* engine{windowPtr->GetEnginePtr()};
        engine->SetGravity(0.0f);
    }
}

void MouseCallback(GLFWwindow* window, int button, int action, int mods)
{   
    (void)mods;

    Renderer* windowPtr{reinterpret_cast<Renderer*>(glfwGetWindowUserPointer(window))};

    if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
    {
        Engine* engine{windowPtr->GetEnginePtr()};
        GLProgram* curProgram{windowPtr->GetCurrentProgram()};

        if(engine && curProgram)
        {
            FreeRoamCamera const& cam{windowPtr->GetCamera()};
            arma::fvec3 pos{BookKeeping::gta3(cam.GetPosition())};
            arma::fvec3 dir{BookKeeping::gta3(cam.GetDirection())};

            arma::fvec3 shootDir{arma::normalise(dir)};
            shootDir *= 40.0f;

            Entity* ent{new Entity{MakeSphere(30.0f, 1.0f, 1.0f, 30, {1.0f, 1.0f, 1.0f, 1.0f}, {1.0f, 0.0f, 0.0f, 1.0f}, pos, shootDir)}};

            windowPtr->AddEntity(*curProgram, ent);
            engine->AddEntity(ent);
            windowPtr->SetModelUniforms(engine->GetEntities());

            IForce* force{new GravForce(0.5f, ent->GetRigidBody())};
            engine->AddForceToSystem(force);

    	    ent->GetRigidBody()->ApplyTorque({10000.0f, 60.0f, 0.0f});
        }
    }
}

glm::vec4 RainbowColoring (float const& t)
{
    if(t <= 0.2)
        return glm::mix(glm::vec4(1.0f, 0.0f, 1.0f, 1.0f), glm::vec4(0.0f, 0.0f, 1.0f, 1.0f), t * 1.0 / 0.2);
    else if(t <= 0.4) 
        return glm::mix(glm::vec4(0.0f, 0.0f, 1.0f, 1.0f), glm::vec4(0.0f, 1.0f, 1.0f, 1.0f), t * 1.0 / 0.4);
    else if(t <= 0.6) 
        return glm::mix(glm::vec4(0.0f, 1.0f, 1.0f, 1.0f), glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), t * 1.0 / 0.4);
    else if(t <= 0.8) 
        return glm::mix(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), glm::vec4(1.0f, 1.0f, 0.0f, 1.0f), t * 1.0 / 0.4);
    
    return glm::mix(glm::vec4(1.0f, 1.0f, 0.0f, 1.0f), glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), t * 1.0 / 0.4);
}

std::vector<Entity*> Stack (int const& n, float const& sep, float const& rad, arma::fvec3 const& initPos={0.0f,0.0f,0.0f}) 
{
    std::vector<Entity*> spheres;
    float sclr{20.0f};
    for(int i = 0; i < n; ++i)
    {
        for(int j = 0; j < i+1; ++j)
        {
            spheres.push_back(new Entity{MakeSphere(3.0f, 1.0f, rad, 8, 
                        RainbowColoring(fmod(sin(sclr *  i   /(GLfloat)n), 1.0f)), 
                        RainbowColoring(fmod(sin(sclr * (i+1)/(GLfloat)n), 1.0f)), 
                        initPos + arma::fvec3{rad*((sep+2.0f)*j - (1.0f+0.5f*sep)*i), 
                                              0.0f, 
											  rad*(sep+2.0f)*i*std::cos((float)M_PI/6)},
						{0.0f, 0.0f, 0.0f})}); 
        }
    }

    return spheres;
}

int main () 
{
    //ADD A WAY TO RETRIEVE "SIMULATION INFO"
	
    //Start the logger 
    const char* logFileName{"../output/LOG.txt"};

    if(!Logger::singleton().init(logFileName))
    {
        std::cerr<<"Failed to initialize logger"<<std::endl;
        exit(0);
    }
    DEBUG_MSG("Beginning main loop");

    //Initalize our SGVGraphics with window width and height and true to initialize GLEW
    Renderer renderer;
    if(!renderer.Initailize(1080.0, 1920.0, true, KeyCallback, MouseCallback))
    {
        std::cerr<<"Failed to Initialize GLFWContext!"<<std::endl;
        ERROR("Failed to Initialize GLFWContext!");
        exit(0);
    }

    FreeRoamCamera camera(10.0f, 1.0f);
    camera.SetPosition(glm::vec3(0.0f, 10.0f, -10.0f));
    camera.SetDirection(glm::vec3(0.0f, -1.0f/sqrt(2.0f), 1.0f/sqrt(2.0f)));
    camera.SetProjection(45.0f, 1080.0f/1920.0f, 0.1f, 200.0f);
    camera.UpdateView();
    renderer.SetCamera(camera);

    //Create a new shader program
    GLProgram program;
    renderer.GetNewProgram(program, "../src/Shaders/vert_simple.glsl", "../src/Shaders/frag_simple.glsl", SGV_POSITION | SGV_NORMAL | SGV_COLOR);
    renderer.BindProgram(program);

    //Create physics engine
    Engine engine;
    renderer.SetEnginePtr(engine);
    renderer.SetProgram(&program);

    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end; 
    float dt;

    //Continuously render OpenGL and update physics systsem 
    bool good{true};
    glfwSetTime(0.0f);
    while(good)
    {
        start = std::chrono::system_clock::now();

        engine.Run(dt);
        
		renderer.SetModelUniforms(engine.GetEntities());
        good = renderer.Render(program.Strip(), {0.1f, 0.5f, 0.1f, 1.0f});

        end = std::chrono::system_clock::now();
        dt = std::chrono::duration<float>(end-start).count();
    }
}
