#include "phys.h"
#include "renderer.h"
#include "bookKeeping.h"
#include "collision_detection.h"
#include "collidable.h"

#include <ostream>
#include <chrono>
#include <iostream>

#include <vector>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>
//IMP NOTE: ADD DESTRUCTORS AND MORE SAFETY CHECKS!!

#define VERT(v) mesh->positions.push_back(std::move(v)) 

Entity MakeSphere (double const& mass, 
                   double const& rad=1.0, 
                   unsigned const& nInc=16, 
                   glm::vec4 const& color1={1.0,1.0,1.0,1.0}, 
                   glm::vec4 const& color2={1.0,1.0,1.0,1.0},
                   arma::vec3 const& pos={0.0,0.0,0.0,0.0}, 
                   arma::vec3 const& vel={0.0,0.0,0.0,0.0}) 
{
    Collidable* col{new SphereCollidable(rad)};


    arma::mat33 inertia;
    inertia.eye();
    inertia *= 2.0/5.0 * mass * rad * rad;

    RigidBody* rb{new RigidBody{mass, inertia, pos, col, vel, 1.0, 1.0}}; 
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

Entity MakeBox (double const& mass, 
                glm::vec3 const& dim=glm::vec3(1.0), 
                glm::vec4 const& color={1.0,1.0,1.0,1.0},
                arma::vec3 const& pos={0.0,0.0,0.0}, 
                arma::vec3 const& vel={0.0,0.0,0.0}) 
{
    std::vector<arma::vec3> positions 
    {
        {-dim.x*0.5, -dim.y*0.5, -dim.z*0.5}, //0: v000
        { dim.x*0.5, -dim.y*0.5, -dim.z*0.5}, //1: v100
        {-dim.x*0.5,  dim.y*0.5, -dim.z*0.5}, //2: v010
        { dim.x*0.5,  dim.y*0.5, -dim.z*0.5}, //3: v110
        {-dim.x*0.5, -dim.y*0.5,  dim.z*0.5}, //4: v001
        { dim.x*0.5, -dim.y*0.5,  dim.z*0.5}, //5: v101
        {-dim.x*0.5,  dim.y*0.5,  dim.z*0.5}, //6: v011
        { dim.x*0.5,  dim.y*0.5,  dim.z*0.5}, //7: v111
    };

    static const std::vector<std::vector<unsigned>> faces 
    {
        {0, 4, 6, 2}, //0: Left face
        {5, 1, 3, 7}, //1: Right face
        {0, 1, 5, 4}, //2: Bottom face
        {7, 3, 2, 6}, //3: Top face
        {0, 2, 3, 1}, //4: Back face
        {4, 5, 7, 6}  //5: Front face
    };

    HalfEdgeMesh* const half_edge_mesh{new HalfEdgeMesh(positions, faces)};
    std::unique_ptr<MeshInstance> mesh_instance{new MeshInstance(half_edge_mesh)};
    Collidable* col{new MeshCollidable(std::move(mesh_instance))};

    arma::mat33 inertia;
    inertia.zeros();
    inertia(0, 0) = 1.0/12.0 * mass * (dim.y * dim.y + dim.z * dim.z);
    inertia(1, 1) = 1.0/12.0 * mass * (dim.x * dim.x + dim.z * dim.z);
    inertia(2, 2) = 1.0/12.0 * mass * (dim.y * dim.y + dim.z * dim.z);

    RigidBody* rb{new RigidBody{mass, inertia, pos, col, vel, 1.0, 1.0}}; 
    Mesh* mesh{new Mesh};
    mesh->positions = 
    {
        {-dim.x*0.5,-dim.y*0.5,-dim.z*0.5},
        {-dim.x*0.5,-dim.y*0.5, dim.z*0.5},
        {-dim.x*0.5, dim.y*0.5, dim.z*0.5},

        { dim.x*0.5, dim.y*0.5,-dim.z*0.5},
        {-dim.x*0.5,-dim.y*0.5,-dim.z*0.5},
        {-dim.x*0.5, dim.y*0.5,-dim.z*0.5},

        { dim.x*0.5,-dim.y*0.5, dim.z*0.5},
        {-dim.x*0.5,-dim.y*0.5,-dim.z*0.5},
        { dim.x*0.5,-dim.y*0.5,-dim.z*0.5},

        { dim.x*0.5, dim.y*0.5,-dim.z*0.5},
        { dim.x*0.5,-dim.y*0.5,-dim.z*0.5},
        {-dim.x*0.5,-dim.y*0.5,-dim.z*0.5},

        {-dim.x*0.5,-dim.y*0.5,-dim.z*0.5},
        {-dim.x*0.5, dim.y*0.5, dim.z*0.5},
        {-dim.x*0.5, dim.y*0.5,-dim.z*0.5},

        { dim.x*0.5,-dim.y*0.5, dim.z*0.5},
        {-dim.x*0.5,-dim.y*0.5, dim.z*0.5},
        {-dim.x*0.5,-dim.y*0.5,-dim.z*0.5},

        {-dim.x*0.5, dim.y*0.5, dim.z*0.5},
        {-dim.x*0.5,-dim.y*0.5, dim.z*0.5},
        { dim.x*0.5,-dim.y*0.5, dim.z*0.5},

        { dim.x*0.5, dim.y*0.5, dim.z*0.5},
        { dim.x*0.5,-dim.y*0.5,-dim.z*0.5},
        { dim.x*0.5, dim.y*0.5,-dim.z*0.5},

        { dim.x*0.5,-dim.y*0.5,-dim.z*0.5},
        { dim.x*0.5, dim.y*0.5, dim.z*0.5},
        { dim.x*0.5,-dim.y*0.5, dim.z*0.5},

        { dim.x*0.5, dim.y*0.5, dim.z*0.5},
        { dim.x*0.5, dim.y*0.5,-dim.z*0.5},
        {-dim.x*0.5, dim.y*0.5,-dim.z*0.5},

        { dim.x*0.5, dim.y*0.5, dim.z*0.5},
        {-dim.x*0.5, dim.y*0.5,-dim.z*0.5},
        {-dim.x*0.5, dim.y*0.5, dim.z*0.5},

        { dim.x*0.5, dim.y*0.5, dim.z*0.5},
        {-dim.x*0.5, dim.y*0.5, dim.z*0.5},
        { dim.x*0.5,-dim.y*0.5, dim.z*0.5}
    };

    mesh->normals = 
    {
        {-1.0, 0.0, 0.0},
        {-1.0, 0.0, 0.0},
        {-1.0, 0.0, 0.0},

        { 0.0, 0.0,-1.0},
        { 0.0, 0.0,-1.0},
        { 0.0, 0.0,-1.0},

        { 0.0,-1.0, 0.0},
        { 0.0,-1.0, 0.0},
        { 0.0,-1.0, 0.0},

        { 0.0, 0.0,-1.0},
        { 0.0, 0.0,-1.0},
        { 0.0, 0.0,-1.0},

        {-1.0, 0.0, 0.0},
        {-1.0, 0.0, 0.0},
        {-1.0, 0.0, 0.0},

        { 0.0,-1.0, 0.0},
        { 0.0,-1.0, 0.0},
        { 0.0,-1.0, 0.0},

        { 0.0, 0.0, 1.0},
        { 0.0, 0.0, 1.0},
        { 0.0, 0.0, 1.0},

        { 1.0, 0.0, 0.0},
        { 1.0, 0.0, 0.0},
        { 1.0, 0.0, 0.0},

        { 1.0, 0.0, 0.0},
        { 1.0, 0.0, 0.0},
        { 1.0, 0.0, 0.0},

        { 0.0, 1.0, 0.0},
        { 0.0, 1.0, 0.0},
        { 0.0, 1.0, 0.0},

        { 0.0, 1.0, 0.0},
        { 0.0, 1.0, 0.0},
        { 0.0, 1.0, 0.0},

        { 0.0, 0.0, 1.0},
        { 0.0, 0.0, 1.0},
        { 0.0, 0.0, 1.0}
    };

    mesh->colors = std::vector<glm::vec4>(36, color);
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

    else if(key == GLFW_KEY_P && action == GLFW_PRESS)
        windowPtr->TogglePause();

    else if(key == GLFW_KEY_LEFT && action == GLFW_PRESS)
    {
        Engine* engine{windowPtr->GetEnginePtr()};

        if(engine)
			engine->GetEntities().back()->GetRigidBody()->IncVelocity({1.0, 0.0, 0.0});
    }
    else if(key == GLFW_KEY_RIGHT && action == GLFW_PRESS)
    {
        Engine* engine{windowPtr->GetEnginePtr()};

        if(engine)
			engine->GetEntities().back()->GetRigidBody()->IncVelocity({-1.0, 0.0, 0.0});
    }
    else if(key == GLFW_KEY_UP && action == GLFW_PRESS)
    {
        Engine* engine{windowPtr->GetEnginePtr()};

        if(engine)
			engine->GetEntities().back()->GetRigidBody()->IncVelocity({0.0, 0.0, 1.0});
    }
    else if(key == GLFW_KEY_DOWN && action == GLFW_PRESS)
    {
        Engine* engine{windowPtr->GetEnginePtr()};

        if(engine)
			engine->GetEntities().back()->GetRigidBody()->IncVelocity({0.0, 0.0, -1.0});
    }
    else if(key == GLFW_KEY_PAGE_UP && action == GLFW_PRESS)
    {
        Engine* engine{windowPtr->GetEnginePtr()};

        if(engine)
			engine->GetEntities().back()->GetRigidBody()->IncVelocity({0.0, 1.0, 0.0});
    }
    else if(key == GLFW_KEY_PAGE_DOWN && action == GLFW_PRESS)
    {
        Engine* engine{windowPtr->GetEnginePtr()};

        if(engine)
			engine->GetEntities().back()->GetRigidBody()->IncVelocity({0.0, -1.0, 0.0});
    }

    else if(key == GLFW_KEY_LEFT_BRACKET && action == GLFW_PRESS)
    {
        Engine* engine{windowPtr->GetEnginePtr()};
        engine->SetGravity(engine->GetGravity() - 5.0);
	}

    else if(key == GLFW_KEY_RIGHT_BRACKET && action == GLFW_PRESS)
    {
        Engine* engine{windowPtr->GetEnginePtr()};
        engine->SetGravity(engine->GetGravity() + 5000.0);
	}

    else if(key == GLFW_KEY_BACKSLASH && action == GLFW_PRESS)
    {
        Engine* engine{windowPtr->GetEnginePtr()};
        engine->SetGravity(0.0);
    }
	
	else 
	{
		FreeRoamCamera& cam{windowPtr->GetCameraRef()};
		if(key == GLFW_KEY_Q && (action != GLFW_RELEASE))
			cam.DownKey();                                                   
		else if(key == GLFW_KEY_W && (action != GLFW_RELEASE))
			cam.ForwardsKey();       
		else if(key == GLFW_KEY_E && (action != GLFW_RELEASE))
			cam.UpKey();            
		else if(key == GLFW_KEY_A && (action != GLFW_RELEASE))
			cam.LeftKey();         
		else if(key == GLFW_KEY_S && (action != GLFW_RELEASE))
			cam.BackwardsKey();   
		else if(key == GLFW_KEY_D && (action != GLFW_RELEASE))
			cam.RightKey();
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
            arma::vec3 pos{BookKeeping::gta3(cam.GetPosition())};
            arma::vec3 dir{BookKeeping::gta3(cam.GetDirection())};

            arma::vec3 shootDir{arma::normalise(dir)};
            shootDir *= 40.0;

            Entity* ent{new Entity{MakeSphere(30.0, 1.0, 30, {1.0, 1.0, 1.0, 1.0}, {1.0, 0.0, 0.0, 1.0}, pos, shootDir)}};

            windowPtr->AddEntity(*curProgram, ent);
            engine->AddEntity(ent);
            windowPtr->SetModelUniforms(engine->GetEntities());

            IForce* force{new GravForce(0.5, ent->GetRigidBody())};
            engine->AddForceToSystem(force);

    	    ent->GetRigidBody()->ApplyTorque({10000.0, 60.0, 0.0});
        }
    }
}

glm::vec4 RainbowColoring (float const& t)
{
    if(t <= 0.2)
        return glm::mix(glm::vec4(1.0, 0.0, 1.0, 1.0), glm::vec4(0.0, 0.0, 1.0, 1.0), t * 1.0 / 0.2);
    else if(t <= 0.4) 
        return glm::mix(glm::vec4(0.0, 0.0, 1.0, 1.0), glm::vec4(0.0, 1.0, 1.0, 1.0), t * 1.0 / 0.4);
    else if(t <= 0.6) 
        return glm::mix(glm::vec4(0.0, 1.0, 1.0, 1.0), glm::vec4(0.0, 1.0, 0.0, 1.0), t * 1.0 / 0.4);
    else if(t <= 0.8) 
        return glm::mix(glm::vec4(0.0, 1.0, 0.0, 1.0), glm::vec4(1.0, 1.0, 0.0, 1.0), t * 1.0 / 0.4);
    
    return glm::mix(glm::vec4(1.0, 1.0, 0.0, 1.0), glm::vec4(1.0, 0.0, 0.0, 1.0), t * 1.0 / 0.4);
}

std::vector<Spring> springs;

std::vector<Entity*> Stack (int const& n, double const& sep, double const& rad, arma::vec3 const& initPos={0.0,0.0,0.0}) 
{
    std::vector<Entity*> spheres;
    double sclr{20.0};
    for(int i = 0; i < n; ++i)
    {
        for(int j = 0; j < i+1; ++j)
        {
            spheres.push_back(new Entity{MakeBox(1.0, {rad * 0.5, rad * 0.5, rad * 0.5}, 
                        RainbowColoring(fmod(sin(sclr *  i   /(GLfloat)n), 1.0)), 
                        initPos + arma::vec3{rad*((sep+2.0)*j - (1.0+0.5*sep)*i), 
                                              0.0, 
											  rad*(sep+2.0)*i*std::cos(M_PI/6)},
						{0.0, 0.0, 0.0})}); 
        }
    }

    for(int i = 0; i < n; ++i)
    {
        for(int j = 0; j < i; ++j)
        {
            RigidBody* rb1{spheres[i*(i+1)/2+j]->GetRigidBody()};
            RigidBody* rb2{spheres[i*(i+1)/2+j+1]->GetRigidBody()};
            springs.push_back({rb1,rb2});
        }
    }

    return spheres;
}

std::vector<Entity*> RopeBride ()
{
    std::vector<Entity*> boxes;
    
    static const glm::vec4 brown{0.82,0.41,0.12,1.0};

    Entity* prev_left_beam, *prev_right_beam;
    static const size_t n = 40;
    for(size_t i = 0; i < n; ++i)
    {
        double zpos = -10.0 + (i / (float)(n-1)) * 20.0;

        Entity* left_beam{new Entity{MakeBox(0.05, {0.05, 0.05, 0.5}, brown, {-1.1, 1.0, zpos})}};
        Entity* right_beam{new Entity{MakeBox(0.05, {0.05, 0.05, 0.5}, brown, {1.1, 1.0, zpos})}};

        // Fix end beams
        if(i == 0 || i == n-1)
        {
            left_beam->GetRigidBody()->Fix();
            right_beam->GetRigidBody()->Fix();
        }

        //Join consecutive beams
        if(i > 0)
        {
            springs.push_back({left_beam->GetRigidBody(), prev_left_beam->GetRigidBody()});
            springs.push_back({right_beam->GetRigidBody(), prev_right_beam->GetRigidBody()});
        }

        boxes.push_back(left_beam);
        boxes.push_back(right_beam);

        Entity* left_join{new Entity{MakeBox(0.05, {0.05, 0.05, 0.05}, brown, {-1.1, 0.0, zpos})}};
        Entity* right_join{new Entity{MakeBox(0.05, {0.05, 0.05, 0.05}, brown, {1.1, 0.0, zpos})}};

        Entity* step{new Entity{MakeBox(0.01, {0.95, 0.05, 0.05}, brown, {0.0, 0.0, zpos})}};

        boxes.push_back(step);
        boxes.push_back(left_join);
        boxes.push_back(right_join);

        springs.push_back({left_beam->GetRigidBody(), left_join->GetRigidBody()});
        springs.push_back({right_beam->GetRigidBody(), right_join->GetRigidBody()});

        springs.push_back({left_join->GetRigidBody(), step->GetRigidBody(), 10.0});
        springs.push_back({step->GetRigidBody(), right_join->GetRigidBody(), 10.0});

        prev_left_beam = left_beam;
        prev_right_beam = right_beam;
    }

    return boxes;
}

//std::vector<Entity*> Square (int const& n, float const& sep, float const& rad, glm::vec3 const& initPos=glm::vec3(0.0)) 
//{
//    std::vector<Entity*> spheres;
//    float sclr{20.0};
//    for(int i = 0; i < n; ++i)
//    {
//        for(int j = 0; j < n; ++j)
//        {
//            spheres.push_back(new Entity{MakeSphere(3.0, 0.99, rad, 4, 
//                        RainbowColoring(fmod(sin(sclr *  i   /(GLfloat)n), 1.0)),
//                        RainbowColoring(fmod(sin(sclr * (i+1)/(GLfloat)n), 1.0)), 
//                        initPos + glm::vec3(rad*((sep+2.0)*j - (1.0*0.5*sep)*n), 
//                                            0.0, rad*(sep+2.0)*i) )}); 
//        }
//    }
//
//    return spheres;
//}

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
    if(!renderer.Initailize(1920.0, 1080.0, true, KeyCallback, MouseCallback))
    {
        std::cerr<<"Failed to Initialize GLFWContext!"<<std::endl;
        ERROR("Failed to Initialize GLFWContext!");
        exit(0);
    }
	glfwSetWindowPos(renderer.Window(), 500, 500);

    FreeRoamCamera camera(10.0, 1.0);
    camera.SetPosition(glm::vec3(-1.0, -15.0, 1.0));
    camera.SetDirection(glm::vec3(-1.0, 0.0, 0.0));
    camera.SetProjection(45.0, 1920.0/1080.0, 0.1, 400.0);
    camera.UpdateView();
    renderer.SetCamera(camera);

    //Create a new shader program
    GLProgram program;
    renderer.GetNewProgram(program, "../src/Shaders/vert_simple.glsl", "../src/Shaders/frag_simple.glsl", SGV_POSITION | SGV_NORMAL | SGV_COLOR);
    renderer.BindProgram(program);

	std::vector<Entity*> spheres;
    
//    spheres = Stack(30, 0.0, 1.3, {0.0, 0.0, 5.0});  
    spheres = RopeBride();  

//    spheres.push_back(new Entity{MakeBox(30.0, glm::vec3{6.0, 30.0, 30.0}, {1.0, 1.0, 1.0, 1.0}, {-17.0, 0.0, 0.0}, {2.0, 0.0, 0.0})});
//    spheres.back()->GetRigidBody()->SetAngularVelocity(0.01 * arma::vec3{1.0, 5.0, 1.0});
//    spheres.push_back(new Entity{MakeBox(30.0, glm::vec3{6.0, 30.0, 30.0}, {1.0, 1.0, 1.0, 1.0}, {17.0, 0.0, 0.0}, {-2.0, 0.0, 2.0})});
//    spheres.back()->GetRigidBody()->SetAngularVelocity(0.01 * arma::vec3{1.0, 5.0, 1.0});

//    spheres.push_back(new Entity{MakeBox(8.0, {1.0, 9.0, 1.0}, {1.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0})});
//
//    spheres.push_back(new Entity{MakeBox(8.0, {1.0, 8.0, 1.0}, {1.0, 0.0, 0.0, 1.0}, {17.0, 0.0, 0.0}, {-1.0, 0.0, 0.0})});
//    spheres.back()->GetRigidBody()->SetAngularVelocity({1.0, 5.0, 1.0});

	unsigned n{(unsigned)spheres.size()};

    renderer.AddEntities(program, spheres);

    //Create physics engine
    Engine engine{spheres, 10};
    renderer.SetEnginePtr(engine);
    renderer.SetProgram(&program);

	engine.SetGravity(0.01);

//	Spring s{spheres[n-3]->GetRigidBody(), spheres[n-2]->GetRigidBody(), 10.0};

    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end; 
    double dt{1.0 / 60.0};

    BroadPhase bp;
    std::vector<RigidBody*> rbs(n);
    for(unsigned i= 0; i < rbs.size(); ++i)
    {
        rbs[i] = spheres[i]->GetRigidBody();
    }
    bp.BatchInsert(rbs);
    bp.InitialOverlapCache();

    std::vector<GraphMesh> springMeshes(springs.size());
    Mesh mesh;
    mesh.positions = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    mesh.normals = {{0.0, 1.0, 0.0}, {0.0, 1.0, 0.0}};
    mesh.colors = {{0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}};
    for(auto& springMesh : springMeshes)
    {
        springMesh = program.AddMesh(mesh, GL_LINES);
    }

	std::vector<GraphMesh> axes(3);
	Mesh xAxis, yAxis, zAxis;
	xAxis.positions = {glm::vec3{0.0, 0.0, 0.0}, 10.0f*glm::vec3{1.0, 0.0, 0.0}};
	yAxis.positions = {glm::vec3{0.0, 0.0, 0.0}, 10.0f*glm::vec3{0.0, 1.0, 0.0}};
	zAxis.positions = {glm::vec3{0.0, 0.0, 0.0}, 10.0f*glm::vec3{0.0, 0.0, 1.0}};
	xAxis.colors = {{1.0, 0.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 1.0}};
	yAxis.colors = {{0.0, 1.0, 0.0, 1.0}, {0.0, 1.0, 0.0, 1.0}};
	zAxis.colors = {{0.0, 0.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 1.0}};
	xAxis.normals = {{0.0,1.0,0.0},{0.0,1.0,0.0}};
	yAxis.normals = {{0.0,1.0,0.0},{0.0,1.0,0.0}};
	zAxis.normals = {{0.0,1.0,0.0},{0.0,1.0,0.0}};
	axes[0] = program.AddMesh(xAxis, GL_LINES);
	axes[1] = program.AddMesh(yAxis, GL_LINES);
	axes[2] = program.AddMesh(zAxis, GL_LINES);

    std::vector<GraphMesh> normals(6);

    //Continuously render OpenGL and update physics system 
    glfwSetTime(0.0);
    double time_since_last_collision{3.0};
    bool slow{false};
    GLfloat color[4]{0.9, 0.9, 0.9, 1.0};

    while(true)
    {
        start = std::chrono::system_clock::now();

        for(auto& ent: spheres)
        {
            ent->GetRigidBody()->SetForce({0.0,0.0,0.0});
        }
        for(auto& spring: springs)
        {
            spring.Apply();
        }

        Engine::RunInfo info;
        if(!renderer.Paused())
            info = engine.Run(dt, bp);
        
		renderer.SetModelUniforms(engine.GetEntities());
        if(!renderer.Render(program.Strip(), color))
            break;

        for(unsigned i = 0; i < springMeshes.size(); ++i)
        {
            auto& springMesh{springMeshes[i]};
            auto& spring{springs[i]};

            glm::vec3 p1{BookKeeping::atg3(spring.GetBody1()->Position())}, p2{BookKeeping::atg3(spring.GetBody2()->Position())};
			glm::vec3 q{p2 - p1};

            glm::mat4x4 model{
				q[0],q[1],q[2],0.0,
				0.0,0.0,0.0,0.0,
				0.0,0.0,0.0,0.0,
				p1[0],p1[1],p1[2],1.0};

            glUniformMatrix4fv(5, 1, GL_FALSE, glm::value_ptr(model));
            Indexer indexer{springMesh.GetSigIndexer()};
            glDrawArrays(GL_LINES, indexer.First(), indexer.Count());
        }

		glUniformMatrix4fv(5, 1, GL_FALSE, glm::value_ptr(glm::mat4x4(1.0)));
        for(auto& axis: axes)
        {
            Indexer indexer{axis.GetSigIndexer()};
            glDrawArrays(GL_LINES, indexer.First(), indexer.Count());
        }

        if(info.num_constraints > 0)
        {
            for(int i = 0; i < 6; ++i)
            {
                arma::vec3 norm = static_cast<MeshCollidable*>(spheres[0]->GetRigidBody()->GetCollidable())->TransformedNormals()[i];
                norm *= 10.0;
                Mesh norm_mesh;
                norm_mesh.positions = {{0.0, 0.0, 0.0}, {norm[0], norm[1], norm[2]}};
                norm_mesh.colors = {{1.0, 1.0, 1.0, 1.0}, {1.0, 1.0, 1.0, 1.0}};
                norm_mesh.normals = {{0.0, 1.0, 0.0}, {0.0, 1.0, 0.0}};
                normals[i] = program.AddMesh(norm_mesh, GL_LINES);
            }
        }

		glUniformMatrix4fv(5, 1, GL_FALSE, glm::value_ptr(glm::mat4x4(1.0)));
        for(auto& nm: normals)
        {
            Indexer indexer{nm.GetSigIndexer()};
            glDrawArrays(GL_LINES, indexer.First(), indexer.Count());
        }

        glfwSwapBuffers(renderer.Window());

		arma::vec3 momentum;
		momentum.zeros();
		for(auto const& ent: engine.GetEntities())
		{
			RigidBody* rb{ent->GetRigidBody()};
			if(rb->InvMass() > FLT_EPSILON)
				momentum += rb->LinearMomentum();
		}

	    DEBUG_MSG("Momentum: %f, %f, %f\n", momentum(0), momentum(1), momentum(2));

        end = std::chrono::system_clock::now();
        dt = std::chrono::duration<float>(end-start).count();
        time_since_last_collision += dt;

        if(info.num_constraints > 0)
        {
            time_since_last_collision = 0.0;
            slow = true;
        }

        if(time_since_last_collision < 2.0)
        {
            color[0] = 0.0;
            color[1] = 0.0;
            color[2] = 1.0;
        }
        else
        {
            color[0] = 0.9;
            color[1] = 0.9;
            color[2] = 0.9;
        }
    }
}
