#include <iostream>
#include <chrono>
#include <phx_vector.hpp>
#include <calculate_collisions.hpp>
#include <phx_circle.hpp>
#include <phx_rect.hpp>
#include <phx_world.hpp>
#include <SDL2/SDL.h>


void drawCircle(SDL_Renderer* renderer, Phx::Circle& circle){

    float r = circle.get_radius();

    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 1);

    for(float i = 0; i < 2*r; i++){
        for(float j = 0; j < 2*r; j++){

            float dx = i - r;
            float dy = j - r;

            if(dx * dx + dy*dy <= (r+1)*(r+1) && dx * dx + dy*dy >= (r-1)*(r-1)){
                SDL_RenderDrawPoint(renderer, circle.get_position().x + dx, circle.get_position().y + dy);
            }
        }
    }
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 1);

}


void drawRect(SDL_Renderer* renderer, Phx::Rect& rect){
    
    SDL_Rect render_rect;



    SDL_RenderDrawLine(renderer, 40, 12, 23, 22);

    for(short i = 0; i < 4; i++)
    {
        if(i < 3)
        {
        SDL_RenderDrawLine(renderer, rect.get_vertices()[i].x, rect.get_vertices()[i].y, 
                                    rect.get_vertices()[i+1].x, rect.get_vertices()[i+1].y);
        SDL_RenderDrawPoint(renderer, rect.get_vertices()[i].x, rect.get_vertices()[i].y);

        }
        else{
            SDL_RenderDrawLine(renderer, rect.get_vertices()[3].x, rect.get_vertices()[3].y, 
                                    rect.get_vertices()[0].x, rect.get_vertices()[0].y);
            SDL_RenderDrawPoint(renderer, rect.get_vertices()[i].x, rect.get_vertices()[i].y);
        }
    } 
    
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 1);

    

}



void drawDebug(SDL_Renderer* renderer, Phx::Rect& r, Phx::Circle& c)
{
    float min_dist = std::numeric_limits<float>::max();
    Phx::Vec2 vert;
    Phx::Vec2 center = c.get_position();


    for(int i = 0; i < 4; i++)
    {   
        float dist = Phx::length( c.get_position() - r.get_vertices()[i]);
        if(dist < min_dist)
            {
                min_dist = dist;
                vert = r.get_vertices()[i];
            }
    }

    float penetrate;
    Phx::Vec2 n;
    Phx::Vec2 cp;

    if(Phx::CircleRectCheckCollision(c, r, n, cp, penetrate))
    {
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 1);
        Phx::Circle point(cp, 5);
        drawCircle(renderer, point);
        
    }

    Phx::Vec2 project_vec1 =  r.get_normal1() * -Phx::dot(c.get_position() - vert, r.get_normal1()); 
    Phx::Vec2 project_vec2 =  r.get_normal2() * -Phx::dot(c.get_position() - vert, r.get_normal2()); 

    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 1);
    SDL_RenderDrawLine(renderer, center.x, center.y, center.x + project_vec1.x, center.y + project_vec1.y);
    SDL_RenderDrawLine(renderer, center.x, center.y, center.x + project_vec2.x, center.y + project_vec2.y);

    SDL_RenderDrawLine(renderer, vert.x, vert.y, center.x, center.y);
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 1);

   //std::cout << "is inside = " << Phx::checkPointInsidePolygon(c.get_position(), r) << std::endl;
}



float speed = 250;
float velocityX = 0;
float velocityY = 0;
float rot_speed = 1;
float rad = 0;


void handleInput(const Uint8* keyboardState) {
        velocityX = 0;
        velocityY = 0;
        rad = 0;

        // Обработка WASD
        if (keyboardState[SDL_SCANCODE_W]) {
            velocityY = -speed;
        }
        if (keyboardState[SDL_SCANCODE_S]) {
            velocityY = speed;
        }
        if (keyboardState[SDL_SCANCODE_A]) {
            velocityX = -speed;
        }
        if (keyboardState[SDL_SCANCODE_D]) {
            velocityX = speed;
        }
        if(keyboardState[SDL_SCANCODE_R]){
            rad += rot_speed;
        }
        // Нормализация диагонального движения
        if (velocityX != 0 && velocityY != 0) {
            velocityX *= 0.7071f; // 1/√2
            velocityY *= 0.7071f;
        }
    }


constexpr int WIDTH = 1080;
constexpr int HEIGHT = 720;

int main(){

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        SDL_Log("Unable to initialize SDL: %s", SDL_GetError());
        return 1;
    }
    // Create a window
    SDL_Window* window = SDL_CreateWindow(
        "Physics Engine Editor",         // Window title
        SDL_WINDOWPOS_UNDEFINED,      // Initial x position
        SDL_WINDOWPOS_UNDEFINED,      // Initial y position
        WIDTH,                          // Width
        HEIGHT,                          // Height
        SDL_WINDOW_SHOWN              // Flags (e.g., SDL_WINDOW_FULLSCREEN)
    );

    if (window == NULL) {
        SDL_Log("Could not create window: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);

    SDL_Surface* screenSurface = SDL_GetWindowSurface(window);
    SDL_FillRect(screenSurface, NULL, SDL_MapRGB(screenSurface->format, 0, 0, 255));


    bool quit = false;
    SDL_Event event;
    
    float x = WIDTH / 2;
    float y = HEIGHT / 2;

    float t = 0;
    auto previousTime = std::chrono::high_resolution_clock::now();
    float deltaTime = 0.0f;

    Phx::Circle *circle1 = new Phx::Circle(540, 510, 50);   
    circle1->set_velocity({0,0});
    circle1->set_elasticity(0.8);
    circle1->set_mass(20);
    circle1->set_move_on(true);
    //circle1->set_gravity_on(true);
    circle1->set_collision_on(true);

    Phx::Circle* circle2 = new Phx::Circle(x, 200, 50);   
    circle2->set_velocity({0,0});
    circle2->set_elasticity(0.5);
    circle2->set_mass(30);
    circle2->set_move_on(true);
    //circle2.set_gravity_on(true);
    circle2->set_collision_on(true);

    Phx::Circle* circle3 = new Phx::Circle(x + 5, y, 50);   
    circle3->set_velocity({0,0});
    circle3->set_elasticity(0.5);
    circle3->set_mass(30);
    circle3->set_move_on(true);
    //circle3.set_gravity_on(true);
    circle3->set_collision_on(true);

    Phx::PhysicsWorld world({WIDTH, HEIGHT});
    //world.generate_circles(700);
    //world.add_circle(circle1);
    //world.add_circle(&circle2);
    //world.add_circle(&circle3);

    std::shared_ptr<Phx::Rect> rect0
    = std::make_shared<Phx::Rect>(Phx::Vec2(500, 392), Phx::Vec2(50,50), 20.f);   
    rect0->set_velocity({0,0});
    rect0->set_elasticity(0.5);
    rect0->set_elasticity(0.8);
    rect0->set_angle_speed(0);
    rect0->set_collision_indicate(true);
    
    std::shared_ptr<Phx::Rect> rect1
    = std::make_shared<Phx::Rect>(Phx::Vec2(0, 560),Phx::Vec2(WIDTH,50), std::numeric_limits<float>::max());   
    rect1->set_velocity({0,0});
    rect1->set_elasticity(0.8);
    rect1->set_angle_speed(0);
    rect1->set_acceleration({0,0});
    rect1->set_collision_indicate(true);


    std::shared_ptr<Phx::Rect> rect2
    = std::make_shared<Phx::Rect>(Phx::Vec2(0, 200),Phx::Vec2(WIDTH/3.f,50), std::numeric_limits<float>::max());   
    rect2->set_velocity({0,0});
    rect2->set_elasticity(0.8);
    rect2->set_angle_speed(0);
    rect2->set_acceleration({0,0});
    rect2->set_collision_indicate(true);
    rect2->set_rotate(30 * 3.1415f / 180.f);
    world.add_rect(rect0);
    world.add_rect(rect1);
    world.add_rect(rect2);


    while(!quit){

        auto currentTime = std::chrono::high_resolution_clock::now();
        deltaTime = std::chrono::duration<float>(currentTime - previousTime).count();
        previousTime = currentTime;


        while(SDL_PollEvent(&event)){
            if(event.type == SDL_QUIT)
                quit = true;
            if(event.type == SDL_MOUSEBUTTONDOWN){

                if(event.button.button == SDL_BUTTON_LEFT)
                {
                    
                    std::shared_ptr<Phx::Circle> circle = std::make_shared<Phx::Circle>(event.button.x, event.button.y, 23);   
                    circle->set_velocity({0,0});
                    circle->set_elasticity(0.8);
                    circle->set_mass(50);
                    circle->set_acceleration({0, 500});
                    circle->set_gravity_on(true);
                    circle->set_collision_on(true);
                    world.add_circle(circle);
                    
                }

                if(event.button.button == SDL_BUTTON_RIGHT)
                {
                    
                    std::shared_ptr<Phx::Rect> rect
                     = std::make_shared<Phx::Rect>(Phx::Vec2(event.button.x, event.button.y),Phx::Vec2(50,50), 20);   
                    rect->set_velocity({0,0});
                    rect->set_elasticity(0.5);
                    rect->set_elasticity(0.8);
                    rect->set_angle_speed(0);
                    rect->set_acceleration({0,500});
                    rect->set_collision_indicate(true);

                    world.add_rect(rect);
                    
                }

            }
            
        }

        

        const Uint8* keyState = SDL_GetKeyboardState(NULL);
        handleInput(keyState);
        rect0->set_velocity({velocityX, velocityY});
        


        rect0->set_rotate(rad*deltaTime);

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        
        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
        
        world.update(deltaTime);


        for(auto it : world.get_circles()){
            drawCircle(renderer, *it);
        }
        Phx::Vec2 n;
        float d;
        if(Phx::AABBcheckCollision(*rect0, *rect1, n, d)){
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        }else{
            SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
        }

        for(auto it : world.get_rects()){
            drawRect(renderer, *it);
        }

        
        //drawDebug(renderer, *rect0, *circle1);

        

        SDL_RenderPresent(renderer);

        SDL_SetWindowTitle(window, std::to_string(1.f/deltaTime).c_str());

        t += 0.05;

    }
    

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    
    return 0;
}