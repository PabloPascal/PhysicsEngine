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

    for(float i = 0; i < 2*r; i++){
        for(float j = 0; j < 2*r; j++){

            float dx = i - r;
            float dy = j - r;

            if(dx * dx + dy*dy < r*r){
                SDL_RenderDrawPoint(renderer, circle.get_position().x + dx, circle.get_position().y + dy);
            }
        }
    }
}


void drawRect(SDL_Renderer* renderer, Phx::Rect& rect){
    
    SDL_Rect render_rect;

    Phx::Vec2 l11 = (rect.get_normal2() - rect.get_normal1()) * rect.get_size().x / 2;
    Phx::Vec2 l12 = (rect.get_normal2() + rect.get_normal1()) * rect.get_size().y /2;


    float px1 = (rect.get_position() + l11).x;
    float py1 = (rect.get_position() + l11).y;

    float px2 = (rect.get_position() + l12).x;
    float py2 = (rect.get_position() + l12).y;

    float px3 = (rect.get_position() - l11).x;
    float py3 = (rect.get_position() - l11).y;
    
    float px4 = (rect.get_position() - l12).x;
    float py4 = (rect.get_position() - l12).y;



    render_rect.x = px1;
    render_rect.y = py1;
    render_rect.w = rect.get_size().x;
    render_rect.h = rect.get_size().y;

    // std::cout << "px1 = " << px1 << std::endl;
    // std::cout << "px2 = " << px2 << std::endl;

    //SDL_RenderDrawRect(renderer, &render_rect);

    SDL_RenderDrawLine(renderer, 40, 12, 23, 22);

    for(short i = 0; i < 4; i++)
    {
        if(i < 3)
        {
        SDL_RenderDrawLine(renderer, rect.get_vertices()[i].x, rect.get_vertices()[i].y, 
                                    rect.get_vertices()[i+1].x, rect.get_vertices()[i+1].y);
        }
        else{
            SDL_RenderDrawLine(renderer, rect.get_vertices()[3].x, rect.get_vertices()[3].y, 
                                    rect.get_vertices()[0].x, rect.get_vertices()[0].y);
        }
    } 
    // SDL_RenderDrawLine(renderer, px1, py1, px2, py2);
    // SDL_RenderDrawLine(renderer, px2, py2, px3, py3);
    // SDL_RenderDrawLine(renderer, px3, py3, px4, py4);
    // SDL_RenderDrawLine(renderer, px4, py4, px1, py1);



    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);


    SDL_RenderDrawPoint(renderer, px1, py1);
    SDL_RenderDrawPoint(renderer, px2, py2);
    SDL_RenderDrawPoint(renderer, px3, py3);
    SDL_RenderDrawPoint(renderer, px4, py4);

    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);

    

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

    Phx::Circle circle1(x, 50, 50);   
    circle1.set_velocity({0,0});
    circle1.set_elasticity(0.8);
    circle1.set_mass(20);
    circle1.set_move_on(true);
    //circle1.set_gravity_on(true);
    circle1.set_collision_on(true);

    Phx::Circle circle2(x, 200, 50);   
    circle2.set_velocity({0,0});
    circle2.set_elasticity(0.5);
    circle2.set_mass(30);
    circle2.set_move_on(true);
    //circle2.set_gravity_on(true);
    circle2.set_collision_on(true);

    Phx::Circle circle3(x + 5, y, 50);   
    circle3.set_velocity({0,0});
    circle3.set_elasticity(0.5);
    circle3.set_mass(30);
    circle3.set_move_on(true);
    //circle3.set_gravity_on(true);
    circle3.set_collision_on(true);

    Phx::PhysicsWorld world({WIDTH, HEIGHT});
    //world.generate_circles(1000);
    //world.add_circle(&circle1);
    //world.add_circle(&circle2);
    //world.add_circle(&circle3);

    std::shared_ptr<Phx::Rect> rect0
    = std::make_shared<Phx::Rect>(Phx::Vec2(x, y), Phx::Vec2(200,200), 20.f);   
    rect0->set_velocity({50,0});
    
    std::shared_ptr<Phx::Rect> rect1
    = std::make_shared<Phx::Rect>(Phx::Vec2(200, 200),Phx::Vec2(200,200), 20.f);   
    rect1->set_velocity({0,0});

    rect0->set_rotate(rad);

    world.add_rect(rect0);
    world.add_rect(rect1);

    Phx::SATcheckCollision(*rect0, *rect1);


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
                    circle->set_mass(5);
                    circle->set_move_on(true);
                    circle->set_gravity_on(true);
                    circle->set_collision_on(true);
                    world.add_circle(circle);
                    
                }

                if(event.button.button == SDL_BUTTON_RIGHT)
                {
                    
                    std::shared_ptr<Phx::Rect> rect
                     = std::make_shared<Phx::Rect>(Phx::Vec2(event.button.x, event.button.y),Phx::Vec2(50,50), 20);   
                    rect->set_velocity({0,0});

                    world.add_rect(rect);
                    
                }

            }
            
        }

        const Uint8* keyState = SDL_GetKeyboardState(NULL);
        handleInput(keyState);
        rect0->set_velocity({velocityX, velocityY});
        // x += velocityX * deltaTime;
        // y += velocityY * deltaTime;

        rect0->set_rotate(rad*deltaTime);
        rect0->update(deltaTime);


        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        
        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
        world.update(deltaTime);
    

        for(auto it : world.get_circles()){
            drawCircle(renderer, *it);
        }

        if(Phx::SATcheckCollision(*rect0, *rect1)){
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        }else{
            SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
        }

        for(auto it : world.get_rects()){
            drawRect(renderer, *it);
        }
        
        SDL_RenderPresent(renderer);

        SDL_SetWindowTitle(window, std::to_string(1.f/deltaTime).c_str());

        t += 0.05;

    }
    

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    
    return 0;
}