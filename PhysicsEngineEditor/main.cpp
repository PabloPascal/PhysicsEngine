#include <SDL2/SDL.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include <algorithm>

#include "phx_World.hpp"
#include "phx_Body.hpp"
#include "phx_Material.hpp"
#include "Colliders/phx_CircleCollider.hpp"
#include "Colliders/phx_RectCollider.hpp"


namespace {

constexpr int   WINDOW_W = 1200;
constexpr int   WINDOW_H = 800;
constexpr float PPM      = 80.f;   // pixels per meter

constexpr SDL_Color BG_COLOR      = { 18,  18,  28, 255};
constexpr SDL_Color STATIC_COLOR  = { 90,  95, 110, 255};
constexpr SDL_Color DYNAMIC_COLOR = {230, 120,  60, 255};
constexpr SDL_Color DEBUG_COLOR   = { 20,  20,  20, 255};


// Мир: x → вправо, y → вверх. Экран: x → вправо, y → вниз.
SDL_FPoint to_screen(Phx::Vec2 p)
{
    return { p.x * PPM, static_cast<float>(WINDOW_H) - p.y * PPM };
}


Phx::Vec2 to_world(int sx, int sy)
{
    return {
        static_cast<float>(sx) / PPM,
        static_cast<float>(WINDOW_H - sy) / PPM
    };
}

void fill_circle(SDL_Renderer* r, int cx, int cy, int radius)
{
    for (int dy = -radius; dy <= radius; ++dy) {
        const int dx = static_cast<int>(std::sqrt(
            static_cast<float>(radius * radius - dy * dy)));
        SDL_RenderDrawLine(r, cx - dx, cy + dy, cx + dx, cy + dy);
    }
}


void draw_circle(SDL_Renderer* r, const Phx::Body& body,
                 const Phx::CircleCollider& c, SDL_Color color)
{
    const SDL_FPoint center = to_screen(body.position);
    const int pixelRadius   = static_cast<int>(c.get_radius() * PPM);

    SDL_SetRenderDrawColor(r, color.r, color.g, color.b, color.a);
    fill_circle(r, static_cast<int>(center.x),
                   static_cast<int>(center.y), pixelRadius);

    // Радиальная метка — чтобы видеть вращение
    const float ca = std::cos(body.angle);
    const float sa = std::sin(body.angle);
    const Phx::Vec2 edgeWorld = body.position
                              + Phx::Vec2(ca * c.get_radius(),
                                          sa * c.get_radius());
    const SDL_FPoint edgeScreen = to_screen(edgeWorld);

    SDL_SetRenderDrawColor(r, DEBUG_COLOR.r, DEBUG_COLOR.g,
                              DEBUG_COLOR.b, DEBUG_COLOR.a);
    SDL_RenderDrawLine(r,
        static_cast<int>(center.x),     static_cast<int>(center.y),
        static_cast<int>(edgeScreen.x), static_cast<int>(edgeScreen.y));
}


void draw_rect(SDL_Renderer* r, const Phx::Body& body,
               const Phx::RectCollider& rc, SDL_Color color)
{
    const float hw = rc.get_width()  * 0.5f;
    const float hh = rc.get_height() * 0.5f;

    const float ca = std::cos(body.angle);
    const float sa = std::sin(body.angle);

    const Phx::Vec2 local[4] = {
        {-hw,  hh}, { hw,  hh}, { hw, -hh}, {-hw, -hh}
    };

    SDL_Vertex verts[4];
    for (int i = 0; i < 4; ++i) {
        const float rx = local[i].x * ca - local[i].y * sa;
        const float ry = local[i].x * sa + local[i].y * ca;
        const Phx::Vec2 world = body.position + Phx::Vec2(rx, ry);
        const SDL_FPoint s = to_screen(world);

        verts[i].position   = s;
        verts[i].color      = color;
        verts[i].tex_coord  = {0.f, 0.f};
    }
    const int indices[6] = {0, 1, 2, 0, 2, 3};
    SDL_RenderGeometry(r, nullptr, verts, 4, indices, 6);

    // Обводка
    SDL_SetRenderDrawColor(r, DEBUG_COLOR.r, DEBUG_COLOR.g,
                              DEBUG_COLOR.b, DEBUG_COLOR.a);
    SDL_FPoint s[4];
    for (int i = 0; i < 4; ++i) s[i] = verts[i].position;
    for (int i = 0; i < 4; ++i)
        SDL_RenderDrawLine(r,
            static_cast<int>(s[i].x),         static_cast<int>(s[i].y),
            static_cast<int>(s[(i+1)%4].x),   static_cast<int>(s[(i+1)%4].y));
}


void draw_body(SDL_Renderer* r, const Phx::Body& body)
{
    if (!body.collider) return;

    const SDL_Color color = body.is_static ? STATIC_COLOR : DYNAMIC_COLOR;

    switch (body.collider->type()) {
    case Phx::ColliderType::Circle:
        draw_circle(r, body,
            static_cast<const Phx::CircleCollider&>(*body.collider), color);
        break;
    case Phx::ColliderType::Rect:
        draw_rect(r, body,
            static_cast<const Phx::RectCollider&>(*body.collider), color);
        break;
    }
}

} // namespace


// ---------------------------------------------------------------------------
// Сцена
// ---------------------------------------------------------------------------

class Scene {
public:
    Phx::PhysicsWorld world;

    Phx::Body* add_circle(float x, float y, float r, float mass, bool isStatic = false)
    {
        auto collider = std::make_unique<Phx::CircleCollider>(r);
        auto body     = std::make_unique<Phx::Body>();

        body->position = {x, y};
        body->collider = collider.get();
        body->material = { 0.4f, 0.5f };

        if (isStatic) {
            body->set_mass(0.f);
            body->set_inertia(0.f);
            body->set_static(true);
        } else {
            body->set_mass(mass);
            body->set_inertia(0.5f * mass * r * r);
        }

        Phx::Body* raw = body.get();
        m_colliders.push_back(std::move(collider));
        world.addBody(std::move(body));
        m_bodies.push_back(raw);
        return raw;
    }

    Phx::Body* add_rect(float x, float y, float w, float h, float mass,
                        bool isStatic = false, float angle = 0.f)
    {
        auto collider = std::make_unique<Phx::RectCollider>(w, h);
        auto body     = std::make_unique<Phx::Body>();

        body->position = {x, y};
        body->angle    = angle;
        body->collider = collider.get();
        body->material = { 0.5f, 0.3f };

        if (isStatic) {
            body->set_mass(0.f);
            body->set_inertia(0.f);
            body->set_static(true);
        } else {
            body->set_mass(mass);
            body->set_inertia(mass * (w*w + h*h) / 12.f);
        }

        Phx::Body* raw = body.get();
        m_colliders.push_back(std::move(collider));
        world.addBody(std::move(body));
        m_bodies.push_back(raw);
        return raw;
    }

    void apply_gravity(float g = -9.81f)
    {
        for (Phx::Body* b : m_bodies) {
            if (b->is_static) continue;
            b->force = b->force + Phx::Vec2(0.f, g) * b->mass;
        }
    }

private:
    std::vector<std::unique_ptr<Phx::Collider>> m_colliders;
    std::vector<Phx::Body*>                     m_bodies;
};


static std::unique_ptr<Scene> build_demo_scene()
{
    auto s = std::make_unique<Scene>();

    // Границы (толщина 0.5 м).
    s->add_rect(7.5f,   0.25f, 15.f, 0.5f, 0.f, true);   // пол
    s->add_rect(0.25f,  5.0f,  0.5f, 10.f, 0.f, true);   // левая стена
    s->add_rect(14.75f, 5.0f,  0.5f, 10.f, 0.f, true);   // правая стена

    // Наклонная плоскость — проверить вращение и трение.
    s->add_rect(4.f, 4.f, 3.f, 0.4f, 0.f, true, 0.35f);

    // Падающие тела.
    s->add_circle(6.f,  9.f,  0.4f, 1.0f);
    s->add_circle(7.f, 11.f,  0.5f, 1.5f);
    s->add_circle(8.f, 13.f,  0.3f, 0.8f);
    s->add_rect  (10.f, 15.f, 1.0f, 0.5f, 1.0f, false, 0.3f);

    return s;
}


int main(int, char**)
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << '\n';
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "Physics Engine Editor",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WINDOW_W, WINDOW_H, SDL_WINDOW_SHOWN);

    if (!window) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << '\n';
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(
        window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    if (!renderer) {
        std::cerr << "SDL_CreateRenderer failed: " << SDL_GetError() << '\n';
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    auto scene = build_demo_scene();

    bool   running = true;
    Uint64 lastTick = SDL_GetPerformanceCounter();
    const  Uint64 freq = SDL_GetPerformanceFrequency();

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            if (e.type == SDL_KEYDOWN) {
                if (e.key.keysym.sym == SDLK_ESCAPE) running = false;
                if (e.key.keysym.sym == SDLK_r) {
                    scene = build_demo_scene();
                }
            }

            if (e.type == SDL_MOUSEBUTTONDOWN) {
                Phx::Vec2 p = to_world(e.button.x, e.button.y);

                // Клампим к безопасной зоне — не даём заспавнить тело внутри стен.
                constexpr float MARGIN = 0.7f;
                p.x = std::clamp(p.x, MARGIN, 15.f - MARGIN);
                p.y = std::clamp(p.y, MARGIN, 10.f - MARGIN);

                if (e.button.button == SDL_BUTTON_LEFT) {
                    scene->add_circle(p.x, p.y, 0.4f, 1.0f);
                }
                else if (e.button.button == SDL_BUTTON_RIGHT) {
                    scene->add_rect(p.x, p.y, 0.8f, 0.8f, 1.0f);
                }
            }
        }

        const Uint64 now = SDL_GetPerformanceCounter();
        float dt = static_cast<float>(now - lastTick)
                 / static_cast<float>(freq);
        lastTick = now;
        if (dt > 0.05f) dt = 0.05f;   // клампим при просадках

        // 1. Гравитация (пока вручную — в Core этого ещё нет).
        scene->apply_gravity(-9.81f);

        // 2. Физический шаг.
        scene->world.step(dt);

        // 3. Отрисовка.
        SDL_SetRenderDrawColor(renderer,
            BG_COLOR.r, BG_COLOR.g, BG_COLOR.b, BG_COLOR.a);
        SDL_RenderClear(renderer);

        for (const auto& body : scene->world.get_bodies())
            draw_body(renderer, *body);

        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}