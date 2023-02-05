// simulation-maze.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <algorithm>
#include <iostream>
#include <complex>
#include <utility>
#include <memory>
#include <string>
#include <random>
#include <vector>
#include <cmath>
#include <ctime>

#if __has_include("visual.hpp")
#include "visual.hpp"
#endif

using namespace std::complex_literals;
typedef std::complex<float> vec2;
typedef std::complex<int> int2;

const float timestep = 1.0f;
const int n_iters = 1;
const bool rand_force = true;

struct particle_t
{
    // physical state
    vec2 pos, vel, force;
    // computation intermediates
    int degree;  // number of bars connected
    int offset;  // offset in solution vector (vx, vy)
    constexpr float mass() { return 1; }
};

struct bar_t
{
    particle_t* start;
    particle_t* end;
    int offset;
    float force;
    bar_t(particle_t* start_, particle_t* end_) : start(start_), end(end_), offset(0), force(0) {}
    bar_t(particle_t& start_, particle_t& end_) : start(&start_), end(&end_), offset(0), force(0) {}
};

struct lildiag_matrix_t
{
    // off-diagonal elements stored in list of lists
    std::vector<std::vector<std::pair<int, float>>> lil;
    // diagonal elements stored as a dense vector
    std::vector<float> diag;

    lildiag_matrix_t(int n) : lil(), diag(n)
    {
        for (int i = 0; i < n; i++)
        {
            lil.emplace_back(0);
            lil[i].reserve(8);
        }
    }
};

float sample(float u) {
    // Making rng static ensures that it stays the same
    // Between different invocations of the function
    static std::default_random_engine rng;

    std::uniform_real_distribution<float> dist(0.0f, u);
    return dist(rng);
}

float sanity(std::vector<particle_t>& particles, std::vector<bar_t>& bars)
{
    float v = 0.0f;
    for (const auto& bar : bars)
    {
        v = std::max(v, std::abs(1.0f - std::abs(bar.start->pos - bar.end->pos)));
    }
    return std::max(1 - v, 0.0f);
}

float length_limit(std::vector<particle_t>& particles, std::vector<bar_t>& bars)
{
    float v = 0.0f;
    for (const auto& bar : bars)
    {
        v = std::max(v, std::abs(bar.start->pos - bar.end->pos));
    }
    return v;
}

// returns:
// +1 - convergence
// -1 - divergence
// 0 - non-determined
int step(std::vector<particle_t>& particles, std::vector<bar_t>& bars, float t, float dt)
{
    auto reference = particles[0].pos;
    for (auto& p : particles)
    {
        p.pos += p.vel * dt - reference;
    }

    for (auto& p : particles)
        if (p.degree == 0)
            p.vel += p.force / p.mass() * dt;

    std::vector<float> x;
    for (auto& p : particles)
    {
        if (p.degree == 0) continue;
        p.offset = x.size();
        x.emplace_back(p.vel.real());
        x.emplace_back(p.vel.imag());
    }
    auto bar_split = x.size();
    for (auto& b : bars)
    {
        b.offset = x.size();
        x.emplace_back(b.force);
    }

    lildiag_matrix_t A(x.size());
    std::vector<float> b(x.size(), 0.0f);
    for (auto& p : particles)
    {
        float co = dt / p.mass();
        // F = ma => v' - dt * f_bar / m = v + dt * f_ext / m
        A.diag[p.offset] = 1;
        A.diag[p.offset + 1] = 1;
        b[p.offset] = p.vel.real() + p.force.real() * co;
        b[p.offset + 1] = p.vel.imag() + p.force.imag() * co;
    }
    for (auto& b : bars)
    {
        // Direction
        vec2 dir = b.end->pos - b.start->pos;
        dir = dir / std::abs(dir);
        // Fill in previous F = ma slots
        A.lil[b.start->offset].emplace_back(b.offset, -dt / b.start->mass() * dir.real());
        A.lil[b.start->offset + 1].emplace_back(b.offset, -dt / b.start->mass() * dir.imag());
        A.lil[b.end->offset].emplace_back(b.offset, dt / b.end->mass() * dir.real());
        A.lil[b.end->offset + 1].emplace_back(b.offset, dt / b.end->mass() * dir.imag());
        // Relative velocity is perpendicular to bar, otherwise bar would be non-rigid
        // (vb - ve) dot dir = 0
        A.lil[b.offset].emplace_back(b.start->offset, dir.real());
        A.lil[b.offset].emplace_back(b.start->offset + 1, dir.imag());
        A.lil[b.offset].emplace_back(b.end->offset, -dir.real());
        A.lil[b.offset].emplace_back(b.end->offset + 1, -dir.imag());
    }

    std::vector<float> an2;
    std::vector<float> sn2;
    float s = 0;
    for (int i = 0; i < x.size(); i++)
    {
        float n2 = A.diag[i] * A.diag[i];
        for (const auto& [j, v] : A.lil[i])
            n2 += v * v;
        an2.push_back(n2);
        s += n2;
        sn2.push_back(s);
    }
    for (int k = 0; k < n_iters; k++)
    {
        // Kaczmarz
        for (unsigned ii = 0; ii < x.size(); ii++)
        {
            int i = ii;  // std::distance(sn2.begin(), std::upper_bound(sn2.begin(), sn2.end(), sample(sn2.back())));
            float p = b[i] - A.diag[i] * x[i];
            for (const auto& [j, v] : A.lil[i])
                p -= v * x[j];
            // std::cout << k << " " << i << " " << p << std::endl;
            p = p / an2[i];
            x[i] += p * A.diag[i];
            for (const auto& [j, v] : A.lil[i])
            {
                x[j] += p * v;
            }
        }
        // Gauss-Seidel
        /*for (unsigned i = 0; i < bar_split; i++)
        {
            float p = b[i];
            for (const auto& [j, v] : A.lil[i])
                p -= v * x[j];
            x[i] = p / A.diag[i];
        }*/
    }

    for (auto& p : particles)
    {
        p.vel = vec2(x[p.offset], x[p.offset + 1]);
    }
    for (auto& b : bars)
    {
        b.force = x[b.offset];
    }

    // if (t > 1 && std::abs(particles.back().vel) < cov_th) return 1;

    std::vector<float> proj;
    proj.reserve(particles.size());
    auto nor = particles.back().pos - particles.front().pos;
    nor = nor / std::abs(nor);
    auto st = (particles.front().pos * std::conj(nor)).real();
    auto ed = (particles.back().pos * std::conj(nor)).real();
    for (const auto& p : particles)
    {
        if (p.degree > 0 || &p == &particles.front() || &p == &particles.back())
        {
            auto dot = (p.pos * std::conj(nor)).real();
            if (dot >= st - 0.01f && dot <= ed + 0.01f)
                proj.emplace_back(dot);
        }
    }
    std::sort(proj.begin(), proj.end());
    auto lim = length_limit(particles, bars);
    for (unsigned i = 0; i + 1 < proj.size(); i++)
        if (proj[i + 1] - proj[i] > lim * 1.01f + 0.5f)
            return -1;
    // if (std::abs(particles.front().pos - particles.back().pos) > div_th) return -1;
    return 0;
}

int main()
{
    int n, m;
    std::cin >> n >> m;
    std::vector<std::string> maze(n, "");
    std::vector<particle_t> particles(n * m);
    std::vector<bar_t> bars;
    for (int i = 0; i < n; i++)
        std::cin >> maze[i];
    auto begin = std::clock();
    auto lidx = [=](int i, int j) { return i * m + j; };
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            auto& p = particles[lidx(i, j)];
            p.pos = vec2(j, i); p.vel = 0;
            if (maze[i][j] == '.' && j + 1 < m && maze[i][j + 1] == '.')
                bars.emplace_back(particles[lidx(i, j)], particles[lidx(i, j + 1)]);
            if (maze[i][j] == '.' && i + 1 < n && maze[i + 1][j] == '.')
                bars.emplace_back(particles[lidx(i, j)], particles[lidx(i + 1, j)]);
        }
    }

    for (const auto& bar : bars)
    {
        bar.start->degree++;
        bar.end->degree++;
    }

#if MAZE_VISUAL
    init_visuals(bars.size());
#endif

    float t = 0.0f;
    if constexpr (rand_force)
        for (auto& p : particles)
            p.force = vec2(sample(2) - 1, sample(2) - 1);
    particles.front().force = 10.0f / (n + m) * vec2(-m, -n);
    particles.back().force = 10.0f / (n + m) * vec2(m, n);
    while (clock() - begin < CLOCKS_PER_SEC * 19 / 20)
    {
        auto lim = length_limit(particles, bars);
#if MAZE_VISUAL
        std::cerr << "t = " << t << ", s = " << sanity(particles, bars) << ", l = " << lim << std::endl;
        std::vector<graph_seg_t> gst;
        for (const auto& bar : bars)
            gst.emplace_back(bar.start->pos.real(), bar.start->pos.imag(), bar.end->pos.real(), bar.end->pos.imag());
        update_visuals(gst);
#endif
        t += timestep;
        auto res = step(particles, bars, t, timestep);
        if (res < 0) { std::cout << "No" << std::endl; return 0; }
    }
    std::cout << "Yes" << std::endl;
#if MAZE_VISUAL
    return shutdown_visuals();
#else
    return 0;
#endif
}
