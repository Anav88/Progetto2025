#ifndef PF_REGRESSION_HPP
#define PF_REGRESSION_HPP

// #include <SFML/Graphics.hpp>
// #include <SFML/Window.hpp>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>
// #include <execution>
#include <iostream>
#include <random>

double const s = 0.4;
double const a = 0.9;
int const d = 50;
double const c = 0.03;
int const ds = 15;
// int const NUM_BOIDS{100};
int const MIN_POS{0};
int const MAX_POS{400};
int const MIN_VEL{-5};
int const MAX_VEL{5};
double const PI{3.141593};

struct Vec {
  double x;
  double y;
};

struct Two_Vec {
  Vec a;
  Vec b;
};

Vec operator-(Vec f1, Vec f2);
Vec operator+(Vec f1, Vec f2);
Vec operator*(Vec f1, Vec f2);
Vec operator*(Vec f1, double d);
Vec operator/(Vec f1, double d);
bool operator==(Vec v1, Vec v2);

class Boid {
 private:
  Vec pos_;
  Vec vel_;
  Vec corr_v1_{0., 0.};
  Vec corr_v2_{0., 0.};
  Vec corr_v3_{0., 0.};

 public:
  Boid(Vec p, Vec v);
  Boid(Two_Vec vec);
  Boid();
  Vec get_pos() const;
  Vec get_vel() const;
  Vec get_corr_v1() const;
  Vec get_corr_v2() const;
  Vec get_corr_v3() const;

  void vel_sep(Vec ds);
  void vel_all(Vec vds);
  void vel_coes(Vec cdm);

  void correction();
  void limit();
  void reset_corr();
  void vel_max();
};


bool operator==(Boid b1, Boid b2);

double abs(Vec f1, Vec f2);
double distance(Boid b1, Boid b2);

int init_size();
int init_size(int);

Vec mean_velocity(std::vector<Boid> const arr);
Vec mean_velocity_algo(std::vector<Boid> const vec);

void mean_deviation(std::vector<Boid> const arr);
void mean_deviation_algo(std::vector<Boid> const vec);

Two_Vec rand_num();

void add_boid(std::vector<Boid> &add_arr);

void evaluate_correction(std::vector<Boid> &arr);
// void add_triangle(std::vector<sf::ConvexShape> &triangles);

// void init_tr(Boid &b, sf::ConvexShape &triangles);

void update_correction(std::vector<Boid> &arr);

// void print(std::vector<sf::ConvexShape> &tr, std::vector<Boid> &arr,
//            sf::RenderWindow &window);

#endif