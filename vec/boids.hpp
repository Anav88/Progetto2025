#ifndef PF_BOIDS_HPP
#define PF_BOIDS_HPP

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

// double const s = 0.4;
// double const a = 0.9;
// int const d = 50;
// double const c = 0.03;
// int const ds = 15;
// int const NUM_BOIDS{100};
int const MIN_POS{0};
int const MAX_POS{400};
int const MIN_VEL{-5};
int const MAX_VEL{5};
std::size_t const MAX_PRED{5};
float const PI{3.141593f};

struct Vec {
  float x;
  float y;
};

struct Two_Vec {
  Vec a;
  Vec b;
};

struct Par {
  float s;
  float a;
  float c;
  int d;
  int ds;
  std::size_t N;
};

Vec operator-(Vec f1, Vec f2);
Vec operator+(Vec f1, Vec f2);
Vec operator*(Vec f1, Vec f2);
Vec operator*(Vec f1, float d);
Vec operator/(Vec f1, float d);
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

  void vel_sep(Vec ds, float);
  void vel_all(Vec vds, float);
  void vel_coes(Vec cdm, float);

  void correction();
  void limit();
  void reset_corr();
  void vel_max();
};

class Predator {
 private:
  Vec pos_;
  Vec vel_;

 public:
  Predator(Vec p, Vec v);
  Predator();
  Vec get_vel();
  Vec get_pos();
};

bool operator==(Boid b1, Boid b2);

float abs(Vec f1, Vec f2);
float distance(Boid b1, Boid b2);

int init_size();
int init_size(int);

Par init_parametres();

Vec mean_velocity(std::vector<Boid> const arr);
Vec mean_velocity_algo(std::vector<Boid> const vec);

void mean_deviation(std::vector<Boid> const arr);
void mean_deviation_algo(std::vector<Boid> const vec);

Two_Vec rand_num();

void add_boid(std::vector<Boid> &add_vec);

void evaluate_correction(std::vector<Boid> &vec, Par);
void add_triangle(std::vector<sf::ConvexShape> &triangles);

void init_tr(Boid &b, sf::ConvexShape &triangles);

sf::CircleShape crt_pred(float, float);

#endif