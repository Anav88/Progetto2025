#ifndef PF_BOIDS_HPP
#define PF_BOIDS_HPP

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

namespace bob {

int const pd = 80;
int const df = 40;
int const MIN_POS{0};
int const MAX_POS{600};
int const MIN_VEL{-10};
int const MAX_VEL{10};
int const VEL_PRED{30};
int const VEL_PRED_SEP{7};
std::size_t const MAX_PRED{5};
constexpr float TIME_STEP{1.f / 60.f};

struct Vec {
  float x;
  float y;

  Vec &operator+=(Vec const &);
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

Vec operator-(Vec const &, Vec const &);
Vec operator+(Vec const &, Vec const &);
Vec operator*(Vec const &, Vec const &);
Vec operator*(Vec const &, float);
Vec operator*(float, Vec const &);
Vec operator/(Vec const &, float);
bool operator==(Vec const &, Vec const &);

class Boid {
 private:
  Vec pos_;
  Vec vel_;
  Vec corr_v1_{0., 0.};
  Vec corr_v2_{0., 0.};
  Vec corr_v3_{0., 0.};
  Vec corr_v_fuga{0., 0.};

 public:
  Boid(Vec p, Vec v);
  Boid(Two_Vec vec);
  Boid();
  Vec get_pos() const;
  Vec get_vel() const;
  Vec get_corr_v1() const;
  Vec get_corr_v2() const;
  Vec get_corr_v3() const;

  void vel_sep(Vec const &, float);
  void vel_all(Vec const &, float);
  void vel_coes(Vec const &, float);
  void corr_vel_fuga(float, float);

  void correction();
  void limit();
  void reset_corr();
  void vel_max();
};

class Predator {
 private:
  Vec pos_;
  Vec vel_{0.f, 0.f};
  Vec corr_v1_{0.f, 0.f};
  Vec corr_v2_{0.f, 0.f};

 public:
  Predator(Vec p);
  Predator();
  Vec get_vel() const;
  Vec get_pos() const;
  void corr_vel_pred_1(float);
  void corr_vel_pred_2(float);
  void correction();
  void zerovel();
  void limit();
};

bool operator==(Boid, Boid);
bool operator==(Predator, Predator);

float abs(Vec const &, Vec const &);
template <typename BP1, typename BP2>
float distance(BP1 const &, BP2 const &);

Par init_parametres();
Par init_parametres(float, float, float, int, int, std::size_t);

namespace statistics {
Vec mean_velocity(std::vector<Boid> const &);
Vec mean_velocity_algo(std::vector<Boid> const &);

Vec mean_deviation(std::vector<Boid> const &);
Vec mean_deviation_algo(std::vector<Boid> const &);

void print_statistics(std::vector<Boid> const &);
}  // namespace statistics

Two_Vec rand_num();

void add_boid(std::vector<Boid> &add_vec);

void evaluate_correction(std::vector<Boid> &, Par const &);
void evaluate_corr_fuga(std::vector<Boid> &, std::vector<Predator> &);

void add_circle(std::vector<sf::CircleShape> &);

void init_cr(Boid const &, sf::CircleShape &);

sf::CircleShape crt_pred(float, float);

void erase_boid(std::vector<Boid> &, std::vector<Predator> &,
                std::vector<sf::CircleShape> &);

void evaluate_pred_correction(std::vector<Predator> &, std::vector<Boid> &);

void update_correction(std::vector<sf::CircleShape> &,
                       std::vector<sf::CircleShape> &, std::vector<Predator> &,
                       std::vector<Boid> &, sf::RenderWindow &);
}  // namespace bob
#endif