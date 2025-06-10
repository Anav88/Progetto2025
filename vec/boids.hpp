#ifndef PF_BOIDS_HPP
#define PF_BOIDS_HPP

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

namespace bob {

int const pd = 80;
int const df = 40;
int const FAT_FUGA{100};
int const MIN_POS{0};
int const MAX_POS{600};
int const MIN_VEL{-10};
int const MAX_VEL{10};
int const VEL_PRED{30};
int const VEL_PRED_SEP{7};
std::size_t const MAX_PRED{5};
constexpr float TIME_STEP{1.f / 60.f};

struct Vec2f {
  float x;
  float y;

  Vec2f &operator+=(Vec2f const &);
  float angle();
  float norm();
};

struct Two_Vec {
  Vec2f a;
  Vec2f b;
};

struct Par {
  float s;
  float a;
  float c;
  int d;
  int ds;
  std::size_t N;
};

Vec2f operator-(Vec2f const &, Vec2f const &);
Vec2f operator+(Vec2f const &, Vec2f const &);
Vec2f operator*(Vec2f const &, Vec2f const &);
Vec2f operator*(Vec2f const &, float);
Vec2f operator*(float, Vec2f const &);
Vec2f operator/(Vec2f const &, float);
bool operator==(Vec2f const &, Vec2f const &);

class Boid {
 private:
  Vec2f pos_;
  Vec2f vel_;
  Vec2f corr_vsep_{0., 0.};
  Vec2f corr_vall_{0., 0.};
  Vec2f corr_vcoes_{0., 0.};
  Vec2f corr_vfuga_{0., 0.};

 public:
  Boid(Vec2f p, Vec2f v);
  Boid(Two_Vec vec);
  Boid();
  Vec2f get_pos() const;
  Vec2f get_vel() const;
  Vec2f get_corr_vsep() const;
  Vec2f get_corr_vall() const;
  Vec2f get_corr_vcoes() const;
  Vec2f get_corr_vfuga() const;

  void vel_sep(Vec2f const &, float);
  void vel_all(Vec2f const &, float);
  void vel_coes(Vec2f const &, float);
  void corr_vel_fuga(float);

  void correction();
  void limit();
  void reset_corr();
  void vel_max();
};

class Predator {
 private:
  Vec2f pos_;
  Vec2f vel_{0.f, 0.f};
  Vec2f corr_vinseg_{0.f, 0.f};
  Vec2f corr_vsep_{0.f, 0.f};

 public:
  Predator(Vec2f p);
  Predator();
  Vec2f get_vel() const;
  Vec2f get_pos() const;
  Vec2f get_vel_inseg() const;
  Vec2f get_vel_sep() const;
  void vel_inseg(float);
  void vel_sep(float);
  void correction();
  void reset_corr();
  void limit();
};

bool operator==(Boid, Boid);
bool operator==(Predator, Predator);

float abs(Vec2f const &, Vec2f const &);
template <typename BP1, typename BP2>
float distance(BP1 const &, BP2 const &);

Par init_parametres();
Par init_parametres(float, float, float, int, int, std::size_t);

namespace statistics {
Vec2f mean_velocity(std::vector<Boid> const &);
Vec2f mean_velocity_algo(std::vector<Boid> const &);

Vec2f mean_deviation(std::vector<Boid> const &);
Vec2f mean_deviation_algo(std::vector<Boid> const &);

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