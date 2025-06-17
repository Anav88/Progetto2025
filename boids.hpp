#ifndef PF_BOIDS_HPP
#define PF_BOIDS_HPP

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

namespace bob {

int const PRED_DIST_SEP{
    80};  // distanza per cui predatori risentono una forza di allontanamento
int const BOID_DIST_FUGA = 30;  // distanza per cui i boid fuggono dai predatori
int const FACT_FUGA{50};

int const MIN_POS{0};
int const MAX_POS{600};

int const BOID_VEL_LIM{10};
int const VEL_PRED{30};
int const VEL_PRED_SEP{7};
std::size_t const MAX_PRED{5};
float const CATCH_RADIUS{0.5f};

constexpr float TIME_STEP{
    1.f / 60.f};  // delta t che intercorre refresh della finestra

struct Vec2f {
  float x;
  float y;

  Vec2f &operator+=(Vec2f const &);
  float angle() const;
  float norm() const;
};

struct Two_Vec {
  Vec2f a;
  Vec2f b;
};

struct Par {
  float s;
  float a;
  float c;
  float d;
  float d_s;
  std::size_t size;
};

Vec2f operator-(Vec2f const &, Vec2f const &);
Vec2f operator+(Vec2f const &, Vec2f const &);
Vec2f operator*(Vec2f const &, Vec2f const &);
Vec2f operator*(Vec2f const &, float);
Vec2f operator*(float, Vec2f const &);
Vec2f operator/(Vec2f const &, float);
bool operator==(Vec2f const &, Vec2f const &);

void limit_func(Vec2f &);

class Boid {
 private:
  Vec2f pos_;
  Vec2f vel_;
  Vec2f corr_vsep_{0.f, 0.f};
  Vec2f corr_vall_{0.f, 0.f};
  Vec2f corr_vcoes_{0.f, 0.f};
  Vec2f corr_vfuga_{0.f, 0.f};

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
  void vel_fuga(float);

  void correction();
  void reset_corr();
  void limit();
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

template <typename BP1, typename BP2>
float distance(BP1 const &bp1, BP2 const &bp2) {
  Vec2f delta_pos = bp1.get_pos() - bp2.get_pos();

  return delta_pos.norm();
}  // La definizione è stata scritta qui nell'hpp perché, essendo una funzione
   // template usata in boids.test.cpp,
// deve essere visibile al momento della compilazione.

Par init_parametres();
Par init_parametres(float, float, float, float, float, std::size_t);

namespace statistics {
Vec2f mean_velocity_algo(std::vector<Boid> const &);

Vec2f mean_deviation_algo(std::vector<Boid> const &);

void print_statistics(std::vector<Boid> const &);
}  // namespace statistics

Two_Vec rand_num();

void add_boid(std::vector<Boid> &add_vec);

void add_circle(std::vector<sf::CircleShape> &);

sf::CircleShape create_pred(float, float);

void evaluate_boid_correction(std::vector<Boid> &, std::vector<Predator> &,
                              Par const &);
void evaluate_boid_corr_fuga(Boid &, std::vector<Predator> &);

void evaluate_pred_correction(std::vector<Predator> &, std::vector<Boid> &);

void erase_boid(std::vector<Boid> &, std::vector<Predator> &,
                std::vector<sf::CircleShape> &);

void update_pred(std::vector<sf::CircleShape> &, std::vector<Predator> &,
                 sf::RenderWindow &);

void update_boid(std::vector<sf::CircleShape> &, std::vector<Boid> &,
                 sf::RenderWindow &);

template <typename BP>
void update_correction(std::vector<sf::CircleShape> &, std::vector<BP> &,
                       sf::RenderWindow &);

template <typename BP>
void init_circle(BP const &, sf::CircleShape &);
}  // namespace bob
#endif