#include "boids.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

namespace bob {
Vec2f operator-(Vec2f const &v1, Vec2f const &v2) {
  return {v1.x - v2.x, v1.y - v2.y};
}
Vec2f operator+(Vec2f const &v1, Vec2f const &v2) {
  return {v1.x + v2.x, v1.y + v2.y};
}
Vec2f &Vec2f::operator+=(Vec2f const &v1) {
  *this = *this + v1;
  return *this;  // Metodo scritto da chatgpt
}
Vec2f operator*(Vec2f const &v1, Vec2f const &v2) {
  return {v1.x * v2.x, v1.y * v2.y};
}
Vec2f operator*(Vec2f const &v1, float f) { return {v1.x * f, v1.y * f}; }
Vec2f operator*(float f, Vec2f const &v1) { return v1 * f; }
Vec2f operator/(Vec2f const &v1, float f) {
  if (f == 0.f) {
    throw std::invalid_argument("It is impossible to divide by 0");
  }
  return {v1.x / f, v1.y / f};
}
bool operator==(Vec2f const &v1, Vec2f const &v2) {
  return v1.x == v2.x && v1.y == v2.y;
}

float Vec2f::angle() const {
  if (x == 0.f && y == 0.f) {
    throw std::invalid_argument("The angle is not defined");
  }
  return atan2f(y, x);
}
float Vec2f::norm() const { return std::sqrt(y * y + x * x); }

void limit_func(Vec2f &pos) {
  if (pos.x < MIN_POS) {
    pos.x += MAX_POS;
  } else if (pos.x > MAX_POS) {
    pos.x -= MAX_POS;
  }

  if (pos.y < MIN_POS) {
    pos.y += MAX_POS;
  } else if (pos.y > MAX_POS) {
    pos.y -= MAX_POS;
  }
}

Boid::Boid(Vec2f p, Vec2f v) : pos_{p}, vel_{v} {}
Boid::Boid(Two_Vec vec) : pos_{vec.a}, vel_{vec.b} {}
Boid::Boid() : pos_{0, 0}, vel_{0, 0} {}

Vec2f Boid::get_pos() const { return pos_; }
Vec2f Boid::get_vel() const { return vel_; }
Vec2f Boid::get_corr_vsep() const { return corr_vsep_; }
Vec2f Boid::get_corr_vall() const { return corr_vall_; }
Vec2f Boid::get_corr_vcoes() const { return corr_vcoes_; }
Vec2f Boid::get_corr_vfuga() const { return corr_vfuga_; }
void Boid::vel_sep(Vec2f const &ds, float s) { corr_vsep_ = ds * (-s); }
void Boid::vel_all(Vec2f const &vds, float a) { corr_vall_ = (vds) * (a); }
void Boid::vel_coes(Vec2f const &cdm, float c) {
  corr_vcoes_ = (cdm - pos_) * c;
}
void Boid::correction() {
  vel_ += corr_vsep_ + corr_vall_ + corr_vcoes_ + corr_vfuga_;
  this->vel_max();
  pos_ += (vel_ * (TIME_STEP));
}
void Boid::limit() { limit_func(pos_); }
void Boid::reset_corr() {
  corr_vsep_ = {0.f, 0.f};
  corr_vall_ = {0.f, 0.f};
  corr_vcoes_ = {0.f, 0.f};
  corr_vfuga_ = {0.f, 0.f};
}
void Boid::vel_max() {
  float max_speed = sqrtf(BOID_VEL_LIM * BOID_VEL_LIM * 2);
  float norm = vel_.norm();
  if (norm > max_speed) {
    float angle = vel_.angle();
    vel_.x = max_speed * cosf(angle);
    vel_.y = max_speed * sinf(angle);
  }
}
void Boid::vel_fuga(float f) {
  corr_vfuga_.x += FACT_FUGA * cosf(f);
  corr_vfuga_.y += FACT_FUGA * sinf(f);
}

Predator::Predator(Vec2f p) : pos_{p} {}
Predator::Predator() : pos_{0.f, 0.f} {}

Vec2f Predator::get_pos() const { return pos_; }
Vec2f Predator::get_vel() const { return vel_; }
Vec2f Predator::get_vel_inseg() const { return corr_vinseg_; }
Vec2f Predator::get_vel_sep() const { return corr_vsep_; }

void Predator::vel_inseg(float f) {
  corr_vinseg_.x = VEL_PRED * cosf(f);
  corr_vinseg_.y = VEL_PRED * sinf(f);
}
void Predator::vel_sep(float f) {
  corr_vsep_.x += VEL_PRED_SEP * cosf(f);
  corr_vsep_.y += VEL_PRED_SEP * sinf(f);
}
void Predator::correction() {
  vel_ = corr_vinseg_ + corr_vsep_;
  pos_ += vel_ * TIME_STEP;
}
void Predator::reset_corr() {
  vel_ = {0.f, 0.f};
  corr_vinseg_ = {0.f, 0.f};
  corr_vsep_ = {0.f, 0.f};
}
void Predator::limit() { limit_func(pos_); }

template <typename BP1, typename BP2>
float distance_templ(BP1 const &bp1, BP2 const &bp2) {
  Vec2f delta_pos = bp1.get_pos() - bp2.get_pos();

  return delta_pos.norm();
}

float distance(Predator const &p1, Predator const &p2) {
  return distance_templ(p1, p2);
}
float distance(Boid const &b1, Boid const &b2) {
  return distance_templ(b1, b2);
}
float distance(Predator const &p, Boid const &b) {
  return distance_templ(p, b);
}
float distance(Boid const &b, Predator const &p) {
  return distance_templ(b, p);
}

Par init_parametres(float s, float a, float c, float d, float ds,
                    std::size_t size) {
  if (s < 0 || s > 1) {
    throw std::domain_error("Parameter s must be in the interval [0,1]");
  }
  if (a < 0 || a > 1) {
    throw std::domain_error("Parameter a must be in the interval [0,1]");
  }
  if (c < 0 || c > 1) {
    throw std::domain_error("Parameter c must be in the interval [0,1]");
  }
  if (d < 0 || ds < 0) {
    throw std::domain_error("Parameter d and ds must be positive");
  }
  if (d < ds) {
    throw std::domain_error("Parameter d must be bigger than ds");
  }

  return Par{s, a, c, d, ds, size};
}
Par init_parametres() {
  Par input;
  int N;
  std::cout << "Enter the values of the parameters s, a, c\n";
  std::cin >> input.s >> input.a >> input.c;
  if (std::cin.fail()) {
    throw std::domain_error("Parameter not valid");
  }
  if (input.s < 0 || input.s > 1) {
    throw std::domain_error("Parameter s must be in the interval [0,1]");
  }
  if (input.a < 0 || input.a > 1) {
    throw std::domain_error("Parameter a must be in the interval [0,1]");
  }
  if (input.c < 0 || input.c > 1) {
    throw std::domain_error("Parameter c must be in the interval [0,1]");
  }

  std::cout << "Enter the values of the parameters d, ds\n";
  std::cin >> input.d >> input.d_s;
  if (std::cin.fail()) {
    throw std::domain_error("Parameter not valid");
  }
  if (input.d < 0 || input.d_s < 0) {
    throw std::domain_error("Parameter d and ds must be positive");
  }
  if (input.d < input.d_s) {
    throw std::domain_error("Parameter d must be bigger than ds");
  }

  std::cout << "Enter the number of boids\n";
  std::cin >> N;
  if (std::cin.fail()) {
    throw std::domain_error("Parameter not valid");
  } else if (N < 0) {
    throw std::domain_error("Number of Boid can't be negative");
  }
  input.size = static_cast<size_t>(N);

  return input;
}

namespace statistics {

Vec2f mean_velocity_algo(std::vector<Boid> const &boids) {
  if (boids.size() <= 0) {
    throw std::domain_error(
        "You cannot calculate the average speed with any boid");
  }
  Vec2f sums = std::accumulate(boids.begin(), boids.end(), Vec2f{0.f, 0.f},
                               [](Vec2f sum, const Boid &b) {
                                 sum += b.get_vel();

                                 return sum;
                               });
  return sums / static_cast<float>(boids.size());
}

Vec2f mean_deviation_algo(std::vector<Boid> const &boids) {
  if (boids.size() <= 1) {
    throw std::domain_error(
        "Standard deviation cannot be calculated with less than two boids");
  }
  Vec2f mean = mean_velocity_algo(boids);

  Vec2f sum_mean_vel_diff_square =
      std::accumulate(boids.begin(), boids.end(), Vec2f{0.f, 0.f},
                      [&mean](Vec2f sum, const Boid &b) {
                        sum += (b.get_vel() - mean) * (b.get_vel() - mean);
                        return sum;
                      });

  Vec2f var =
      sum_mean_vel_diff_square / (static_cast<float>(boids.size()) - 1.f);
  return {sqrtf(var.x), sqrtf(var.y)};
}

void print_statistics(std::vector<Boid> const &boids) {
  Vec2f mean = mean_velocity_algo(boids);
  Vec2f deviation = mean_deviation_algo(boids);

  std::cout << "\nMean velocity & standard deviation in x: " << mean.x << ' '
            << deviation.x;
  std::cout << "\nMean velocity & standard deviation in y: " << mean.y << ' '
            << deviation.y << '\n';
}
}  // namespace statistics

Two_Vec rand_num() {
  std::random_device r;
  std::default_random_engine eng{r()};

  std::uniform_real_distribution<float> pos_x{MIN_POS, MAX_POS};
  std::uniform_real_distribution<float> pos_y{MIN_POS, MAX_POS};
  std::uniform_real_distribution<float> vel_x{-BOID_VEL_LIM, BOID_VEL_LIM};
  std::uniform_real_distribution<float> vel_y{-BOID_VEL_LIM, BOID_VEL_LIM};

  return {{pos_x(eng), pos_y(eng)}, {vel_x(eng), vel_y(eng)}};
}

void add_boid(std::vector<Boid> &add_vec) {
  std::generate(add_vec.begin(), add_vec.end(),
                []() { return (Boid(rand_num())); });
}

void add_circle(std::vector<sf::CircleShape> &circles) {
  std::generate(circles.begin(), circles.end(), []() {
    float radius{1.f};
    sf::CircleShape c{radius};
    c.setOrigin(radius, radius);
    c.setFillColor(sf::Color::Black);
    return c;
  });
}

sf::CircleShape create_pred(float x, float y) {
  sf::CircleShape circle{3.0f};
  circle.setOrigin(3.f, 3.f);
  circle.setPosition({x, y});
  circle.setFillColor(sf::Color::Red);

  return circle;
}

void evaluate_boid_correction(std::vector<Boid> &boids,
                              std::vector<Predator> &predators,
                              Par const &parametres) {
  for (Boid &boid_i : boids) {
    if (boid_i.get_pos().x < MIN_POS || boid_i.get_pos().y < MIN_POS ||
        boid_i.get_pos().x > MAX_POS || boid_i.get_pos().y > MAX_POS) {
      throw std::domain_error("The boid is out of bounds");
    }
    if (boid_i.get_vel().norm() >
        sqrtf(BOID_VEL_LIM * BOID_VEL_LIM * 2) + 0.1f) {
      throw std::domain_error(
          "The boid has no speed within the allowed limits");
    }

    evaluate_boid_corr_fuga(boid_i, predators);

    Vec2f i_pos = boid_i.get_pos();
    Vec2f i_vel = boid_i.get_vel();

    struct Stats {
      int n;
      Vec2f all;
      Vec2f coes;
      int nSep;
      Vec2f sep;
    };

    auto result = std::accumulate(
        boids.begin(), boids.end(),
        Stats{0, {0.f, 0.f}, {0.f, 0.f}, 0, {0.f, 0.f}},
        [&](Stats acc, Boid const &boid_j) {
          if (&boid_i != &boid_j) {
            float dist = distance(boid_i, boid_j);

            if (dist < parametres.d) {
              Vec2f j_pos = boid_j.get_pos();
              Vec2f j_vel = boid_j.get_vel();
              ++acc.n;
              acc.all += (j_vel - i_vel);  // calcola la sommatoria(vj-vi) per
                                           // il calcolo di corr_vall_
              acc.coes += j_pos;  // calcola la sommatoria(xj) per il calcolo di
                                  // corr_vcoes_

              if (dist < parametres.d_s) {
                acc.sep += (j_pos - i_pos);  // calcola la sommatoria(xj-xi) per
                                             // il calcolo di corr_vsep_
                ++acc.nSep;
              }
            }
          }
          return acc;
        });

    if (result.nSep > 0) {
      boid_i.vel_sep(result.sep, parametres.s);
    }

    if (result.n > 0) {
      boid_i.vel_all(result.all / static_cast<float>(result.n), parametres.a);
      boid_i.vel_coes(result.coes / static_cast<float>(result.n), parametres.c);
    }
  }
}

void evaluate_boid_corr_fuga(Boid &boid, std::vector<Predator> &predators) {
  for (auto &pred : predators) {
    if (distance(boid, pred) < BOID_DIST_FUGA) {
      Vec2f delta_pos = boid.get_pos() - pred.get_pos();
      boid.vel_fuga(delta_pos.angle());
    }
  }
}

void evaluate_pred_correction(std::vector<Predator> &predators,
                              std::vector<Boid> &boids) {
  for (auto &pred_i : predators) {
    if (pred_i.get_pos().x < MIN_POS || pred_i.get_pos().y < MIN_POS ||
        pred_i.get_pos().x > MAX_POS || pred_i.get_pos().y > MAX_POS) {
      throw std::domain_error("The predator is out of bounds");
    }

    if (!boids.empty()) {
      auto it = std::min_element(
          boids.begin(), boids.end(), [&](Boid const &b1, Boid const &b2) {
            return (distance(pred_i, b1) < distance(pred_i, b2));
          });                   // individua il boid più vicino
      if (it != boids.end()) {  // condizione superflua ma utile per una
                                // maggiore chiarezza e controllo
        Vec2f delta_pos = (*it).get_pos() - pred_i.get_pos();
        pred_i.vel_inseg(delta_pos.angle());
      }
    }

    for (auto &pred_j : predators) {
      if (distance(pred_i, pred_j) < PRED_DIST_SEP && (&pred_i != &pred_j)) {
        Vec2f delta_pos = pred_i.get_pos() - pred_j.get_pos();
        pred_i.vel_sep(delta_pos.angle());
      }
    }
  }
}

void erase_boid(std::vector<Boid> &boids, std::vector<Predator> &predators,
                std::vector<sf::CircleShape> &circles) {
  for (auto it_p = predators.begin(); it_p != predators.end(); ++it_p) {
    for (size_t i = 0; i < boids.size(); ++i) {
      if (distance((*it_p), boids[i]) <
          CATCH_RADIUS) {  // se i-esimo boid è abbastanza vicino al predatore
        boids.erase(boids.begin() +
                    static_cast<std::vector<Boid>::iterator::difference_type>(
                        i));  // scritto da chat gpt per eliminare il warning
        circles.erase(
            circles.begin() +
            static_cast<
                std::vector<sf::CircleShape>::iterator::difference_type>(i));
        --it_p;  // eliminato il boid e il cerchio invalido gli iteratori,
                 // quindi esco dal ciclo e rincomincio
        break;
      }
    }
  }
}

void update_pred(std::vector<sf::CircleShape> &circles,
                 std::vector<Predator> &predators, sf::RenderWindow &window) {
  update_correction(circles, predators, window);
}

void update_boid(std::vector<sf::CircleShape> &circles,
                 std::vector<Boid> &boids, sf::RenderWindow &window) {
  update_correction(circles, boids, window);
}

template <typename BP>
void update_correction(std::vector<sf::CircleShape> &circles,
                       std::vector<BP> &boid_pred, sf::RenderWindow &window) {
  auto it_c = circles.begin();
  for (auto it = boid_pred.begin(); it != boid_pred.end(); ++it, ++it_c) {
    (*it).correction();
    (*it).limit();
    init_circle(*it, *it_c);

    (*it).reset_corr();
    window.draw(*it_c);
  }
}

template <typename BP>
void init_circle(BP const &bp, sf::CircleShape &c) {
  sf::Vector2f pos{bp.get_pos().x, bp.get_pos().y};
  c.setPosition(pos);
}
}  // namespace bob