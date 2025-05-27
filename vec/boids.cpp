#include "boids.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <execution>
#include <iostream>
#include <random>
#include <vector>

namespace bob {
Vec operator-(Vec f1, Vec f2) { return {f1.x - f2.x, f1.y - f2.y}; }
Vec operator+(Vec f1, Vec f2) { return {f1.x + f2.x, f1.y + f2.y}; }
Vec operator*(Vec f1, Vec f2) { return {f1.x * f2.x, f1.y * f2.y}; }
Vec operator*(Vec f1, float d) { return {f1.x * d, f1.y * d}; }
Vec operator/(Vec f1, float d) { return {f1.x / d, f1.y / d}; }
bool operator==(Vec v1, Vec v2) {
  if (v1.x == v2.x && v1.y == v2.y) {
    return true;
  } else {
    return false;
  }
}

Boid::Boid(Vec p, Vec v) : pos_{p}, vel_{v} {}
Boid::Boid(Two_Vec vec) : pos_{vec.a}, vel_{vec.b} {}
Boid::Boid() : pos_{0, 0}, vel_{0, 0} {}

Vec Boid::get_pos() const { return pos_; }
Vec Boid::get_vel() const { return vel_; }
Vec Boid::get_corr_v1() const { return corr_v1_; }
Vec Boid::get_corr_v2() const { return corr_v2_; }
Vec Boid::get_corr_v3() const { return corr_v3_; }

void Boid::vel_sep(Vec ds, float s) { corr_v1_ = ds * (-s); }
void Boid::vel_all(Vec vds, float a) { corr_v2_ = (vds) * (a); }
void Boid::vel_coes(Vec cdm, float c) { corr_v3_ = (cdm - pos_) * c; }

void Boid::correction() {
  vel_ = vel_ + corr_v1_ + corr_v2_ + corr_v3_ + corr_v_fuga;
  this->vel_max();
  pos_ = pos_ + (vel_ * (0.017f));
}
void Boid::limit() {
  if (pos_.x < MIN_POS + 100.f) {
    pos_.x += MAX_POS;
  }
  if (pos_.y < MIN_POS + 100.f) {
    pos_.y += MAX_POS;
  }
  if (pos_.x > MAX_POS + 100.f) {
    pos_.x -= MAX_POS;
  }
  if (pos_.y > MAX_POS + 100.f) {
    pos_.y -= MAX_POS;
  }
}
void Boid::reset_corr() {
  corr_v1_ = {0.f, 0.f};
  corr_v2_ = {0.f, 0.f};
  corr_v3_ = {0.f, 0.f};
  corr_v_fuga = {0.f, 0.f};
}

void Boid::vel_max() {
  if (vel_.x > MAX_VEL) {
    vel_.y *= (MAX_VEL / vel_.x);
    vel_.x = MAX_VEL;
  }
  if (vel_.y > MAX_VEL) {
    vel_.x *= (MAX_VEL / vel_.y);
    vel_.y = MAX_VEL;
  }
  if (vel_.x < MIN_VEL) {
    vel_.y *= (MIN_VEL / vel_.x);
    vel_.x = MIN_VEL;
  }
  if (vel_.y < MIN_VEL) {
    vel_.x *= (MIN_VEL / vel_.y);
    vel_.y = MIN_VEL;
  }
}
void Boid::corr_vel_fuga(float f, float dis) {
  corr_v_fuga.x = corr_v_fuga.x + dis * cosf(f);
  corr_v_fuga.y = corr_v_fuga.y + dis * sinf(f);
}

bool operator==(Boid b1, Boid b2) {
  if (b1.get_pos() == b2.get_pos() && b1.get_vel() == b2.get_vel()) {
    return true;
  } else {
    return false;
  }
}

Predator::Predator(Vec p) : pos_{p} {}
Predator::Predator() : pos_{0.f, 0.f} {}
Vec Predator::get_pos() { return pos_; }
Vec Predator::get_vel() { return vel_; }
void Predator::corr_vel_pred_1(float f) {
  corr_v1_.x = VEL_PRED * std::cosf(f);
  corr_v1_.y = VEL_PRED * std::sinf(f);
}
void Predator::corr_vel_pred_2(float f) {
  corr_v2_.x = corr_v2_.x + VEL_PRED_SEP * std::cosf(f);
  corr_v2_.y = corr_v2_.y + VEL_PRED_SEP * std::sinf(f);
}
void Predator::correction() {
  vel_ = corr_v1_ + corr_v2_;
  pos_ = pos_ + vel_ * 0.017f;
}
void Predator::zerovel() {
  vel_ = {0.f, 0.f};
  corr_v1_ = {0.f, 0.f};
  corr_v2_ = {0.f, 0.f};
}
void Predator::limit() {
  if (pos_.x < MIN_POS + 100.f) {
    pos_.x += MAX_POS;
  } else if (pos_.x > MAX_POS + 100.f) {
    pos_.x -= MAX_POS;
  }

  if (pos_.y < MIN_POS + 100.f) {
    pos_.y += MAX_POS;
  } else if (pos_.y > MAX_POS + 100.f) {
    pos_.y -= MAX_POS;
  }
}
bool operator==(Predator p1, Predator p2) {
  if (p1.get_pos() == p2.get_pos() && p1.get_vel() == p2.get_vel()) {
    return true;
  } else {
    return false;
  }
}

int init_size() {
  int n;
  std::cout << "Inserire il numero di uccelli \n";
  std::cin >> n;
  if (n < 0) {
    throw std::runtime_error{"Impossibile avere un numero negativo di boid"};
  }
  if (std::cin.fail()) {
    throw std::runtime_error{"Input non valido"};
  }
  return n;
}
int init_size(int n) {
  if (n < 0) {
    throw std::runtime_error{"Impossibile avere un numero negativo di boid"};
  }
  return n;
}

Par init_parametres() {
  Par in;
  int n;

  std::cout << "Inserisci i valori dei parametri s, a, c\n";
  std::cin >> in.s >> in.a >> in.c;
  if (std::cin.fail()) {
    throw std::invalid_argument("Parameter not valid");
  }
  if (in.s < 0 || in.s > 1) {
    throw std::invalid_argument("Parameter s must be in the interval [0,1]");
  }
  assert(in.s >= 0 && in.s <= 1);
  if (in.a < 0 || in.a > 1) {
    throw std::invalid_argument("Parameter a must be in the interval [0,1]");
  }
  assert(in.a >= 0 && in.a <= 1);
  if (in.c < 0 || in.c > 1) {
    throw std::invalid_argument("Parameter c must be in the interval [0,1]");
  }
  assert(in.c >= 0 && in.c <= 1);

  std::cout << "Inserisci i valori dei parametri d, ds\n";
  std::cin >> in.d >> in.ds;
  if (std::cin.fail()) {
    throw std::invalid_argument("Parameter not valid");
  }
  if (in.d < 0 || in.ds < 0) {
    throw std::invalid_argument("Parameter d and ds must be positive");
  }
  if (in.d < in.ds) {
    throw std::invalid_argument("Parameter d must be bigger than ds");
  }
  assert(in.ds <= in.d && in.ds >= 0);

  std::cout << "Inserisci il numero dei boid\n";
  std::cin >> n;
  if (std::cin.fail()) {
    throw std::invalid_argument("Parameter not valid");
  }
  if (n < 0) {
    throw std::invalid_argument("Parameter N can't be negative");
  }

  in.N = static_cast<std::size_t>(n);  // chat gpt per evitare warning

  return in;
}

float abs(Vec f1, Vec f2) {
  return sqrt(
      pow(f1.x - f2.x, 2.0f) +
      pow(f1.y - f2.y, 2.0f));  // calcola il modulo della diff di sue vettori
}

template <typename BP1, typename BP2>
float distance(BP1 bp1, BP2 bp2) {
  return abs(bp1.get_pos(), bp2.get_pos());
}

Vec mean_velocity(std::vector<Boid> const vec) {
  Vec sum_vel{0., 0.};
  for (auto it = vec.begin(); it != vec.end(); ++it) {
    sum_vel = sum_vel + (*it).get_vel();
  }
  std::cout << "Mean: " << sum_vel.x / vec.size() << ' '
            << sum_vel.y / vec.size() << '\n';
  return sum_vel / vec.size();
}
Vec mean_velocity_algo(std::vector<Boid> const vec) {
  Vec sums = std::accumulate(vec.begin(), vec.end(), Vec{0.f, 0.f},
                             [](Vec sum, const Boid &b) {
                               sum = sum + b.get_vel();

                               return sum;
                             });
  std::cout << "Mean: " << sums.x / vec.size() << ' ' << sums.y / vec.size()
            << '\n';
  return sums / vec.size();
}

void mean_deviation(std::vector<Boid> const vec) {
  Vec sum_mean_vel_diff_square{0., 0.};
  Vec mean = mean_velocity(vec);
  for (auto it = vec.begin(); it != vec.end(); ++it) {
    sum_mean_vel_diff_square =
        sum_mean_vel_diff_square +
        ((*it).get_vel() - mean) * ((*it).get_vel() - mean);
  }
  Vec var = sum_mean_vel_diff_square / (vec.size());
  std::cout << "Standard deviation: " << std::sqrt(var.x) << ' '
            << std::sqrt(var.y) << '\n';
}
void mean_deviation_algo(std::vector<Boid> const vec) {
  Vec mean = mean_velocity(vec);

  Vec sum_mean_vel_diff_square = std::accumulate(
      vec.begin(), vec.end(), Vec{0.f, 0.f}, [&mean](Vec sum, const Boid &b) {
        sum = sum + (b.get_vel() - mean) * (b.get_vel() - mean);
        return sum;
      });

  Vec var = sum_mean_vel_diff_square / (vec.size());
  std::cout << "Standard deviation: " << std::sqrt(var.x) << ' '
            << std::sqrt(var.y) << '\n';
}

void add_boid(std::vector<Boid> &add_vec) {
  std::generate(add_vec.begin(), add_vec.end(),
                []() { return (Boid(rand_num())); });
}

Two_Vec rand_num() {
  std::random_device r;
  std::default_random_engine eng{r()};
  // static std::random_device r;
  // static std::default_random_engine eng{r()};
  // Gli static ce li ha suggeriti chatgpt per migliorare prestazioni
  // e poichè potrebbe dare qualche problema

  std::uniform_real_distribution<float> pos_x{MIN_POS + 100.f, MAX_POS + 100.f};
  std::uniform_real_distribution<float> pos_y{MIN_POS + 100.f, MAX_POS + 100.f};
  std::uniform_real_distribution<float> vel_x{MIN_VEL, MAX_VEL};
  std::uniform_real_distribution<float> vel_y{MIN_VEL, MAX_VEL};

  return {{pos_x(eng), pos_y(eng)}, {vel_x(eng), vel_y(eng)}};
}

void evaluate_correction(std::vector<Boid> &vec, Par const &parametres) {
  for (Boid &boid_i : vec) {
    Vec sum_diff_pos = {0., 0.};  // per il calcolo vel sep
    Vec sum_diff_vel = {0., 0.};  // per il calcolo vel all
    Vec sum_coord = {0., 0.};     // per il calcolo del cdm/vel coes
    int neighbor_count = 0;
    int separation_count = 0;
    for (Boid const &boid_j : vec) {
      if (!(boid_i == boid_j)) {
        float dist = distance(boid_i, boid_j);

        Vec j_vel = boid_j.get_vel();
        Vec j_pos = boid_j.get_pos();
        Vec i_vel = boid_i.get_vel();
        Vec i_pos = boid_i.get_pos();

        if (dist < parametres.d) {
          ++neighbor_count;
          sum_diff_vel = sum_diff_vel + (j_vel - i_vel);
          sum_coord = sum_coord + j_pos;

          if (dist < parametres.ds) {
            sum_diff_pos = sum_diff_pos + (j_pos - i_pos);
            ++separation_count;
          }
        }
      }
    }

    if (separation_count > 0) {
      boid_i.vel_sep(sum_diff_pos, parametres.s);
    }

    if (neighbor_count > 1) {
      Vec mean_vel = sum_diff_vel / (neighbor_count - 1);
      Vec cdm = sum_coord / (neighbor_count - 1);
      boid_i.vel_all(mean_vel, parametres.a);
      boid_i.vel_coes(cdm, parametres.c);
    }
  }
}

void evaluate_corr_fuga(std::vector<Boid> &boids,
                        std::vector<Predator> &predators) {
  for (auto it_b = boids.begin(); it_b != boids.end(); ++it_b) {
    for (auto it_p = predators.begin(); it_p != predators.end(); ++it_p) {
      float dis = distance((*it_b), (*it_p));
      if (dis < df) {
        Vec delta_pos = (*it_b).get_pos() - (*it_p).get_pos();
        (*it_b).corr_vel_fuga(std::atan2f(delta_pos.y, delta_pos.x), 100.f);
      }
    }
  }
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

void init_cr(Boid const &b, sf::CircleShape &c) {
  sf::Vector2f pos(b.get_pos().x, b.get_pos().y);
  c.setPosition(pos);
}

sf::CircleShape crt_pred(float x, float y) {
  sf::CircleShape circle{3.0f};
  circle.setOrigin(3.f, 3.f);
  circle.setPosition({x, y});
  circle.setFillColor(sf::Color::Red);

  return circle;
}

void erase_boid(std::vector<Boid> &boids, std::vector<Predator> &predators,
                std::vector<sf::CircleShape> &circles) {
  for (auto it_p = predators.begin(); it_p != predators.end(); ++it_p) {
    auto it_t = circles.begin();
    for (auto it_b = boids.begin(); it_b != boids.end(); ++it_b, ++it_t) {
      if ((*it_p).get_pos().x - (*it_b).get_pos().x < 5e-1f &&
          (*it_p).get_pos().x - (*it_b).get_pos().x > -5e-1f &&
          (*it_p).get_pos().y - (*it_b).get_pos().y < 5e-1f &&
          (*it_p).get_pos().y - (*it_b).get_pos().y > -5e-1f) {
        boids.erase(it_b);
        circles.erase(it_t);
        --it_p;
        break;
      }
    }
  }
}

void evaluate_pred_correction(std::vector<Predator> &predators,
                              std::vector<Boid> &boids) {
  for (auto it_p = predators.begin(); it_p != predators.end(); ++it_p) {
    float min_ds{0};
    Vec delta_pos{0.f, 0.f};
    for (auto it_b = boids.begin(); it_b != boids.end(); ++it_b) {
      if (it_b == boids.begin()) {
        min_ds = distance(*it_b, *it_p);
        delta_pos = (*it_b).get_pos() - (*it_p).get_pos();
      } else {
        float dist = distance(*it_b, *it_p);
        if (dist < min_ds) {
          min_ds = dist;
          delta_pos = (*it_b).get_pos() - (*it_p).get_pos();
        }
      }
    }
    if (!boids.empty()) {
      (*it_p).corr_vel_pred_1(std::atan2f(delta_pos.y, delta_pos.x));
    }
  }
  for (auto it_i = predators.begin(); it_i != predators.end(); ++it_i) {
    for (auto it_j = predators.begin(); it_j != predators.end(); ++it_j) {
      if (distance((*it_i), (*it_j)) < pd && !((*it_i) == (*it_j))) {
        Vec delta_pos = (*it_i).get_pos() - (*it_j).get_pos();
        (*it_i).corr_vel_pred_2(std::atan2f(delta_pos.y, delta_pos.x));
      }
    }
  }
}

void update_correction(std::vector<sf::CircleShape> &circles_pred,
                       std::vector<sf::CircleShape> &circles_boid,
                       std::vector<Predator> &predators,
                       std::vector<Boid> &boids, sf::RenderWindow &window) {
  {
    auto it_c = circles_pred.begin();
    for (auto it_p = predators.begin(); it_p != predators.end();
         ++it_p, ++it_c) {
      (*it_p).correction();
      (*it_p).zerovel();
      (*it_p).limit();
      (*it_c).setPosition({(*it_p).get_pos().x, (*it_p).get_pos().y});

      window.draw(*it_c);
    }
  }

  {
    auto it_b = boids.begin();
    for (auto it = circles_boid.begin(); it != circles_boid.end();
         ++it, ++it_b) {
      // init_tr((*it_b), (*it));
      bob::init_cr((*it_b), (*it));
      (*it_b).correction();
      (*it_b).reset_corr();
      (*it_b).limit();

      window.draw(*it);
    }
  }
}
}  // namespace bob