#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>
// #include <execution>
#include <iostream>
#include <random>

#include "boids.hpp"

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
  vel_ = vel_ + corr_v1_ + corr_v2_ + corr_v3_;
  pos_ = pos_ + (vel_ * (0.017f));
}
void Boid::limit() {
  if (pos_.x < MIN_POS) {
    pos_.x += MAX_POS;
  }
  if (pos_.y < MIN_POS) {
    pos_.y += MAX_POS;
  }
  if (pos_.x > MAX_POS) {
    pos_.x -= MAX_POS;
  }
  if (pos_.y > MAX_POS) {
    pos_.y -= MAX_POS;
  }
}
void Boid::reset_corr() {
  corr_v1_ = {0, 0};
  corr_v2_ = {0, 0};
  corr_v3_ = {0, 0};
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

bool operator==(Boid b1, Boid b2) {
  if (b1.get_pos() == b2.get_pos() && b1.get_vel() == b2.get_vel()) {
    return true;
  } else {
    return false;
  }
}

Predator::Predator(Vec p, Vec v) : pos_{p}, vel_{v} {}
Predator::Predator() : pos_{0, 0}, vel_{0, 0} {}
Vec Predator::get_pos() { return pos_; }
Vec Predator::get_vel() { return vel_; }

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

float distance(Boid b1, Boid b2) { return abs(b1.get_pos(), b2.get_pos()); }

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

Two_Vec rand_num() {
  std::random_device r;
  std::default_random_engine eng{r()};
  std::uniform_real_distribution<float> pos_x{MIN_POS, MAX_POS};
  std::uniform_real_distribution<float> pos_y{MIN_POS, MAX_POS};
  std::uniform_real_distribution<float> vel_x{MIN_VEL, MAX_VEL};
  std::uniform_real_distribution<float> vel_y{MIN_VEL, MAX_VEL};

  return {{pos_x(eng), pos_y(eng)}, {vel_x(eng), vel_x(eng)}};
}

void add_boid(std::vector<Boid> &add_vec) {
  std::generate(add_vec.begin(), add_vec.end(),
                []() { return (Boid(rand_num())); });
}

void evaluate_correction(std::vector<Boid> &vec, Par parametres) {
  for (auto it_i = vec.begin(); it_i != vec.end(); ++it_i) {
    Vec sum_diff_pos = {0., 0.};  // per il calcolo vel sep
    Vec sum_diff_vel = {0., 0.};  // per il calcolo vel all
    Vec sum_coord = {0., 0.};     // per il calcolo del cdm/vel coes
    int neighbor_count = 0;
    int separation_count = 0;

    for (auto it_j = vec.begin(); it_j != vec.end(); ++it_j) {
      if (it_i != it_j) {
        float dist = distance(*it_i, *it_j);

        if (dist < parametres.d) {
          ++neighbor_count;
          sum_diff_vel = sum_diff_vel + ((*it_j).get_vel() - (*it_i).get_vel());
          sum_coord = sum_coord + (*it_j).get_pos();

          if (dist < parametres.ds) {
            sum_diff_pos =
                sum_diff_pos + ((*it_j).get_pos() - (*it_i).get_pos());
            ++separation_count;
          }
        }
      }
    }
    if (separation_count > 0) {
      (*it_i).vel_sep(sum_diff_pos, parametres.s);
    }

    if (neighbor_count > 1) {
      Vec mean_vel = sum_diff_vel / (neighbor_count - 1);
      Vec cdm = sum_coord / (neighbor_count - 1);
      (*it_i).vel_all(mean_vel, parametres.a);
      (*it_i).vel_coes(cdm, parametres.c);
    }
  }
}

void add_triangle(std::vector<sf::ConvexShape> &triangles) {
  for (auto it = triangles.begin(); it != triangles.end(); ++it) {
    (*it).setPointCount(3);
  }
}

void init_tr(Boid &b, sf::ConvexShape &triangles) {
  triangles.setPoint(0, {3.0f, 0.f});
  triangles.setPoint(1, {-1.0f, -1.5f});
  triangles.setPoint(2, {-1.0f, 1.5f});
  sf::Vector2f pos(b.get_pos().x + 100.0f, b.get_pos().y + 100.0f);
  triangles.setPosition(pos);

  triangles.setFillColor(sf::Color::Black);
  triangles.setRotation(std::atan2(b.get_vel().y, b.get_vel().x) * 180.f / PI);
}

sf::CircleShape crt_pred(float x, float y) {
  sf::CircleShape circle{3.0f};
  // circle.setOrigin(5.0f, 5.f);
  circle.setPosition({x, y});
  circle.setFillColor(sf::Color::Blue);

  return circle;
}