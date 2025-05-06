#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boids.hpp"

#include "doctest.h"

TEST_CASE("Prova") { CHECK(1 == 1); }
TEST_CASE("Prova") {
  SUBCASE("n<0") { CHECK_THROWS(init_size(-3)); }
}

TEST_CASE("Verifica dei comportamenti per velocità superiori alla massima") {
  Boid b1 = {{0.f, 0.f}, {3.f, 4.f}};
  Boid b2 = {{0.f, 0.f}, {1.f, 7.f}};
  Boid b3 = {{0.f, 0.f}, {6.f, 8.f}};
  Boid b4 = {{0.f, 0.f}, {-9.f, -3.f}};
  Boid b5 = {{0.f, 0.f}, {-8.f, -10.f}};
  b1.vel_max();
  b2.vel_max();
  b3.vel_max();
  b4.vel_max();
  b5.vel_max();
  CHECK(b1.get_vel().x == 3.f);
  CHECK(b1.get_vel().y == 4.f);
  CHECK(b2.get_vel().x == doctest::Approx(0.714).epsilon(0.01));
  CHECK(b2.get_vel().y == 5.f);
  CHECK(b3.get_vel().x == doctest::Approx(3.75).epsilon(0.01));
  CHECK(b3.get_vel().y == 5.f);
  CHECK(b4.get_vel().x == -5.f);
  CHECK(b4.get_vel().y == doctest::Approx(-1.667).epsilon(0.01));
  CHECK(b5.get_vel().x == doctest::Approx(-4).epsilon(0.01));
  CHECK(b5.get_vel().y == -5.f);
}
TEST_CASE("Verifica dei comportamenti ai bordi") {
  Boid b1 = {{407.f, 410.f}, {0.f, 0.f}};
  Boid b2 = {{450.f, 35.f}, {0.f, 0.f}};
  Boid b3 = {{-3.f, 5.f}, {0.f, 0.f}};
  Boid b4 = {{-1.f, -1.f}, {0.f, 0.f}};
  b1.limit();
  b2.limit();
  b3.limit();
  b4.limit();
  CHECK(b1.get_pos().x == 7.f);
  CHECK(b1.get_pos().y == 10.f);
  CHECK(b2.get_pos().x == 50.f);
  CHECK(b2.get_pos().y == 35.f);
  CHECK(b3.get_pos().x == 397.f);
  CHECK(b3.get_pos().y == 5.f);
  CHECK(b4.get_pos().x == 399.f);
  CHECK(b4.get_pos().y == 399.f);
}
TEST_CASE("Verifica distanza tra due boid") {
  Boid b1 = {{0.f, 0.f}, {0.f, 0.f}};
  Boid b2 = {{4.f, 3.f}, {0.f, 0.f}};
  Boid b3 = {{-4.f, -3.f}, {0.f, 0.f}};
  Boid b4 = {{-2.f, 2.f}, {0.f, 0.f}};

  CHECK(distance(b1, b1) == 0);
  CHECK(distance(b2, b2) == 0);
  CHECK(distance(b1, b2) == 5.);
  CHECK(distance(b1, b3) == 5.);
  CHECK(distance(b1, b4) == doctest::Approx(2 * sqrt(2)));
}
TEST_CASE("Correzione delle velocità") {
  std::vector<Boid> vec;

  SUBCASE("Nessuna interazione") {
    Boid b1 = {{0.f, 0.f}, {0.f, -1.f}};
    Boid b2 = {{100.f, 100.f}, {-1.f, -3.f}};
    Boid b3 = {{200.f, 200.f}, {2.f, 1.f}};
    Boid b4 = {{300.f, 300.f}, {3.f, 4.f}};

    vec.push_back(b1);
    vec.push_back(b2);
    vec.push_back(b3);
    vec.push_back(b4);

    evaluate_correction(vec);
    Vec v = {0.f, 0.f};
    CHECK(b1.get_corr_v1() == v);
    CHECK(b1.get_corr_v2() == v);
    CHECK(b1.get_corr_v3() == v);

    // CHECK(b1.get_corr_v1().x == 0.f);
    // CHECK(b1.get_corr_v1().y == 0.f);
    // CHECK(b1.get_corr_v2().x == 0.f);
    // CHECK(b1.get_corr_v2().y == 0.f);
    // CHECK(b1.get_corr_v3().x == 0.f);
    // CHECK(b1.get_corr_v3().y == 0.f);
  }

  SUBCASE("") {
    Boid b1 = {{100.f, 100.f}, {2.f, 2.f}};
    Boid b2 = {{120.f, 120.f}, {-1.f, 3.f}};
    Boid b3 = {{80.f, 80.f}, {4.f, -2.f}};

    vec.push_back(b1);
    vec.push_back(b2);
    vec.push_back(b3);

    evaluate_correction(vec);

    CHECK(b3.get_corr_v1().x == 0.f);
    CHECK(b3.get_corr_v1().y == 0.f);
    CHECK(b3.get_corr_v2().x == 0.f);
    CHECK(b3.get_corr_v2().y == 0.f);
    CHECK(b3.get_corr_v3().x == 0.f);
    CHECK(b3.get_corr_v3().y == 0.f);
  }
  SUBCASE("Velocità di separazione") {}
}