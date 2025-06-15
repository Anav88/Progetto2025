#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boids.hpp"

#include "doctest.h"

namespace bob {

TEST_CASE("Vec2f Operations") {
  SUBCASE("Addition (+)") {
    Vec2f a{1.5f, 2.0f};
    Vec2f b{3.0f, -1.5f};
    Vec2f result = a + b;

    CHECK(result.x == doctest::Approx(4.5f));
    CHECK(result.y == doctest::Approx(0.5f));
  }

  SUBCASE("Compound Addition (+=)") {
    Vec2f a{5.0f, 10.0f};
    Vec2f b{2.5f, -3.0f};
    a += b;

    CHECK(a.x == doctest::Approx(7.5f));
    CHECK(a.y == doctest::Approx(7.0f));
  }

  SUBCASE("Subtraction (-)") {
    Vec2f a{10.0f, 8.0f};
    Vec2f b{4.0f, 9.0f};
    Vec2f result = a - b;

    CHECK(result.x == doctest::Approx(6.0f));
    CHECK(result.y == doctest::Approx(-1.0f));
  }

  SUBCASE("Multiplication (Scalar * Vec2f)") {
    Vec2f v{2.0f, -3.0f};
    Vec2f result1 = 2.5f * v;
    Vec2f result2 = v * 2.0f;

    CHECK(result1.x == doctest::Approx(5.0f));
    CHECK(result1.y == doctest::Approx(-7.5f));
    CHECK(result2.x == doctest::Approx(4.0f));
    CHECK(result2.y == doctest::Approx(-6.0f));
  }

  SUBCASE("Division (/)") {
    Vec2f v{6.0f, 4.0f};
    Vec2f result = v / 2.0f;

    CHECK(result.x == doctest::Approx(3.0f));
    CHECK(result.y == doctest::Approx(2.0f));

    CHECK_THROWS(v / 0.0f);
  }

  SUBCASE("Element-wise Multiplication (*)") {
    Vec2f a{3.0f, 5.0f};
    Vec2f b{2.0f, -1.0f};
    Vec2f result = a * b;

    CHECK(result.x == doctest::Approx(6.0f));
    CHECK(result.y == doctest::Approx(-5.0f));
  }
}

TEST_CASE("Norm") {
  Vec2f v1{3.f, 4.f};
  Vec2f v2{0.f, 0.f};

  CHECK(v1.norm() == 5.f);
  CHECK(v2.norm() == 0.f);
}

TEST_CASE("Angle") {
  Vec2f v1{1.f, 0.f};
  Vec2f v2{0.f, 1.f};
  Vec2f v3{0.f, 0.f};

  CHECK(v1.angle() == 0.f);
  CHECK(v2.angle() == doctest::Approx(1.571).epsilon(0.01));
  CHECK_THROWS(v3.angle());
}

TEST_CASE("Test inizializzazionen parametri") {
  CHECK_THROWS(init_parametres(0.8f, 0.3f, 0.1f, 15, 20, 3));
  CHECK_THROWS(init_parametres(1.8f, 0.3f, 0.1f, 15, 10, 3));
  CHECK_THROWS(init_parametres(0.8f, 1.3f, 0.1f, 15, 10, 3));
  CHECK_THROWS(init_parametres(0.8f, 0.3f, -1.1f, 15, 10, 3));
  CHECK_THROWS(init_parametres(0.8f, 0.3f, 0.1f, 15, -10, 3));
}

TEST_CASE("Verifica dei comportamenti per velocità superiori alla massima") {
  bob::Boid b1 = {{0.f, 0.f}, {3.f, 4.f}};
  bob::Boid b2 = {{0.f, 0.f}, {-9.f, 13.f}};
  bob::Boid b3 = {{0.f, 0.f}, {0.f, 0.f}};
  bob::Boid b4 = {{0.f, 0.f}, {12.f, 10.f}};
  bob::Boid b5 = {{0.f, 0.f}, {-13.f, -15.f}};
  b1.vel_max();
  b2.vel_max();
  b3.vel_max();
  b4.vel_max();
  b5.vel_max();
  CHECK(b1.get_vel().x == 3.f);
  CHECK(b1.get_vel().y == 4.f);
  CHECK(b2.get_vel().x == doctest::Approx(-8.050f).epsilon(0.01));
  CHECK(b2.get_vel().y == doctest::Approx(11.627f).epsilon(0.01));
  CHECK(b3.get_vel().x == 0.f);
  CHECK(b3.get_vel().y == 0.f);
  CHECK(b4.get_vel().x == doctest::Approx(10.866f).epsilon(0.01));
  CHECK(b4.get_vel().y == doctest::Approx(9.055f).epsilon(0.01));
  CHECK(b5.get_vel().x == doctest::Approx(-9.261f).epsilon(0.01));
  CHECK(b5.get_vel().y == doctest::Approx(-10.686f).epsilon(0.01));
}

TEST_CASE("Verifica distanza") {
  bob::Boid b1 = {{0.f, 0.f}, {0.f, 0.f}};
  bob::Boid p2 = {{4.f, 3.f}, {0.f, 0.f}};
  bob::Boid b3 = {{-4.f, -3.f}, {0.f, 0.f}};
  bob::Boid p4 = {{-2.f, 2.f}, {0.f, 0.f}};

  CHECK(distance(b1, b1) == 0.f);
  CHECK(distance(p2, p2) == 0.f);
  CHECK(distance(b1, p2) == 5.f);
  CHECK(distance(b1, b3) == 5.f);
  CHECK(distance(b1, p4) == doctest::Approx(2 * sqrt(2)));
}

TEST_CASE("Comportamento ai bordi") {
  SUBCASE("Predatori") {
    bob::Predator p1 = bob::Vec2f{601.f, 613.f};
    bob::Predator p2 = bob::Vec2f{750.f, 35.f};
    bob::Predator p3 = bob::Vec2f{-3.f, 204.f};
    bob::Predator p4 = bob::Vec2f{-20.f, -5.f};
    p1.limit();
    p2.limit();
    p3.limit();
    p4.limit();

    CHECK(p1.get_pos().x == 1.f);
    CHECK(p1.get_pos().y == 13.f);
    CHECK(p2.get_pos().x == 150.f);
    CHECK(p2.get_pos().y == 35.f);
    CHECK(p3.get_pos().x == 597.f);
    CHECK(p3.get_pos().y == 204.f);
    CHECK(p4.get_pos().x == 580.f);
    CHECK(p4.get_pos().y == 595.f);
  }
  SUBCASE("Boids") {
    bob::Boid b1 = {{607.f, 610.f}, {0.f, 0.f}};
    bob::Boid b2 = {{650.f, 35.f}, {0.f, 0.f}};
    bob::Boid b3 = {{-3.f, 5.f}, {0.f, 0.f}};
    bob::Boid b4 = {{-1.f, -1.f}, {0.f, 0.f}};
    b1.limit();
    b2.limit();
    b3.limit();
    b4.limit();

    CHECK(b1.get_pos().x == 7.f);
    CHECK(b1.get_pos().y == 10.f);
    CHECK(b2.get_pos().x == 50.f);
    CHECK(b2.get_pos().y == 35.f);
    CHECK(b3.get_pos().x == 597.f);
    CHECK(b3.get_pos().y == 5.f);
    CHECK(b4.get_pos().x == 599.f);
    CHECK(b4.get_pos().y == 599.f);
  }
}

TEST_CASE("Verifica funzionamento evaluate correction Boid") {
  SUBCASE("Velocità di separazione") {
    Boid b1(Two_Vec{{0.f, 0.f}, {1.f, 0.f}});
    Boid b2(Two_Vec{{2.f, 0.f}, {-1.f, 0.f}});
    std::vector<Boid> boids = {b1, b2};
    std::vector<Predator> predators;
    Par params = init_parametres(0.8f, 0.f, 0.f, 5, 3, 2);

    evaluate_boid_correction(boids, predators, params);
    Vec2f vel = (-params.s) * (b2.get_pos() - b1.get_pos());
    CHECK(boids[0].get_corr_vsep().x == doctest::Approx(vel.x).epsilon(0.01));
    CHECK(boids[0].get_corr_vsep().y == doctest::Approx(vel.y).epsilon(0.01));
    CHECK(boids[0].get_corr_vall().x == doctest::Approx(0.f).epsilon(0.01));
    CHECK(boids[0].get_corr_vcoes().x == doctest::Approx(0.f).epsilon(0.01));
  }  // namespace bob

  SUBCASE("Velocità di allineamento - 3 Boids") {
    Boid b1(Two_Vec{{0.f, 0.f}, {1.f, 1.f}});
    Boid b2(Two_Vec{{2.f, 2.f}, {3.f, 3.f}});
    Boid b3(Two_Vec{{3.f, 3.f}, {5.f, 1.f}});
    std::vector<Boid> boids = {b1, b2, b3};
    std::vector<Predator> predators;
    Par params = init_parametres(0.f, 0.5f, 0.f, 10, 1, 3);

    evaluate_boid_correction(boids, predators, params);

    Vec2f vel =
        ((b2.get_vel() - b1.get_vel()) + (b3.get_vel() - b1.get_vel())) / 2 *
        params.a;

    CHECK(boids[0].get_corr_vall().x == doctest::Approx(vel.x).epsilon(0.01));
    CHECK(boids[0].get_corr_vall().y == doctest::Approx(vel.y).epsilon(0.01));

    CHECK(boids[0].get_corr_vsep().x == doctest::Approx(0.f).epsilon(0.01));
    CHECK(boids[0].get_corr_vcoes().x == doctest::Approx(0.f).epsilon(0.01));
  }

  SUBCASE("Velocità di coesione - 3 Boids") {
    Boid b1(Two_Vec{{0.f, 0.f}, {1.f, 1.f}});
    Boid b2(Two_Vec{{10.f, 0.f}, {0.f, 0.f}});
    Boid b3(Two_Vec{{6.f, 4.f}, {0.f, 0.f}});
    std::vector<Boid> boids = {b1, b2, b3};
    std::vector<Predator> predators;
    Par params = init_parametres(0.f, 0.f, 0.8f, 15, 1, 3);

    evaluate_boid_correction(boids, predators, params);

    Vec2f vel =
        (((b2.get_pos() + b3.get_pos()) / 1.f) - b1.get_pos()) / 2 * params.c;

    CHECK(boids[0].get_corr_vcoes().x == doctest::Approx(vel.x).epsilon(0.01));
    CHECK(boids[0].get_corr_vcoes().y == doctest::Approx(vel.y).epsilon(0.01));

    CHECK(boids[0].get_corr_vsep().x == doctest::Approx(0.f).epsilon(0.01));
    CHECK(boids[0].get_corr_vall().x == doctest::Approx(0.f).epsilon(0.01));
  }

  SUBCASE("Tutte le velocità - 3 Boids") {
    Boid b1(Two_Vec{{0.f, 0.f}, {0.f, 0.f}});
    Boid b2(Two_Vec{{1.f, 0.f}, {1.f, 0.f}});
    Boid b3(Two_Vec{{0.f, 1.f}, {0.f, 1.f}});
    std::vector<Boid> boids = {b1, b2, b3};
    std::vector<Predator> predators;
    Par params = init_parametres(0.6f, 0.5f, 0.3f, 5, 2, 3);  // s, a, c attivi
    evaluate_boid_correction(boids, predators, params);

    Vec2f sep =
        ((b2.get_pos() - b1.get_pos()) + (b3.get_pos() - b1.get_pos())) *
        (-params.s);

    Vec2f all =
        ((b2.get_vel() - b1.get_vel()) + (b3.get_vel() - b1.get_vel())) / 2 *
        params.a;

    Vec2f coes = (b2.get_pos() + b3.get_pos() - b1.get_pos()) / 2 * params.c;

    CHECK(boids[0].get_corr_vsep().x == doctest::Approx(sep.x));
    CHECK(boids[0].get_corr_vsep().y == doctest::Approx(sep.y));
    CHECK(boids[0].get_corr_vall().x == doctest::Approx(all.x));
    CHECK(boids[0].get_corr_vall().y == doctest::Approx(all.y));
    CHECK(boids[0].get_corr_vcoes().x == doctest::Approx(coes.x));
    CHECK(boids[0].get_corr_vcoes().y == doctest::Approx(coes.y));
  }

  SUBCASE("Tutte le velocità - 4 Boids") {
    Boid b1(Two_Vec{{0.f, 0.f}, {0.f, 0.f}});
    Boid b2(Two_Vec{{1.f, 0.f}, {1.f, 0.f}});
    Boid b3(Two_Vec{{0.f, 1.f}, {0.f, 1.f}});
    Boid b4(Two_Vec{{1.f, 1.f}, {-1.f, -1.f}});
    std::vector<Boid> boids = {b1, b2, b3, b4};
    std::vector<Predator> predators;
    Par params = init_parametres(1.f, 1.f, 1.f, 5, 2, 4);

    evaluate_boid_correction(boids, predators, params);

    Vec2f sep = ((b2.get_pos() - b1.get_pos()) + (b3.get_pos() - b1.get_pos()) +
                 (b4.get_pos() - b1.get_pos())) *
                (-params.s);

    Vec2f all = ((b2.get_vel() - b1.get_vel()) + (b3.get_vel() - b1.get_vel()) +
                 (b4.get_vel() - b1.get_vel())) /
                3 * params.a;

    Vec2f coes =
        (((b2.get_pos() + b3.get_pos() + b4.get_pos()) / 3) - b1.get_pos()) *
        params.c;

    CHECK(boids[0].get_corr_vsep().x == doctest::Approx(sep.x));
    CHECK(boids[0].get_corr_vsep().y == doctest::Approx(sep.y));
    CHECK(boids[0].get_corr_vall().x == doctest::Approx(all.x));
    CHECK(boids[0].get_corr_vall().y == doctest::Approx(all.y));
    CHECK(boids[0].get_corr_vcoes().x == doctest::Approx(coes.x));
    CHECK(boids[0].get_corr_vcoes().y == doctest::Approx(coes.y));
  }

  SUBCASE("Tutte le velocità - 5 Boids") {
    Boid b1(Two_Vec{{0.f, 0.f}, {0.f, 0.f}});
    Boid b2(Two_Vec{{1.f, 0.f}, {1.f, 0.f}});
    Boid b3(Two_Vec{{0.f, 1.f}, {0.f, 1.f}});
    Boid b4(Two_Vec{{100.f, 100.f}, {5.f, 5.f}});
    Boid b5(Two_Vec{{1.f, 1.f}, {-1.f, -1.f}});
    std::vector<Boid> boids = {b1, b2, b3, b4, b5};
    std::vector<Predator> predators;
    Par params = init_parametres(1.f, 1.f, 1.f, 5, 2, 5);

    evaluate_boid_correction(boids, predators, params);

    Vec2f sep = ((b2.get_pos() - b1.get_pos()) + (b3.get_pos() - b1.get_pos()) +
                 (b5.get_pos() - b1.get_pos())) *
                (-params.s);

    Vec2f all = ((b2.get_vel() - b1.get_vel()) + (b3.get_vel() - b1.get_vel()) +
                 (b5.get_vel() - b1.get_vel())) /
                3 * params.a;

    Vec2f coes =
        (((b2.get_pos() + b3.get_pos() + b5.get_pos()) / 3) - b1.get_pos()) *
        params.c;

    CHECK(boids[0].get_corr_vsep().x == doctest::Approx(sep.x));
    CHECK(boids[0].get_corr_vsep().y == doctest::Approx(sep.y));
    CHECK(boids[0].get_corr_vall().x == doctest::Approx(all.x));
    CHECK(boids[0].get_corr_vall().y == doctest::Approx(all.y));
    CHECK(boids[0].get_corr_vcoes().x == doctest::Approx(coes.x));
    CHECK(boids[0].get_corr_vcoes().y == doctest::Approx(coes.y));
  }

  SUBCASE("Posizione non valida") {
    SUBCASE("Coordinata negativa") {
      Boid b = Two_Vec{{305.f, -12.f}, {0.f, 0.f}};
      std::vector<Boid> boids{b};
      std::vector<Predator> predators;
      Par params{0.2f, 0.3f, 0.9f, 6, 3, 1};

      CHECK_THROWS(evaluate_boid_correction(boids, predators, params));
    }

    SUBCASE("Coordinata oltre il limite") {
      Boid b1 = Two_Vec{{305.f, 12.f}, {0.f, 0.f}};
      Boid b2 = Two_Vec{{201.f, 812.f}, {0.f, 0.f}};
      std::vector<Boid> boids{b1, b2};
      std::vector<Predator> predators;
      Par params{0.2f, 0.3f, 0.9f, 6, 3, 2};

      CHECK_THROWS(evaluate_boid_correction(boids, predators, params));
    }
  }

  SUBCASE("Invalid speed") {
    Boid b = Two_Vec{{0.f, 0.f}, {8.f, -14.f}};
    std::vector<Boid> boids{b};
    std::vector<Predator> predators;
    Par params{0.2f, 0.3f, 0.9f, 6, 3, 1};

    CHECK_THROWS(evaluate_boid_correction(boids, predators, params));
  }
}

TEST_CASE("Azioni dei predatori") {
  SUBCASE("Predatore si muove verso un boid") {
    Predator p({0.f, 0.f});
    Boid b(Two_Vec{{1.f, 0.f}, {0.f, 0.f}});

    Vec2f diff_pos1 = b.get_pos() - p.get_pos();
    float angle = diff_pos1.angle();

    p.vel_inseg(angle);
    p.correction();

    CHECK(p.get_vel().x == doctest::Approx(VEL_PRED).epsilon(0.01));
    CHECK(p.get_vel().y == doctest::Approx(0.f).epsilon(0.01));
    CHECK(p.get_pos().x == doctest::Approx(VEL_PRED * TIME_STEP).epsilon(0.01));
    CHECK(p.get_pos().y == doctest::Approx(0.f).epsilon(0.01));
  }

  SUBCASE("Predatori si allontanano perchè vicini") {
    SUBCASE("2 orizzontalmente") {
      Predator p1({0.f, 0.f});
      Predator p2({1.f, 0.f});

      Vec2f diff_pos1 = p1.get_pos() - p2.get_pos();
      float angle1 = diff_pos1.angle();
      Vec2f diff_pos2 = p2.get_pos() - p1.get_pos();
      float angle2 = diff_pos2.angle();

      p1.vel_sep(angle1);
      p2.vel_sep(angle2);
      p1.correction();
      p2.correction();

      float vx1 = VEL_PRED_SEP * std::cos(angle1);
      float vy1 = VEL_PRED_SEP * std::sin(angle1);
      float vx2 = VEL_PRED_SEP * std::cos(angle2);
      float vy2 = VEL_PRED_SEP * std::sin(angle2);

      CHECK(p1.get_vel().x == doctest::Approx(vx1).epsilon(0.01));
      CHECK(p1.get_vel().y == doctest::Approx(vy1).epsilon(0.01));
      CHECK(p2.get_vel().x == doctest::Approx(vx2).epsilon(0.01));
      CHECK(p2.get_vel().y == doctest::Approx(vy2).epsilon(0.01));
      CHECK(p1.get_pos().x == doctest::Approx(vx1 * TIME_STEP).epsilon(0.01));
    }
    SUBCASE("2 verticalmente") {
      Predator p1({100.f, 100.f});
      Predator p2({100.f, 90.f});  // sopra

      Vec2f diff_pos1 = p1.get_pos() - p2.get_pos();
      float angle1 = diff_pos1.angle();
      Vec2f diff_pos2 = p2.get_pos() - p1.get_pos();
      float angle2 = diff_pos2.angle();
      p1.vel_sep(angle1);
      p1.correction();
      p2.vel_sep(angle2);
      p2.correction();

      float vx1 = VEL_PRED_SEP * std::cos(angle1);
      float vy1 = VEL_PRED_SEP * std::sin(angle1);
      float vx2 = VEL_PRED_SEP * std::cos(angle2);
      float vy2 = VEL_PRED_SEP * std::sin(angle2);

      CHECK(p1.get_vel().x == doctest::Approx(vx1).epsilon(0.01));
      CHECK(p1.get_vel().y == doctest::Approx(vy1).epsilon(0.01));

      CHECK(p2.get_vel().x == doctest::Approx(vx2).epsilon(0.01));
      CHECK(p2.get_vel().y == doctest::Approx(vy2).epsilon(0.01));
    }
    SUBCASE("2 diagonalmente") {
      Predator p1({50.f, 50.f});
      Predator p2({55.f, 55.f});  // diagonale in alto a destra

      Vec2f diff_pos1 = p1.get_pos() - p2.get_pos();
      float angle1 = diff_pos1.angle();
      Vec2f diff_pos2 = p2.get_pos() - p1.get_pos();
      float angle2 = diff_pos2.angle();

      p1.vel_sep(angle1);
      p1.correction();
      p2.vel_sep(angle2);
      p2.correction();

      float vx = VEL_PRED_SEP * std::cos(angle1);
      float vy = VEL_PRED_SEP * std::sin(angle1);

      float vx2 = VEL_PRED_SEP * std::cos(angle2);
      float vy2 = VEL_PRED_SEP * std::sin(angle2);

      CHECK(p1.get_vel().x == doctest::Approx(vx).epsilon(0.01));
      CHECK(p1.get_vel().y == doctest::Approx(vy).epsilon(0.01));
      CHECK(p2.get_vel().x == doctest::Approx(vx2).epsilon(0.01));
      CHECK(p2.get_vel().y == doctest::Approx(vy2).epsilon(0.01));
    }
  }

  SUBCASE("Inseguimento e separazione") {
    Predator p1({0.f, 0.f});
    Boid b(Two_Vec{{1.f, 1.f}, {0.f, 0.f}});
    Predator p2({0.f, 1.f});

    Vec2f dist_ins = b.get_pos() - p1.get_pos();
    float angle_ins = dist_ins.angle();
    Vec2f dist_sep = p1.get_pos() - p2.get_pos();
    float angle_sep = dist_sep.angle();

    p1.vel_inseg(angle_ins);
    p1.vel_sep(angle_sep);
    p1.correction();

    Vec2f v_expected = {
        cosf(angle_ins) * VEL_PRED + cosf(angle_sep) * VEL_PRED_SEP,
        sinf(angle_ins) * VEL_PRED + sinf(angle_sep) * VEL_PRED_SEP};

    CHECK(p1.get_vel().x == doctest::Approx(v_expected.x).epsilon(0.01));
    CHECK(p1.get_vel().y == doctest::Approx(v_expected.y).epsilon(0.01));
  }

  SUBCASE("Boid fugge dal predatore") {
    Boid b(Two_Vec{{100.f, 100.f}, {0.f, 0.f}});
    Predator p({90.f, 100.f});

    Vec2f dist_fug = b.get_pos() - p.get_pos();
    float angle = dist_fug.angle();

    std::vector<Boid> boids = {b};
    std::vector<Predator> predators = {p};
    Par params = init_parametres(1.f, 1.f, 1.f, 5, 2, 1);
    evaluate_boid_correction(boids, predators, params);

    CHECK(b.get_corr_vsep().x == doctest::Approx(0.f).epsilon(0.01));
    CHECK(b.get_corr_vall().x == doctest::Approx(0.f).epsilon(0.01));
    CHECK(b.get_corr_vcoes().x == doctest::Approx(0.f).epsilon(0.01));
    CHECK(b.get_corr_vsep().x == doctest::Approx(0.f).epsilon(0.01));
    CHECK(boids[0].get_corr_vfuga().x ==
          doctest::Approx(FACT_FUGA * cosf(angle)).epsilon(0.01));
    CHECK(boids[0].get_corr_vfuga().y ==
          doctest::Approx(FACT_FUGA * sinf(angle)).epsilon(0.01));
  }

  SUBCASE("Boid non scappa dal predatore distante") {
    Boid b(Two_Vec{{100.f, 100.f}, {0.f, 0.f}});
    Predator p({300.f, 300.f});

    std::vector<Boid> boids = {b};
    std::vector<Predator> predators = {p};
    Par params{0.2f, 0.3f, 0.1f, 100, 50, 1};
    evaluate_boid_correction(boids, predators, params);

    CHECK(boids[0].get_corr_vfuga().x == doctest::Approx(0.f).epsilon(0.01));
    CHECK(boids[0].get_corr_vfuga().y == doctest::Approx(0.f).epsilon(0.01));
  }

  SUBCASE("Predatore individua il boid più vicino") {
    Boid b1(Two_Vec{{100.f, 100.f}, {0.f, 0.f}});
    Boid b2(Two_Vec{{200.f, 100.f}, {0.f, 0.f}});
    Predator p({110.f, 100.f});

    std::vector<Boid> boids = {b1, b2};
    std::vector<Predator> predators = {p};

    evaluate_pred_correction(predators, boids);

    Vec2f vel = predators[0].get_vel_inseg();
    CHECK(vel.x < 0.f);
  }

  SUBCASE("Position not valid") {
    Predator p = Vec2f{809.f, 103};
    std::vector<Predator> predators = {p};
    std::vector<Boid> boids;
    CHECK_THROWS(evaluate_pred_correction(predators, boids));
  }
}

TEST_CASE("Test della funzione reset_corr") {
  SUBCASE("Boid") {
    Boid b(Two_Vec{{0.f, 0.f}, {1.f, 1.f}});
    b.vel_sep({1.f, 1.f}, 1.f);
    b.vel_all({1.f, 1.f}, 1.f);
    b.vel_coes({1.f, 1.f}, 1.f);
    b.vel_fuga(0.f);

    b.reset_corr();

    CHECK(b.get_corr_vsep() == Vec2f{0.f, 0.f});
    CHECK(b.get_corr_vall() == Vec2f{0.f, 0.f});
    CHECK(b.get_corr_vcoes() == Vec2f{0.f, 0.f});
  }

  SUBCASE("Predator") {
    Predator p({100.f, 100.f});
    p.vel_inseg(0.0f);
    p.vel_sep(3.14f);

    p.reset_corr();

    CHECK(p.get_vel_inseg() == Vec2f{0.f, 0.f});
    CHECK(p.get_vel_sep() == Vec2f{0.f, 0.f});
  }
}

namespace statistics {
TEST_CASE("Statistics") {
  SUBCASE("Velocità media dei boids") {
    Boid b1(Two_Vec{{0.f, 0.f}, {2.f, 0.f}});
    Boid b2(Two_Vec{{0.f, 0.f}, {0.f, 2.f}});
    Boid b3(Two_Vec{{0.f, 0.f}, {2.f, 2.f}});
    std::vector<Boid> boids = {b1, b2, b3};

    Vec2f expected_mean = {(2.f + 0.f + 2.f) / 3.f, (0.f + 2.f + 2.f) / 3.f};

    CHECK(mean_velocity_algo(boids).x == doctest::Approx(expected_mean.x));
    CHECK(mean_velocity_algo(boids).y == doctest::Approx(expected_mean.y));
  }

  SUBCASE("Deaviazione standard") {
    Boid b1(Two_Vec{{0.f, 0.f}, {1.f, 1.f}});
    Boid b2(Two_Vec{{0.f, 0.f}, {3.f, 1.f}});
    Boid b3(Two_Vec{{0.f, 0.f}, {5.f, 1.f}});
    std::vector<Boid> boids = {b1, b2, b3};

    float dev_x = std::sqrt((4.f + 0.f + 4.f) / 3.f);
    float dev_y = 0.f;

    Vec2f deviation_algo = mean_deviation_algo(boids);

    CHECK(deviation_algo.x == doctest::Approx(dev_x));
    CHECK(deviation_algo.y == doctest::Approx(dev_y));
  }

  SUBCASE("Deviazione nulla per velocità identiche") {
    Boid b1(Two_Vec{{0.f, 0.f}, {2.f, 3.f}});
    Boid b2(Two_Vec{{0.f, 0.f}, {2.f, 3.f}});
    Boid b3(Two_Vec{{0.f, 0.f}, {2.f, 3.f}});
    std::vector<Boid> boids = {b1, b2, b3};

    Vec2f dev_algo = mean_deviation_algo(boids);

    CHECK(dev_algo.x == doctest::Approx(0.f));
    CHECK(dev_algo.y == doctest::Approx(0.f));
  }

  SUBCASE("Media con nessun boid") {
    std::vector<Boid> boids;
    CHECK_THROWS(mean_velocity_algo(boids));
  }

  SUBCASE("Deviazione standard con 1 boid") {
    Boid b = Two_Vec{{4.f, 5.f}, {0.f, 0.f}};
    std::vector<Boid> boids{b};
    CHECK_THROWS(mean_deviation_algo(boids));
  }
}
}  // namespace statistics

TEST_CASE("Test metodo erase_boid") {
  Boid b(Two_Vec{{100.f, 100.f}, {0.f, 0.f}});
  Predator p({100.f, 100.f});
  sf::CircleShape shape(1.f);
  shape.setPosition({100.f, 100.f});

  std::vector<Boid> boids = {b};
  std::vector<Predator> predators = {p};
  std::vector<sf::CircleShape> circles = {shape};

  erase_boid(boids, predators, circles);

  CHECK(boids.empty());
  CHECK(circles.empty());
}
}  // namespace bob