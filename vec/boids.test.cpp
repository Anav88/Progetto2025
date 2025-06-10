#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boids.hpp"

#include "doctest.h"
using namespace bob;

TEST_CASE("Test inizializzazionen parametri"){

  CHECK_THROWS(init_parametres(0.8f,0.3f,0.1f,15,20,3));
  CHECK_THROWS(init_parametres(1.8f,0.3f,0.1f,15,10,3));
  CHECK_THROWS(init_parametres(0.8f,1.3f,0.1f,15,10,3));
  CHECK_THROWS(init_parametres(0.8f,0.3f,-1.1f,15,10,3));
  CHECK_THROWS(init_parametres(0.8f,0.3f,0.1f,15,-10,3));
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

  CHECK(distance(b1, b1) == 0);
  CHECK(distance(p2, p2) == 0);
  CHECK(distance(b1, p2) == 5.);
  CHECK(distance(b1, b3) == 5.);
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

    Par params = init_parametres(1.f, 0.f, 0.f, 5, 3, 2);  // s=1, ds=3, d=5
    evaluate_correction(boids, params);

    CHECK(boids[0].get_corr_vsep().x ==
          doctest::Approx(-2.f));  // separation: -(2-0)
    CHECK(boids[0].get_corr_vsep().y == doctest::Approx(0.f));
    CHECK(boids[0].get_corr_vall().x == doctest::Approx(0.f));
    CHECK(boids[0].get_corr_vcoes().x == doctest::Approx(0.f));
  }

  SUBCASE("Velocità di allineamento - 3 Boids") {
    Boid b1(Two_Vec{{0.f, 0.f}, {1.f, 1.f}});
    Boid b2(Two_Vec{{2.f, 2.f}, {3.f, 3.f}});
    Boid b3(Two_Vec{{3.f, 3.f}, {5.f, 1.f}});
    std::vector<Boid> boids = {b1, b2, b3};

    Par params =
        init_parametres(0.f, 0.5f, 0.f, 10, 1, 3);  // solo allineamento
    evaluate_correction(boids, params);

    // Calcolo corretto: ( (b2.vel - b1.vel) + (b3.vel - b1.vel) ) / (2 - 1)
    Vec2f expected =
        ((b2.get_vel() - b1.get_vel()) + (b3.get_vel() - b1.get_vel())) * 0.5f;

    CHECK(boids[0].get_corr_vall().x == doctest::Approx(expected.x));
    CHECK(boids[0].get_corr_vall().y == doctest::Approx(expected.y));

    CHECK(boids[0].get_corr_vsep().x == doctest::Approx(0.f));
    CHECK(boids[0].get_corr_vcoes().x == doctest::Approx(0.f));
  }

  SUBCASE("Velocità di coesione - 3 Boids") {
    Boid b1(Two_Vec{{0.f, 0.f}, {1.f, 1.f}});
    Boid b2(Two_Vec{{10.f, 0.f}, {0.f, 0.f}});
    Boid b3(Two_Vec{{6.f, 4.f}, {0.f, 0.f}});
    std::vector<Boid> boids = {b1, b2, b3};

    Par params =
        init_parametres(0.f, 0.f, 1.f, 15, 1, 3);  // solo coesione attiva
    evaluate_correction(boids, params);

    // Centro di massa (cdm) = (b2.pos + b3.pos) / (2 - 1) = {16, 4} / 1 = {16,
    // 4}
    Vec2f expected_cdm = (b2.get_pos() + b3.get_pos()) / 1.f;
    Vec2f expected = (expected_cdm - b1.get_pos()) * params.c;

    CHECK(boids[0].get_corr_vcoes().x == doctest::Approx(expected.x));
    CHECK(boids[0].get_corr_vcoes().y == doctest::Approx(expected.y));

    CHECK(boids[0].get_corr_vsep().x == doctest::Approx(0.f));
    CHECK(boids[0].get_corr_vall().x == doctest::Approx(0.f));
  }

  SUBCASE("Tutte le velocità - 3 Boids") {
    Boid b1(Two_Vec{{0.f, 0.f}, {0.f, 0.f}});
    Boid b2(Two_Vec{{1.f, 0.f}, {1.f, 0.f}});
    Boid b3(Two_Vec{{0.f, 1.f}, {0.f, 1.f}});
    std::vector<Boid> boids = {b1, b2, b3};

    Par params = init_parametres(1.f, 1.f, 1.f, 5, 2, 3);  // s, a, c attivi
    evaluate_correction(boids, params);

    // Separazione
    Vec2f sep = (b2.get_pos() - b1.get_pos()) + (b3.get_pos() - b1.get_pos());
    Vec2f corr_sep = sep * (-params.s);

    // Allineamento
    Vec2f all = (b2.get_vel() - b1.get_vel()) + (b3.get_vel() - b1.get_vel());
    Vec2f corr_all = all / (2 - 1) * params.a;

    // Coesione
    Vec2f cdm = (b2.get_pos() + b3.get_pos()) / (2 - 1);
    Vec2f corr_coes = (cdm - b1.get_pos()) * params.c;

    CHECK(boids[0].get_corr_vsep().x == doctest::Approx(corr_sep.x));
    CHECK(boids[0].get_corr_vall().x == doctest::Approx(corr_all.x));
    CHECK(boids[0].get_corr_vcoes().x == doctest::Approx(corr_coes.x));
  }

  SUBCASE("Tutte le velocità - 4 Boids") {
    Boid b1(Two_Vec{{0.f, 0.f}, {0.f, 0.f}});
    Boid b2(Two_Vec{{1.f, 0.f}, {1.f, 0.f}});
    Boid b3(Two_Vec{{0.f, 1.f}, {0.f, 1.f}});
    Boid b4(Two_Vec{{1.f, 1.f}, {-1.f, -1.f}});
    std::vector<Boid> boids = {b1, b2, b3, b4};

    Par params = init_parametres(1.f, 1.f, 1.f, 5, 2, 4);  // d=5, ds=2
    evaluate_correction(boids, params);

    // Separazione
    Vec2f sep = (b2.get_pos() - b1.get_pos()) + (b3.get_pos() - b1.get_pos()) +
              (b4.get_pos() - b1.get_pos());
    Vec2f corr_sep = sep * (-params.s);

    // Allineamento
    Vec2f all = (b2.get_vel() - b1.get_vel()) + (b3.get_vel() - b1.get_vel()) +
              (b4.get_vel() - b1.get_vel());
    Vec2f corr_all = all / (3 - 1) * params.a;

    // Coesione
    Vec2f cdm = (b2.get_pos() + b3.get_pos() + b4.get_pos()) / (3 - 1);
    Vec2f corr_coes = (cdm - b1.get_pos()) * params.c;

    CHECK(boids[0].get_corr_vsep().x == doctest::Approx(corr_sep.x));
    CHECK(boids[0].get_corr_vall().x == doctest::Approx(corr_all.x));
    CHECK(boids[0].get_corr_vcoes().x == doctest::Approx(corr_coes.x));
  }

  SUBCASE("Tutte le velocità - 5 Boids") {
    Boid b1(Two_Vec{{0.f, 0.f}, {0.f, 0.f}});
    Boid b2(Two_Vec{{1.f, 0.f}, {1.f, 0.f}});
    Boid b3(Two_Vec{{0.f, 1.f}, {0.f, 1.f}});
    Boid b4(Two_Vec{{1.f, 1.f}, {-1.f, -1.f}});
    Boid b5(Two_Vec{{100.f, 100.f}, {5.f, 5.f}});  // troppo lontano
    std::vector<Boid> boids = {b1, b2, b3, b4, b5};

    Par params = init_parametres(1.f, 1.f, 1.f, 5, 2, 5);
    evaluate_correction(boids, params);

    // Ignora b5, quindi stesso calcolo del test con 4 boid
    Vec2f sep = (b2.get_pos() - b1.get_pos()) + (b3.get_pos() - b1.get_pos()) +
              (b4.get_pos() - b1.get_pos());
    Vec2f corr_sep = sep * (-params.s);

    Vec2f all = (b2.get_vel() - b1.get_vel()) + (b3.get_vel() - b1.get_vel()) +
              (b4.get_vel() - b1.get_vel());
    Vec2f corr_all = all / (3 - 1) * params.a;

    Vec2f cdm = (b2.get_pos() + b3.get_pos() + b4.get_pos()) / (3 - 1);
    Vec2f corr_coes = (cdm - b1.get_pos()) * params.c;

    CHECK(boids[0].get_corr_vsep().x == doctest::Approx(corr_sep.x));
    CHECK(boids[0].get_corr_vall().x == doctest::Approx(corr_all.x));
    CHECK(boids[0].get_corr_vcoes().x == doctest::Approx(corr_coes.x));
  }
}

TEST_CASE("Predatore si muove verso un boid") {
  Predator p({0.f, 0.f});
  Boid b(Two_Vec{{1.f, 0.f}, {0.f, 0.f}});

  float angle =
      std::atan2f(b.get_pos().y - p.get_pos().y, b.get_pos().x - p.get_pos().x);

  p.vel_inseg(angle);
  p.correction();

  CHECK(p.get_vel().x == doctest::Approx(VEL_PRED));
  CHECK(p.get_vel().y == doctest::Approx(0.f));
  CHECK(p.get_pos().x == doctest::Approx(VEL_PRED * TIME_STEP));
  CHECK(p.get_pos().y == doctest::Approx(0.f));
}

TEST_CASE("Predatori si allontanano perchè vicini") {
  SUBCASE("2 orizzontalmente") {
    Predator p1({0.f, 0.f});
    Predator p2({1.f, 0.f});

    float angle = std::atan2f(p1.get_pos().y - p2.get_pos().y,
                              p1.get_pos().x - p2.get_pos().x);
    float angle2 = std::atan2f(p2.get_pos().y - p1.get_pos().y,
                               p2.get_pos().x - p1.get_pos().x);

    p1.vel_sep(angle);
    p2.vel_sep(angle2);
    p1.correction();
    p2.correction();

    CHECK(p1.get_vel().x == doctest::Approx(-7));
    CHECK(p1.get_vel().y == doctest::Approx(0.f));
    CHECK(p2.get_vel().x == doctest::Approx(7));
    CHECK(p2.get_vel().y == doctest::Approx(0.f));
    CHECK(p1.get_pos().x == doctest::Approx(-VEL_PRED_SEP * TIME_STEP));
  }
  SUBCASE("2 verticalmente") {
    Predator p1({100.f, 100.f});
    Predator p2({100.f, 90.f});  // sopra

    float angle = std::atan2(10.f, 0.f);
    p1.vel_sep(angle);
    p1.correction();
    p2.vel_sep(-angle);
    p2.correction();

    // angolo tra p1 e p2: f = atan2(100 - 90, 0) = atan2(10, 0) = π/2 (90°)

    float vx = VEL_PRED_SEP * std::cos(angle);  // = 0
    float vy = VEL_PRED_SEP * std::sin(angle);  // = 7

    CHECK(p1.get_vel().x == doctest::Approx(vx));
    CHECK(p1.get_vel().y == doctest::Approx(vy));

    // p2 separazione opposta → angolo = -π/2
    CHECK(p2.get_vel().x == doctest::Approx(vx));
    CHECK(p2.get_vel().y == doctest::Approx(-vy));
  }
  SUBCASE("2 diagonalmente") {
    Predator p1({50.f, 50.f});
    Predator p2({55.f, 55.f});  // diagonale in alto a destra

    float angle = std::atan2(-5.f, -5.f);  // p1 rispetto a p2
    p1.vel_sep(angle);
    p1.correction();

    float angle2 = std::atan2(5.f, 5.f);  // p2 rispetto a p1
    p2.vel_sep(angle2);
    p2.correction();

    float vx = VEL_PRED_SEP * std::cos(angle);
    float vy = VEL_PRED_SEP * std::sin(angle);

    float vx2 = VEL_PRED_SEP * std::cos(angle2);
    float vy2 = VEL_PRED_SEP * std::sin(angle2);

    CHECK(p1.get_vel().x == doctest::Approx(vx));
    CHECK(p1.get_vel().y == doctest::Approx(vy));
    CHECK(p2.get_vel().x == doctest::Approx(vx2));
    CHECK(p2.get_vel().y == doctest::Approx(vy2));
  }

  SUBCASE("3 predatori") {
    Predator p1({50.f, 50.f});
    Predator p2({55.f, 50.f});  // destra
    Predator p3({50.f, 55.f});  // sopra

    // p1 si allontana da p2
    float angle1 = std::atan2(0.f, -5.f);  // verso sinistra
    // p1 si allontana da p3
    float angle2 = std::atan2(-5.f, 0.f);  // verso il basso

    p1.vel_sep(angle1);
    p1.vel_sep(angle2);
    p1.correction();

    // Calcolo separazione totale
    float vx_total = VEL_PRED_SEP * (std::cos(angle1) + std::cos(angle2));
    float vy_total = VEL_PRED_SEP * (std::sin(angle1) + std::sin(angle2));

    CHECK(p1.get_vel().x == doctest::Approx(vx_total));
    CHECK(p1.get_vel().y == doctest::Approx(vy_total));
  }
}

TEST_CASE("Inseguimento e separazione") {
  Predator p({0.f, 0.f});
  Boid b(Two_Vec{{1.f, 1.f}, {0.f, 0.f}});
  Predator other({0.f, 1.f});

  float angle_ins =
      std::atan2f(b.get_pos().y - p.get_pos().y, b.get_pos().x - p.get_pos().x);
  float angle_fug = std::atan2f(p.get_pos().y - other.get_pos().y,
                                p.get_pos().x - other.get_pos().x);

  p.vel_inseg(angle_ins);
  p.vel_sep(angle_fug);
  p.correction();

  Vec2f v_expected = {
      std::cosf(angle_ins) * VEL_PRED + std::cosf(angle_fug) * VEL_PRED_SEP,
      std::sinf(angle_ins) * VEL_PRED + std::sinf(angle_fug) * VEL_PRED_SEP};

  CHECK(p.get_vel().x == doctest::Approx(v_expected.x));
  CHECK(p.get_vel().y == doctest::Approx(v_expected.y));
}

TEST_CASE("Boid scappa") {
  Boid b(Two_Vec{{100.f, 100.f}, {0.f, 0.f}});
  Predator p({90.f, 100.f});

  float angle =
      std::atan2f(b.get_pos().y - p.get_pos().y, b.get_pos().x - p.get_pos().x);
  b.corr_vel_fuga(angle);

  CHECK(b.get_corr_vsep().x == doctest::Approx(0.f));
  CHECK(b.get_corr_vall().x == doctest::Approx(0.f));
  CHECK(b.get_corr_vcoes().x == doctest::Approx(0.f));
  CHECK(b.get_corr_vsep().x == 0.f);
  CHECK(b.get_corr_vfuga().x == FAT_FUGA * cosf(angle));
  CHECK(b.get_corr_vfuga().y == FAT_FUGA * sinf(angle));
}

TEST_CASE("Boid non scappa dal predatore distante") {
  Boid b(Two_Vec{{100.f, 100.f}, {0.f, 0.f}});
  Predator p({300.f, 300.f});

  std::vector<Boid> boids = {b};
  std::vector<Predator> predators = {p};

  evaluate_corr_fuga(boids, predators);

  CHECK(boids[0].get_corr_vfuga().x == 0.f);
  CHECK(boids[0].get_corr_vfuga().y == 0.f);
}
TEST_CASE("Predatore individua il boid più vicino") {
  Boid b1(Two_Vec{{100.f, 100.f}, {0.f, 0.f}});
  Boid b2(Two_Vec{{300.f, 300.f}, {0.f, 0.f}});
  Predator p({90.f, 100.f});

  std::vector<Boid> boids = {b1, b2};
  std::vector<Predator> predators = {p};

  evaluate_pred_correction(predators, boids);

  Vec2f vel = predators[0].get_vel_inseg();
  CHECK(vel.x > 0.f);
}

TEST_CASE("Test della funzione reset_corr") {
  SUBCASE("Boid") {
    Boid b(Two_Vec{{0.f, 0.f}, {1.f, 1.f}});
    b.vel_sep({1.f, 1.f}, 1.f);
    b.vel_all({1.f, 1.f}, 1.f);
    b.vel_coes({1.f, 1.f}, 1.f);
    b.corr_vel_fuga(0.f);

    b.reset_corr();

    CHECK(b.get_corr_vsep() == Vec2f{0.f, 0.f});
    CHECK(b.get_corr_vall() == Vec2f{0.f, 0.f});
    CHECK(b.get_corr_vcoes() == Vec2f{0.f, 0.f});
  }
  SUBCASE("Predator") {
    Predator p({100.f, 100.f});
    p.vel_inseg(0.0f);
    p.vel_sep(3.14f);

    p.correction();
    p.reset_corr();

    CHECK(p.get_vel().x == 0.f);
    CHECK(p.get_vel().y == 0.f);
  }
}
TEST_CASE("Statistics") {
  using namespace statistics;
  SUBCASE("Velocità media dei boids") {
    Boid b1(Two_Vec{{0.f, 0.f}, {2.f, 0.f}});
    Boid b2(Two_Vec{{0.f, 0.f}, {0.f, 2.f}});
    Boid b3(Two_Vec{{0.f, 0.f}, {2.f, 2.f}});
    std::vector<Boid> boids = {b1, b2, b3};

    Vec2f expected_mean = {(2.f + 0.f + 2.f) / 3.f, (0.f + 2.f + 2.f) / 3.f};

    CHECK(mean_velocity(boids).x == doctest::Approx(expected_mean.x));
    CHECK(mean_velocity(boids).y == doctest::Approx(expected_mean.y));
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

    Vec2f deviation = mean_deviation(boids);
    Vec2f deviation_algo = mean_deviation_algo(boids);

    CHECK(deviation.x == doctest::Approx(dev_x));
    CHECK(deviation.y == doctest::Approx(dev_y));
    CHECK(deviation_algo.x == doctest::Approx(dev_x));
    CHECK(deviation_algo.y == doctest::Approx(dev_y));
  }

  SUBCASE("Deviazione nulla per velocità identiche") {
    Boid b1(Two_Vec{{0.f, 0.f}, {2.f, 3.f}});
    Boid b2(Two_Vec{{0.f, 0.f}, {2.f, 3.f}});
    Boid b3(Two_Vec{{0.f, 0.f}, {2.f, 3.f}});
    std::vector<Boid> boids = {b1, b2, b3};

    Vec2f dev = mean_deviation(boids);
    Vec2f dev_algo = mean_deviation_algo(boids);

    CHECK(dev.x == doctest::Approx(0.f));
    CHECK(dev.y == doctest::Approx(0.f));
    CHECK(dev_algo.x == doctest::Approx(0.f));
    CHECK(dev_algo.y == doctest::Approx(0.f));
  }

  SUBCASE(
      "Verifica uguaglianza dei due metodi calcolo media") {
    Boid b1(Two_Vec{{0.f, 0.f}, {2.f, 1.f}});
    Boid b2(Two_Vec{{0.f, 0.f}, {4.f, 3.f}});
    Boid b3(Two_Vec{{0.f, 0.f}, {6.f, 5.f}});
    std::vector<Boid> boids = {b1, b2, b3};

    Vec2f m1 = mean_velocity(boids);
    Vec2f m2 = mean_velocity_algo(boids);

    CHECK(m1.x == doctest::Approx(m2.x));
    CHECK(m1.y == doctest::Approx(m2.y));
  }
  SUBCASE("Verifica uguaglianza dei due metodi calcolo deviazioni") {
    Boid b1(Two_Vec{{0.f, 0.f}, {1.f, 1.f}});
    Boid b2(Two_Vec{{0.f, 0.f}, {3.f, 1.f}});
    Boid b3(Two_Vec{{0.f, 0.f}, {5.f, 1.f}});
    std::vector<Boid> boids = {b1, b2, b3};

    Vec2f d1 = mean_deviation(boids);
    Vec2f d2 = mean_deviation_algo(boids);

    CHECK(d1.x == doctest::Approx(d2.x));
    CHECK(d1.y == doctest::Approx(d2.y));
  }
}

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
