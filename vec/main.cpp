

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>
// #include <execution>
#include <iostream>
#include <random>

#include "boids.hpp"

int main() {
  try {
    Par parametres = init_parametres();
    std::vector<Boid> boids(parametres.N);

    sf::RenderWindow window(sf::VideoMode(MAX_POS + 200, 200 + MAX_POS),
                            "My window");
    window.setFramerateLimit(60);

    sf::RectangleShape rectangle({MAX_POS, MAX_POS});
    rectangle.setPosition({100.0f, 100.0f});
    rectangle.setOutlineColor(sf::Color::Black);
    rectangle.setOutlineThickness(2.0f);

    std::vector<sf::CircleShape> circles;
    std::vector<Predator> predators;
    add_boid(boids);

    std::vector<sf::ConvexShape> triangles(parametres.N);
    add_triangle(triangles);

    while (window.isOpen()) {
      sf::Event event;
      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
          window.close();
        }
        if (event.type == sf::Event::MouseButtonPressed) {
          float x = event.mouseButton.x;
          float y = event.mouseButton.y;
          if (x > 100.f && x < MAX_POS + 100.f && y > 100.f &&
              y < MAX_POS + 100.f && circles.size() < MAX_PRED) {
            Predator p(Vec{x, y});
            predators.push_back(p);

            sf::CircleShape c = crt_pred(x, y);
            circles.push_back(c);
          }
        }
      }
      assert(predators.size() <= 5);

      window.clear(sf::Color::White);
      window.draw(rectangle);

      evaluate_pred_correction(predators, boids);
      evaluate_corr_fuga(boids, predators);
      evaluate_correction(boids, parametres);

      {
        auto it_c = circles.begin();
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
        for (auto it = triangles.begin(); it != triangles.end(); ++it, ++it_b) {
          init_tr((*it_b), (*it));

          (*it_b).correction();
          (*it_b).reset_corr();
          (*it_b).limit();

          window.draw(*it);
        }
      }

      erase_boid(boids, predators, triangles);
      window.display();
    }
  } catch (std::exception const& e) {
    std::cerr << "Caught exception: '" << e.what() << "'\n";
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Caught unknown exception\n";
    return EXIT_FAILURE;
  }
}
