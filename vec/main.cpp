

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

    sf::RenderWindow window(sf::VideoMode(600, 600), "My window");
    window.setFramerateLimit(60);

    sf::RectangleShape rectangle({400.0f, 400.0f});
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
            Predator p(Vec{x, y}, Vec{0.f, 0.f});
            predators.push_back(p);

            sf::CircleShape c = crt_pred(x, y);
            circles.push_back(c);
          }
        }
      }
      assert(predators.size() <= 5);

      window.clear(sf::Color::White);
      window.draw(rectangle);

      {
        auto it_c = circles.begin();

        for (auto it_p = predators.begin(); it_p != predators.end();
             ++it_p, ++it_c) {
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
          if (delta_pos == Vec{0.f, 0.f}) {
          } else {
            (*it_p).corr_vel_pred(std::atan2f(delta_pos.y, delta_pos.x));
          }
          (*it_c).setPosition({(*it_p).get_pos().x, (*it_p).get_pos().y});
          window.draw(*it_c);
        }
      }
      
      if(erase_boid(boids, predators, triangles)){
        assert(boids.size() < parametres.N);
        assert(triangles.size() < parametres.N);
      }
      evaluate_correction(boids, parametres);
      {
        auto it_b = boids.begin();
        for (auto it = triangles.begin(); it != triangles.end(); ++it, ++it_b) {
          init_tr((*it_b), (*it));

          (*it_b).correction();
          (*it_b).limit();
          (*it_b).vel_max();

          window.draw(*it);
        }
      }

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
